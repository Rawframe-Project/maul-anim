// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Decimal to binary by the multiprecision decimal method (mani-0007):
// the number's digits kept in decimal, halved or doubled by shifts of up
// to 60 bits until the value lies in [1/2, 1), the binary exponent
// counted on the way, then the format's bits taken and rounded half to
// even. 800 digits hold every halfway point between binary64 values
// exactly; nonzero digits dropped past them only ever push a halfway
// case up, which a sticky flag records. The method is the one Go's
// strconv and wuffs describe; this is its own writing.

#include "decimal.h"

#include <stdbool.h>
#include <stdint.h>

#define MANI_DECIMAL_DIGITS 800
// The widest shift that keeps digit * 2^k plus a carry in 64 bits.
#define MANI_DECIMAL_MAX_SHIFT 60

// A decimal 0.d1 d2 ... dn x 10^point: digits 0 to 9, the first not 0,
// trailing zeros trimmed; no digits for 0.
typedef struct Decimal
{
    uint8_t digits[MANI_DECIMAL_DIGITS];
    uint32_t count;
    int32_t point;
    bool negative;
    // Nonzero digits were dropped past the buffer.
    bool truncated;
} Decimal;

static void Trim(Decimal* d)
{
    while (d->count > 0 && d->digits[d->count - 1] == 0)
    {
        --d->count;
    }
    if (d->count == 0)
    {
        d->point = 0;
    }
}

static bool IsDigit(char c)
{
    return c >= '0' && c <= '9';
}

// The digits before any exponent, into d; returns the point they give
// and leaves *at at the exponent's letter or the end.
static int64_t ParseDigits(const char* text, size_t length, size_t* at, Decimal* d)
{
    int64_t point = 0;
    bool fraction = false;
    for (; *at < length && text[*at] != 'e' && text[*at] != 'E'; ++*at)
    {
        char c = text[*at];
        if (c == '.')
        {
            fraction = true;
            continue;
        }
        if (c == '0' && d->count == 0)
        {
            // A leading zero: after the point it moves the point down.
            point -= fraction ? 1 : 0;
            continue;
        }
        if (d->count < MANI_DECIMAL_DIGITS)
        {
            d->digits[d->count++] = (uint8_t)(c - '0');
        }
        else if (c != '0')
        {
            d->truncated = true;
        }
        point += fraction ? 0 : 1;
    }
    return point;
}

// The exponent after its letter at `at`, saturated far past any value a
// format holds.
static int64_t ParseExponent(const char* text, size_t length, size_t at)
{
    bool negative = at < length && text[at] == '-';
    at += at < length && (text[at] == '-' || text[at] == '+') ? 1 : 0;
    int64_t exponent = 0;
    for (; at < length && IsDigit(text[at]); ++at)
    {
        exponent = exponent < 100000000 ? exponent * 10 + (text[at] - '0') : exponent;
    }
    return negative ? -exponent : exponent;
}

// The digits and the point of a JSON number.
static void Parse(const char* text, size_t length, Decimal* d)
{
    d->count = 0;
    d->truncated = false;
    d->negative = length > 0 && text[0] == '-';
    size_t at = d->negative ? 1 : 0;
    int64_t point = ParseDigits(text, length, &at, d);
    if (at < length)
    {
        point += ParseExponent(text, length, at + 1);
    }
    point = point > 100000000 ? 100000000 : (point < -100000000 ? -100000000 : point);
    d->point = (int32_t)point;
    Trim(d);
}

// Divides by 2^k, 1 <= k <= 60.
static void ShiftRight(Decimal* d, uint32_t k)
{
    uint32_t read = 0;
    uint32_t write = 0;
    uint64_t n = 0;
    // Enough leading digits for a first nonzero digit out.
    while ((n >> k) == 0)
    {
        if (read >= d->count)
        {
            while ((n >> k) == 0)
            {
                n *= 10;
                ++read;
            }
            break;
        }
        n = n * 10 + d->digits[read++];
    }
    d->point -= (int32_t)read - 1;
    uint64_t mask = ((uint64_t)1 << k) - 1;
    for (; read < d->count; ++read)
    {
        uint8_t next = d->digits[read];
        d->digits[write++] = (uint8_t)(n >> k);
        n = (n & mask) * 10 + next;
    }
    while (n > 0)
    {
        uint8_t digit = (uint8_t)(n >> k);
        n = (n & mask) * 10;
        if (write < MANI_DECIMAL_DIGITS)
        {
            d->digits[write++] = digit;
        }
        else if (digit > 0)
        {
            d->truncated = true;
        }
    }
    d->count = write;
    Trim(d);
}

// Multiplies by 2^k, 1 <= k <= 60.
static void ShiftLeft(Decimal* d, uint32_t k)
{
    // The product's digits, least significant first: at most 19 more
    // than the decimal had.
    uint8_t out[MANI_DECIMAL_DIGITS + 20];
    uint32_t made = 0;
    uint64_t n = 0;
    for (uint32_t r = d->count; r-- > 0;)
    {
        n += (uint64_t)d->digits[r] << k;
        uint64_t quotient = n / 10;
        out[made++] = (uint8_t)(n - 10 * quotient);
        n = quotient;
    }
    while (n > 0)
    {
        uint64_t quotient = n / 10;
        out[made++] = (uint8_t)(n - 10 * quotient);
        n = quotient;
    }
    d->point += (int32_t)(made - d->count);
    uint32_t keep = made < MANI_DECIMAL_DIGITS ? made : MANI_DECIMAL_DIGITS;
    for (uint32_t j = 0; j < made; ++j)
    {
        uint8_t digit = out[made - 1 - j];
        if (j < keep)
        {
            d->digits[j] = digit;
        }
        else if (digit > 0)
        {
            d->truncated = true;
        }
    }
    d->count = keep;
    Trim(d);
}

// Multiplies by 2^k for k above 0, divides by 2^-k below.
static void Shift(Decimal* d, int32_t k)
{
    while (k > 0 && d->count > 0)
    {
        uint32_t step = k > MANI_DECIMAL_MAX_SHIFT ? MANI_DECIMAL_MAX_SHIFT : (uint32_t)k;
        ShiftLeft(d, step);
        k -= (int32_t)step;
    }
    while (k < 0 && d->count > 0)
    {
        uint32_t step = -k > MANI_DECIMAL_MAX_SHIFT ? MANI_DECIMAL_MAX_SHIFT : (uint32_t)-k;
        ShiftRight(d, step);
        k += (int32_t)step;
    }
}

// Whether rounding to the first `at` digits goes up: past halfway, or
// exactly halfway (a last digit 5) to an odd digit, or halfway with
// digits dropped, which is really past it.
static bool RoundsUp(const Decimal* d, int32_t at)
{
    if (at < 0 || (uint32_t)at >= d->count)
    {
        return false;
    }
    if (d->digits[at] == 5 && (uint32_t)at + 1 == d->count)
    {
        return d->truncated || (at > 0 && d->digits[at - 1] % 2 == 1);
    }
    return d->digits[at] >= 5;
}

// The integer part rounded half to even; the point is at most 19 here.
static uint64_t RoundedInteger(const Decimal* d)
{
    uint64_t n = 0;
    int32_t i = 0;
    for (; i < d->point && (uint32_t)i < d->count; ++i)
    {
        n = n * 10 + d->digits[i];
    }
    for (; i < d->point; ++i)
    {
        n *= 10;
    }
    return n + (RoundsUp(d, d->point) ? 1u : 0u);
}

// The shift that moves the point by about `point` decimal places without
// overshooting: 2^powers[p] < 10^p.
static int32_t PointShift(int32_t point)
{
    static const int32_t kPowers[9] = {1, 3, 6, 9, 13, 16, 19, 23, 26};
    return point < 9 ? kPowers[point] : 27;
}

bool maniDecimalToBinary(const char* text, size_t length, maniBinaryFormat format,
                         uint64_t* bitsOut)
{
    const int32_t mantissaBits = format == maniBinary32 ? 23 : 52;
    const int32_t exponentBits = format == maniBinary32 ? 8 : 11;
    const int32_t bias = -((1 << (exponentBits - 1)) - 1);
    const int32_t maxField = (1 << exponentBits) - 1;
    Decimal d;
    Parse(text, length, &d);
    uint64_t sign = d.negative ? (uint64_t)1 << (mantissaBits + exponentBits) : 0;
    if (d.count == 0 || d.point < -330)
    {
        *bitsOut = sign;
        return true;
    }
    if (d.point > 310)
    {
        return false;
    }
    // Scale into [1/2, 1), counting the binary exponent.
    int32_t exponent = 0;
    while (d.point > 0)
    {
        int32_t n = PointShift(d.point);
        Shift(&d, -n);
        exponent += n;
    }
    while (d.point < 0 || (d.point == 0 && d.digits[0] < 5))
    {
        int32_t n = PointShift(-d.point);
        Shift(&d, n);
        exponent -= n;
    }
    // [1/2, 1) to the formats' [1, 2); below the least normal exponent,
    // a subnormal.
    --exponent;
    if (exponent < bias + 1)
    {
        int32_t n = bias + 1 - exponent;
        Shift(&d, -n);
        exponent += n;
    }
    if (exponent - bias >= maxField)
    {
        return false;
    }
    Shift(&d, mantissaBits + 1);
    uint64_t mantissa = RoundedInteger(&d);
    if (mantissa == (uint64_t)2 << mantissaBits)
    {
        // Rounding carried into a new bit.
        mantissa >>= 1;
        ++exponent;
        if (exponent - bias >= maxField)
        {
            return false;
        }
    }
    if ((mantissa & ((uint64_t)1 << mantissaBits)) == 0)
    {
        exponent = bias;
    }
    uint64_t field = (uint64_t)(exponent - bias);
    *bitsOut = sign | (field << mantissaBits) | (mantissa & (((uint64_t)1 << mantissaBits) - 1));
    return true;
}

bool maniDecimalToInteger(const char* text, size_t length, int64_t minimum, int64_t maximum,
                          int64_t* valueOut)
{
    Decimal d;
    Parse(text, length, &d);
    // Trailing zeros are trimmed, so a digit past the point is a
    // fraction; 19 places hold every int64.
    if (d.truncated || d.count > (uint32_t)(d.point > 0 ? d.point : 0) || d.point > 19)
    {
        return false;
    }
    uint64_t magnitude = 0;
    for (int32_t i = 0; i < d.point; ++i)
    {
        magnitude = magnitude * 10 + ((uint32_t)i < d.count ? d.digits[i] : 0u);
    }
    int64_t value = 0;
    if (d.negative)
    {
        if (magnitude > (uint64_t)INT64_MAX + 1)
        {
            return false;
        }
        value = magnitude == (uint64_t)INT64_MAX + 1 ? INT64_MIN : -(int64_t)magnitude;
    }
    else
    {
        if (magnitude > (uint64_t)INT64_MAX)
        {
            return false;
        }
        value = (int64_t)magnitude;
    }
    if (value < minimum || value > maximum)
    {
        return false;
    }
    *valueOut = value;
    return true;
}
