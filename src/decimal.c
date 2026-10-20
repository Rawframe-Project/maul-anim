// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Decimal to binary (mani-0007), first by Eisel and Lemire's method
// (Lemire 2021): the first 19 significant digits as an integer w, times
// 5^q truncated to 128 bits from a table, the high word enough for those
// digits (Mushtak and Lemire 2023); past 19 digits, w and w + 1 must
// round alike. Otherwise by the multiprecision decimal method: the
// number's digits kept in decimal, halved or doubled by shifts of up to
// 60 bits until the value lies in [1/2, 1), the binary exponent counted
// on the way, then the format's bits taken and rounded half to even. 800
// digits hold every halfway point between binary64 values exactly;
// nonzero digits dropped past them only ever push a halfway case up,
// which a sticky flag records. The methods are the ones fast_float, Go's
// strconv and wuffs describe; this is their own writing.

#include "decimal.h"

#include "bytes.h"

#include "generated/decimal_powers.h"

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

// A number as w x 10^q from its first 19 significant digits; `more` when
// a later digit is not 0.
typedef struct Short
{
    uint64_t w;
    int64_t q;
    bool negative;
    bool more;
} Short;

// Past 19 significant digits: the first 19 taken, the rest counted into
// q and noted when not 0, a digit at a time.
static void ParseLong(const char* text, size_t length, size_t at, Short* s)
{
    bool fraction = false;
    uint32_t taken = 0;
    s->w = 0;
    s->q = 0;
    for (; at < length && text[at] != 'e' && text[at] != 'E'; ++at)
    {
        char c = text[at];
        if (c == '.')
        {
            fraction = true;
            continue;
        }
        uint32_t digit = (uint32_t)(c - '0');
        if (taken == 0 && digit == 0)
        {
            s->q -= fraction ? 1 : 0;
            continue;
        }
        if (taken < 19)
        {
            s->w = s->w * 10 + digit;
            ++taken;
            s->q -= fraction ? 1 : 0;
        }
        else
        {
            s->more = s->more || digit != 0;
            s->q += fraction ? 0 : 1;
        }
    }
}

// Whether eight bytes are all ASCII digits: each byte's high half 3, and
// adding 6 leaves it 3 only from '0' to '9'.
static bool EightDigits(uint64_t v)
{
    return ((v & 0xF0F0F0F0F0F0F0F0u) | (((v + 0x0606060606060606u) & 0xF0F0F0F0F0F0F0F0u) >> 4)) ==
           0x3333333333333333u;
}

// Eight ASCII digits, the first in the lowest byte, as their value: pairs,
// then fours, then the eight, by three products.
static uint64_t EightValue(uint64_t v)
{
    v -= 0x3030303030303030u;
    v = v * 10 + (v >> 8);
    const uint64_t mask = 0x000000FF000000FFu;
    return (((v & mask) * 0x000F424000000064u) + (((v >> 16) & mask) * 0x0000271000000001u)) >> 32;
}

// Where a run of digits ends and w with them summed in.
typedef struct Digits
{
    size_t at;
    uint64_t w;
} Digits;

// Digits from at while they last, summed into w: eight at a time where
// eight are there, then one at a time.
static Digits ReadDigits(const char* text, size_t length, size_t at, uint64_t w)
{
    while (length - at >= 8 && EightDigits(maniLoadU64((const uint8_t*)text + at)))
    {
        w = w * 100000000u + EightValue(maniLoadU64((const uint8_t*)text + at));
        at += 8;
    }
    for (; at < length && IsDigit(text[at]); ++at)
    {
        w = w * 10 + (uint64_t)(text[at] - '0');
    }
    return (Digits){at, w};
}

// The digits read in tight loops, the integer then the fraction, w
// summed as it goes: exact while at most 19 digits are significant
// (below 10^19, so below 2^64); past them, read again by ParseLong.
static Short ParseShort(const char* text, size_t length)
{
    bool negative = length > 0 && text[0] == '-';
    size_t first = negative ? 1 : 0;
    Digits d = ReadDigits(text, length, first, 0);
    size_t digits = d.at - first;
    int64_t q = 0;
    if (d.at < length && text[d.at] == '.')
    {
        size_t point = d.at + 1;
        d = ReadDigits(text, length, point, d.w);
        q = -(int64_t)(d.at - point);
        digits += d.at - point;
    }
    Short s = {d.w, q, negative, false};
    if (digits > 19)
    {
        // Leading zeros are not significant: count them before reading
        // again.
        size_t zeros = 0;
        for (size_t k = first; k < d.at && (text[k] == '0' || text[k] == '.'); ++k)
        {
            zeros += text[k] == '0' ? 1 : 0;
        }
        if (digits - zeros > 19)
        {
            ParseLong(text, length, first, &s);
        }
    }
    if (d.at < length)
    {
        s.q += ParseExponent(text, length, d.at + 1);
    }
    return s;
}

// a x b as two words: the compiler's 128-bit product where it has one,
// else from 32-bit halves; the same words either way.
static void Multiply(uint64_t a, uint64_t b, uint64_t* high, uint64_t* low)
{
#if defined(__SIZEOF_INT128__)
    unsigned __int128 p = (unsigned __int128)a * b;
    *low = (uint64_t)p;
    *high = (uint64_t)(p >> 64);
#else
    uint64_t a0 = a & 0xFFFFFFFFu;
    uint64_t a1 = a >> 32;
    uint64_t b0 = b & 0xFFFFFFFFu;
    uint64_t b1 = b >> 32;
    uint64_t p00 = a0 * b0;
    uint64_t p01 = a0 * b1;
    uint64_t p10 = a1 * b0;
    uint64_t p11 = a1 * b1;
    uint64_t middle = (p00 >> 32) + (p01 & 0xFFFFFFFFu) + (p10 & 0xFFFFFFFFu);
    *low = (middle << 32) | (p00 & 0xFFFFFFFFu);
    *high = p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32);
#endif
}

// x not 0: the compiler's count where it has one, else halves, quarters
// and on, six steps.
static uint32_t LeadingZeros(uint64_t x)
{
#if defined(__GNUC__) || defined(__clang__)
    return (uint32_t)__builtin_clzll(x);
#else
    uint32_t n = 0;
    for (uint32_t step = 32; step > 0; step >>= 1)
    {
        if ((x >> (64 - step)) == 0)
        {
            x <<= step;
            n += step;
        }
    }
    return n;
#endif
}

// A format as the method reads it: its mantissa's stored bits, its
// least exponent and its exponent field of all ones; the powers of ten
// below which any w rounds to 0 and past which any w overflows; the
// powers within which a product can fall exactly halfway.
typedef struct Format
{
    int32_t mantissaBits;
    int32_t minimumExponent;
    int32_t infinite;
    int32_t smallestPower;
    int32_t largestPower;
    int32_t evenLow;
    int32_t evenHigh;
} Format;

static const Format kFormats[2] = {
    {23, -127, 0xFF, -65, 38, -17, 10},
    {52, -1023, 0x7FF, MANI_DECIMAL_SMALLEST_POWER, MANI_DECIMAL_LARGEST_POWER, -4, 23}};

// w x 10^q (w not 0, q within the format's powers) as a mantissa and a
// biased exponent: infinite for overflow, 0 and 0 for zero.
static void Compute(const Format* f, uint64_t w, int64_t q, uint64_t* mantissaOut,
                    int32_t* powerOut)
{
    uint32_t lz = LeadingZeros(w);
    w <<= lz;
    size_t index = 2 * (size_t)(q - MANI_DECIMAL_SMALLEST_POWER);
    uint64_t high = 0;
    uint64_t low = 0;
    Multiply(w, kPowersOfFive[index], &high, &low);
    uint64_t mask = UINT64_MAX >> (f->mantissaBits + 3);
    if ((high & mask) == mask)
    {
        uint64_t high2 = 0;
        uint64_t low2 = 0;
        Multiply(w, kPowersOfFive[index + 1], &high2, &low2);
        low += high2;
        high += high2 > low ? 1u : 0u;
    }
    uint32_t upper = (uint32_t)(high >> 63);
    uint32_t drop = upper + 64u - (uint32_t)f->mantissaBits - 3u;
    uint64_t mantissa = high >> drop;
    // floor(q log2 10), as floor(q log2 5) + q: exact for |q| below 400.
    int32_t power = (int32_t)(((152170 + 65536) * q) >> 16) + 63 + (int32_t)upper - (int32_t)lz -
                    f->minimumExponent;
    if (power <= 0)
    {
        if (-power + 1 >= 64)
        {
            *mantissaOut = 0;
            *powerOut = 0;
            return;
        }
        mantissa >>= -power + 1;
        mantissa += mantissa & 1u;
        mantissa >>= 1;
        *powerOut = mantissa < ((uint64_t)1 << f->mantissaBits) ? 0 : 1;
        *mantissaOut = mantissa & (((uint64_t)1 << f->mantissaBits) - 1);
        return;
    }
    // Halfway between two values only when nothing but zeros was dropped.
    if (low <= 1 && q >= f->evenLow && q <= f->evenHigh && (mantissa & 3u) == 1u &&
        (mantissa << drop) == high)
    {
        mantissa &= ~(uint64_t)1;
    }
    mantissa += mantissa & 1u;
    mantissa >>= 1;
    if (mantissa >= ((uint64_t)2 << f->mantissaBits))
    {
        mantissa = (uint64_t)1 << f->mantissaBits;
        ++power;
    }
    mantissa &= ~((uint64_t)1 << f->mantissaBits);
    if (power >= f->infinite)
    {
        power = f->infinite;
        mantissa = 0;
    }
    *mantissaOut = mantissa;
    *powerOut = power;
}

// The fast path's answer, or false when the exact method must decide (a
// value past 19 digits whose neighbours round apart).
static bool Fast(const char* text, size_t length, maniBinaryFormat format, bool* fitsOut,
                 uint64_t* bitsOut)
{
    const Format* f = &kFormats[format == maniBinary32 ? 0 : 1];
    Short s = ParseShort(text, length);
    uint64_t sign =
        s.negative ? (uint64_t)1 << (f->mantissaBits + (format == maniBinary32 ? 8 : 11)) : 0;
    *fitsOut = true;
    if (s.w == 0 || s.q < f->smallestPower)
    {
        *bitsOut = sign;
        return true;
    }
    if (s.q > f->largestPower)
    {
        *fitsOut = false;
        return true;
    }
    uint64_t mantissa = 0;
    int32_t power = 0;
    Compute(f, s.w, s.q, &mantissa, &power);
    if (s.more)
    {
        uint64_t mantissa2 = 0;
        int32_t power2 = 0;
        Compute(f, s.w + 1, s.q, &mantissa2, &power2);
        if (mantissa2 != mantissa || power2 != power)
        {
            return false;
        }
    }
    if (power == f->infinite)
    {
        *fitsOut = false;
        return true;
    }
    *bitsOut = sign | ((uint64_t)power << f->mantissaBits) | mantissa;
    return true;
}

// The multiprecision method, for what the fast one leaves open.
static bool Exact(const char* text, size_t length, maniBinaryFormat format, uint64_t* bitsOut)
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

bool maniDecimalToBinary(const char* text, size_t length, maniBinaryFormat format,
                         uint64_t* bitsOut)
{
    bool fits = true;
    uint64_t bits = 0;
    if (Fast(text, length, format, &fits, &bits))
    {
        if (fits)
        {
            *bitsOut = bits;
        }
        return fits;
    }
    return Exact(text, length, format, bitsOut);
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
