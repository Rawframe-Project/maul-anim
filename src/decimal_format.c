// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binary32 to decimal: the value's exact decimal expansion in
// a small base-1e9 integer, then each length from 1 to 9 digits tried,
// its two nearest candidates read back through maniDecimalToBinary; the
// first length that reads back wins, the nearer candidate (the even one
// of a tie). Then ECMAScript's layout (ES5.1 9.8.1).

#include "decimal_format.h"

#include "decimal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// 2^128 and 5^149 times 2^24 both fit in 16 limbs of 1e9.
#define LIMBS 16u
// The exact expansion's longest digit string.
#define DIGITS (9u * LIMBS)

typedef struct Big
{
    uint32_t limb[LIMBS];
    uint32_t count;
} Big;

static void MultiplySmall(Big* b, uint32_t factor)
{
    uint64_t carry = 0;
    for (uint32_t i = 0; i < b->count; ++i)
    {
        uint64_t x = (uint64_t)b->limb[i] * factor + carry;
        b->limb[i] = (uint32_t)(x % 1000000000u);
        carry = x / 1000000000u;
    }
    if (carry != 0 && b->count < LIMBS)
    {
        b->limb[b->count++] = (uint32_t)carry;
    }
}

// The digits of b, most significant first, without leading zeros.
static uint32_t Digits(const Big* b, char* out)
{
    uint32_t n = 0;
    for (uint32_t i = b->count; i-- > 0;)
    {
        uint32_t v = b->limb[i];
        char group[9];
        for (int k = 8; k >= 0; --k)
        {
            group[k] = (char)('0' + v % 10u);
            v /= 10u;
        }
        for (int k = 0; k < 9; ++k)
        {
            if (n > 0 || group[k] != '0')
            {
                out[n++] = group[k];
            }
        }
    }
    return n;
}

// The exact value of m times 2^e as digits and a power of ten: digits
// times 10^power, trailing zeros taken into the power.
static uint32_t Exact(uint32_t m, int32_t e, char* digits, int32_t* powerOut)
{
    // m is below 2^24, one limb; the factors go in by 2^29 and 5^13, the
    // most a limb times them keeps within 64 bits.
    Big b = {{m}, 1};
    for (int32_t k = e; k > 0; k -= 29)
    {
        MultiplySmall(&b, (uint32_t)1 << (k < 29 ? k : 29));
    }
    for (int32_t k = -e; k > 0; k -= 13)
    {
        uint32_t factor = 1;
        for (int32_t i = 0; i < (k < 13 ? k : 13); ++i)
        {
            factor *= 5u;
        }
        MultiplySmall(&b, factor);
    }
    uint32_t n = Digits(&b, digits);
    int32_t power = e < 0 ? e : 0;
    while (n > 1 && digits[n - 1] == '0')
    {
        --n;
        ++power;
    }
    *powerOut = power;
    return n;
}

// Writes digits times 10^power as a JSON number; returns its length.
static size_t Scientific(const char* digits, uint32_t n, int32_t power, char* out)
{
    memcpy(out, digits, n);
    size_t at = n;
    out[at++] = 'e';
    int32_t p = power;
    if (p < 0)
    {
        out[at++] = '-';
        p = -p;
    }
    char reversed[12];
    uint32_t r = 0;
    do
    {
        reversed[r++] = (char)('0' + p % 10);
        p /= 10;
    } while (p != 0);
    while (r > 0)
    {
        out[at++] = reversed[--r];
    }
    return at;
}

// Whether digits times 10^power reads back to bits.
static bool ReadsBack(const char* digits, uint32_t n, int32_t power, uint32_t bits)
{
    char text[64];
    size_t length = Scientific(digits, n, power, text);
    uint64_t read = 0;
    return maniDecimalToBinary(text, length, maniBinary32, &read) && (uint32_t)read == bits;
}

// The candidate one unit above the first n digits, into out; returns 1
// when it carries into a new digit (out then holds 1 and n - 1 zeros, a
// power of ten higher).
static uint32_t Above(const char* digits, uint32_t n, char* out)
{
    memcpy(out, digits, n);
    uint32_t i = n;
    while (i > 0 && out[i - 1] == '9')
    {
        out[--i] = '0';
    }
    if (i == 0)
    {
        out[0] = '1';
        return 1;
    }
    out[i - 1] = (char)(out[i - 1] + 1);
    return 0;
}

// How the digits past n compare with half a unit: -1 below, 0 at, 1 above.
static int Half(const char* digits, uint32_t total, uint32_t n)
{
    if (digits[n] != '5')
    {
        return digits[n] < '5' ? -1 : 1;
    }
    for (uint32_t i = n + 1; i < total; ++i)
    {
        if (digits[i] != '0')
        {
            return 1;
        }
    }
    return 0;
}

// The n-digit candidate nearer the exact digits (the even one of a tie)
// of those that read back, into best with its power of ten in *power;
// false when neither does.
static bool Candidate(const char* digits, uint32_t total, uint32_t n, uint32_t bits, char* best,
                      int32_t* power)
{
    int32_t shift = *power + (int32_t)(total - n);
    char up[16];
    int32_t carry = (int32_t)Above(digits, n, up);
    bool below = ReadsBack(digits, n, shift, bits);
    bool above = ReadsBack(up, n, shift + carry, bits);
    if (!below && !above)
    {
        return false;
    }
    int half = Half(digits, total, n);
    bool even = ((digits[n - 1] - '0') % 2) == 0;
    bool pickBelow = below && (!above || half < 0 || (half == 0 && even));
    memcpy(best, pickBelow ? digits : up, n);
    *power = pickBelow ? shift : shift + carry;
    return true;
}

// The shortest digits that read back: into best, their count returned,
// their power of ten in *power. A length that reads back makes every
// longer one read back (its candidates lie nearer), so the lengths are
// searched by halves.
static uint32_t Shortest(const char* digits, uint32_t total, int32_t* power, uint32_t bits,
                         char* best)
{
    uint32_t low = 1;
    uint32_t high = total <= 9 ? total : 9;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        char scratch[16];
        int32_t scratchPower = *power;
        if (Candidate(digits, total, mid, bits, scratch, &scratchPower))
        {
            high = mid;
        }
        else
        {
            low = mid + 1;
        }
    }
    // Nine digits always read back; fewer, the exact digits do.
    if (low < total && Candidate(digits, total, low, bits, best, power))
    {
        return low;
    }
    memcpy(best, digits, total);
    return total;
}

// ECMAScript's layout of s (k digits) times 10^(n - k).
static size_t Layout(const char* s, uint32_t k, int32_t n, char* out)
{
    size_t at = 0;
    if ((int32_t)k <= n && n <= 21)
    {
        memcpy(out, s, k);
        at = k;
        for (int32_t i = (int32_t)k; i < n; ++i)
        {
            out[at++] = '0';
        }
        return at;
    }
    if (0 < n && n <= 21)
    {
        memcpy(out, s, (size_t)n);
        out[n] = '.';
        memcpy(out + n + 1, s + n, k - (uint32_t)n);
        return k + 1;
    }
    if (-6 < n && n <= 0)
    {
        out[at++] = '0';
        out[at++] = '.';
        for (int32_t i = 0; i < -n; ++i)
        {
            out[at++] = '0';
        }
        memcpy(out + at, s, k);
        return at + k;
    }
    out[at++] = s[0];
    if (k > 1)
    {
        out[at++] = '.';
        memcpy(out + at, s + 1, k - 1);
        at += k - 1;
    }
    out[at++] = 'e';
    int32_t x = n - 1;
    out[at++] = x < 0 ? '-' : '+';
    x = x < 0 ? -x : x;
    char reversed[12];
    uint32_t r = 0;
    do
    {
        reversed[r++] = (char)('0' + x % 10);
        x /= 10;
    } while (x != 0);
    while (r > 0)
    {
        out[at++] = reversed[--r];
    }
    return at;
}

size_t maniFormatBinary32(float value, char out[MANI_DECIMAL_MAX])
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    uint32_t exponent = (bits >> 23) & 0xFFu;
    uint32_t fraction = bits & 0x7FFFFFu;
    size_t at = 0;
    if (exponent == 0xFFu)
    {
        return 0;
    }
    if ((bits >> 31) != 0)
    {
        out[at++] = '-';
    }
    if (exponent == 0 && fraction == 0)
    {
        out[at++] = '0';
        return at;
    }
    uint32_t m = exponent == 0 ? fraction : fraction | 0x800000u;
    int32_t e = exponent == 0 ? -149 : (int32_t)exponent - 150;
    char digits[DIGITS];
    int32_t power = 0;
    uint32_t total = Exact(m, e, digits, &power);
    char best[DIGITS];
    uint32_t k = Shortest(digits, total, &power, bits & 0x7FFFFFFFu, best);
    return at + Layout(best, k, power + (int32_t)k, out + at);
}
