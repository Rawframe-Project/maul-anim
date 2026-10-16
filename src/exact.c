// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Integers of up to 36 32-bit limbs, sign and magnitude: a difference of
// two scaled coordinates takes 278 bits, an in-circle determinant 1117.
// A binary32 is (2^23 + m) 2^(e - 150) when normal, m 2^-149 when not.

#include "exact.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define LIMBS 36u

typedef struct Exact
{
    uint32_t limb[LIMBS];
    uint32_t length;
    int sign;
} Exact;

static void Trim(Exact* e)
{
    while (e->length > 0 && e->limb[e->length - 1] == 0)
    {
        e->length -= 1;
    }
    e->sign = e->length == 0 ? 0 : e->sign;
}

// A binary32 times 2^149.
static Exact FromFloat(float f)
{
    Exact e;
    memset(&e, 0, sizeof e);
    uint32_t bits = 0;
    memcpy(&bits, &f, sizeof bits);
    uint32_t exponent = (bits >> 23) & 0xFFu;
    uint32_t mantissa = bits & 0x7FFFFFu;
    uint32_t shift = exponent == 0 ? 0 : exponent - 1;
    uint64_t m = exponent == 0 ? mantissa : mantissa + 0x800000u;
    uint32_t at = shift / 32;
    uint64_t moved = m << (shift % 32);
    e.limb[at] = (uint32_t)moved;
    e.limb[at + 1] = (uint32_t)(moved >> 32);
    e.length = at + 2;
    e.sign = (bits >> 31) != 0 ? -1 : 1;
    Trim(&e);
    return e;
}

// The magnitudes compared: -1, 0 or 1.
static int Compare(const Exact* a, const Exact* b)
{
    if (a->length != b->length)
    {
        return a->length < b->length ? -1 : 1;
    }
    for (uint32_t k = a->length; k-- > 0;)
    {
        if (a->limb[k] != b->limb[k])
        {
            return a->limb[k] < b->limb[k] ? -1 : 1;
        }
    }
    return 0;
}

// |a| + |b|, signed s.
static Exact AddMagnitudes(const Exact* a, const Exact* b, int s)
{
    Exact r;
    memset(&r, 0, sizeof r);
    uint32_t n = a->length > b->length ? a->length : b->length;
    uint64_t carry = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        uint64_t sum = carry + (k < a->length ? a->limb[k] : 0) + (k < b->length ? b->limb[k] : 0);
        r.limb[k] = (uint32_t)sum;
        carry = sum >> 32;
    }
    r.limb[n] = (uint32_t)carry;
    r.length = n + 1;
    r.sign = s;
    Trim(&r);
    return r;
}

// |a| - |b| for |a| >= |b|, signed s.
static Exact SubtractMagnitudes(const Exact* a, const Exact* b, int s)
{
    Exact r;
    memset(&r, 0, sizeof r);
    uint64_t borrow = 0;
    for (uint32_t k = 0; k < a->length; ++k)
    {
        uint64_t x = a->limb[k];
        uint64_t y = (k < b->length ? b->limb[k] : 0) + borrow;
        r.limb[k] = (uint32_t)(x - y);
        borrow = x < y ? 1 : 0;
    }
    r.length = a->length;
    r.sign = s;
    Trim(&r);
    return r;
}

static Exact Add(const Exact* a, const Exact* b)
{
    if (a->sign == 0 || b->sign == 0)
    {
        return a->sign == 0 ? *b : *a;
    }
    if (a->sign == b->sign)
    {
        return AddMagnitudes(a, b, a->sign);
    }
    int c = Compare(a, b);
    return c >= 0 ? SubtractMagnitudes(a, b, a->sign) : SubtractMagnitudes(b, a, b->sign);
}

static Exact Subtract(const Exact* a, const Exact* b)
{
    Exact negated = *b;
    negated.sign = -negated.sign;
    return Add(a, &negated);
}

static Exact Multiply(const Exact* a, const Exact* b)
{
    Exact r;
    memset(&r, 0, sizeof r);
    for (uint32_t i = 0; i < a->length; ++i)
    {
        uint64_t carry = 0;
        for (uint32_t j = 0; j < b->length; ++j)
        {
            uint64_t t = (uint64_t)a->limb[i] * b->limb[j] + r.limb[i + j] + carry;
            r.limb[i + j] = (uint32_t)t;
            carry = t >> 32;
        }
        r.limb[i + b->length] = (uint32_t)carry;
    }
    r.length = a->length + b->length;
    r.sign = a->sign * b->sign;
    Trim(&r);
    return r;
}

// The 2 by 2 determinant p q - r s.
static Exact Cross(const Exact* p, const Exact* q, const Exact* r, const Exact* s)
{
    Exact pq = Multiply(p, q);
    Exact rs = Multiply(r, s);
    return Subtract(&pq, &rs);
}

int maniExactOrient(const float* a, const float* b, const float* c)
{
    Exact ax = FromFloat(a[0]);
    Exact ay = FromFloat(a[1]);
    Exact bx = FromFloat(b[0]);
    Exact by = FromFloat(b[1]);
    Exact cx = FromFloat(c[0]);
    Exact cy = FromFloat(c[1]);
    Exact ux = Subtract(&bx, &ax);
    Exact uy = Subtract(&by, &ay);
    Exact vx = Subtract(&cx, &ax);
    Exact vy = Subtract(&cy, &ay);
    return Cross(&ux, &vy, &uy, &vx).sign;
}

int maniExactInCircle(const float* a, const float* b, const float* c, const float* d)
{
    const float* p[3] = {a, b, c};
    Exact dx = FromFloat(d[0]);
    Exact dy = FromFloat(d[1]);
    Exact x[3];
    Exact y[3];
    Exact lift[3];
    for (int k = 0; k < 3; ++k)
    {
        Exact px = FromFloat(p[k][0]);
        Exact py = FromFloat(p[k][1]);
        x[k] = Subtract(&px, &dx);
        y[k] = Subtract(&py, &dy);
        Exact xx = Multiply(&x[k], &x[k]);
        Exact yy = Multiply(&y[k], &y[k]);
        lift[k] = Add(&xx, &yy);
    }
    Exact sum;
    memset(&sum, 0, sizeof sum);
    for (int k = 0; k < 3; ++k)
    {
        int i = (k + 1) % 3;
        int j = (k + 2) % 3;
        Exact minor = Cross(&x[i], &y[j], &x[j], &y[i]);
        Exact term = Multiply(&lift[k], &minor);
        sum = Add(&sum, &term);
    }
    return sum.sign;
}
