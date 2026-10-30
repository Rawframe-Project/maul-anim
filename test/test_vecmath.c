// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Vectors, quaternions and affine matrices against binary64 references
// of the textbook formulas, with their properties and edge cases.

#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of HashAll on the reference build; every platform must match.
#define PINNED_HASH 0x8bfb0de0498f9bc5ull

// A fixed sequence of numbers in [-1, 1], the same everywhere.
static uint32_t s_state = 12345u;

static float Next(void)
{
    s_state = s_state * 1664525u + 1013904223u;
    return (float)(s_state >> 8) / 8388608.0f - 1.0f;
}

// The numbers are drawn into locals one at a time: C leaves the order in
// which a call's arguments or an initializer's elements are evaluated
// unspecified, and compilers differ.
static maniQuat RandomUnitQuat(void)
{
    double x = (double)Next();
    double y = (double)Next();
    double z = (double)Next();
    double w = (double)Next();
    double n = sqrt(x * x + y * y + z * z + w * w);
    return (maniQuat){(float)(x / n), (float)(y / n), (float)(z / n), (float)(w / n)};
}

static maniVec3 RandomVec(float scale)
{
    float x = Next() * scale;
    float y = Next() * scale;
    float z = Next() * scale;
    return maniVec3Make(x, y, z);
}

static bool Near(float a, double b, double tolerance)
{
    return fabs((double)a - b) <= tolerance;
}

static bool NearF(float a, float b, double tolerance)
{
    return Near(a, (double)b, tolerance);
}

static bool SameBits(const void* a, const void* b, size_t size)
{
    return memcmp(a, b, size) == 0;
}

// The binary64 rotation of v by q (q v q*).
static void RotateDouble(maniQuat q, maniVec3 v, double out[3])
{
    double x = (double)q.x;
    double y = (double)q.y;
    double z = (double)q.z;
    double w = (double)q.w;
    double m[3][3] = {
        {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
        {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
        {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)},
    };
    double in[3] = {(double)v.x, (double)v.y, (double)v.z};
    for (int32_t r = 0; r < 3; ++r)
    {
        out[r] = m[r][0] * in[0] + m[r][1] * in[1] + m[r][2] * in[2];
    }
}

static void TestRotateMatchesTheMatrix(void)
{
    bool ok = true;
    for (int32_t i = 0; i < 10000; ++i)
    {
        maniQuat q = RandomUnitQuat();
        maniVec3 v = RandomVec(10.0f);
        maniVec3 r = maniQuatRotate(q, v);
        double expect[3];
        RotateDouble(q, v, expect);
        ok = ok && Near(r.x, expect[0], 1e-5) && Near(r.y, expect[1], 1e-5) &&
             Near(r.z, expect[2], 1e-5);
    }
    CHECK(ok, "rotation within 1e-5 of the binary64 matrix at length 10");
}

static void TestProductTurnsBySecondThenFirst(void)
{
    bool ok = true;
    for (int32_t i = 0; i < 10000; ++i)
    {
        maniQuat a = RandomUnitQuat();
        maniQuat b = RandomUnitQuat();
        maniVec3 v = RandomVec(1.0f);
        maniVec3 both = maniQuatRotate(maniQuatMul(a, b), v);
        double first[3];
        double second[3];
        RotateDouble(b, v, first);
        RotateDouble(a, maniVec3Make((float)first[0], (float)first[1], (float)first[2]), second);
        ok = ok && Near(both.x, second[0], 2e-6) && Near(both.y, second[1], 2e-6) &&
             Near(both.z, second[2], 2e-6);
    }
    CHECK(ok, "a * b turns by b, then a");
    maniQuat q = RandomUnitQuat();
    maniQuat r = maniQuatMul(q, maniQuatConjugate(q));
    CHECK(Near(r.w, 1.0, 1e-6) && Near(r.x, 0.0, 1e-6), "q times its conjugate is the identity");
}

static void TestNormalizeIsTotal(void)
{
    maniQuat id = maniQuatIdentity();
    maniQuat z = maniQuatNormalize((maniQuat){0.0f, 0.0f, 0.0f, 0.0f});
    maniQuat n = maniQuatNormalize((maniQuat){NAN, 0.0f, 0.0f, 1.0f});
    maniQuat inf = maniQuatNormalize((maniQuat){INFINITY, 0.0f, 0.0f, 1.0f});
    CHECK(SameBits(&z, &id, sizeof id), "zero gives the identity");
    CHECK(SameBits(&n, &id, sizeof id), "NaN gives the identity");
    CHECK(SameBits(&inf, &id, sizeof id), "infinity gives the identity");
    maniQuat big = maniQuatNormalize((maniQuat){3e30f, 0.0f, 0.0f, 4e30f});
    CHECK(Near(big.x, 0.6, 1e-7) && Near(big.w, 0.8, 1e-7), "huge components normalize");
    maniQuat tiny = maniQuatNormalize((maniQuat){3e-30f, 0.0f, 0.0f, 4e-30f});
    CHECK(Near(tiny.x, 0.6, 1e-7) && Near(tiny.w, 0.8, 1e-7), "tiny components normalize");
    maniVec3 fallback = maniVec3Make(0.0f, 1.0f, 0.0f);
    maniVec3 v = maniVec3Normalize(maniVec3Make(0.0f, 0.0f, 0.0f), fallback);
    CHECK(SameBits(&v, &fallback, sizeof v), "a zero vector gives the fallback");
    v = maniVec3Normalize(maniVec3Make(0.0f, 0.0f, 1e-30f), fallback);
    CHECK(v.z == 1.0f && v.x == 0.0f, "a tiny vector normalizes");
    CHECK(maniVec3Length(maniVec3Make(3e30f, 4e30f, 0.0f)) == 5e30f, "length without overflow");
}

static void TestNlerpHemispheresAndEnds(void)
{
    bool ok = true;
    for (int32_t i = 0; i < 1000; ++i)
    {
        maniQuat a = RandomUnitQuat();
        maniQuat b = RandomUnitQuat();
        maniQuat nb = {-b.x, -b.y, -b.z, -b.w};
        float t = (Next() + 1.0f) * 0.5f;
        maniQuat p = maniQuatNlerp(a, b, t);
        maniQuat q = maniQuatNlerp(a, nb, t);
        ok = ok && SameBits(&p, &q, sizeof p) && maniQuatDot(p, a) >= 0.0f;
    }
    CHECK(ok, "a negated end gives the same bits, on a's side");
    maniQuat a = RandomUnitQuat();
    maniQuat b = RandomUnitQuat();
    maniQuat at0 = maniQuatNlerp(a, b, 0.0f);
    CHECK(Near(maniQuatDot(at0, a), 1.0, 1e-6), "t = 0 is a");
}

static void TestSlerpMatchesTheReference(void)
{
    bool ok = true;
    bool ends = true;
    for (int32_t i = 0; i < 10000; ++i)
    {
        maniQuat a = RandomUnitQuat();
        maniQuat b = RandomUnitQuat();
        float t = (Next() + 1.0f) * 0.5f;
        maniQuat s = maniQuatSlerp(a, b, t);
        double ad[4] = {(double)a.x, (double)a.y, (double)a.z, (double)a.w};
        double bd[4] = {(double)b.x, (double)b.y, (double)b.z, (double)b.w};
        double sd[4] = {(double)s.x, (double)s.y, (double)s.z, (double)s.w};
        double d = ad[0] * bd[0] + ad[1] * bd[1] + ad[2] * bd[2] + ad[3] * bd[3];
        double sign = d < 0.0 ? -1.0 : 1.0;
        double theta = acos(fmin(1.0, fabs(d)));
        double td = (double)t;
        double wa = sin((1.0 - td) * theta) / sin(theta);
        double wb = sign * sin(td * theta) / sin(theta);
        for (int32_t k = 0; k < 4; ++k)
        {
            ok = ok && fabs(sd[k] - (wa * ad[k] + wb * bd[k])) <= 2e-6;
        }
        maniQuat s1 = maniQuatSlerp(a, b, 1.0f);
        ends = ends && Near(fabsf(maniQuatDot(s1, b)), 1.0, 1e-6);
    }
    CHECK(ok, "within 2e-6 of binary64 slerp");
    CHECK(ends, "t = 1 is b up to sign");
    maniQuat a = RandomUnitQuat();
    maniQuat same = maniQuatSlerp(a, a, 0.3f);
    CHECK(Near(maniQuatDot(same, a), 1.0, 1e-6), "equal ends give that rotation");
}

static void TestAxisAngle(void)
{
    maniQuat q = maniQuatFromAxisAngle(maniVec3Make(0.0f, 0.0f, 1.0f), 1.5707964f);
    maniVec3 r = maniQuatRotate(q, maniVec3Make(1.0f, 0.0f, 0.0f));
    CHECK(Near(r.x, 0.0, 1e-6) && Near(r.y, 1.0, 1e-6), "a quarter turn about z takes x to y");
    maniQuat zero = maniQuatFromAxisAngle(maniVec3Make(1.0f, 0.0f, 0.0f), 0.0f);
    maniQuat id = maniQuatIdentity();
    CHECK(SameBits(&zero, &id, sizeof id), "no angle is the identity");
}

static void TestBetweenTakesFromOntoTo(void)
{
    bool ok = true;
    for (int32_t i = 0; i < 10000; ++i)
    {
        maniVec3 f = maniVec3Normalize(RandomVec(1.0f), maniVec3Make(1.0f, 0.0f, 0.0f));
        maniVec3 t = maniVec3Normalize(RandomVec(1.0f), maniVec3Make(0.0f, 1.0f, 0.0f));
        maniVec3 r = maniQuatRotate(maniQuatBetween(f, t), f);
        ok = ok && NearF(r.x, t.x, 2e-6) && NearF(r.y, t.y, 2e-6) && NearF(r.z, t.z, 2e-6);
    }
    CHECK(ok, "from turns onto to");
    const maniVec3 axes[] = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.6f, 0.0f, 0.8f}};
    for (size_t i = 0; i < sizeof axes / sizeof axes[0]; ++i)
    {
        maniVec3 f = axes[i];
        maniVec3 back = maniVec3Scale(f, -1.0f);
        maniQuat q = maniQuatBetween(f, back);
        maniVec3 r = maniQuatRotate(q, f);
        CHECK(NearF(r.x, back.x, 1e-6) && NearF(r.y, back.y, 1e-6) && NearF(r.z, back.z, 1e-6),
              "opposite directions turn half way");
        CHECK(Near(maniQuatDot(q, q), 1.0, 1e-6), "a unit turn");
    }
    const maniVec3 bad[] = {{0.0f, 0.0f, 0.0f}, {NAN, 0.0f, 0.0f}, {INFINITY, 1.0f, 0.0f}};
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; ++i)
    {
        maniQuat q = maniQuatBetween(bad[i], axes[1]);
        maniQuat r = maniQuatBetween(axes[1], bad[i]);
        CHECK(Near(maniQuatDot(q, q), 1.0, 1e-6) && Near(maniQuatDot(r, r), 1.0, 1e-6),
              "a zero or non-finite direction still gives a unit turn");
    }
    maniQuat same = maniQuatBetween(axes[3], axes[3]);
    CHECK(Near(same.w, 1.0, 1e-7), "the same direction does not turn");
}

static void TestAffineAgainstBinary64(void)
{
    bool ok = true;
    for (int32_t i = 0; i < 10000; ++i)
    {
        maniVec3 t = RandomVec(5.0f);
        maniQuat q = RandomUnitQuat();
        maniVec3 s = RandomVec(2.0f);
        maniAffine m = maniAffineFromTrs(t, q, s);
        maniVec3 p = RandomVec(3.0f);
        maniVec3 got = maniAffineTransformPoint(&m, p);
        double rotated[3];
        RotateDouble(q, maniVec3Make(p.x * s.x, p.y * s.y, p.z * s.z), rotated);
        ok = ok && Near(got.x, rotated[0] + (double)t.x, 1e-5) &&
             Near(got.y, rotated[1] + (double)t.y, 1e-5) &&
             Near(got.z, rotated[2] + (double)t.z, 1e-5);
        maniVec3 dir = maniAffineTransformVector(&m, p);
        ok = ok && Near(dir.x, rotated[0], 1e-5) && Near(dir.y, rotated[1], 1e-5) &&
             Near(dir.z, rotated[2], 1e-5);
    }
    CHECK(ok, "scale, turn, translate as binary64 does");
}

static void TestAffineProductAndInverse(void)
{
    bool ok = true;
    for (int32_t i = 0; i < 10000; ++i)
    {
        maniAffine a =
            maniAffineFromTrs(RandomVec(5.0f), RandomUnitQuat(), maniVec3Make(1.5f, -0.5f, 2.0f));
        maniAffine b =
            maniAffineFromTrs(RandomVec(5.0f), RandomUnitQuat(), maniVec3Make(0.7f, 1.0f, 1.3f));
        maniAffine ab = maniAffineMul(&a, &b);
        maniVec3 p = RandomVec(2.0f);
        maniVec3 direct = maniAffineTransformPoint(&ab, p);
        maniVec3 stepped = maniAffineTransformPoint(&a, maniAffineTransformPoint(&b, p));
        ok = ok && NearF(direct.x, stepped.x, 1e-4) && NearF(direct.y, stepped.y, 1e-4) &&
             NearF(direct.z, stepped.z, 1e-4);
        maniAffine inverse;
        ok = ok && maniAffineInverse(&ab, &inverse);
        maniVec3 back = maniAffineTransformPoint(&inverse, direct);
        ok = ok && NearF(back.x, p.x, 1e-4) && NearF(back.y, p.y, 1e-4) && NearF(back.z, p.z, 1e-4);
    }
    CHECK(ok, "the product is b then a, and the inverse undoes it");
    maniAffine flat = maniAffineFromTrs(maniVec3Make(1.0f, 2.0f, 3.0f), maniQuatIdentity(),
                                        maniVec3Make(1.0f, 0.0f, 1.0f));
    maniAffine untouched = flat;
    CHECK(!maniAffineInverse(&flat, &untouched), "a flat matrix has no inverse");
    CHECK(SameBits(&untouched, &flat, sizeof flat), "and the output is left as it was");
    maniAffine nan = flat;
    nan.m[0] = NAN;
    CHECK(!maniAffineInverse(&nan, &untouched), "nor has one with NaN");
}

// Results are hashed with the family hash in blocks, as every platform
// must produce the same bits.
typedef struct Hasher
{
    uint64_t hash;
    int32_t count;
    float block[1024];
} Hasher;

static void HashFloat(Hasher* h, float value)
{
    h->block[h->count++] = value;
    if (h->count == 1024)
    {
        h->hash = maniHash64(h->hash, h->block, (int64_t)sizeof h->block);
        h->count = 0;
    }
}

static uint64_t HashEnd(Hasher* h)
{
    return maniHash64(h->hash, h->block, h->count * (int64_t)sizeof(float));
}

static uint64_t HashAll(void)
{
    s_state = 777u;
    Hasher h = {MANI_HASH_INIT, 0, {0}};
    for (int32_t i = 0; i < 20000; ++i)
    {
        maniQuat a = RandomUnitQuat();
        maniQuat b = RandomUnitQuat();
        float t = (Next() + 1.0f) * 0.5f;
        maniQuat s = maniQuatSlerp(a, b, t);
        maniQuat n = maniQuatNlerp(a, b, t);
        maniVec3 v = RandomVec(4.0f);
        maniQuat between = maniQuatBetween(maniVec3Normalize(v, maniVec3Make(1.0f, 0.0f, 0.0f)),
                                           maniVec3Make(0.0f, 0.0f, 1.0f));
        maniAffine m = maniAffineFromTrs(v, a, maniVec3Make(1.0f, t, 2.0f));
        maniAffine mm = maniAffineMul(&m, &m);
        maniAffine inv;
        if (!maniAffineInverse(&mm, &inv))
        {
            memset(&inv, 0, sizeof inv);
        }
        const float* parts[] = {&s.x, &n.x, &between.x};
        for (size_t p = 0; p < 3; ++p)
        {
            for (int32_t k = 0; k < 4; ++k)
            {
                HashFloat(&h, parts[p][k]);
            }
        }
        for (int32_t k = 0; k < 12; ++k)
        {
            HashFloat(&h, inv.m[k]);
        }
    }
    return HashEnd(&h);
}

static void TestSameBitsEverywhere(void)
{
    uint64_t hash = HashAll();
    if (hash != PINNED_HASH)
    {
        printf("hash %016llx\n", (unsigned long long)hash);
    }
    CHECK(hash == PINNED_HASH, "the pinned results");
}

int main(void)
{
    TestRotateMatchesTheMatrix();
    TestProductTurnsBySecondThenFirst();
    TestNormalizeIsTotal();
    TestNlerpHemispheresAndEnds();
    TestSlerpMatchesTheReference();
    TestAxisAngle();
    TestBetweenTakesFromOntoTo();
    TestAffineAgainstBinary64();
    TestAffineProductAndInverse();
    TestSameBitsEverywhere();
    return s_failures == 0 ? 0 : 1;
}
