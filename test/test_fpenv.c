// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library's floating-point environment under a host that set
// flush-to-zero, denormals-are-zero and round toward zero: inside, results
// are the round-to-nearest, denormals-kept ones; after, the host's setting
// is back exactly.

#include "fpenv.h"
#include "scalar.h"
#include "test_harness.h"

#include <stdint.h>
#include <string.h>

// The register is read and written through the library's accessors,
// which no compiler may drop or reorder; the arithmetic helpers are out of
// line, since compilers move float arithmetic freely (fpenv.h).
#if defined(__x86_64__) || defined(_M_X64)
// Flush-to-zero, denormals-are-zero, round toward zero, exceptions masked.
static const uint64_t s_hostSetting = 0x1f80u | (1u << 15) | (1u << 6) | (3u << 13);
static const int s_hostCanChange = 1;
#elif defined(__aarch64__) || defined(_M_ARM64)
// Flush-to-zero and round toward zero.
static const uint64_t s_hostSetting = (1ull << 24) | (3ull << 22);
static const int s_hostCanChange = 1;
#else
static const uint64_t s_hostSetting = 0;
static const int s_hostCanChange = 0;
#endif

static uint64_t ReadControl(void)
{
    return maniFloatControlRead();
}

static void WriteControl(uint64_t value)
{
    maniFloatControlWrite(value);
}

static bool SameBits(float a, float b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

// Values the environment decides, computed through volatile so the
// compiler cannot fold them at build time.
MANI_NOINLINE static float Subnormal(void)
{
    volatile float small = 1e-38f;
    volatile float scale = 0.001f;
    return small * scale;
}

MANI_NOINLINE static float AboveHalfUlp(void)
{
    volatile float one = 1.0f;
    volatile float bit = 0x1.8p-24f;
    return one + bit;
}

static void TestTheHostSettingTakesEffect(void)
{
    uint64_t before = ReadControl();
    WriteControl(s_hostSetting);
    float subnormal = Subnormal();
    float rounded = AboveHalfUlp();
    WriteControl(before);
    if (s_hostCanChange)
    {
        CHECK(subnormal == 0.0f, "the host's flush-to-zero flushes");
        CHECK(rounded == 1.0f, "the host's rounding toward zero rounds down");
    }
}

static void TestInsideIsNearestWithDenormals(void)
{
    uint64_t before = ReadControl();
    WriteControl(s_hostSetting);
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    float subnormal = Subnormal();
    float rounded = AboveHalfUlp();
    float sine = maniSin(1e-40f);
    maniFloatEnvLeave(&env);
    uint64_t after = ReadControl();
    WriteControl(before);
    CHECK(subnormal != 0.0f, "denormals kept inside");
    CHECK(SameBits(rounded, 1.0f + 0x1p-23f), "round to nearest inside");
    CHECK(SameBits(sine, 1e-40f), "a subnormal sine survives inside");
    CHECK(after == s_hostSetting, "the host's setting is back after");
}

static void TestNestedEntriesRestoreInOrder(void)
{
    uint64_t before = ReadControl();
    WriteControl(s_hostSetting);
    maniFloatEnv outer;
    maniFloatEnv inner;
    maniFloatEnvEnter(&outer);
    uint64_t inside = ReadControl();
    maniFloatEnvEnter(&inner);
    maniFloatEnvLeave(&inner);
    CHECK(ReadControl() == inside, "an inner call leaves the library's setting");
    maniFloatEnvLeave(&outer);
    CHECK(ReadControl() == s_hostSetting, "the outer call leaves the host's");
    WriteControl(before);
}

int main(void)
{
    TestTheHostSettingTakesEffect();
    TestInsideIsNearestWithDenormals();
    TestNestedEntriesRestoreInOrder();
    return s_failures == 0 ? 0 : 1;
}
