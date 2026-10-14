// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The family's 64-bit hash: its frozen values.

#include "test_harness.h"

#include "maul-anim/base.h"

#include <stdint.h>

static void TestNoBytesIsTheSeed(void)
{
    CHECK(maniHash64(MANI_HASH_INIT, NULL, 0) == MANI_HASH_INIT, "no bytes");
    CHECK(maniHash64(7u, NULL, 0) == 7u, "any seed");
}

static void TestOneRoundAndOneLeftoverByte(void)
{
    // One full eight-byte round and one leftover byte, computed from the
    // family's frozen constants; the word is read in the host's byte order.
    const uint8_t bytes[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    uint64_t word = 0;
    for (int32_t i = 7; i >= 0; --i)
    {
        word = (word << 8) | bytes[i];
    }
    uint64_t expected = (MANI_HASH_INIT ^ word) * 0x9E3779B97F4A7C15ull;
    expected ^= expected >> 32;
    expected = (expected ^ 9u) * 0x100000001B3ull;
    CHECK(maniHash64(MANI_HASH_INIT, bytes, 9) == expected, "nine bytes");
}

static void TestContinuingEqualsOneCall(void)
{
    const uint8_t bytes[16] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 11, 12, 13, 14, 15, 16};
    uint64_t whole = maniHash64(MANI_HASH_INIT, bytes, 16);
    uint64_t parts = maniHash64(maniHash64(MANI_HASH_INIT, bytes, 8), bytes + 8, 8);
    CHECK(whole == parts, "a hash continued at a word boundary equals one call");
}

int main(void)
{
    TestNoBytesIsTheSeed();
    TestOneRoundAndOneLeftoverByte();
    TestContinuingEqualsOneCall();
    return s_failures == 0 ? 0 : 1;
}
