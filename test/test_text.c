// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// UTF-8 well-formedness at every boundary of the Unicode Standard's table
// of well-formed byte sequences.

#include "test_harness.h"
#include "text.h"

#include <string.h>

static bool Valid(const char* bytes)
{
    return maniValidUtf8((const uint8_t*)bytes, strlen(bytes));
}

static void TestWellFormed(void)
{
    CHECK(maniValidUtf8(NULL, 0), "empty");
    CHECK(Valid("plain ascii"), "ascii");
    CHECK(Valid("\xC2\x80\xDF\xBF"), "two bytes at the ends");
    CHECK(Valid("\xE0\xA0\x80\xED\x9F\xBF\xEE\x80\x80\xEF\xBF\xBF"), "three bytes at the ends");
    CHECK(Valid("\xF0\x90\x80\x80\xF4\x8F\xBF\xBF\xF3\xBF\xBF\xBF"), "four bytes at the ends");
}

static void TestIllFormed(void)
{
    CHECK(!Valid("\x80"), "a lone continuation");
    CHECK(!Valid("\xC0\x80"), "overlong two");
    CHECK(!Valid("\xC1\xBF"), "overlong two, high");
    CHECK(!Valid("\xE0\x9F\xBF"), "overlong three");
    CHECK(!Valid("\xED\xA0\x80"), "a surrogate");
    CHECK(!Valid("\xF0\x8F\xBF\xBF"), "overlong four");
    CHECK(!Valid("\xF4\x90\x80\x80"), "past U+10FFFF");
    CHECK(!Valid("\xF5\x80\x80\x80"), "a lead byte past 0xF4");
    CHECK(!Valid("\xE1\x80"), "truncated three");
    CHECK(!Valid("\xE1\x80\x41"), "a bad third byte");
    CHECK(!Valid("\xF1\x80\x80\xC0"), "a bad fourth byte");
    CHECK(!Valid("\xC2"), "truncated two");
}

int main(void)
{
    TestWellFormed();
    TestIllFormed();
    return s_failures == 0 ? 0 : 1;
}
