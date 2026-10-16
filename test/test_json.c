// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The JSON reader (mani-0007): numbers against an exact rational oracle in
// both formats, binary32 written shortest against another (mani-0023),
// whole numbers, the grammar's refusals with their offsets, every proper
// prefix of a document refused, the layout of values, lookups where names
// repeat, escapes, depth and allocation failure. Run as test_json --seed
// <file> it writes the fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "decimal.h"
#include "decimal_format.h"
#include "json.h"
#include "json_format_cases.h"
#include "json_number_cases.h"
#include "test_harness.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const maniAllocator kDefault = {0};

static const char kDocument[] =
    "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"name\":\"hip\\u00e9\",\"translation\":[0,1."
    "5e0,-2]},{\"name\":\"a\\\"b\",\"children\":[]}],\"name\":\"first\",\"name\":\"last\","
    "\"e\":\"\\ud83d\\ude00\\n\",\"t\":true,\"f\":false,\"z\":null}";

static maniJsonResult Read(const char* text, maniJson* json)
{
    return maniReadJson(&kDefault, (const uint8_t*)text, strlen(text), json);
}

static void TestNumbers(void)
{
    size_t wrong = 0;
    for (size_t i = 0; i < sizeof kNumberCases / sizeof kNumberCases[0]; ++i)
    {
        const NumberCase* c = &kNumberCases[i];
        uint64_t bits = 0;
        bool ok = maniDecimalToBinary(c->text, strlen(c->text), maniBinary32, &bits);
        wrong += ok == c->overflow32 || (ok && bits != c->bits32) ? 1 : 0;
        ok = maniDecimalToBinary(c->text, strlen(c->text), maniBinary64, &bits);
        wrong += ok == c->overflow64 || (ok && bits != c->bits64) ? 1 : 0;
    }
    CHECK(wrong == 0, "every case correctly rounded");
    if (wrong != 0)
    {
        printf("%zu wrong\n", wrong);
    }
    // Exponents past any int64 saturate: no overflow on the way.
    uint64_t bits = 1;
    const char kHuge[] = "1e99999999999999999999999";
    CHECK(!maniDecimalToBinary(kHuge, sizeof kHuge - 1, maniBinary64, &bits), "a huge exponent");
    const char kTiny[] = "-1e-99999999999999999999999";
    CHECK(maniDecimalToBinary(kTiny, sizeof kTiny - 1, maniBinary64, &bits) &&
              bits == 0x8000000000000000u,
          "a tiny one is -0");
}

static void TestFormat(void)
{
    size_t wrong = 0;
    char text[MANI_DECIMAL_MAX];
    for (size_t i = 0; i < sizeof kFormatCases / sizeof kFormatCases[0]; ++i)
    {
        const FormatCase* c = &kFormatCases[i];
        float value = 0.0f;
        memcpy(&value, &c->bits, sizeof value);
        size_t length = maniFormatBinary32(value, text);
        wrong += length != strlen(c->text) || memcmp(text, c->text, length) != 0 ? 1 : 0;
    }
    CHECK(wrong == 0, "every case written as the oracle writes it");
    // A sweep of the whole range: each text reads back to its value.
    size_t lost = 0;
    for (uint64_t bits = 0; bits < 0x7F800000u; bits += 32771u)
    {
        uint32_t b = (uint32_t)bits;
        float value = 0.0f;
        memcpy(&value, &b, sizeof value);
        size_t length = maniFormatBinary32(value, text);
        uint64_t read = 0;
        lost += maniDecimalToBinary(text, length, maniBinary32, &read) && read == b ? 0 : 1;
    }
    CHECK(lost == 0, "every text reads back");
    uint32_t infinity = 0x7F800000u;
    uint32_t nan = 0x7FC00000u;
    float value = 0.0f;
    memcpy(&value, &infinity, sizeof value);
    CHECK(maniFormatBinary32(value, text) == 0, "infinity is not written");
    memcpy(&value, &nan, sizeof value);
    CHECK(maniFormatBinary32(value, text) == 0, "NaN is not written");
}

static bool Integer(const char* text, int64_t minimum, int64_t maximum, int64_t* value)
{
    return maniDecimalToInteger(text, strlen(text), minimum, maximum, value);
}

static void TestIntegers(void)
{
    int64_t v = 7;
    CHECK(Integer("0", 0, 10, &v) && v == 0, "0");
    CHECK(Integer("-0", 0, 10, &v) && v == 0, "-0 is 0");
    CHECK(Integer("1e2", 0, 1000, &v) && v == 100, "an exponent");
    CHECK(Integer("3.0", 0, 10, &v) && v == 3, "a zero fraction");
    CHECK(Integer("100e-2", 0, 10, &v) && v == 1, "a negative exponent, whole");
    CHECK(Integer("0.000e5", 0, 10, &v) && v == 0, "zero with an exponent");
    CHECK(!Integer("1.5", 0, 10, &v), "a fraction");
    CHECK(!Integer("1e-1", 0, 10, &v), "a fraction by exponent");
    CHECK(Integer("4294967295", 0, UINT32_MAX, &v) && v == UINT32_MAX, "the range's top");
    CHECK(!Integer("4294967296", 0, UINT32_MAX, &v), "past the range");
    CHECK(!Integer("-1", 0, 10, &v), "below the range");
    CHECK(Integer("9223372036854775807", INT64_MIN, INT64_MAX, &v) && v == INT64_MAX, "int64 max");
    CHECK(Integer("-9223372036854775808", INT64_MIN, INT64_MAX, &v) && v == INT64_MIN, "int64 min");
    CHECK(!Integer("9223372036854775808", INT64_MIN, INT64_MAX, &v), "past int64");
    CHECK(!Integer("-9223372036854775809", INT64_MIN, INT64_MAX, &v), "below int64");
    CHECK(!Integer("1e19", INT64_MIN, INT64_MAX, &v), "ten to the 19");
    CHECK(!Integer("1e999999999999", INT64_MIN, INT64_MAX, &v), "a huge exponent");
    CHECK(v == INT64_MIN, "refusals leave the value");
}

typedef struct Bad
{
    const char* text;
    maniJsonReason reason;
    uint64_t offset;
} Bad;

static void TestRefusals(void)
{
    static const Bad kBad[] = {
        {"", maniJsonReasonSyntax, 0},
        {"  ", maniJsonReasonSyntax, 2},
        {"01", maniJsonReasonSyntax, 1},
        {"1.", maniJsonReasonSyntax, 2},
        {".5", maniJsonReasonSyntax, 0},
        {"-", maniJsonReasonSyntax, 1},
        {"-a", maniJsonReasonSyntax, 1},
        {"1e", maniJsonReasonSyntax, 2},
        {"1e+", maniJsonReasonSyntax, 3},
        {"+1", maniJsonReasonSyntax, 0},
        {"[1,]", maniJsonReasonSyntax, 3},
        {"{\"a\":1,}", maniJsonReasonSyntax, 7},
        {"{\"a\" 1}", maniJsonReasonSyntax, 5},
        {"{1:2}", maniJsonReasonSyntax, 1},
        {"[1 2]", maniJsonReasonSyntax, 3},
        {"[1}", maniJsonReasonSyntax, 2},
        {"{\"a\":1]", maniJsonReasonSyntax, 6},
        {"tru", maniJsonReasonSyntax, 0},
        {"nul", maniJsonReasonSyntax, 0},
        {"NaN", maniJsonReasonSyntax, 0},
        {"Infinity", maniJsonReasonSyntax, 0},
        {"'a'", maniJsonReasonSyntax, 0},
        {"//c\n1", maniJsonReasonSyntax, 0},
        {"[1] x", maniJsonReasonSyntax, 4},
        {"1 2", maniJsonReasonSyntax, 2},
        {"\"abc", maniJsonReasonSyntax, 4},
        {"\"a\\x\"", maniJsonReasonEscape, 2},
        {"\"\\u12g4\"", maniJsonReasonEscape, 1},
        {"\"\\uD800\"", maniJsonReasonEscape, 1},
        {"\"\\uDC00\"", maniJsonReasonEscape, 1},
        {"\"\\uD800\\u0041\"", maniJsonReasonEscape, 1},
        {"\"\\uD800\\uE000\"", maniJsonReasonEscape, 1},
        {"\"a\x01\"", maniJsonReasonControl, 2},
        {"\"a\tb\"", maniJsonReasonControl, 2},
        {"\"a\x1F\"", maniJsonReasonControl, 2},
        {"\"\xC3\x28\"", maniJsonReasonUtf8, 1},
        {"\"\xED\xA0\x80\"", maniJsonReasonUtf8, 1},
        {"\"\xC0\xAF\"", maniJsonReasonUtf8, 1},
        {"\xEF\xBB", maniJsonReasonSyntax, 0},
        {"\xEF\xBB\xBF\xEF\xBB\xBF{}", maniJsonReasonSyntax, 3},
    };
    for (size_t i = 0; i < sizeof kBad / sizeof kBad[0]; ++i)
    {
        maniJson json;
        maniJsonResult r = Read(kBad[i].text, &json);
        bool right = r.result == mani_errorFormat && r.reason == kBad[i].reason &&
                     r.offset == kBad[i].offset;
        CHECK(right, kBad[i].text);
        if (!right)
        {
            printf("  case %zu: reason %d offset %llu\n", i, (int)r.reason,
                   (unsigned long long)r.offset);
        }
    }
}

static void TestPrefixes(void)
{
    size_t size = strlen(kDocument);
    for (size_t n = 0; n < size; ++n)
    {
        maniJson json;
        maniJsonResult r = maniReadJson(&kDefault, (const uint8_t*)kDocument, n, &json);
        CHECK(r.result == mani_errorFormat, "a proper prefix");
    }
}

static void TestLayout(void)
{
    maniJson json;
    CHECK(Read(kDocument, &json).result == mani_success, "reads");
    const maniJsonValue* v = json.values;
    CHECK(v[0].kind == maniJsonObject && v[0].count == 8 && v[0].next == json.count, "the root");
    CHECK(v[0].start == 0 && v[0].length == strlen(kDocument), "its span");
    uint32_t nodes = maniJsonFind(&json, 0, "nodes", 5);
    CHECK(nodes != MANI_JSON_NONE && v[nodes].kind == maniJsonArray && v[nodes].count == 2,
          "nodes");
    uint32_t first = nodes + 1;
    uint32_t second = v[first].next;
    CHECK(v[second].kind == maniJsonObject && v[second].next == v[nodes].next, "siblings");
    uint32_t translation = maniJsonFind(&json, first, "translation", 11);
    CHECK(v[translation].count == 3, "three numbers");
    uint64_t bits = 0;
    uint32_t y = v[translation + 1].next;
    CHECK(maniJsonBinary(&json, y, maniBinary32, &bits) && bits == 0x3FC00000u, "1.5e0");
    int64_t whole = 0;
    CHECK(maniJsonInteger(&json, v[y].next, -5, 5, &whole) && whole == -2, "-2");
    CHECK(!maniJsonInteger(&json, y, -5, 5, &whole), "1.5 is not whole");
    CHECK(!maniJsonBinary(&json, translation, maniBinary32, &bits), "an array is no number");
    uint32_t children = maniJsonFind(&json, second, "children", 8);
    CHECK(v[children].count == 0 && v[children].next == children + 1, "an empty array");
    // A repeated name: the last wins.
    uint32_t name = maniJsonFind(&json, 0, "name", 4);
    CHECK(maniJsonStringEquals(&json, name, "last", 4), "the later value");
    CHECK(maniJsonFind(&json, 0, "missing", 7) == MANI_JSON_NONE, "missing");
    CHECK(maniJsonFind(&json, nodes, "name", 4) == MANI_JSON_NONE, "an array has no names");
    CHECK(maniJsonFind(&json, 9999, "name", 4) == MANI_JSON_NONE, "past the values");
    // Escapes decode.
    uint32_t hip = maniJsonFind(&json, first, "name", 4);
    CHECK(maniJsonStringEquals(&json, hip, "hip\xC3\xA9", 5), "\\u00e9");
    CHECK(!maniJsonStringEquals(&json, hip, "hip", 3), "a prefix is not equal");
    CHECK(!maniJsonStringEquals(&json, hip, "hip\xC3\xA9x", 6), "nor a longer one");
    uint32_t quote = maniJsonFind(&json, second, "name", 4);
    CHECK(maniJsonStringEquals(&json, quote, "a\"b", 3), "an escaped quote");
    uint8_t out[16];
    uint32_t e = maniJsonFind(&json, 0, "e", 1);
    size_t n = maniJsonStringDecode(&json, e, out, sizeof out);
    CHECK(n == 5 && memcmp(out, "\xF0\x9F\x98\x80\n", 5) == 0, "a surrogate pair");
    CHECK(maniJsonStringDecode(&json, e, out, 3) == 0, "no partial code point");
    CHECK(maniJsonStringDecode(&json, e, out, 4) == 4, "whole code points only");
    CHECK(v[maniJsonFind(&json, 0, "t", 1)].kind == maniJsonTrue, "true");
    CHECK(v[maniJsonFind(&json, 0, "f", 1)].kind == maniJsonFalse, "false");
    CHECK(v[maniJsonFind(&json, 0, "z", 1)].kind == maniJsonNull, "null");
    maniReleaseJson(&kDefault, &json);
    CHECK(json.values == NULL && json.count == 0, "released");
}

static void TestScalarsAndSpace(void)
{
    maniJson json;
    CHECK(Read(" \t\r\n-0.5e-3 \n", &json).result == mani_success, "a number alone");
    uint64_t bits = 0;
    CHECK(json.count == 1 && maniJsonBinary(&json, 0, maniBinary64, &bits), "one value");
    maniReleaseJson(&kDefault, &json);
    CHECK(Read("\xEF\xBB\xBF[]", &json).result == mani_success && json.count == 1, "a BOM");
    maniReleaseJson(&kDefault, &json);
    CHECK(Read("\"\"", &json).result == mani_success && json.values[0].length == 0, "empty");
    maniReleaseJson(&kDefault, &json);
    CHECK(Read("\"\\\"\\\\\\/\\b\\f\\n\\r\\t\"", &json).result == mani_success, "escapes");
    uint8_t out[8];
    CHECK(maniJsonStringDecode(&json, 0, out, sizeof out) == 8 &&
              memcmp(out, "\"\\/\b\f\n\r\t", 8) == 0,
          "decoded");
    maniReleaseJson(&kDefault, &json);
    // Each encoded length's last and first code point, upper-case hex.
    CHECK(Read("\"\\u07FF\\u0800\\uFFFF\\uD800\\uDC00\"", &json).result == mani_success,
          "boundaries");
    uint8_t wide[16];
    CHECK(maniJsonStringDecode(&json, 0, wide, sizeof wide) == 12 &&
              memcmp(wide, "\xDF\xBF\xE0\xA0\x80\xEF\xBF\xBF\xF0\x90\x80\x80", 12) == 0,
          "two, three and four bytes");
    maniReleaseJson(&kDefault, &json);
    // A backslash before a NUL byte is no escape.
    const uint8_t kNul[] = {'"', '\\', 0, '"'};
    maniJsonResult r = maniReadJson(&kDefault, kNul, sizeof kNul, &json);
    CHECK(r.result == mani_errorFormat && r.reason == maniJsonReasonEscape && r.offset == 1,
          "\\ and NUL");
}

static void TestDepth(void)
{
    static char deep[2 * MANI_JSON_MAX_DEPTH + 3];
    for (uint32_t i = 0; i < MANI_JSON_MAX_DEPTH; ++i)
    {
        deep[i] = '[';
        deep[MANI_JSON_MAX_DEPTH + i] = ']';
    }
    maniJson json;
    CHECK(Read(deep, &json).result == mani_success && json.count == MANI_JSON_MAX_DEPTH,
          "the deepest");
    maniReleaseJson(&kDefault, &json);
    memmove(deep + 1, deep, 2 * MANI_JSON_MAX_DEPTH);
    deep[0] = '[';
    deep[2 * MANI_JSON_MAX_DEPTH + 1] = ']';
    maniJsonResult r = Read(deep, &json);
    CHECK(r.result == mani_errorFormat && r.reason == maniJsonReasonDepth &&
              r.offset == MANI_JSON_MAX_DEPTH,
          "one deeper");
}

static void* FailingAlloc(size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    return NULL;
}

static void FailingFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)memory;
    (void)size;
    (void)alignment;
    (void)context;
}

static void TestAllocationFailure(void)
{
    maniAllocator failing = {FailingAlloc, FailingFree, NULL};
    maniJson json;
    maniJsonResult r = maniReadJson(&failing, (const uint8_t*)"[1]", 3, &json);
    CHECK(r.result == mani_errorCapacity && json.values == NULL, "no memory");
}

static int WriteSeed(const char* path)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t size = strlen(kDocument);
    size_t written = fwrite(kDocument, 1, size, out);
    return fclose(out) == 0 && written == size ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestNumbers();
    TestFormat();
    TestIntegers();
    TestRefusals();
    TestPrefixes();
    TestLayout();
    TestScalarsAndSpace();
    TestDepth();
    TestAllocationFailure();
    return s_failures == 0 ? 0 : 1;
}
