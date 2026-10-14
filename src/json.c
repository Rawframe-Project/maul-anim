// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The JSON reader (mani-0007). One scanner runs twice over the text: first to
// check it and count its values, then, with the array allocated, to
// fill them. It is iterative: a stack of open containers, at most
// MANI_JSON_MAX_DEPTH deep, replaces recursion.

#include "json.h"

#include "allocator.h"
#include "decimal.h"
#include "text.h"

#include <string.h>

// An object's brackets, by value: the family's source checks count braces
// in character literals as code.
enum
{
    kObjectOpen = 0x7B,
    kObjectClose = 0x7D,
};

typedef struct Scanner
{
    const uint8_t* text;
    size_t size;
    size_t at;
    // NULL on the counting pass.
    maniJsonValue* values;
    uint32_t count;
    uint32_t depth;
    // Each open container: its value's index and its children so far.
    uint32_t open[MANI_JSON_MAX_DEPTH];
    uint32_t children[MANI_JSON_MAX_DEPTH];
} Scanner;

static maniJsonResult Fail(maniJsonReason reason, size_t offset)
{
    return (maniJsonResult){mani_errorFormat, reason, offset};
}

static maniJsonResult Success(void)
{
    return (maniJsonResult){mani_success, maniJsonReasonNone, 0};
}

static void SkipSpace(Scanner* s)
{
    while (s->at < s->size)
    {
        uint8_t c = s->text[s->at];
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r')
        {
            return;
        }
        ++s->at;
    }
}

static bool At(const Scanner* s, uint8_t c)
{
    return s->at < s->size && s->text[s->at] == c;
}

static bool IsDigit(uint8_t c)
{
    return c >= '0' && c <= '9';
}

static void Emit(Scanner* s, maniJsonKind kind, size_t start, size_t length)
{
    if (s->values != NULL)
    {
        s->values[s->count] =
            (maniJsonValue){kind, (uint32_t)start, (uint32_t)length, 0, s->count + 1};
    }
    ++s->count;
}

static int32_t HexDigit(uint8_t c)
{
    if (IsDigit(c))
    {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    return -1;
}

// Four hex digits after "\u" at `at`; -1 when they are not there.
static int32_t ReadUnit(const uint8_t* text, size_t size, size_t at)
{
    if (size - at < 6 || text[at] != '\\' || text[at + 1] != 'u')
    {
        return -1;
    }
    int32_t unit = 0;
    for (size_t k = 2; k < 6; ++k)
    {
        int32_t digit = HexDigit(text[at + k]);
        if (digit < 0)
        {
            return -1;
        }
        unit = unit * 16 + digit;
    }
    return unit;
}

// An escape at s->at (its backslash): checked, and stepped over.
static maniJsonResult ScanEscape(Scanner* s)
{
    size_t start = s->at;
    if (s->size - start < 2)
    {
        return Fail(maniJsonReasonSyntax, s->size);
    }
    uint8_t c = s->text[start + 1];
    if (c != 'u')
    {
        if (strchr("\"\\/bfnrt", c) == NULL || c == 0)
        {
            return Fail(maniJsonReasonEscape, start);
        }
        s->at += 2;
        return Success();
    }
    int32_t unit = ReadUnit(s->text, s->size, start);
    if (unit < 0)
    {
        return Fail(maniJsonReasonEscape, start);
    }
    s->at += 6;
    if (unit >= 0xDC00 && unit <= 0xDFFF)
    {
        return Fail(maniJsonReasonEscape, start);
    }
    if (unit >= 0xD800 && unit <= 0xDBFF)
    {
        int32_t low = ReadUnit(s->text, s->size, s->at);
        if (low < 0xDC00 || low > 0xDFFF)
        {
            return Fail(maniJsonReasonEscape, start);
        }
        s->at += 6;
    }
    return Success();
}

// A string at s->at (its opening quote), emitted as the span between the
// quotes.
static maniJsonResult ScanString(Scanner* s)
{
    size_t start = ++s->at;
    while (s->at < s->size)
    {
        uint8_t c = s->text[s->at];
        if (c == '"')
        {
            Emit(s, maniJsonString, start, s->at - start);
            ++s->at;
            return Success();
        }
        if (c < 0x20)
        {
            return Fail(maniJsonReasonControl, s->at);
        }
        if (c == '\\')
        {
            maniJsonResult r = ScanEscape(s);
            if (r.result != mani_success)
            {
                return r;
            }
            continue;
        }
        size_t step = c < 0x80 ? 1 : maniUtf8Sequence(s->text + s->at, s->size - s->at);
        if (step == 0)
        {
            return Fail(maniJsonReasonUtf8, s->at);
        }
        s->at += step;
    }
    return Fail(maniJsonReasonSyntax, s->size);
}

static void SkipDigits(Scanner* s)
{
    while (s->at < s->size && IsDigit(s->text[s->at]))
    {
        ++s->at;
    }
}

// An optional minus; 0, or a digit 1 to 9 and more digits; optionally a
// point and digits; optionally e or E, a sign, and digits.
static maniJsonResult ScanNumber(Scanner* s)
{
    size_t start = s->at;
    s->at += At(s, '-') ? 1 : 0;
    if (At(s, '0'))
    {
        ++s->at;
    }
    else if (s->at < s->size && IsDigit(s->text[s->at]))
    {
        SkipDigits(s);
    }
    else
    {
        return Fail(maniJsonReasonSyntax, s->at);
    }
    if (At(s, '.'))
    {
        ++s->at;
        if (!(s->at < s->size && IsDigit(s->text[s->at])))
        {
            return Fail(maniJsonReasonSyntax, s->at);
        }
        SkipDigits(s);
    }
    if (At(s, 'e') || At(s, 'E'))
    {
        ++s->at;
        s->at += At(s, '+') || At(s, '-') ? 1 : 0;
        if (!(s->at < s->size && IsDigit(s->text[s->at])))
        {
            return Fail(maniJsonReasonSyntax, s->at);
        }
        SkipDigits(s);
    }
    Emit(s, maniJsonNumber, start, s->at - start);
    return Success();
}

static maniJsonResult ScanLiteral(Scanner* s)
{
    static const char* const kWords[3] = {"null", "false", "true"};
    static const maniJsonKind kKinds[3] = {maniJsonNull, maniJsonFalse, maniJsonTrue};
    for (uint32_t i = 0; i < 3; ++i)
    {
        size_t length = strlen(kWords[i]);
        if (s->size - s->at >= length && memcmp(s->text + s->at, kWords[i], length) == 0)
        {
            Emit(s, kKinds[i], s->at, length);
            s->at += length;
            return Success();
        }
    }
    return Fail(maniJsonReasonSyntax, s->at);
}

// A member's name and its colon, leaving s->at at the value.
static maniJsonResult ScanName(Scanner* s)
{
    if (!At(s, '"'))
    {
        return Fail(maniJsonReasonSyntax, s->at);
    }
    maniJsonResult r = ScanString(s);
    if (r.result != mani_success)
    {
        return r;
    }
    SkipSpace(s);
    if (!At(s, ':'))
    {
        return Fail(maniJsonReasonSyntax, s->at);
    }
    ++s->at;
    SkipSpace(s);
    return Success();
}

static void Close(Scanner* s)
{
    uint32_t index = s->open[--s->depth];
    if (s->values != NULL)
    {
        maniJsonValue* v = &s->values[index];
        v->length = (uint32_t)(s->at - v->start);
        v->count = s->children[s->depth];
        v->next = s->count;
    }
}

static bool IsObject(const Scanner* s)
{
    return s->text[s->values != NULL ? s->values[s->open[s->depth - 1]].start
                                     : s->open[s->depth - 1]] == kObjectOpen;
}

// Opens an array or object at s->at; *valueNext tells the caller a value
// comes next (false: the container closed at once, empty).
static maniJsonResult Open(Scanner* s, bool* valueNext)
{
    if (s->depth == MANI_JSON_MAX_DEPTH)
    {
        return Fail(maniJsonReasonDepth, s->at);
    }
    uint8_t bracket = s->text[s->at];
    // The counting pass keeps the bracket's offset, the filling pass the
    // value's index.
    s->open[s->depth] = s->values != NULL ? s->count : (uint32_t)s->at;
    s->children[s->depth++] = 0;
    Emit(s, bracket == kObjectOpen ? maniJsonObject : maniJsonArray, s->at, 0);
    ++s->at;
    SkipSpace(s);
    if (At(s, bracket == kObjectOpen ? kObjectClose : ']'))
    {
        ++s->at;
        Close(s);
        *valueNext = false;
        return Success();
    }
    *valueNext = true;
    return bracket == kObjectOpen ? ScanName(s) : Success();
}

// One value at s->at; *valueNext as Open's.
static maniJsonResult ScanValue(Scanner* s, bool* valueNext)
{
    *valueNext = false;
    if (s->at >= s->size)
    {
        return Fail(maniJsonReasonSyntax, s->at);
    }
    uint8_t c = s->text[s->at];
    if (c == kObjectOpen || c == '[')
    {
        return Open(s, valueNext);
    }
    if (c == '"')
    {
        return ScanString(s);
    }
    if (c == '-' || IsDigit(c))
    {
        return ScanNumber(s);
    }
    return ScanLiteral(s);
}

// After a value: a comma and the next value, or closing brackets. *done
// when the outermost value ended and only space follows.
static maniJsonResult AfterValue(Scanner* s, bool* done)
{
    for (;;)
    {
        SkipSpace(s);
        if (s->depth == 0)
        {
            *done = true;
            return s->at == s->size ? Success() : Fail(maniJsonReasonSyntax, s->at);
        }
        ++s->children[s->depth - 1];
        bool object = IsObject(s);
        if (At(s, ','))
        {
            ++s->at;
            SkipSpace(s);
            *done = false;
            return object ? ScanName(s) : Success();
        }
        if (!At(s, object ? kObjectClose : ']'))
        {
            return Fail(maniJsonReasonSyntax, s->at);
        }
        ++s->at;
        Close(s);
    }
}

static maniJsonResult Scan(Scanner* s)
{
    s->at = s->size >= 3 && memcmp(s->text, "\xEF\xBB\xBF", 3) == 0 ? 3 : 0;
    s->count = 0;
    s->depth = 0;
    SkipSpace(s);
    for (;;)
    {
        bool valueNext = false;
        maniJsonResult r = ScanValue(s, &valueNext);
        if (r.result != mani_success || valueNext)
        {
            if (r.result != mani_success)
            {
                return r;
            }
            continue;
        }
        bool done = false;
        r = AfterValue(s, &done);
        if (r.result != mani_success || done)
        {
            return r;
        }
    }
}

maniJsonResult maniReadJson(const maniAllocator* allocator, const uint8_t* text, size_t size,
                            maniJson* jsonOut)
{
    *jsonOut = (maniJson){0};
    if (size >= UINT32_MAX)
    {
        return Fail(maniJsonReasonSize, UINT32_MAX);
    }
    Scanner s = {.text = text, .size = size};
    maniJsonResult r = Scan(&s);
    if (r.result != mani_success)
    {
        return r;
    }
    maniJsonValue* values = maniAllocate(allocator, s.count, sizeof(maniJsonValue), 8);
    if (values == NULL)
    {
        return (maniJsonResult){mani_errorCapacity, maniJsonReasonNone, 0};
    }
    s.values = values;
    r = Scan(&s);
    *jsonOut = (maniJson){text, values, s.count};
    return r;
}

void maniReleaseJson(const maniAllocator* allocator, maniJson* json)
{
    maniRelease(allocator, json->values, json->count, sizeof(maniJsonValue), 8);
    *json = (maniJson){0};
}

// One decoded code point of a checked string at *at, as UTF-8 into out;
// returns its byte count.
static size_t DecodeNext(const uint8_t* raw, size_t length, size_t* at, uint8_t out[4])
{
    uint8_t c = raw[*at];
    if (c != '\\')
    {
        size_t step = c < 0x80 ? 1 : maniUtf8Sequence(raw + *at, length - *at);
        memcpy(out, raw + *at, step);
        *at += step;
        return step;
    }
    uint8_t e = raw[*at + 1];
    if (e != 'u')
    {
        static const char kFrom[] = "\"\\/bfnrt";
        static const char kTo[] = "\"\\/\b\f\n\r\t";
        out[0] = (uint8_t)kTo[strchr(kFrom, e) - kFrom];
        *at += 2;
        return 1;
    }
    uint32_t point = (uint32_t)ReadUnit(raw, length, *at);
    *at += 6;
    if (point >= 0xD800 && point <= 0xDBFF)
    {
        uint32_t low = (uint32_t)ReadUnit(raw, length, *at);
        *at += 6;
        point = 0x10000 + ((point - 0xD800) << 10) + (low - 0xDC00);
    }
    if (point < 0x80)
    {
        out[0] = (uint8_t)point;
        return 1;
    }
    if (point < 0x800)
    {
        out[0] = (uint8_t)(0xC0 | (point >> 6));
        out[1] = (uint8_t)(0x80 | (point & 0x3F));
        return 2;
    }
    if (point < 0x10000)
    {
        out[0] = (uint8_t)(0xE0 | (point >> 12));
        out[1] = (uint8_t)(0x80 | ((point >> 6) & 0x3F));
        out[2] = (uint8_t)(0x80 | (point & 0x3F));
        return 3;
    }
    out[0] = (uint8_t)(0xF0 | (point >> 18));
    out[1] = (uint8_t)(0x80 | ((point >> 12) & 0x3F));
    out[2] = (uint8_t)(0x80 | ((point >> 6) & 0x3F));
    out[3] = (uint8_t)(0x80 | (point & 0x3F));
    return 4;
}

static bool IsKind(const maniJson* json, uint32_t value, maniJsonKind kind)
{
    return value < json->count && json->values[value].kind == (uint32_t)kind;
}

bool maniJsonStringEquals(const maniJson* json, uint32_t string, const char* bytes, size_t length)
{
    if (!IsKind(json, string, maniJsonString))
    {
        return false;
    }
    const maniJsonValue* v = &json->values[string];
    const uint8_t* raw = json->text + v->start;
    size_t at = 0;
    size_t matched = 0;
    while (at < v->length)
    {
        uint8_t unit[4];
        size_t n = DecodeNext(raw, v->length, &at, unit);
        if (length - matched < n || memcmp(bytes + matched, unit, n) != 0)
        {
            return false;
        }
        matched += n;
    }
    return matched == length;
}

size_t maniJsonStringDecode(const maniJson* json, uint32_t string, uint8_t* out, size_t capacity)
{
    if (!IsKind(json, string, maniJsonString))
    {
        return 0;
    }
    const maniJsonValue* v = &json->values[string];
    const uint8_t* raw = json->text + v->start;
    size_t at = 0;
    size_t written = 0;
    while (at < v->length)
    {
        uint8_t unit[4];
        size_t n = DecodeNext(raw, v->length, &at, unit);
        if (capacity - written < n)
        {
            break;
        }
        memcpy(out + written, unit, n);
        written += n;
    }
    return written;
}

uint32_t maniJsonFind(const maniJson* json, uint32_t object, const char* name, size_t length)
{
    if (!IsKind(json, object, maniJsonObject))
    {
        return MANI_JSON_NONE;
    }
    uint32_t found = MANI_JSON_NONE;
    uint32_t key = object + 1;
    for (uint32_t m = 0; m < json->values[object].count; ++m)
    {
        if (maniJsonStringEquals(json, key, name, length))
        {
            found = key + 1;
        }
        key = json->values[key + 1].next;
    }
    return found;
}

bool maniJsonBinary(const maniJson* json, uint32_t number, maniBinaryFormat format,
                    uint64_t* bitsOut)
{
    if (!IsKind(json, number, maniJsonNumber))
    {
        return false;
    }
    const maniJsonValue* v = &json->values[number];
    return maniDecimalToBinary((const char*)json->text + v->start, v->length, format, bitsOut);
}

bool maniJsonInteger(const maniJson* json, uint32_t number, int64_t minimum, int64_t maximum,
                     int64_t* valueOut)
{
    if (!IsKind(json, number, maniJsonNumber))
    {
        return false;
    }
    const maniJsonValue* v = &json->values[number];
    return maniDecimalToInteger((const char*)json->text + v->start, v->length, minimum, maximum,
                                valueOut);
}
