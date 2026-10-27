// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The binary FBX container: files written here record by
// record, in both header widths and both byte orders, read back as the
// tree they were written as; values, raw and zlib arrays read on
// request; and each way the container can be wrong refused at its byte.
// Run as test_fbx --seed <file> it writes the fuzz target's seeds, the
// binary scene and (beside it) the ASCII one.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_value.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A file being written: records open on a stack, their headers patched
// when they close.
typedef struct Writer
{
    uint8_t b[16384];
    size_t n;
    uint32_t version;
    bool big;
    int depth;
    size_t start[80];
    size_t values[80];
    size_t valuesEnd[80];
    uint32_t count[80];
    bool children[80];
} Writer;

static Writer s_w;

static size_t HeaderSize(const Writer* w)
{
    return w->version >= 7500 ? 25 : 13;
}

static void Put(Writer* w, const void* bytes, size_t size)
{
    memcpy(w->b + w->n, bytes, size);
    w->n += size;
}

static void Word(uint8_t* p, uint64_t v, int size, bool big)
{
    for (int i = 0; i < size; ++i)
    {
        p[big ? size - 1 - i : i] = (uint8_t)(v >> (8 * i));
    }
}

static void PutWord(Writer* w, uint64_t v, int size)
{
    Word(w->b + w->n, v, size, w->big);
    w->n += (size_t)size;
}

static void Start(uint32_t version, bool big)
{
    Writer* w = &s_w;
    memset(w, 0, sizeof *w);
    w->version = version;
    w->big = big;
    Put(w, "Kaydara FBX Binary  \0\x1A", 22);
    uint8_t endian = big ? 1 : 0;
    Put(w, &endian, 1);
    PutWord(w, version, 4);
    w->depth = 0;
}

static void Begin(const char* name)
{
    Writer* w = &s_w;
    if (w->depth > 0 && !w->children[w->depth])
    {
        w->children[w->depth] = true;
        w->valuesEnd[w->depth] = w->n;
    }
    ++w->depth;
    w->start[w->depth] = w->n;
    memset(w->b + w->n, 0, HeaderSize(w));
    w->n += HeaderSize(w);
    w->b[w->n - 1] = (uint8_t)strlen(name);
    Put(w, name, strlen(name));
    w->values[w->depth] = w->n;
    w->count[w->depth] = 0;
    w->children[w->depth] = false;
}

static void Null(Writer* w)
{
    memset(w->b + w->n, 0, HeaderSize(w));
    w->n += HeaderSize(w);
}

static void End(void)
{
    Writer* w = &s_w;
    int d = w->depth;
    if (w->children[d])
    {
        Null(w);
    }
    else
    {
        w->valuesEnd[d] = w->n;
    }
    int size = w->version >= 7500 ? 8 : 4;
    uint8_t* h = w->b + w->start[d];
    Word(h, w->n, size, w->big);
    Word(h + size, w->count[d], size, w->big);
    Word(h + 2 * size, w->valuesEnd[d] - w->values[d], size, w->big);
    --w->depth;
}

// The top-level list's null record and a footer of no meaning.
static void Finish(void)
{
    Null(&s_w);
    Put(&s_w, "footer bytes....", 16);
}

static void Scalar(char type, uint64_t bits, int size)
{
    Put(&s_w, &type, 1);
    PutWord(&s_w, bits, size);
    ++s_w.count[s_w.depth];
}

static void PropI(int32_t v)
{
    Scalar('I', (uint32_t)v, 4);
}

static void PropL(int64_t v)
{
    Scalar('L', (uint64_t)v, 8);
}

static void PropD(double v)
{
    uint64_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    Scalar('D', bits, 8);
}

static void PropS(const char* s, size_t length)
{
    Put(&s_w, "S", 1);
    PutWord(&s_w, length, 4);
    Put(&s_w, s, length);
    ++s_w.count[s_w.depth];
}

// A raw array of `count` elements of `size` bytes, given as words.
static void PropArray(char type, const uint64_t* words, uint32_t count, int size)
{
    Put(&s_w, &type, 1);
    PutWord(&s_w, count, 4);
    PutWord(&s_w, 0, 4);
    PutWord(&s_w, (uint64_t)count * (uint64_t)size, 4);
    for (uint32_t i = 0; i < count; ++i)
    {
        PutWord(&s_w, words[i], size);
    }
    ++s_w.count[s_w.depth];
}

static uint32_t Adler32(const uint8_t* p, size_t n)
{
    uint32_t a = 1;
    uint32_t b = 0;
    for (size_t i = 0; i < n; ++i)
    {
        a = (a + p[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    return b << 16 | a;
}

// The same as a zlib stream of one stored block; `corrupt` breaks its
// checksum.
static void PropZlib(char type, const uint64_t* words, uint32_t count, int size, bool corrupt)
{
    uint8_t data[512];
    for (uint32_t i = 0; i < count; ++i)
    {
        Word(data + i * (uint32_t)size, words[i], size, s_w.big);
    }
    uint16_t length = (uint16_t)(count * (uint32_t)size);
    Put(&s_w, &type, 1);
    PutWord(&s_w, count, 4);
    PutWord(&s_w, 1, 4);
    PutWord(&s_w, (uint64_t)length + 11, 4);
    const uint8_t head[5] = {0x78, 0x01, 0x01, (uint8_t)length, (uint8_t)(length >> 8)};
    Put(&s_w, head, sizeof head);
    const uint8_t inverse[2] = {(uint8_t)~length, (uint8_t)(~length >> 8)};
    Put(&s_w, inverse, 2);
    Put(&s_w, data, length);
    uint32_t adler = Adler32(data, length) ^ (corrupt ? 1u : 0u);
    const uint8_t sum[4] = {(uint8_t)(adler >> 24), (uint8_t)(adler >> 16), (uint8_t)(adler >> 8),
                            (uint8_t)adler};
    Put(&s_w, sum, 4);
    ++s_w.count[s_w.depth];
}

static uint64_t Bits(double v)
{
    uint64_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

static uint64_t Bits32(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

// A small scene: a header node, a model with a name, a Properties70
// entry and arrays of every kind.
static void Scene(uint32_t version, bool big)
{
    Start(version, big);
    Begin("FBXHeaderExtension");
    Begin("FBXVersion");
    PropI((int32_t)version);
    End();
    End();
    Begin("Objects");
    Begin("Model");
    PropL(1234567890123LL);
    PropS("hip\0\1Model", 11);
    PropS("LimbNode", 8);
    Begin("Properties70");
    Begin("P");
    PropS("Lcl Translation", 15);
    PropS("Lcl Translation", 15);
    PropS("", 0);
    PropS("A", 1);
    PropD(1.5);
    PropD(-2.0);
    PropD(0.25);
    End();
    End();
    Begin("KeyTime");
    const uint64_t times[3] = {(uint64_t)-46186158000LL, 46186158000ull, 92372316000ull};
    PropArray('l', times, 3, 8);
    End();
    Begin("KeyValueFloat");
    const uint64_t values[3] = {Bits32(1.0f), Bits32(-0.5f), Bits32(3.25f)};
    PropZlib('f', values, 3, 4, false);
    End();
    Begin("Indexes");
    const uint64_t indexes[2] = {(uint32_t)-7, 9};
    PropArray('i', indexes, 2, 4);
    End();
    Begin("Weights");
    const uint64_t weights[2] = {Bits(0.75), Bits(0.25)};
    PropArray('d', weights, 2, 8);
    End();
    End();
    End();
    Finish();
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static uint32_t Path(const maniFbx* fbx, const char* const* names, int count)
{
    uint32_t node = 0;
    for (int i = 0; i < count && node != MANI_FBX_NONE; ++i)
    {
        node = maniFbxFindChild(fbx, node, names[i], strlen(names[i]));
    }
    return node;
}

static void CheckScene(uint32_t version, bool big, const char* what)
{
    Scene(version, big);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, s_w.b, s_w.n, &fbx);
    CHECK(r.result == mani_success, what);
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu\n", r.reason, (unsigned long long)r.offset);
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbxVersion(fbx) == version, "the version");
    CHECK(fbx->nodeCount == 11, "eleven nodes and the root");
    const char* const header[2] = {"FBXHeaderExtension", "FBXVersion"};
    int64_t v = 0;
    CHECK(maniFbxInteger(fbx, maniFbxNodeProperty(fbx, Path(fbx, header, 2), 0), 0, 9999, &v) &&
              v == version,
          "an int32 property");
    const char* const model[2] = {"Objects", "Model"};
    uint32_t m = Path(fbx, model, 2);
    CHECK(m != MANI_FBX_NONE && fbx->nodes[m].propertyCount == 3, "a model, three properties");
    CHECK(maniFbxInteger(fbx, maniFbxNodeProperty(fbx, m, 0), INT64_MIN, INT64_MAX, &v) &&
              v == 1234567890123LL,
          "an int64 id");
    const uint8_t* text = NULL;
    uint32_t length = 0;
    CHECK(maniFbxString(fbx, maniFbxNodeProperty(fbx, m, 1), &text, &length) && length == 11 &&
              memcmp(text, "hip\0\1Model", 11) == 0,
          "a name with its class, raw");
    CHECK(maniFbxNodeProperty(fbx, m, 3) == MANI_FBX_NONE, "past the properties");
    uint8_t name[16];
    CHECK(fbx->objectCount == 1 && maniFbxObjectName(fbx, 0, name, sizeof name) == 3 &&
              memcmp(name, "hip", 3) == 0 && maniFbxFindObject(fbx, 1234567890123LL) == 0,
          "a binary object, its name before \\x00\\x01");
    const char* const p[4] = {"Objects", "Model", "Properties70", "P"};
    uint32_t entry = Path(fbx, p, 4);
    double d = 0;
    CHECK(maniFbxNumber(fbx, maniFbxNodeProperty(fbx, entry, 5), &d) && d == -2.0, "a double");
    CHECK(!maniFbxNumber(fbx, maniFbxNodeProperty(fbx, entry, 0), &d), "a string is no number");
    CHECK(maniFbxInteger(fbx, maniFbxNodeProperty(fbx, entry, 5), -5, 5, &v) && v == -2,
          "a whole double as an integer");
    CHECK(!maniFbxInteger(fbx, maniFbxNodeProperty(fbx, entry, 4), -5, 5, &v), "1.5 is not whole");
    const char* const times[3] = {"Objects", "Model", "KeyTime"};
    uint32_t t = maniFbxNodeProperty(fbx, Path(fbx, times, 3), 0);
    int64_t ticks[3];
    CHECK(maniFbxArrayCount(fbx, t) == 3 &&
              maniFbxReadIntegers(fbx, t, ticks).result == mani_success &&
              ticks[0] == -46186158000LL && ticks[1] == 46186158000LL && ticks[2] == 92372316000LL,
          "an int64 array");
    double seconds[3];
    CHECK(maniFbxReadDoubles(fbx, t, seconds).result == mani_success &&
              seconds[0] == -46186158000.0 && seconds[2] == 92372316000.0,
          "an int64 array as doubles, signed");
    const char* const keys[3] = {"Objects", "Model", "KeyValueFloat"};
    uint32_t k = maniFbxNodeProperty(fbx, Path(fbx, keys, 3), 0);
    double out[3];
    CHECK(maniFbxReadDoubles(fbx, k, out).result == mani_success && out[0] == 1.0 &&
              out[1] == -0.5 && out[2] == 3.25,
          "a zlib float array");
    CHECK(maniFbxReadIntegers(fbx, k, ticks).result == mani_errorFormat, "floats are no integers");
    const char* const indexes[3] = {"Objects", "Model", "Indexes"};
    uint32_t i = maniFbxNodeProperty(fbx, Path(fbx, indexes, 3), 0);
    CHECK(maniFbxReadIntegers(fbx, i, ticks).result == mani_success && ticks[0] == -7 &&
              ticks[1] == 9,
          "int32 sign extended");
    CHECK(maniFbxReadDoubles(fbx, i, out).result == mani_success && out[0] == -7.0,
          "integers as doubles");
    const char* const weights[3] = {"Objects", "Model", "Weights"};
    uint32_t wt = maniFbxNodeProperty(fbx, Path(fbx, weights, 3), 0);
    CHECK(maniFbxReadDoubles(fbx, wt, out).result == mani_success && out[0] == 0.75 &&
              out[1] == 0.25,
          "a double array");
    uint32_t words[3];
    CHECK(maniFbxReadFloatBits(fbx, k, words).result == mani_success && words[0] == 0x3f800000u &&
              words[1] == 0xbf000000u &&
              maniFbxReadFloatBits(fbx, i, words).result == mani_success && words[0] == 0xfffffff9u,
          "float and int32 arrays as binary32 words");
    uint32_t named = maniFbxNodeProperty(fbx, entry, 0);
    uint32_t again = maniFbxNodeProperty(fbx, entry, 1);
    uint32_t empty = maniFbxNodeProperty(fbx, entry, 2);
    uint32_t a = maniFbxNodeProperty(fbx, entry, 3);
    CHECK(maniFbxSameString(fbx, named, again) && !maniFbxSameString(fbx, empty, a) &&
              !maniFbxSameString(fbx, a, empty) && !maniFbxSameString(fbx, named, a) &&
              !maniFbxSameString(fbx, named, k),
          "strings the same or not, by length too");
    CHECK(maniFbxStringHash(fbx, named) == maniFbxNameHash("Lcl Translation", 15) &&
              maniFbxStringHash(fbx, empty) == maniFbxNameHash("", 0) &&
              maniFbxStringHash(fbx, k) == maniFbxNameHash("", 0),
          "a string's hash, a non-string's the empty one");
    CHECK(maniFbxReadFloatBits(fbx, wt, words).result == mani_errorFormat &&
              maniFbxReadFloatBits(fbx, t, words).result == mani_errorFormat,
          "no words of doubles or int64s");
    CHECK(maniFbxArrayCount(fbx, maniFbxNodeProperty(fbx, m, 0)) == 0, "a scalar is no array");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(maniImportReason reason, uint64_t offset, const char* what)
{
    // The file alone in a block of its size, so that a read past its end
    // is a sanitizer's error, not a read of the writer's spare room.
    uint8_t* exact = malloc(s_w.n > 0 ? s_w.n : 1);
    if (exact == NULL)
    {
        return;
    }
    memcpy(exact, s_w.b, s_w.n);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, exact, s_w.n, &fbx);
    free(exact);
    bool right =
        r.result == mani_errorFormat && r.reason == reason && r.offset == offset && fbx == NULL;
    maniCloseFbx(fbx);
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu (want %llu)\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, (unsigned long long)offset);
    }
    maniDestroyBuilder(builder);
}

// Whether the written file opens.
static bool Opens(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool open = maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return open;
}

// The offset of a node's header in the written file: the n-th record
// named so.
static size_t RecordAt(const char* name)
{
    size_t length = strlen(name);
    for (size_t at = 27; at + length < s_w.n; ++at)
    {
        size_t h = HeaderSize(&s_w);
        if (at + h + length <= s_w.n && s_w.b[at + h - 1] == length &&
            memcmp(s_w.b + at + h, name, length) == 0)
        {
            return at;
        }
    }
    return 0;
}

static uint64_t ReadWord(const uint8_t* p, int size, bool big)
{
    uint64_t v = 0;
    for (int i = 0; i < size; ++i)
    {
        v |= (uint64_t)p[big ? size - 1 - i : i] << (8 * i);
    }
    return v;
}

// The offset of a record's k-th property's type letter.
static size_t PropertyAt(size_t record, int k)
{
    size_t at = record + HeaderSize(&s_w) + s_w.b[record + HeaderSize(&s_w) - 1];
    for (int i = 0; i < k; ++i)
    {
        uint8_t type = s_w.b[at];
        size_t step = type == 'I' ? 5 : type == 'L' || type == 'D' ? 9 : 0;
        if (type == 'S')
        {
            step = 5 + (size_t)ReadWord(s_w.b + at + 1, 4, s_w.big);
        }
        at += step;
    }
    return at;
}

static void TestRefusals(void)
{
    Scene(7400, false);
    s_w.b[21] = 0x1B;
    ExpectRefusal(maniImportReasonContainer, 21, "the byte after the magic");
    Scene(7400, false);
    s_w.b[22] = 2;
    ExpectRefusal(maniImportReasonContainer, 21, "an endianness byte neither 0 nor 1");
    Scene(2900, false);
    ExpectRefusal(maniImportReasonVersion, 23, "version below 3000");
    Scene(7800, false);
    ExpectRefusal(maniImportReasonVersion, 23, "version past 7700");
    Scene(7400, false);
    s_w.n = 24;
    ExpectRefusal(maniImportReasonContainer, 24, "a header cut short");
    // A record ending past the file.
    Scene(7400, false);
    size_t model = RecordAt("Model");
    Word(s_w.b + model, s_w.n + 100, 4, false);
    ExpectRefusal(maniImportReasonContainer, model, "an end past the file");
    // A record ending one byte before its children's null record does:
    // refused at that null record.
    Scene(7400, false);
    model = RecordAt("Model");
    uint64_t end = ReadWord(s_w.b + model, 4, false);
    Word(s_w.b + model, end - 1, 4, false);
    ExpectRefusal(maniImportReasonContainer, end - 13, "an end before the record does");
    Scene(7400, false);
    model = RecordAt("Model");
    s_w.b[PropertyAt(model, 0)] = 'Q';
    ExpectRefusal(maniImportReasonContainer, PropertyAt(model, 0), "a type FBX lacks");
    Scene(7400, false);
    model = RecordAt("Model");
    Word(s_w.b + model + 4, 2, 4, false);
    ExpectRefusal(maniImportReasonContainer, PropertyAt(model, 2), "fewer properties than bytes");
    Scene(7400, false);
    model = RecordAt("Model");
    Word(s_w.b + model + 4, 1000, 4, false);
    ExpectRefusal(maniImportReasonContainer, model, "more properties than bytes");
    Scene(7400, false);
    model = RecordAt("Model");
    Word(s_w.b + PropertyAt(model, 1) + 1, 100000, 4, false);
    ExpectRefusal(maniImportReasonContainer, PropertyAt(model, 1), "a string past its record");
    Scene(7400, false);
    size_t times = RecordAt("KeyTime");
    size_t array = PropertyAt(times, 0) + 1;
    Word(s_w.b + array + 4, 2, 4, false);
    ExpectRefusal(maniImportReasonContainer, array + 4, "an array encoding FBX lacks");
    Scene(7400, false);
    Word(s_w.b + array + 8, 23, 4, false);
    ExpectRefusal(maniImportReasonContainer, array + 8, "a raw array of the wrong size");
    Scene(7400, false);
    Word(s_w.b + array, 4, 4, false);
    ExpectRefusal(maniImportReasonContainer, array + 8, "a raw array's count past its bytes");
    // A deflated array (23 bytes) may give 1032 times its bytes, no more.
    Scene(7400, false);
    size_t values = PropertyAt(RecordAt("KeyValueFloat"), 0) + 1;
    Word(s_w.b + values, 23 * 1032 / 4 + 1, 4, false);
    ExpectRefusal(maniImportReasonContainer, values, "a deflated array larger than DEFLATE gives");
    Scene(7400, false);
    Word(s_w.b + values, 23 * 1032 / 4, 4, false);
    maniBuilder* largest = NewBuilder();
    maniFbx* opened = NULL;
    CHECK(maniOpenFbx(largest, s_w.b, s_w.n, &opened).result == mani_success,
          "a deflated array as large as DEFLATE gives opens");
    maniCloseFbx(opened);
    maniDestroyBuilder(largest);
    // A scalar running past its record's property bytes.
    Scene(7400, false);
    size_t version = RecordAt("FBXVersion");
    Word(s_w.b + version + 8, 4, 4, false);
    ExpectRefusal(maniImportReasonContainer, PropertyAt(version, 0), "a scalar past its bytes");
    // An array's header, then its bytes, cut by the property bytes.
    Scene(7400, false);
    times = RecordAt("KeyTime");
    array = PropertyAt(times, 0) + 1;
    Word(s_w.b + times + 8, 1 + 8, 4, false);
    ExpectRefusal(maniImportReasonContainer, array, "an array header past its bytes");
    Scene(7400, false);
    Word(s_w.b + times + 8, 1 + 12 + 8, 4, false);
    ExpectRefusal(maniImportReasonContainer, array + 8, "array bytes past their record's");
    // A child ending past its parent, within the file.
    Scene(7400, false);
    model = RecordAt("Model");
    size_t child = RecordAt("Weights");
    Word(s_w.b + child, ReadWord(s_w.b + model, 4, false) + 1, 4, false);
    ExpectRefusal(maniImportReasonContainer, child, "a child past its parent");
    // A list's null record with a property count still ends it, as ufbx
    // reads it: no end and no name.
    Scene(7400, false);
    model = RecordAt("Model");
    end = ReadWord(s_w.b + model, 4, false);
    Word(s_w.b + end - 13 + 4, 1, 4, false);
    CHECK(Opens(), "a null record with a property count");
    // A leaf record whose end is 0 ends with its values, as Maya 2019
    // writes them; one with children then leaves its children's null
    // record short of its parent's end.
    Scene(7400, false);
    Word(s_w.b + RecordAt("Weights"), 0, 4, false);
    CHECK(Opens(), "a leaf record with an end of 0");
    Scene(7400, false);
    model = RecordAt("Model");
    end = ReadWord(s_w.b + model, 4, false);
    Word(s_w.b + model, 0, 4, false);
    ExpectRefusal(maniImportReasonContainer, end - 13, "a record with children and an end of 0");
    // A header cut by the end of the file.
    Scene(7400, false);
    s_w.n -= 16 + 5;
    ExpectRefusal(maniImportReasonContainer, s_w.n - 8, "a header cut short");
    // One property more than a record holds, the byte after it an array
    // letter: refused at the record's end, not read as an array.
    bool padded = false;
    for (size_t pad = 1; pad < 256 && !padded; ++pad)
    {
        Start(7400, false);
        Begin("X");
        PropI(1);
        End();
        char name[256];
        memset(name, 'y', pad);
        name[pad] = 0;
        Begin(name);
        End();
        Finish();
        size_t y = RecordAt("X") + 13 + 1 + 5;
        if (s_w.b[y] != 'i')
        {
            continue;
        }
        Word(s_w.b + RecordAt("X") + 4, 2, 4, false);
        ExpectRefusal(maniImportReasonContainer, y, "a property at its record's end");
        padded = true;
    }
    CHECK(padded, "a padding that puts an array letter there");
    // No null record ends the top level.
    Scene(7400, false);
    s_w.n -= 16 + 13;
    ExpectRefusal(maniImportReasonContainer, s_w.n, "the top level unended");
    // Nesting past 64.
    Start(7400, false);
    for (int i = 0; i < 66; ++i)
    {
        Begin("N");
    }
    for (int i = 0; i < 66; ++i)
    {
        End();
    }
    Finish();
    size_t deep = 27;
    for (int i = 0; i < 64; ++i)
    {
        deep += 13 + 1;
    }
    ExpectRefusal(maniImportReasonContainer, deep - 14, "nesting past 64");
    Start(7400, false);
    for (int i = 0; i < 64; ++i)
    {
        Begin("N");
    }
    for (int i = 0; i < 64; ++i)
    {
        End();
    }
    Finish();
    {
        maniBuilder* builder = NewBuilder();
        maniFbx* fbx = NULL;
        CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success, "64 levels");
        maniCloseFbx(fbx);
        maniDestroyBuilder(builder);
    }
}

static const char kAscii[] =
    "; FBX 7.4.0 project file\n"
    "; a comment\n"
    "FBXHeaderExtension:  {\n"
    "\tFBXVersion: 7400\n"
    "}\n"
    "Objects:  {\n"
    "\tModel: 1234567890123, \"Model::hip\", \"LimbNode\" {\n"
    "\t\tProperties70:  {\n"
    "\t\t\tP: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1.5,-2,+0.25\n"
    "\t\t}\n"
    "\t\tShading: T\n"
    "\t\tKeyTime: *3 {\n"
    "\t\t\ta: 0,46186158000, ; a comment inside\n"
    "\t\t\t92372316000\n"
    "\t\t}\n"
    "\t\tEmpty: *0 {\n"
    "\t\t}\n"
    "\t\tWeird: 1.#INF, -1.#IND\n"
    "\t\tBadKeys: *2 {\n"
    "\t\t\ta: 1,1.#QNAN\n"
    "\t\t}\n"
    "\t\tQuote: \"say &quot;hi&quot;&lf;& more\"\n"
    "\t\t\"Spaced Name\": , 1\n"
    "\t}\n"
    "}\n";

static void SetText(const char* text)
{
    s_w.n = strlen(text);
    memcpy(s_w.b, text, s_w.n);
}

static void TestAscii(void)
{
    SetText(kAscii);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, s_w.b, s_w.n, &fbx);
    CHECK(r.result == mani_success, "an ASCII file");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu near: %.30s\n", r.reason, (unsigned long long)r.offset,
               (const char*)s_w.b + r.offset);
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbxVersion(fbx) == 7400, "the version from the first comment");
    const char* const model[2] = {"Objects", "Model"};
    uint32_t m = Path(fbx, model, 2);
    int64_t v = 0;
    CHECK(m != MANI_FBX_NONE &&
              maniFbxInteger(fbx, maniFbxNodeProperty(fbx, m, 0), INT64_MIN, INT64_MAX, &v) &&
              v == 1234567890123LL,
          "an id");
    CHECK(maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, m, 1), "Model::hip", 10),
          "a quoted string");
    const char* const p[4] = {"Objects", "Model", "Properties70", "P"};
    uint32_t entry = Path(fbx, p, 4);
    double d = 0;
    CHECK(maniFbxNumber(fbx, maniFbxNodeProperty(fbx, entry, 4), &d) && d == 1.5, "1.5");
    CHECK(maniFbxNumber(fbx, maniFbxNodeProperty(fbx, entry, 6), &d) && d == 0.25,
          "+0.25, exactly");
    CHECK(maniFbxInteger(fbx, maniFbxNodeProperty(fbx, entry, 5), -5, 5, &v) && v == -2, "-2");
    CHECK(maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, entry, 2), "", 0), "an empty string");
    const char* const shading[3] = {"Objects", "Model", "Shading"};
    CHECK(maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, Path(fbx, shading, 3), 0), "T", 1),
          "a bare word");
    const char* const times[3] = {"Objects", "Model", "KeyTime"};
    uint32_t t = maniFbxNodeProperty(fbx, Path(fbx, times, 3), 0);
    int64_t ticks[3];
    double values[3];
    CHECK(maniFbxArrayCount(fbx, t) == 3 &&
              maniFbxReadIntegers(fbx, t, ticks).result == mani_success &&
              ticks[1] == 46186158000LL && ticks[2] == 92372316000LL,
          "an array across lines and a comment");
    CHECK(maniFbxReadDoubles(fbx, t, values).result == mani_success && values[2] == 92372316000.0,
          "as doubles");
    CHECK(fbx->nodes[Path(fbx, times, 3)].firstChild == MANI_FBX_NONE, "its a: folded in");
    const char* const empty[3] = {"Objects", "Model", "Empty"};
    uint32_t e = maniFbxNodeProperty(fbx, Path(fbx, empty, 3), 0);
    CHECK(e != MANI_FBX_NONE && maniFbxArrayCount(fbx, e) == 0, "an empty array");
    const char* const weird[3] = {"Objects", "Model", "Weird"};
    uint32_t w = Path(fbx, weird, 3);
    CHECK(fbx->nodes[w].propertyCount == 2 &&
              !maniFbxNumber(fbx, maniFbxNodeProperty(fbx, w, 0), &d),
          "1.#INF is kept, and is no number");
    const char* const bad[3] = {"Objects", "Model", "BadKeys"};
    uint32_t b = maniFbxNodeProperty(fbx, Path(fbx, bad, 3), 0);
    r = maniFbxReadDoubles(fbx, b, values);
    const char* qnan = strstr(kAscii, "1.#QNAN");
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData &&
              r.offset == (uint64_t)(qnan - kAscii),
          "a non-number element, at its text");
    const char* const quote[3] = {"Objects", "Model", "Quote"};
    uint32_t q = maniFbxNodeProperty(fbx, Path(fbx, quote, 3), 0);
    uint8_t out[32];
    size_t n = maniFbxStringDecode(fbx, q, out, sizeof out);
    CHECK(n == 15 && memcmp(out, "say \"hi\"\n& more", 15) == 0, "entities decoded");
    CHECK(maniFbxStringEquals(fbx, q, "say \"hi\"\n& more", 15) &&
              !maniFbxStringEquals(fbx, q, "say \"hi\"\n& mor", 14),
          "and compared");
    const char* const spaced[3] = {"Objects", "Model", "Spaced Name"};
    uint32_t sp = Path(fbx, spaced, 3);
    CHECK(sp != MANI_FBX_NONE && fbx->nodes[sp].propertyCount == 1,
          "a quoted name, a leading comma");
    maniCloseFbx(fbx);
    // The version from the header when no comment gives it.
    SetText("FBXHeaderExtension: {\n FBXVersion: 6100\n}\n");
    CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success &&
              maniFbxVersion(fbx) == 6100,
          "the version from FBXHeaderExtension");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestAsciiEdges(void)
{
    SetText("; FBX 7.4.0 project file\n; FBX 6.1.0 project file\n"
            "Odd: 1.#QNAN, -nan(ind), +infinity, 2.#SNAN7\n"
            "Spaced : 1\n"
            "Lines: \"a\nb\"\n"
            "Short: -5\n");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success, "opens");
    if (fbx != NULL)
    {
        CHECK(maniFbxVersion(fbx) == 7400, "the first comment's version, not a later one's");
        uint32_t odd = maniFbxFindChild(fbx, 0, "Odd", 3);
        double d = 0;
        bool none = true;
        for (uint32_t k = 0; k < 4; ++k)
        {
            none = none && !maniFbxNumber(fbx, maniFbxNodeProperty(fbx, odd, k), &d);
        }
        CHECK(fbx->nodes[odd].propertyCount == 4 && none,
              "infinities and NaNs open, numbers that are none");
        CHECK(maniFbxFindChild(fbx, 0, "Spaced", 6) != MANI_FBX_NONE, "a space before the colon");
        uint32_t lines = maniFbxNodeProperty(fbx, maniFbxFindChild(fbx, 0, "Lines", 5), 0);
        CHECK(maniFbxStringEquals(fbx, lines, "a\nb", 3), "a string across lines");
        int64_t v = 0;
        uint32_t shortValue = maniFbxNodeProperty(fbx, maniFbxFindChild(fbx, 0, "Short", 5), 0);
        CHECK(!maniFbxInteger(fbx, shortValue, -4, 4, &v), "below the range");
        CHECK(!maniFbxStringEquals(fbx, lines, "a\nbc", 4) &&
                  !maniFbxStringEquals(fbx, lines, "a\n", 2),
              "neither longer nor shorter");
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
    // A number-like token that is neither a number nor an infinity or NaN
    // whole is refused at open, as ufbx fails on it.
    static const char* const kOdd[] = {"1x5",       ".",      "1e",   "-",      "1.#INDIE",
                                       "-nan(a-b)", "1.Xinf", "-nab", "-nan(a#"};
    for (uint32_t k = 0; k < sizeof kOdd / sizeof kOdd[0]; ++k)
    {
        char text[96];
        (void)snprintf(text, sizeof text, "; FBX 7.4.0 project file\nOdd: 1, %s\n", kOdd[k]);
        SetText(text);
        ExpectRefusal(maniImportReasonSyntax,
                      (uint64_t)(strstr((const char*)s_w.b, "1, ") + 3 - (const char*)s_w.b),
                      kOdd[k]);
    }
    // A token cut by the end of the file is read within it.
    SetText("; FBX 7.4.0 project file\nOdd: 1, -na");
    ExpectRefusal(maniImportReasonSyntax,
                  (uint64_t)(strstr((const char*)s_w.b, "-na") - (const char*)s_w.b),
                  "a token at the end of the file");
    // Words in an array open, as ufbx reads them; decoding the array
    // refuses at the first (Data).
    SetText("; FBX 7.4.0 project file\nV: *4 {\n a: 1,inf,nan(ind),-1.#IND\n}\n");
    builder = NewBuilder();
    fbx = NULL;
    bool open = maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success;
    double values[4];
    uint32_t array = open ? maniFbxNodeProperty(fbx, maniFbxFindChild(fbx, 0, "V", 1), 0) : 0;
    maniImportResult r = open ? maniFbxReadDoubles(fbx, array, values)
                              : (maniImportResult){mani_errorInvalid, 0, 0, 0};
    const char* word = strstr((const char*)s_w.b, "inf");
    CHECK(open && maniFbxArrayCount(fbx, array) == 4 && r.reason == maniImportReasonData &&
              r.offset == (uint64_t)(word - (const char*)s_w.b),
          "words in an array: opened, refused when read");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    // A nested value list ending in a comma.
    SetText("; FBX 7.4.0 project file\nA: {\nB: 1,\n}\n");
    ExpectRefusal(maniImportReasonSyntax,
                  (uint64_t)(strchr((const char*)s_w.b, '}') - (const char*)s_w.b),
                  "a nested trailing comma");
}

static void TestAsciiRefusals(void)
{
    struct
    {
        const char* text;
        maniImportReason reason;
        const char* at;
        const char* what;
    } kBad[] = {
        {"; FBX 7.4.0 project file\nA: \"open", maniImportReasonSyntax, "\"open",
         "a string unended"},
        {"; FBX 7.4.0 project file\n}", maniImportReasonSyntax, "}", "a brace closing nothing"},
        {"; FBX 7.4.0 project file\nA: {\nB: 1\n", maniImportReasonSyntax, NULL,
         "a brace unclosed"},
        {"; FBX 7.4.0 project file\nA: *3 {\na: 1,2\n}", maniImportReasonSyntax, "*3",
         "three promised, two given"},
        {"; FBX 7.4.0 project file\nA: *2 {\nb: 1,2\n}", maniImportReasonSyntax,
         "b:", "an array's values not under a:"},
        {"; FBX 7.4.0 project file\nA: *2 {\na: 1,\"x\"\n}", maniImportReasonSyntax, "\"x\"",
         "a string in an array"},
        {"; FBX 7.4.0 project file\nA: 1,\n}", maniImportReasonSyntax, "}", "a trailing comma"},
        {"; FBX 7.4.0 project file\nA: 1 @", maniImportReasonSyntax, "@", "a character FBX lacks"},
        {"; FBX 7.4.0 project file\n1, 2", maniImportReasonSyntax, "1, 2", "values without a name"},
        {"; FBX 7.4.0 project file\nA: *x {\n}", maniImportReasonSyntax, "*x",
         "a count not a number"},
        {"A: 1\n", maniImportReasonVersion, NULL, "no version"},
        {"; FBX 2.9.0 project file\nA: 1\n", maniImportReasonVersion, NULL, "a version below 3000"},
    };
    for (size_t i = 0; i < sizeof kBad / sizeof kBad[0]; ++i)
    {
        SetText(kBad[i].text);
        uint64_t at = kBad[i].at != NULL
                          ? (uint64_t)(strstr(kBad[i].text, kBad[i].at) - kBad[i].text)
                          : (kBad[i].reason == maniImportReasonVersion ? 0 : s_w.n);
        ExpectRefusal(kBad[i].reason, at, kBad[i].what);
    }
    // Nesting past 64 levels.
    char deep[1024] = "; FBX 7.4.0 project file\n";
    for (int i = 0; i < 64; ++i)
    {
        strcat(deep, "A: {\n");
    }
    SetText(deep);
    ExpectRefusal(maniImportReasonSyntax, (uint64_t)(strrchr(deep, '{') - deep), "65 levels");
}

static void TestValueRefusals(void)
{
    Start(7400, false);
    Begin("Bad");
    const uint64_t values[2] = {Bits32(1.0f), Bits32(2.0f)};
    PropZlib('f', values, 2, 4, true);
    End();
    Begin("NaN");
    const uint64_t nan[2] = {Bits(1.0), 0x7FF8000000000000ull};
    PropArray('d', nan, 2, 8);
    End();
    Finish();
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success, "opens");
    double out[2];
    uint32_t bad = maniFbxNodeProperty(fbx, maniFbxFindChild(fbx, 0, "Bad", 3), 0);
    maniImportResult r = maniFbxReadDoubles(fbx, bad, out);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData &&
              r.offset == fbx->properties[bad].offset,
          "a checksum that fails");
    uint32_t nanArray = maniFbxNodeProperty(fbx, maniFbxFindChild(fbx, 0, "NaN", 3), 0);
    r = maniFbxReadDoubles(fbx, nanArray, out);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData &&
              r.offset == fbx->properties[nanArray].offset + 8,
          "a NaN, at its element");
    CHECK(maniFbxReadDoubles(fbx, MANI_FBX_NONE, out).result == mani_errorFormat, "no property");
    maniCloseFbx(fbx);
    // Misuse.
    CHECK(maniOpenFbx(builder, NULL, 0, &fbx).result == mani_errorInvalid && fbx == NULL,
          "no bytes");
    CHECK(maniOpenFbx(NULL, s_w.b, s_w.n, &fbx).result == mani_errorInvalid, "no builder");
    CHECK(maniBuilderMisuses(builder) == 1, "a misuse counted");
    maniCloseFbx(NULL);
    CHECK(maniFbxVersion(NULL) == 0, "no document");
    maniDestroyBuilder(builder);
}

static void Entry(const char* name, double v)
{
    Begin("P");
    PropS(name, strlen(name));
    PropS("Vector3D", 8);
    PropS("Vector", 6);
    PropS("", 0);
    PropD(v);
    PropD(v);
    PropD(v);
    End();
}

// A binary model's properties found together as one by one: names only
// alike (a prefix, a longer name) are other properties.
static void TestFindProperties(void)
{
    Start(7400, false);
    Begin("Objects");
    Begin("Model");
    PropL(1);
    PropS("m\0\1Model", 9);
    PropS("LimbNode", 8);
    Begin("Properties70");
    Entry("RotationPivot", 1.0);
    Entry("RotationPiv", 2.0);
    Entry("RotationPivots", 3.0);
    Entry("ScalingPivot", 4.0);
    End();
    End();
    End();
    Finish();
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success, "opens");
    static const char* const kNames[3] = {"RotationPivot", "ScalingPivot", "RotationOffset"};
    uint32_t entries[3];
    maniFbxFindProperties(fbx, 0, kNames, 3, NULL, entries);
    bool same = true;
    for (uint32_t k = 0; k < 3; ++k)
    {
        same = same && entries[k] == maniFbxFindProperty(fbx, 0, kNames[k], strlen(kNames[k]));
    }
    CHECK(same && entries[0] != MANI_FBX_NONE && entries[2] == MANI_FBX_NONE,
          "properties found together as one by one");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// A binary int16, negative.
// Opens the written file with at most `limit` properties: whether it
// opens, else where it is refused; `count` the file's properties when
// it opens.
static maniImportResult OpenLimited(uint32_t limit, uint32_t* count)
{
    uint8_t* exact = malloc(s_w.n > 0 ? s_w.n : 1);
    if (exact == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    memcpy(exact, s_w.b, s_w.n);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniFbxOpenLimited(builder, exact, s_w.n, limit, &fbx);
    *count = fbx != NULL ? fbx->propertyCount : 0;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    free(exact);
    return r;
}

// A file of n properties opens with a limit of n and is refused with one
// of n - 1 at its last property's first byte (a binary type letter, an
// ASCII value), in both readers.
static void TestPropertyLimit(void)
{
    const uint32_t versions[2] = {7400, 7500};
    for (int v = 0; v < 2; ++v)
    {
        Scene(versions[v], false);
        uint32_t n = 0;
        CHECK(OpenLimited(MANI_FBX_MAX_PROPERTIES, &n).result == mani_success && n > 1,
              "a binary file under the property limit opens");
        size_t last = PropertyAt(RecordAt("Weights"), 0);
        uint32_t opened = 0;
        CHECK(OpenLimited(n, &opened).result == mani_success && opened == n,
              "a binary file of as many properties as the limit opens");
        maniImportResult r = OpenLimited(n - 1, &opened);
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonContainer &&
                  r.offset == last,
              "a binary file of one property past the limit is refused at it");
        r = OpenLimited(0, &opened);
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonContainer &&
                  r.offset == PropertyAt(RecordAt("FBXVersion"), 0),
              "a binary file is refused at its first property with no room for one");
    }
    SetText(kAscii);
    uint32_t n = 0;
    CHECK(OpenLimited(MANI_FBX_MAX_PROPERTIES, &n).result == mani_success && n > 1,
          "an ASCII file under the property limit opens");
    uint32_t opened = 0;
    CHECK(OpenLimited(n, &opened).result == mani_success && opened == n,
          "an ASCII file of as many properties as the limit opens");
    maniImportResult r = OpenLimited(n - 1, &opened);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonSyntax &&
              r.offset == (uint64_t)(strstr(kAscii, ", 1\n") + 2 - kAscii),
          "an ASCII file of one property past the limit is refused at it");
    r = OpenLimited(0, &opened);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonSyntax &&
              r.offset == (uint64_t)(strstr(kAscii, "7400\n") - kAscii),
          "an ASCII file is refused at its first property with no room for one");
}

static void TestShortInteger(void)
{
    Start(7400, false);
    Begin("Short");
    Scalar('Y', (uint16_t)-5, 2);
    End();
    Finish();
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success, "opens");
    int64_t v = 0;
    double d = 0;
    uint32_t p = maniFbxNodeProperty(fbx, maniFbxFindChild(fbx, 0, "Short", 5), 0);
    CHECK(maniFbxInteger(fbx, p, -10, 10, &v) && v == -5 && maniFbxNumber(fbx, p, &d) && d == -5.0,
          "an int16 sign extended");
    CHECK(!maniFbxInteger(fbx, p, -4, 4, &v), "a binary integer below its range");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static int WriteFile(const char* path)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(s_w.b, 1, s_w.n, out);
    return fclose(out) == 0 && written == s_w.n ? 0 : 1;
}

// The binary scene at the path, the ASCII one beside it.
static int WriteSeed(const char* path)
{
    Scene(7400, false);
    int failed = WriteFile(path);
    char ascii[512];
    snprintf(ascii, sizeof ascii, "%s-ascii", path);
    SetText(kAscii);
    return failed | WriteFile(ascii);
}

// Binary strings read as their bytes when no byte needs making valid:
// cut at a capacity, compared by their bytes; a NUL or a byte past ASCII
// still made U+FFFD.
static void TestPlainStrings(void)
{
    Start(7400, false);
    Begin("S");
    PropS("abc", 3);
    PropS("abd", 3);
    PropS("a\0b", 3);
    PropS("a\x80"
          "b",
          3);
    End();
    Finish();
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_w.b, s_w.n, &fbx).result == mani_success, "strings open");
    uint32_t node = fbx != NULL ? maniFbxFindChild(fbx, 0, "S", 1) : MANI_FBX_NONE;
    if (node != MANI_FBX_NONE)
    {
        uint32_t abc = maniFbxNodeProperty(fbx, node, 0);
        uint32_t abd = maniFbxNodeProperty(fbx, node, 1);
        uint8_t out[8] = {0};
        CHECK(maniFbxStringDecode(fbx, abc, out, 2) == 2 && memcmp(out, "ab\0", 3) == 0,
              "as many bytes as fit");
        CHECK(!maniFbxSameString(fbx, abc, abd) && maniFbxSameString(fbx, abc, abc),
              "as long, other bytes: not the same");
        static const char kMade[] = "a\xEF\xBF\xBD"
                                    "b";
        for (uint32_t k = 2; k < 4; ++k)
        {
            uint32_t made = maniFbxNodeProperty(fbx, node, k);
            CHECK(maniFbxStringDecode(fbx, made, out, sizeof out) == 5 &&
                      memcmp(out, kMade, 5) == 0,
                  k == 2 ? "a NUL made valid" : "a byte past ASCII made valid");
        }
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    CheckScene(7400, false, "a 7.4 file");
    CheckScene(7500, false, "a 7.5 file, 64-bit headers");
    CheckScene(7700, true, "a big-endian 7.7 file");
    CheckScene(7100, true, "a big-endian 7.1 file");
    TestRefusals();
    TestValueRefusals();
    TestAscii();
    TestAsciiEdges();
    TestAsciiRefusals();
    TestPropertyLimit();
    TestShortInteger();
    TestFindProperties();
    TestPlainStrings();
    return s_failures == 0 ? 0 : 1;
}
