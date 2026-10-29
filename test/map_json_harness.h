// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the maps' JSON tests share: builders over an allocator
// failing at one allocation, the byte offset of a marker in a text, and
// texts refused with their reason, fault and offset.

#ifndef MAUL_ANIM_TEST_MAP_JSON_HARNESS_H
#define MAUL_ANIM_TEST_MAP_JSON_HARNESS_H

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static maniBuilder* NewBuilder(const maniAllocator* allocator)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    if (allocator != NULL)
    {
        def.allocator = *allocator;
    }
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

// An allocator failing at one allocation, counting the live blocks;
// aligned by hand (no aligned_alloc everywhere): the block's start just
// before what it returns.
typedef struct Failing
{
    uint32_t calls;
    uint32_t failAt;
    uint32_t live;
} Failing;

static void* FailingAlloc(size_t size, size_t alignment, void* context)
{
    Failing* f = context;
    if (f->calls++ == f->failAt)
    {
        return NULL;
    }
    uint8_t* block = malloc(size + alignment + sizeof(void*));
    if (block == NULL)
    {
        return NULL;
    }
    uint8_t* at = block + sizeof(void*);
    at += (alignment - (uintptr_t)at % alignment) % alignment;
    memcpy(at - sizeof(void*), (const void*)&block, sizeof block);
    ++f->live;
    return at;
}

static void FailingFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    Failing* f = context;
    void* block = NULL;
    memcpy((void*)&block, (uint8_t*)memory - sizeof(void*), sizeof block);
    free(block);
    --f->live;
}

// A call of a builder: writing (reading false) or reading a text.
typedef maniBuildResult (*BuildCall)(maniBuilder* builder, bool reading);

// One run with the failAt-th allocation failing (none past the count):
// whether one failed, and whether it failed cleanly (Capacity, nothing
// kept, nothing leaked; success when none failed).
static bool FailOnce(BuildCall call, bool reading, uint32_t failAt, bool* cleanOut)
{
    Failing f = {0, UINT32_MAX, 0};
    maniAllocator allocator = {FailingAlloc, FailingFree, &f};
    maniBuilder* builder = NewBuilder(&allocator);
    f.failAt = f.calls + failAt;
    maniBuildResult r = call(builder, reading);
    size_t size = 1;
    bool failed = f.calls > f.failAt;
    bool clean = failed ? r.result == mani_errorCapacity &&
                              maniBuilder_CopyBuilt(builder, NULL, 0, &size) == mani_errorInvalid
                        : r.result == mani_success;
    maniDestroyBuilder(builder);
    *cleanOut = clean && f.live == 0;
    return failed;
}

// Each allocation of the writer, then of the reader, failing in turn.
static void TestAllocationFailure(BuildCall call)
{
    for (int reading = 0; reading < 2; ++reading)
    {
        bool clean = true;
        uint32_t failAt = 0;
        bool once = true;
        while (FailOnce(call, reading != 0, failAt, &once))
        {
            clean = clean && once;
            ++failAt;
        }
        CHECK(clean && once && failAt > 2, "each failure Capacity, nothing kept or leaked");
    }
}

// A text refused: the reason, the fault (or the item, without a marker)
// and the byte offset of the marker's occurrence (inside a string's
// quotes).
typedef struct Refusal
{
    const char* text;
    maniBuildReason reason;
    uint32_t other;
    const char* marker;
    uint32_t occurrence;
} Refusal;

// The byte offset of a marker's occurrence, inside a string's quotes.
static uint32_t OffsetOf(const char* text, const char* marker, uint32_t occurrence)
{
    const char* at = text - 1;
    for (uint32_t k = 0; k < occurrence && at != NULL; ++k)
    {
        at = strstr(at + 1, marker);
    }
    return at == NULL ? UINT32_MAX : (uint32_t)(at - text) + (marker[0] == '"' ? 1u : 0u);
}

// Reads a text; its result.
typedef maniBuildResult (*ReadText)(const char* text);

static void CheckRefusals(ReadText read, const Refusal* cases, size_t count)
{
    uint32_t wrong = 0;
    for (size_t i = 0; i < count; ++i)
    {
        const Refusal* c = &cases[i];
        maniBuildResult r = read(c->text);
        bool ok = r.result == mani_errorInvalid && r.reason == c->reason;
        ok = ok && (c->marker != NULL ? r.other == c->other &&
                                            r.item == OffsetOf(c->text, c->marker, c->occurrence)
                                      : r.item == c->other);
        if (!ok)
        {
            printf("case %zu: %d %u %u %u\n", i, (int)r.result, r.reason, r.item, r.other);
            ++wrong;
        }
    }
    CHECK(wrong == 0, "each text refused at its offset");
}

// The text kept in a builder after a write, into out.
static size_t CopyText(maniBuilder* builder, char* out, size_t capacity)
{
    size_t size = 0;
    CHECK(maniBuilder_CopyBuilt(builder, out, capacity, &size) == mani_success, "the text copied");
    return size;
}

static int WriteSeed(const char* path, const char* text, size_t size)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(text, 1, size, out);
    return fclose(out) == 0 && written == size ? 0 : 1;
}

#endif // MAUL_ANIM_TEST_MAP_JSON_HARNESS_H
