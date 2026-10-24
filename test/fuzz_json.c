// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The JSON reader: reading any bytes, then every value read back:
// containers' spans and links in order, every string decoded no longer
// than its text and equal to itself, every number converted in both
// formats and as a whole number, every object's names looked up. Run as
// fuzz_json <corpus dir>; the seed comes from test_json --seed <file>.

#include "json.h"

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Check(const maniJson* json, uint32_t i, uint8_t* scratch)
{
    const maniJsonValue* v = &json->values[i];
    if (v->next <= i || v->next > json->count)
    {
        abort();
    }
    if (v->kind == maniJsonString)
    {
        size_t n = maniJsonStringDecode(json, i, scratch, v->length);
        if (n > v->length || !maniJsonStringEquals(json, i, (const char*)scratch, n))
        {
            abort();
        }
    }
    if (v->kind == maniJsonNumber)
    {
        uint64_t bits = 0;
        int64_t whole = 0;
        (void)maniJsonBinary(json, i, maniBinary32, &bits);
        (void)maniJsonBinary(json, i, maniBinary64, &bits);
        (void)maniJsonInteger(json, i, INT64_MIN, INT64_MAX, &whole);
    }
    if (v->kind == maniJsonObject)
    {
        uint32_t key = i + 1;
        for (uint32_t m = 0; m < v->count; ++m)
        {
            size_t n = maniJsonStringDecode(json, key, scratch, json->values[key].length);
            if (maniJsonFind(json, i, (const char*)scratch, n) == MANI_JSON_NONE)
            {
                abort();
            }
            key = json->values[key + 1].next;
        }
        if (key != v->next)
        {
            abort();
        }
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    const maniAllocator allocator = {0};
    maniJson json;
    if (maniReadJson(&allocator, data, size, &json).result != mani_success)
    {
        return 0;
    }
    uint8_t* scratch = malloc(size + 1);
    if (scratch != NULL)
    {
        for (uint32_t i = 0; i < json.count; ++i)
        {
            Check(&json, i, scratch);
        }
        free(scratch);
    }
    maniReleaseJson(&allocator, &json);
    return 0;
}
