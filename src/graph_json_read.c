// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_json_read.h"

#include "arena.h"
#include "container.h"
#include "decimal.h"
#include "json.h"
#include "sort.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

bool maniGraphJsonFault(maniGraphReader* r, uint32_t value, maniGraphForm form)
{
    if (r->fault.result == mani_success)
    {
        r->fault = (maniBuildResult){mani_errorInvalid, maniBuildReasonForm,
                                     r->json->values[value].start, form};
    }
    return false;
}

bool maniGraphJsonNoMemory(maniGraphReader* r)
{
    if (r->fault.result == mani_success)
    {
        r->fault = (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0};
    }
    return false;
}

static bool IsKind(const maniGraphReader* r, uint32_t value, maniJsonKind kind)
{
    return r->json->values[value].kind == (uint32_t)kind;
}

// Which of keys a member's key string is, or count.
static uint32_t KeyOf(const maniGraphReader* r, uint32_t key, const char* const* keys,
                      uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (maniJsonStringEquals(r->json, key, keys[k], strlen(keys[k])))
        {
            return k;
        }
    }
    return count;
}

bool maniGraphJsonMembers(maniGraphReader* r, uint32_t object, const char* const* keys,
                          uint32_t count, uint32_t required, uint32_t* found)
{
    if (!IsKind(r, object, maniJsonObject))
    {
        return maniGraphJsonFault(r, object, maniGraphFormValue);
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        found[k] = MANI_JSON_NONE;
    }
    uint32_t key = object + 1;
    for (uint32_t m = 0; m < r->json->values[object].count; ++m)
    {
        uint32_t k = KeyOf(r, key, keys, count);
        if (k == count)
        {
            return maniGraphJsonFault(r, key, maniGraphFormMember);
        }
        if (found[k] != MANI_JSON_NONE)
        {
            return maniGraphJsonFault(r, key, maniGraphFormRepeat);
        }
        found[k] = key + 1;
        key = r->json->values[key + 1].next;
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        if ((required >> k & 1u) != 0 && found[k] == MANI_JSON_NONE)
        {
            return maniGraphJsonFault(r, object, maniGraphFormMissing);
        }
    }
    return true;
}

bool maniGraphJsonArray(maniGraphReader* r, uint32_t value, uint32_t* countOut)
{
    if (!IsKind(r, value, maniJsonArray))
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    *countOut = r->json->values[value].count;
    return true;
}

bool maniGraphJsonName(maniGraphReader* r, uint32_t value, const char** nameOut,
                       uint32_t* lengthOut)
{
    if (!IsKind(r, value, maniJsonString))
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    uint32_t capacity = r->json->values[value].length;
    char* name = maniArenaTake(&r->arena, capacity, 1);
    if (name == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    *lengthOut = (uint32_t)maniJsonStringDecode(r->json, value, (uint8_t*)name, capacity);
    *nameOut = name;
    return true;
}

static uint64_t NameKey(const char* name, uint32_t length)
{
    return maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length);
}

bool maniGraphJsonTable(maniGraphReader* r, const char* const* names, const uint32_t* lengths,
                        uint32_t count, maniNameTable* tableOut)
{
    uint64_t* hashes = maniArenaTake(&r->arena, count, sizeof(uint64_t));
    uint64_t* keys = maniArenaTake(&r->arena, count, sizeof(uint64_t));
    uint32_t* items = maniArenaTake(&r->arena, count, sizeof(uint32_t));
    uint32_t* scratch = maniArenaTake(&r->arena, count, sizeof(uint32_t));
    if (hashes == NULL || keys == NULL || items == NULL || scratch == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        hashes[i] = NameKey(names[i], lengths[i]);
    }
    maniSortByKey(hashes, count, items, scratch);
    for (uint32_t i = 0; i < count; ++i)
    {
        keys[i] = hashes[items[i]];
    }
    *tableOut = (maniNameTable){keys, items, names, lengths, count};
    return true;
}

uint32_t maniGraphJsonLookup(const maniNameTable* table, const char* name, uint32_t length)
{
    uint64_t key = NameKey(name, length);
    uint32_t low = 0;
    uint32_t high = table->count;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (table->keys[mid] < key)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    // Equal hashes in item order: the first equal name is the first item.
    for (uint32_t i = low; i < table->count && table->keys[i] == key; ++i)
    {
        uint32_t item = table->items[i];
        if (table->lengths[item] == length &&
            (length == 0 || memcmp(table->names[item], name, length) == 0))
        {
            return item;
        }
    }
    return MANI_JSON_NONE;
}

bool maniGraphJsonRefer(maniGraphReader* r, const maniNameTable* table, uint32_t value,
                        uint32_t* itemOut)
{
    const char* name = NULL;
    uint32_t length = 0;
    if (!maniGraphJsonName(r, value, &name, &length))
    {
        return false;
    }
    *itemOut = maniGraphJsonLookup(table, name, length);
    return *itemOut != MANI_JSON_NONE || maniGraphJsonFault(r, value, maniGraphFormName);
}

bool maniGraphJsonFloat(maniGraphReader* r, uint32_t value, float* out)
{
    uint64_t bits = 0;
    if (!maniJsonBinary(r->json, value, maniBinary32, &bits))
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    uint32_t word = (uint32_t)bits;
    memcpy(out, &word, sizeof word);
    return true;
}

bool maniGraphJsonBool(maniGraphReader* r, uint32_t value, bool* out)
{
    if (!IsKind(r, value, maniJsonTrue) && !IsKind(r, value, maniJsonFalse))
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    *out = IsKind(r, value, maniJsonTrue);
    return true;
}

bool maniGraphJsonIndex(maniGraphReader* r, uint32_t value, uint32_t* out)
{
    int64_t whole = 0;
    if (!maniJsonInteger(r->json, value, 0, (int64_t)UINT32_MAX - 1, &whole))
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    *out = (uint32_t)whole;
    return true;
}

bool maniGraphJsonScalar(maniGraphReader* r, const maniGraphScope* scope, uint32_t value,
                         maniGraphScalar* out)
{
    if (IsKind(r, value, maniJsonNumber))
    {
        out->parameter = MANI_NO_PARAMETER;
        return maniGraphJsonFloat(r, value, &out->value);
    }
    static const char* const kKeys[] = {"parameter"};
    uint32_t found = 0;
    out->value = 0.0f;
    return maniGraphJsonMembers(r, value, kKeys, 1, 1u, &found) &&
           maniGraphJsonRefer(r, &scope->parameters, found, &out->parameter);
}

uint32_t maniGraphJsonWord(const maniGraphReader* r, uint32_t value, const char* const* words,
                           uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (r->json->values[value].kind == maniJsonString &&
            maniJsonStringEquals(r->json, value, words[k], strlen(words[k])))
        {
            return k;
        }
    }
    return count;
}
