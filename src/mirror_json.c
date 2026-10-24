// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A mirror map's JSON form: built before written, its axis
// and places as words, its bones by name, its reference pose as the
// bones not at their rest, pairs, rules, name pairs and names on a line
// each. Read strictly, then built as maniBuildMirrorMap builds its
// source.

#include "arena.h"
#include "builder.h"
#include "fpenv.h"
#include "graph_json_ik.h"
#include "graph_json_read.h"
#include "graph_json_write.h"
#include "json.h"
#include "map_json.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ABSENT MANI_JSON_NONE

typedef maniGraphReader Reader;
typedef maniGraphWriter Writer;

static const char kFormat[] = "maul-anim mirror map";
static const char* const kAxes[] = {"x", "y", "z"};
static const char* const kPlaces[] = {"prefix", "suffix", "anywhere"};

// Reads an object or string into its place.
typedef bool (*ReadItem)(Reader* r, uint32_t value, void* item);

// A list's items, each read by one into its place of count items of size
// bytes in the arena; NULL and 0 when absent or empty.
static bool List(Reader* r, uint32_t value, size_t size, ReadItem one, void** items,
                 uint32_t* countOut)
{
    *items = NULL;
    if (!maniMapJsonCount(r, value, countOut))
    {
        return false;
    }
    if (*countOut == 0)
    {
        return true;
    }
    uint8_t* list = maniArenaTake(&r->arena, *countOut, size);
    if (list == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < *countOut; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        if (!one(r, e, list + (size_t)i * size))
        {
            return false;
        }
    }
    *items = list;
    return true;
}

static bool Pair(Reader* r, uint32_t object, void* item)
{
    static const char* const kKeys[] = {"first", "second"};
    maniMirrorPairSource* p = item;
    uint32_t f[2];
    return maniGraphJsonMembers(r, object, kKeys, 2, 0x3u, f) &&
           maniGraphJsonBone(r, f[0], &p->first) && maniGraphJsonBone(r, f[1], &p->second);
}

static bool Rule(Reader* r, uint32_t object, void* item)
{
    static const char* const kKeys[] = {"find", "place", "replace"};
    maniMirrorRuleSource* rule = item;
    uint32_t f[3];
    *rule = (maniMirrorRuleSource){0};
    return maniGraphJsonMembers(r, object, kKeys, 3, 0x7u, f) &&
           maniGraphJsonName(r, f[0], &rule->find, &rule->findLength) &&
           maniMapJsonWord(r, f[1], kPlaces, 3, &rule->place) &&
           maniGraphJsonName(r, f[2], &rule->replace, &rule->replaceLength);
}

static bool NamePair(Reader* r, uint32_t object, void* item)
{
    static const char* const kKeys[] = {"first", "second"};
    maniMirrorNamePairSource* p = item;
    uint32_t f[2];
    return maniGraphJsonMembers(r, object, kKeys, 2, 0x3u, f) &&
           maniGraphJsonName(r, f[0], &p->first, &p->firstLength) &&
           maniGraphJsonName(r, f[1], &p->second, &p->secondLength);
}

static bool Name(Reader* r, uint32_t value, void* item)
{
    maniMirrorNameSource* n = item;
    *n = (maniMirrorNameSource){0};
    return maniGraphJsonName(r, value, &n->name, &n->nameLength);
}

static bool ReadMap(Reader* r, maniMirrorMapSource* out)
{
    static const char* const kKeys[] = {"format", "version", "axis",      "reference",
                                        "pairs",  "rules",   "namePairs", "names"};
    uint32_t f[8];
    void* lists[4] = {NULL, NULL, NULL, NULL};
    *out = (maniMirrorMapSource){.skeleton = r->skeleton};
    bool ok =
        maniGraphJsonMembers(r, 0, kKeys, 8, 0x7u, f) && maniMapJsonHeader(r, f, kFormat) &&
        maniMapJsonWord(r, f[2], kAxes, 3, &out->axis) &&
        maniMapJsonReference(r, f[3], &out->reference) &&
        List(r, f[4], sizeof(maniMirrorPairSource), Pair, &lists[0], &out->pairCount) &&
        List(r, f[5], sizeof(maniMirrorRuleSource), Rule, &lists[1], &out->ruleCount) &&
        List(r, f[6], sizeof(maniMirrorNamePairSource), NamePair, &lists[2], &out->namePairCount) &&
        List(r, f[7], sizeof(maniMirrorNameSource), Name, &lists[3], &out->nameCount);
    out->pairs = lists[0];
    out->rules = lists[1];
    out->namePairs = lists[2];
    out->names = lists[3];
    return ok;
}

static MANI_NOINLINE maniBuildResult BuildMirrorMapJsonWork(maniBuilder* builder, const char* text,
                                                            size_t length,
                                                            const maniSkeleton* skeleton)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    if ((text == NULL && length > 0) || skeleton == NULL)
    {
        maniBuilderMisuse(builder);
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    maniJson json;
    Reader r;
    maniBuildResult result;
    if (!maniMapJsonOpen(builder, text, length, &json, &r, &result))
    {
        return result;
    }
    r.skeleton = skeleton;
    maniMirrorMapSource map;
    result = ReadMap(&r, &map) ? maniBuildMirrorMap(builder, &map) : r.fault;
    maniMapJsonClose(builder, &json, &r);
    return result;
}

maniBuildResult maniBuildMirrorMapJson(maniBuilder* builder, const char* text, size_t length,
                                       const maniSkeleton* skeleton)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniBuildResult result = BuildMirrorMapJsonWork(builder, text, length, skeleton);
    maniFloatEnvLeave(&env);
    return result;
}

// "{"first": a, "second": b}" of two strings.
static void PutTwo(Writer* w, const char* first, uint32_t firstLength, const char* second,
                   uint32_t secondLength)
{
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "first");
    maniGraphJsonPutString(w, first, firstLength);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "second");
    maniGraphJsonPutString(w, second, secondLength);
    maniGraphJsonPutText(w, "}");
}

static void PutRule(Writer* w, const maniMirrorRuleSource* rule)
{
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "find");
    maniGraphJsonPutString(w, rule->find, rule->findLength);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "place");
    maniGraphJsonPutText(w, "\"");
    maniGraphJsonPutText(w, kPlaces[rule->place]);
    maniGraphJsonPutText(w, "\", ");
    maniGraphJsonPutKey(w, "replace");
    maniGraphJsonPutString(w, rule->replace, rule->replaceLength);
    maniGraphJsonPutText(w, "}");
}

static void Draw(Writer* w, const void* source)
{
    const maniMirrorMapSource* s = source;
    maniGraphJsonPutText(w, "{");
    maniMapJsonPutHeader(w, kFormat);
    maniGraphJsonLine(w, 1);
    maniGraphJsonPutKey(w, "axis");
    maniGraphJsonPutText(w, "\"");
    maniGraphJsonPutText(w, kAxes[s->axis]);
    maniGraphJsonPutText(w, "\",");
    maniMapJsonPutReference(w, "reference", s->reference);
    maniMapJsonPutOpen(w, "pairs");
    for (uint32_t i = 0; i < s->pairCount; ++i)
    {
        uint32_t first = 0;
        uint32_t second = 0;
        const char* a = maniSkeletonBoneName(s->skeleton, s->pairs[i].first, &first);
        const char* b = maniSkeletonBoneName(s->skeleton, s->pairs[i].second, &second);
        maniMapJsonPutItem(w, i);
        PutTwo(w, a, first, b, second);
    }
    maniMapJsonPutClose(w, s->pairCount, false);
    maniMapJsonPutOpen(w, "rules");
    for (uint32_t i = 0; i < s->ruleCount; ++i)
    {
        maniMapJsonPutItem(w, i);
        PutRule(w, &s->rules[i]);
    }
    maniMapJsonPutClose(w, s->ruleCount, false);
    maniMapJsonPutOpen(w, "namePairs");
    for (uint32_t i = 0; i < s->namePairCount; ++i)
    {
        const maniMirrorNamePairSource* p = &s->namePairs[i];
        maniMapJsonPutItem(w, i);
        PutTwo(w, p->first, p->firstLength, p->second, p->secondLength);
    }
    maniMapJsonPutClose(w, s->namePairCount, false);
    maniMapJsonPutOpen(w, "names");
    for (uint32_t i = 0; i < s->nameCount; ++i)
    {
        maniMapJsonPutItem(w, i);
        maniGraphJsonPutString(w, s->names[i].name, s->names[i].nameLength);
    }
    maniMapJsonPutClose(w, s->nameCount, true);
    maniGraphJsonPutText(w, "\n}\n");
}

static MANI_NOINLINE maniBuildResult WriteMirrorMapJsonWork(maniBuilder* builder,
                                                            const maniMirrorMapSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    maniBuildResult r = maniBuildMirrorMap(builder, source);
    return r.result == mani_success ? maniMapJsonKeep(builder, r, source->skeleton, Draw, source)
                                    : r;
}

maniBuildResult maniWriteMirrorMapJson(maniBuilder* builder, const maniMirrorMapSource* source)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniBuildResult result = WriteMirrorMapJsonWork(builder, source);
    maniFloatEnvLeave(&env);
    return result;
}
