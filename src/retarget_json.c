// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A retargeting map's JSON form: built before written, its
// bones by name against the source and target skeletons, each reference
// pose as the bones not at their rest, pairs, chains and the pelvis on a
// line each, modes as words. Read strictly, then built as
// maniBuildRetargetMap builds its source.

#include "arena.h"
#include "builder.h"
#include "graph_json_ik.h"
#include "graph_json_read.h"
#include "graph_json_write.h"
#include "json.h"
#include "map_json.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ABSENT MANI_JSON_NONE

typedef maniGraphReader Reader;
typedef maniGraphWriter Writer;

static const char kFormat[] = "maul-anim retarget map";
static const char* const kTranslations[] = {"own", "scaled", "absolute"};
static const char* const kRotations[] = {"none", "interpolated", "oneToOne", "oneToOneReversed"};

// The two skeletons a map's bones name.
typedef struct Sides
{
    const maniSkeleton* source;
    const maniSkeleton* target;
} Sides;

// A bone of a skeleton by name.
static bool BoneOf(Reader* r, const maniSkeleton* skeleton, uint32_t value, uint32_t* out)
{
    r->skeleton = skeleton;
    return maniGraphJsonBone(r, value, out);
}

// A list's objects, each read by one into its place of count items of
// size bytes in the arena.
typedef bool (*ReadItem)(Reader* r, const Sides* sides, uint32_t object, void* item);

static bool List(Reader* r, const Sides* sides, uint32_t value, size_t size, ReadItem one,
                 void** items, uint32_t* countOut)
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
        if (!one(r, sides, e, list + (size_t)i * size))
        {
            return false;
        }
    }
    *items = list;
    return true;
}

static bool Pair(Reader* r, const Sides* sides, uint32_t object, void* item)
{
    static const char* const kKeys[] = {"target", "source", "rotationWeight", "translation",
                                        "scale"};
    maniRetargetPairSource* p = item;
    uint32_t f[5];
    *p = (maniRetargetPairSource){.translation = maniRetargetTranslationOwn};
    return maniGraphJsonMembers(r, object, kKeys, 5, 0x7u, f) &&
           BoneOf(r, sides->target, f[0], &p->target) &&
           BoneOf(r, sides->source, f[1], &p->source) &&
           maniGraphJsonFloat(r, f[2], &p->rotationWeight) &&
           maniMapJsonWord(r, f[3], kTranslations, 3, &p->translation) &&
           (f[4] == ABSENT || maniGraphJsonBool(r, f[4], &p->scale));
}

static bool Chain(Reader* r, const Sides* sides, uint32_t object, void* item)
{
    static const char* const kKeys[] = {"sourceStart", "sourceEnd",      "targetStart", "targetEnd",
                                        "rotation",    "rotationWeight", "translation", "scale"};
    maniRetargetChainSource* c = item;
    uint32_t f[8];
    *c = (maniRetargetChainSource){.translation = maniRetargetTranslationOwn};
    return maniGraphJsonMembers(r, object, kKeys, 8, 0x3Fu, f) &&
           BoneOf(r, sides->source, f[0], &c->sourceStart) &&
           BoneOf(r, sides->source, f[1], &c->sourceEnd) &&
           BoneOf(r, sides->target, f[2], &c->targetStart) &&
           BoneOf(r, sides->target, f[3], &c->targetEnd) &&
           maniMapJsonWord(r, f[4], kRotations, 4, &c->rotation) &&
           maniGraphJsonFloat(r, f[5], &c->rotationWeight) &&
           maniMapJsonWord(r, f[6], kTranslations, 3, &c->translation) &&
           (f[7] == ABSENT || maniGraphJsonBool(r, f[7], &c->scale));
}

// The pelvis, NULL when null or absent.
static bool Pelvis(Reader* r, const Sides* sides, uint32_t object,
                   const maniRetargetPelvisSource** out)
{
    static const char* const kKeys[] = {
        "source",        "target",   "up",       "horizontal",
        "vertical",      "absolute", "toSource", "translationWeight",
        "rotationWeight"};
    *out = NULL;
    if (object == ABSENT || r->json->values[object].kind == maniJsonNull)
    {
        return true;
    }
    uint32_t f[9];
    float up[3] = {0.0f, 0.0f, 0.0f};
    maniRetargetPelvisSource* p = maniArenaTake(&r->arena, 1, sizeof *p);
    if (p == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    *p = (maniRetargetPelvisSource){0};
    *out = p;
    bool ok =
        maniGraphJsonMembers(r, object, kKeys, 9, 0x19Fu, f) &&
        BoneOf(r, sides->source, f[0], &p->source) && BoneOf(r, sides->target, f[1], &p->target) &&
        maniGraphJsonFloats(r, f[2], up, 3) && maniGraphJsonFloat(r, f[3], &p->horizontal) &&
        maniGraphJsonFloat(r, f[4], &p->vertical) && maniGraphJsonVector(r, f[5], &p->absolute) &&
        maniGraphJsonVector(r, f[6], &p->toSource) &&
        maniGraphJsonFloat(r, f[7], &p->translationWeight) &&
        maniGraphJsonFloat(r, f[8], &p->rotationWeight);
    p->up = (maniVec3){up[0], up[1], up[2]};
    return ok;
}

static bool ReadMap(Reader* r, const Sides* sides, maniRetargetMapSource* out)
{
    static const char* const kKeys[] = {"format", "version", "sourceReference", "targetReference",
                                        "pairs",  "chains",  "pelvis",          "matchNames"};
    uint32_t f[8];
    void* pairs = NULL;
    void* chains = NULL;
    *out = (maniRetargetMapSource){.source = sides->source, .target = sides->target};
    bool ok = maniGraphJsonMembers(r, 0, kKeys, 8, 0x3u, f) && maniMapJsonHeader(r, f, kFormat);
    r->skeleton = sides->source;
    ok = ok && maniMapJsonReference(r, f[2], &out->sourceReference);
    r->skeleton = sides->target;
    ok = ok && maniMapJsonReference(r, f[3], &out->targetReference) &&
         List(r, sides, f[4], sizeof(maniRetargetPairSource), Pair, &pairs, &out->pairCount) &&
         List(r, sides, f[5], sizeof(maniRetargetChainSource), Chain, &chains, &out->chainCount) &&
         Pelvis(r, sides, f[6], &out->pelvis) &&
         (f[7] == ABSENT || maniGraphJsonBool(r, f[7], &out->matchNames));
    out->pairs = pairs;
    out->chains = chains;
    return ok;
}

maniBuildResult maniBuildRetargetMapJson(maniBuilder* builder, const char* text, size_t length,
                                         const maniSkeleton* source, const maniSkeleton* target)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    if ((text == NULL && length > 0) || source == NULL || target == NULL)
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
    const Sides sides = {source, target};
    maniRetargetMapSource map;
    result = ReadMap(&r, &sides, &map) ? maniBuildRetargetMap(builder, &map) : r.fault;
    maniMapJsonClose(builder, &json, &r);
    return result;
}

// ", "key": word".
static void PutWord(Writer* w, const char* key, const char* word)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutText(w, "\"");
    maniGraphJsonPutText(w, word);
    maniGraphJsonPutText(w, "\"");
}

// ", "key": number".
static void PutFloat(Writer* w, const char* key, float value)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutFloat(w, value);
}

// ", "key": true" or false.
static void PutBool(Writer* w, const char* key, bool value)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutBool(w, value);
}

// ", "key": bone" of a skeleton (or without the comma when first).
static void PutBone(Writer* w, const maniSkeleton* skeleton, const char* key, uint32_t bone,
                    bool first)
{
    maniGraphJsonPutText(w, first ? "" : ", ");
    maniGraphJsonPutKey(w, key);
    w->skeleton = skeleton;
    maniGraphJsonPutBone(w, bone);
}

static void PutPair(Writer* w, const maniRetargetMapSource* s, const maniRetargetPairSource* p)
{
    maniGraphJsonPutText(w, "{");
    PutBone(w, s->target, "target", p->target, true);
    PutBone(w, s->source, "source", p->source, false);
    PutFloat(w, "rotationWeight", p->rotationWeight);
    PutWord(w, "translation", kTranslations[p->translation]);
    PutBool(w, "scale", p->scale);
    maniGraphJsonPutText(w, "}");
}

static void PutChain(Writer* w, const maniRetargetMapSource* s, const maniRetargetChainSource* c)
{
    maniGraphJsonPutText(w, "{");
    PutBone(w, s->source, "sourceStart", c->sourceStart, true);
    PutBone(w, s->source, "sourceEnd", c->sourceEnd, false);
    PutBone(w, s->target, "targetStart", c->targetStart, false);
    PutBone(w, s->target, "targetEnd", c->targetEnd, false);
    PutWord(w, "rotation", kRotations[c->rotation]);
    PutFloat(w, "rotationWeight", c->rotationWeight);
    PutWord(w, "translation", kTranslations[c->translation]);
    PutBool(w, "scale", c->scale);
    maniGraphJsonPutText(w, "}");
}

static void PutPelvis(Writer* w, const maniRetargetMapSource* s)
{
    const maniRetargetPelvisSource* p = s->pelvis;
    maniGraphJsonLine(w, 1);
    maniGraphJsonPutKey(w, "pelvis");
    if (p == NULL)
    {
        maniGraphJsonPutText(w, "null,");
        return;
    }
    const float up[3] = {p->up.x, p->up.y, p->up.z};
    const float absolute[3] = {p->absolute.x, p->absolute.y, p->absolute.z};
    const float toSource[3] = {p->toSource.x, p->toSource.y, p->toSource.z};
    maniGraphJsonPutText(w, "{");
    PutBone(w, s->source, "source", p->source, true);
    PutBone(w, s->target, "target", p->target, false);
    maniMapJsonPutFloats(w, "up", up, 3);
    PutFloat(w, "horizontal", p->horizontal);
    PutFloat(w, "vertical", p->vertical);
    maniMapJsonPutFloats(w, "absolute", absolute, 3);
    maniMapJsonPutFloats(w, "toSource", toSource, 3);
    PutFloat(w, "translationWeight", p->translationWeight);
    PutFloat(w, "rotationWeight", p->rotationWeight);
    maniGraphJsonPutText(w, "},");
}

static void Draw(Writer* w, const void* source)
{
    const maniRetargetMapSource* s = source;
    maniGraphJsonPutText(w, "{");
    maniMapJsonPutHeader(w, kFormat);
    w->skeleton = s->source;
    maniMapJsonPutReference(w, "sourceReference", s->sourceReference);
    w->skeleton = s->target;
    maniMapJsonPutReference(w, "targetReference", s->targetReference);
    maniMapJsonPutOpen(w, "pairs");
    for (uint32_t i = 0; i < s->pairCount; ++i)
    {
        maniMapJsonPutItem(w, i);
        PutPair(w, s, &s->pairs[i]);
    }
    maniMapJsonPutClose(w, s->pairCount, false);
    maniMapJsonPutOpen(w, "chains");
    for (uint32_t i = 0; i < s->chainCount; ++i)
    {
        maniMapJsonPutItem(w, i);
        PutChain(w, s, &s->chains[i]);
    }
    maniMapJsonPutClose(w, s->chainCount, false);
    PutPelvis(w, s);
    maniGraphJsonLine(w, 1);
    maniGraphJsonPutKey(w, "matchNames");
    maniGraphJsonPutBool(w, s->matchNames);
    maniGraphJsonPutText(w, "\n}\n");
}

maniBuildResult maniWriteRetargetMapJson(maniBuilder* builder, const maniRetargetMapSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    maniBuildResult r = maniBuildRetargetMap(builder, source);
    return r.result == mani_success ? maniMapJsonKeep(builder, r, source->source, Draw, source) : r;
}
