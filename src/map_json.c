// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "map_json.h"

#include "allocator.h"
#include "arena.h"
#include "builder.h"
#include "graph_json_ik.h"
#include "graph_json_read.h"
#include "graph_json_write.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ABSENT MANI_JSON_NONE

bool maniMapJsonOpen(maniBuilder* builder, const char* text, size_t length, maniJson* json,
                     maniGraphReader* r, maniBuildResult* result)
{
    maniJsonResult read = maniReadJson(&builder->allocator, (const uint8_t*)text, length, json);
    if (read.result != mani_success)
    {
        maniReleaseJson(&builder->allocator, json);
        *result = read.result == mani_errorCapacity
                      ? (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0}
                      : (maniBuildResult){mani_errorInvalid, maniBuildReasonJson,
                                          (uint32_t)read.offset, read.reason};
        return false;
    }
    *r = (maniGraphReader){.json = json, .arena = maniArenaMake(&builder->allocator)};
    return true;
}

void maniMapJsonClose(maniBuilder* builder, maniJson* json, maniGraphReader* r)
{
    maniArenaRelease(&r->arena);
    maniReleaseJson(&builder->allocator, json);
}

bool maniMapJsonHeader(maniGraphReader* r, const uint32_t* f, const char* format)
{
    int64_t version = 0;
    bool named = r->json->values[f[0]].kind == maniJsonString &&
                 maniJsonStringEquals(r->json, f[0], format, strlen(format));
    if (!named)
    {
        return maniGraphJsonFault(r, f[0], maniGraphFormVersion);
    }
    return maniJsonInteger(r->json, f[1], 1, 1, &version) ||
           maniGraphJsonFault(r, f[1], maniGraphFormVersion);
}

bool maniMapJsonCount(maniGraphReader* r, uint32_t value, uint32_t* countOut)
{
    *countOut = 0;
    return value == ABSENT || maniGraphJsonArray(r, value, countOut);
}

bool maniMapJsonWord(maniGraphReader* r, uint32_t value, const char* const* words, uint32_t count,
                     uint32_t* out)
{
    if (value == ABSENT)
    {
        return true;
    }
    uint32_t word = maniGraphJsonWord(r, value, words, count);
    if (word == count)
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    *out = word;
    return true;
}

// Numbers into out, out as it is when absent.
static bool Floats(maniGraphReader* r, uint32_t value, float* out, uint32_t count)
{
    return value == ABSENT || maniGraphJsonFloats(r, value, out, count);
}

// One entry over its bone's rest; seen marks the bones read before.
static bool Entry(maniGraphReader* r, uint32_t object, maniTransform* pose, uint8_t* seen)
{
    static const char* const kKeys[] = {"bone", "translation", "rotation", "scale"};
    uint32_t f[4];
    uint32_t bone = 0;
    if (!maniGraphJsonMembers(r, object, kKeys, 4, 1u, f) || !maniGraphJsonBone(r, f[0], &bone))
    {
        return false;
    }
    if (seen[bone] != 0)
    {
        return maniGraphJsonFault(r, f[0], maniGraphFormRepeat);
    }
    seen[bone] = 1;
    maniTransform* t = &pose[bone];
    float translation[3] = {t->translation.x, t->translation.y, t->translation.z};
    float rotation[4] = {t->rotation.x, t->rotation.y, t->rotation.z, t->rotation.w};
    float scale[3] = {t->scale.x, t->scale.y, t->scale.z};
    if (!Floats(r, f[1], translation, 3) || !Floats(r, f[2], rotation, 4) ||
        !Floats(r, f[3], scale, 3))
    {
        return false;
    }
    *t = (maniTransform){{translation[0], translation[1], translation[2]},
                         {rotation[0], rotation[1], rotation[2], rotation[3]},
                         {scale[0], scale[1], scale[2]}};
    return true;
}

bool maniMapJsonReference(maniGraphReader* r, uint32_t value, const maniTransform** out)
{
    uint32_t count = 0;
    *out = NULL;
    if (!maniMapJsonCount(r, value, &count))
    {
        return false;
    }
    if (count == 0)
    {
        return true;
    }
    uint32_t bones = maniSkeletonBoneCount(r->skeleton);
    maniTransform* pose = maniArenaTake(&r->arena, bones, sizeof *pose);
    uint8_t* seen = maniArenaTake(&r->arena, bones, 1);
    if (pose == NULL || seen == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    for (uint32_t b = 0; b < bones; ++b)
    {
        pose[b] = maniSkeletonRest(r->skeleton, b);
        seen[b] = 0;
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        if (!Entry(r, e, pose, seen))
        {
            return false;
        }
    }
    *out = pose;
    return true;
}

void maniMapJsonPutHeader(maniGraphWriter* w, const char* format)
{
    maniGraphJsonLine(w, 1);
    maniGraphJsonPutKey(w, "format");
    maniGraphJsonPutString(w, format, (uint32_t)strlen(format));
    maniGraphJsonPutText(w, ",");
    maniGraphJsonLine(w, 1);
    maniGraphJsonPutKey(w, "version");
    maniGraphJsonPutText(w, "1,");
}

void maniMapJsonPutFloats(maniGraphWriter* w, const char* key, const float* values, uint32_t count)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutText(w, "[");
    for (uint32_t k = 0; k < count; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutFloat(w, values[k]);
    }
    maniGraphJsonPutText(w, "]");
}

void maniMapJsonPutOpen(maniGraphWriter* w, const char* key)
{
    maniGraphJsonLine(w, 1);
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutText(w, "[");
}

void maniMapJsonPutItem(maniGraphWriter* w, uint32_t i)
{
    maniGraphJsonPutText(w, i > 0 ? "," : "");
    maniGraphJsonLine(w, 2);
}

void maniMapJsonPutClose(maniGraphWriter* w, uint32_t count, bool last)
{
    if (count > 0)
    {
        maniGraphJsonLine(w, 1);
    }
    maniGraphJsonPutText(w, last ? "]" : "],");
}

static_assert(sizeof(maniTransform) == 10 * sizeof(float), "ten floats");

// Whether a local is its bone's rest bit for bit.
static bool AtRest(const maniSkeleton* skeleton, uint32_t bone, const maniTransform* t)
{
    maniTransform rest = maniSkeletonRest(skeleton, bone);
    uint32_t a[10];
    uint32_t b[10];
    memcpy(a, &rest, sizeof a);
    memcpy(b, t, sizeof b);
    bool same = true;
    for (uint32_t k = 0; k < 10; ++k)
    {
        same = same && a[k] == b[k];
    }
    return same;
}

void maniMapJsonPutReference(maniGraphWriter* w, const char* key, const maniTransform* given)
{
    uint32_t bones = given != NULL ? maniSkeletonBoneCount(w->skeleton) : 0u;
    uint32_t n = 0;
    maniMapJsonPutOpen(w, key);
    for (uint32_t b = 0; b < bones; ++b)
    {
        const maniTransform* t = &given[b];
        if (AtRest(w->skeleton, b, t))
        {
            continue;
        }
        const float translation[3] = {t->translation.x, t->translation.y, t->translation.z};
        const float rotation[4] = {t->rotation.x, t->rotation.y, t->rotation.z, t->rotation.w};
        const float scale[3] = {t->scale.x, t->scale.y, t->scale.z};
        maniMapJsonPutItem(w, n++);
        maniGraphJsonPutText(w, "{");
        maniGraphJsonPutKey(w, "bone");
        maniGraphJsonPutBone(w, b);
        maniMapJsonPutFloats(w, "translation", translation, 3);
        maniMapJsonPutFloats(w, "rotation", rotation, 4);
        maniMapJsonPutFloats(w, "scale", scale, 3);
        maniGraphJsonPutText(w, "}");
    }
    maniMapJsonPutClose(w, n, false);
}

maniBuildResult maniMapJsonKeep(maniBuilder* builder, maniBuildResult built,
                                const maniSkeleton* skeleton, maniMapJsonDraw draw,
                                const void* source)
{
    maniGraphWriter w = {NULL, 0, skeleton};
    draw(&w, source);
    size_t size = w.at;
    // Kept as files are, 8-byte aligned.
    char* text = maniAllocate(&builder->allocator, size, 1, 8);
    if (text == NULL)
    {
        maniBuilderClear(builder);
        return (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0};
    }
    w = (maniGraphWriter){text, 0, skeleton};
    draw(&w, source);
    maniBuilderKeep(builder, MANI_BUILT_TEXT, (uint8_t*)text, size, NULL, 0);
    return built;
}
