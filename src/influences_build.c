// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building an influences file: each vertex's weighted
// influences checked, sorted by decreasing weight then joint (so the
// heaviest leads the dual quaternion signs, the same whatever the
// source's order), padded to groups of four and renormalized in binary64.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "influence_file.h"

#include "maul-anim/build.h"
#include "maul-anim/skin.h"

#include <stdbool.h>
#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

#define MAX_INFLUENCES (MANI_INFLUENCE_MAX_GROUPS * MANI_INFLUENCE_GROUP)

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// A weighted influence: its joint and weight, and where the source had it.
typedef struct Entry
{
    uint32_t joint;
    float weight;
    uint32_t index;
} Entry;

// Vertex v's weighted influences into entries (count out): each checked,
// a joint weighted twice refused.
static maniBuildResult Gather(const maniInfluenceSource* s, uint32_t v, Entry* entries,
                              uint32_t* countOut)
{
    uint32_t joints = maniSkinJointCount(s->skin);
    uint32_t n = 0;
    for (uint32_t i = s->offsets[v]; i < s->offsets[v + 1]; ++i)
    {
        float w = s->weights[i];
        if (!Finite(w))
        {
            return Result(mani_errorInvalid, maniBuildReasonNotFinite, v, i);
        }
        if (w < 0.0f)
        {
            return Result(mani_errorInvalid, maniBuildReasonWeight, v, i);
        }
        if (s->joints[i] >= joints)
        {
            return Result(mani_errorInvalid, maniBuildReasonJoint, v, i);
        }
        if (w == 0.0f)
        {
            continue;
        }
        for (uint32_t e = 0; e < n; ++e)
        {
            if (entries[e].joint == s->joints[i])
            {
                return Result(mani_errorInvalid, maniBuildReasonRepeatedJoint, v, entries[e].index);
            }
        }
        if (n == MAX_INFLUENCES)
        {
            return Result(mani_errorInvalid, maniBuildReasonCount, v, i);
        }
        entries[n++] = (Entry){s->joints[i], w, i};
    }
    *countOut = n;
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

// Every vertex checked; the groups the heaviest vertex needs.
static maniBuildResult Check(const maniInfluenceSource* s, uint32_t* groupsOut)
{
    if (s->vertexCount == 0 || s->offsets[0] != 0)
    {
        return Result(mani_errorInvalid, maniBuildReasonCount, 0, 0);
    }
    uint32_t most = 0;
    Entry entries[MAX_INFLUENCES];
    for (uint32_t v = 0; v < s->vertexCount; ++v)
    {
        if (s->offsets[v + 1] < s->offsets[v] ||
            (s->offsets[v + 1] > s->offsets[v] && (s->joints == NULL || s->weights == NULL)))
        {
            return Result(mani_errorInvalid, maniBuildReasonCount, v, 0);
        }
        uint32_t n = 0;
        maniBuildResult r = Gather(s, v, entries, &n);
        if (r.result != mani_success)
        {
            return r;
        }
        most = n > most ? n : most;
        float b = s->dualQuaternionWeights != NULL ? s->dualQuaternionWeights[v] : 0.0f;
        if (!(b >= 0.0f && b <= 1.0f))
        {
            return Result(mani_errorInvalid, maniBuildReasonWeight, v, 0);
        }
    }
    *groupsOut = most == 0 ? 1 : (most + MANI_INFLUENCE_GROUP - 1) / MANI_INFLUENCE_GROUP;
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

// Heavier first, then the lower joint.
static void Sort(Entry* entries, uint32_t n)
{
    for (uint32_t i = 1; i < n; ++i)
    {
        Entry e = entries[i];
        uint32_t j = i;
        while (j > 0 && (entries[j - 1].weight < e.weight ||
                         (entries[j - 1].weight == e.weight && entries[j - 1].joint > e.joint)))
        {
            entries[j] = entries[j - 1];
            --j;
        }
        entries[j] = e;
    }
}

// Vertex v's slots: sorted, renormalized, padded.
static void Lay(const maniInfluenceSource* s, uint32_t v, uint32_t per, uint8_t* joints,
                uint8_t* weights)
{
    Entry entries[MAX_INFLUENCES];
    uint32_t n = 0;
    (void)Gather(s, v, entries, &n);
    Sort(entries, n);
    double sum = 0.0;
    for (uint32_t e = 0; e < n; ++e)
    {
        sum += (double)entries[e].weight;
    }
    for (uint32_t k = 0; k < per; ++k)
    {
        size_t slot = (size_t)v * per + k;
        float w = k < n ? (float)((double)entries[k].weight / sum) : 0.0f;
        maniStoreU16(joints + 2 * slot, k < n ? (uint16_t)entries[k].joint : 0);
        memcpy(weights + 4 * slot, &w, sizeof w);
    }
}

static maniBuildResult Write(maniBuilder* builder, const maniInfluenceSource* s, uint32_t groups)
{
    uint32_t per = groups * MANI_INFLUENCE_GROUP;
    // Joints (2 bytes) and weights (4) a slot, a weight a vertex: sizes
    // checked, as size_t may be 32 bits.
    size_t slots = 0;
    size_t six = 0;
    size_t dq = 0;
    size_t bytes = 0;
    if (ckd_mul(&slots, (size_t)s->vertexCount, (size_t)per) || ckd_mul(&six, slots, (size_t)6) ||
        ckd_mul(&dq, (size_t)s->vertexCount, s->dualQuaternionWeights != NULL ? (size_t)4 : 0) ||
        ckd_add(&bytes, six, dq))
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    uint8_t* block = maniAllocate(&builder->allocator, bytes, 1, 8);
    if (block == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    uint8_t binding[MANI_INFLUENCE_BINDING_SIZE] = {0};
    maniStoreU64(binding, maniSkinIdentity(s->skin));
    maniStoreU32(binding + 8, maniSkinJointCount(s->skin));
    maniStoreU32(binding + 12, s->vertexCount);
    maniStoreU32(binding + 16, groups);
    for (uint32_t v = 0; v < s->vertexCount; ++v)
    {
        Lay(s, v, per, block, block + 2 * slots);
    }
    if (dq > 0)
    {
        memcpy(block + 6 * slots, s->dualQuaternionWeights, dq);
    }
    const uint8_t* data[MANI_INFLUENCE_SECTIONS] = {binding, block, block + 2 * slots,
                                                    block + 6 * slots, block};
    const uint64_t sizes[MANI_INFLUENCE_SECTIONS] = {sizeof binding, 2 * slots, 4 * slots, dq, 0};
    maniSectionSource sections[MANI_INFLUENCE_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_INFLUENCE_SECTIONS; ++i)
    {
        if (sizes[i] > 0)
        {
            sections[count++] = (maniSectionSource){i + 1, data[i], sizes[i]};
        }
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindInfluences, MANI_INFLUENCE_FORMAT_VERSION,
                           maniInfluenceComputeIdentity(data, sizes), sections, count);
    }
    maniRelease(&builder->allocator, block, bytes, 1, 8);
    if (file == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    maniBuilderKeep(builder, maniKindInfluences, file, (size_t)fileSize, NULL, 0);
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

static MANI_NOINLINE maniBuildResult BuildInfluencesWork(maniBuilder* builder,
                                                         const maniInfluenceSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (source == NULL || source->skin == NULL || source->offsets == NULL)
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    uint32_t groups = 0;
    maniBuildResult r = Check(source, &groups);
    return r.result == mani_success ? Write(builder, source, groups) : r;
}

maniBuildResult maniBuildInfluences(maniBuilder* builder, const maniInfluenceSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildInfluencesWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
