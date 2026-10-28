// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The fields of every source struct a builder records, down to scalars,
// pointers and arrays, as byte ranges: every other byte of the struct is
// padding. A suite fills a source's padding with one byte pattern and
// builds with a builder whose storage starts filled with it, then does
// the same with another pattern, and requires the same bytes: a builder
// reads its source field by field, never whole, and writes every byte it
// stores. A field left out of a table is
// filled like padding, so its build differs and the suite fails. A union
// has a table per member: the struct's other fields and that member's.

#ifndef MAUL_ANIM_TEST_SOURCE_PADDING_H
#define MAUL_ANIM_TEST_SOURCE_PADDING_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/import.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/learnedtrain.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/retargetbuild.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct TestField
{
    size_t at;
    size_t size;
} TestField;

#define TEST_FIELD(type, path) {offsetof(type, path), sizeof(((type*)0)->path)}

// Writes pattern to every byte of the size bytes at p that no field
// covers.
static inline void TestFillPadding(void* p, size_t size, const TestField* fields, size_t count,
                                   uint8_t pattern)
{
    uint8_t* bytes = (uint8_t*)p;
    for (size_t b = 0; b < size; ++b)
    {
        bool covered = false;
        for (size_t f = 0; f < count && !covered; ++f)
        {
            covered = b >= fields[f].at && b < fields[f].at + fields[f].size;
        }
        bytes[b] = covered ? bytes[b] : pattern;
    }
}

// The two patterns a suite builds under.
static const uint8_t kTestPadPatterns[2] = {0xA5, 0x5A};

// An allocator whose blocks start filled with the pattern its context
// carries, so a stored byte the library never wrote differs between the
// two builds. Aligned by hand, as aligned_alloc is not everywhere: the
// block's own pointer sits just before the aligned one.
static inline void* TestPatternAlloc(size_t size, size_t alignment, void* context)
{
    size_t align = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    uint8_t* raw = (uint8_t*)malloc(size + align + sizeof(void*));
    if (raw == NULL)
    {
        return NULL;
    }
    uintptr_t at = ((uintptr_t)(raw + sizeof(void*)) + align - 1) & ~(uintptr_t)(align - 1);
    uint8_t* block = (uint8_t*)at;
    memcpy(block - sizeof(void*), &raw, sizeof raw);
    memset(block, (int)(uintptr_t)context, size);
    return block;
}

static inline void TestPatternFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    if (memory != NULL)
    {
        void* raw = NULL;
        memcpy(&raw, (uint8_t*)memory - sizeof raw, sizeof raw);
        free(raw);
    }
}

// The allocator that fills its blocks with pattern.
static inline maniAllocator TestPatternAllocator(uint8_t pattern)
{
    return (maniAllocator){TestPatternAlloc, TestPatternFree, (void*)(uintptr_t)pattern};
}

// A builder whose storage starts filled with pattern (NULL on failure).
static inline maniBuilder* TestPatternBuilder(uint8_t pattern)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    def.allocator = TestPatternAllocator(pattern);
    maniBuilder* builder = NULL;
    return maniCreateBuilder(&def, &builder) == mani_success ? builder : NULL;
}

// The builder's last result in a heap block of its size (NULL when the
// build failed); *sizeOut its size.
static inline uint8_t* TestCopyBuilt(maniBuilder* builder, size_t* sizeOut)
{
    *sizeOut = 0;
    if (maniCopyBuilt(builder, NULL, 0, sizeOut) != mani_errorCapacity)
    {
        return NULL;
    }
    uint8_t* bytes = (uint8_t*)malloc(*sizeOut);
    if (bytes != NULL && maniCopyBuilt(builder, bytes, *sizeOut, sizeOut) != mani_success)
    {
        free(bytes);
        bytes = NULL;
    }
    return bytes;
}

// Whether both builds were made and hold the same bytes; frees both.
static inline bool TestSameBuilt(uint8_t* built[2], const size_t sizes[2])
{
    bool same = built[0] != NULL && built[1] != NULL && sizes[0] == sizes[1] &&
                memcmp(built[0], built[1], sizes[0]) == 0;
    free(built[0]);
    free(built[1]);
    return same;
}

// n structs of size bytes from p in a heap block of their own (NULL for
// none; freed by the caller).
static inline void* TestCopyOf(const void* p, size_t n, size_t size)
{
    void* q = p != NULL && n > 0 ? malloc(n * size) : NULL;
    if (q != NULL)
    {
        memcpy(q, p, n * size);
    }
    return q;
}

// The padding of n structs of a type at p, by its table.
#define TEST_PAD(p, n, table, pattern)                                                             \
    do                                                                                             \
    {                                                                                              \
        for (size_t test_pad_i = 0; test_pad_i < (size_t)(n); ++test_pad_i)                        \
        {                                                                                          \
            TestFillPadding(&(p)[test_pad_i], sizeof *(p), table,                                  \
                            sizeof(table) / sizeof(table)[0], pattern);                            \
        }                                                                                          \
    } while (0)

static const TestField kFieldsBodyBone[] = {
    TEST_FIELD(maniBodyBone, bone),
    TEST_FIELD(maniBodyBone, limit.kind),
    TEST_FIELD(maniBodyBone, limit.axis.x),
    TEST_FIELD(maniBodyBone, limit.axis.y),
    TEST_FIELD(maniBodyBone, limit.axis.z),
    TEST_FIELD(maniBodyBone, limit.cone),
    TEST_FIELD(maniBodyBone, limit.low),
    TEST_FIELD(maniBodyBone, limit.high),
    TEST_FIELD(maniBodyBone, limit.twistLow),
    TEST_FIELD(maniBodyBone, limit.twistHigh),
    TEST_FIELD(maniBodyBone, preferred.x),
    TEST_FIELD(maniBodyBone, preferred.y),
    TEST_FIELD(maniBodyBone, preferred.z),
    TEST_FIELD(maniBodyBone, rotationStiffness),
    TEST_FIELD(maniBodyBone, positionStiffness),
};

static const TestField kFieldsBoneSource[] = {
    TEST_FIELD(maniBoneSource, name),
    TEST_FIELD(maniBoneSource, nameLength),
    TEST_FIELD(maniBoneSource, parent),
    TEST_FIELD(maniBoneSource, rest.translation.x),
    TEST_FIELD(maniBoneSource, rest.translation.y),
    TEST_FIELD(maniBoneSource, rest.translation.z),
    TEST_FIELD(maniBoneSource, rest.rotation.x),
    TEST_FIELD(maniBoneSource, rest.rotation.y),
    TEST_FIELD(maniBoneSource, rest.rotation.z),
    TEST_FIELD(maniBoneSource, rest.rotation.w),
    TEST_FIELD(maniBoneSource, rest.scale.x),
    TEST_FIELD(maniBoneSource, rest.scale.y),
    TEST_FIELD(maniBoneSource, rest.scale.z),
};

static const TestField kFieldsBoneTrackSource[] = {
    TEST_FIELD(maniBoneTrackSource, bone),
    TEST_FIELD(maniBoneTrackSource, translation.times),
    TEST_FIELD(maniBoneTrackSource, translation.values),
    TEST_FIELD(maniBoneTrackSource, translation.keyCount),
    TEST_FIELD(maniBoneTrackSource, translation.interpolation),
    TEST_FIELD(maniBoneTrackSource, rotation.times),
    TEST_FIELD(maniBoneTrackSource, rotation.values),
    TEST_FIELD(maniBoneTrackSource, rotation.keyCount),
    TEST_FIELD(maniBoneTrackSource, rotation.interpolation),
    TEST_FIELD(maniBoneTrackSource, scale.times),
    TEST_FIELD(maniBoneTrackSource, scale.values),
    TEST_FIELD(maniBoneTrackSource, scale.keyCount),
    TEST_FIELD(maniBoneTrackSource, scale.interpolation),
};

static const TestField kFieldsCenterSource[] = {
    TEST_FIELD(maniCenterSource, influences),    TEST_FIELD(maniCenterSource, positions),
    TEST_FIELD(maniCenterSource, stride),        TEST_FIELD(maniCenterSource, triangles),
    TEST_FIELD(maniCenterSource, triangleCount), TEST_FIELD(maniCenterSource, sigma),
};

static const TestField kFieldsChannelSource[] = {
    TEST_FIELD(maniChannelSource, times),
    TEST_FIELD(maniChannelSource, values),
    TEST_FIELD(maniChannelSource, keyCount),
    TEST_FIELD(maniChannelSource, interpolation),
};

static const TestField kFieldsClipSource[] = {
    TEST_FIELD(maniClipSource, skeleton),
    TEST_FIELD(maniClipSource, boneTracks),
    TEST_FIELD(maniClipSource, floatTracks),
    TEST_FIELD(maniClipSource, discreteTracks),
    TEST_FIELD(maniClipSource, events),
    TEST_FIELD(maniClipSource, markers),
    TEST_FIELD(maniClipSource, boneTrackCount),
    TEST_FIELD(maniClipSource, floatTrackCount),
    TEST_FIELD(maniClipSource, discreteTrackCount),
    TEST_FIELD(maniClipSource, eventCount),
    TEST_FIELD(maniClipSource, markerCount),
    TEST_FIELD(maniClipSource, loop),
    TEST_FIELD(maniClipSource, sampleRate),
    TEST_FIELD(maniClipSource, duration),
    TEST_FIELD(maniClipSource, additive),
    TEST_FIELD(maniClipSource, rootMotion),
};

static const TestField kFieldsColliderGroupSource[] = {
    TEST_FIELD(maniColliderGroupSource, name),
    TEST_FIELD(maniColliderGroupSource, nameLength),
    TEST_FIELD(maniColliderGroupSource, colliderCount),
    TEST_FIELD(maniColliderGroupSource, colliders),
};

static const TestField kFieldsColliderSource[] = {
    TEST_FIELD(maniColliderSource, bone),     TEST_FIELD(maniColliderSource, shape),
    TEST_FIELD(maniColliderSource, offset.x), TEST_FIELD(maniColliderSource, offset.y),
    TEST_FIELD(maniColliderSource, offset.z), TEST_FIELD(maniColliderSource, tail.x),
    TEST_FIELD(maniColliderSource, tail.y),   TEST_FIELD(maniColliderSource, tail.z),
    TEST_FIELD(maniColliderSource, radius),   TEST_FIELD(maniColliderSource, inside),
};

static const TestField kFieldsCompressOptions[] = {
    TEST_FIELD(maniCompressOptions, precision),
    TEST_FIELD(maniCompressOptions, shellDistance),
    TEST_FIELD(maniCompressOptions, bonePrecisions),
    TEST_FIELD(maniCompressOptions, boneShellDistances),
    TEST_FIELD(maniCompressOptions, floatPrecision),
    TEST_FIELD(maniCompressOptions, floatPrecisions),
};

static const TestField kFieldsDiscreteTrackSource[] = {
    TEST_FIELD(maniDiscreteTrackSource, name),     TEST_FIELD(maniDiscreteTrackSource, nameLength),
    TEST_FIELD(maniDiscreteTrackSource, keyCount), TEST_FIELD(maniDiscreteTrackSource, times),
    TEST_FIELD(maniDiscreteTrackSource, values),
};

static const TestField kFieldsEventSource[] = {
    TEST_FIELD(maniEventSource, time),
    TEST_FIELD(maniEventSource, payload),
    TEST_FIELD(maniEventSource, name),
    TEST_FIELD(maniEventSource, nameLength),
};

static const TestField kFieldsFbxOptions[] = {
    TEST_FIELD(maniFbxOptions, axes),           TEST_FIELD(maniFbxOptions, convertAxes),
    TEST_FIELD(maniFbxOptions, foldPlacement),  TEST_FIELD(maniFbxOptions, unitMeters),
    TEST_FIELD(maniFbxOptions, inheritHelpers),
};

static const TestField kFieldsFloatTrackSource[] = {
    TEST_FIELD(maniFloatTrackSource, name),
    TEST_FIELD(maniFloatTrackSource, nameLength),
    TEST_FIELD(maniFloatTrackSource, keys.times),
    TEST_FIELD(maniFloatTrackSource, keys.values),
    TEST_FIELD(maniFloatTrackSource, keys.keyCount),
    TEST_FIELD(maniFloatTrackSource, keys.interpolation),
};

static const TestField kFieldsGltfOptions[] = {
    TEST_FIELD(maniGltfOptions, axes),
    TEST_FIELD(maniGltfOptions, foldPlacement),
};

static const TestField kFieldsGraphAdditiveNodeSource[] = {
    TEST_FIELD(maniGraphAdditiveNodeSource, weights),
};

static const TestField kFieldsGraphAimNodeSource[] = {
    TEST_FIELD(maniGraphAimNodeSource, bones),
    TEST_FIELD(maniGraphAimNodeSource, shares),
    TEST_FIELD(maniGraphAimNodeSource, count),
    TEST_FIELD(maniGraphAimNodeSource, goal),
    TEST_FIELD(maniGraphAimNodeSource, pole),
    TEST_FIELD(maniGraphAimNodeSource, forward.x),
    TEST_FIELD(maniGraphAimNodeSource, forward.y),
    TEST_FIELD(maniGraphAimNodeSource, forward.z),
    TEST_FIELD(maniGraphAimNodeSource, offset.x),
    TEST_FIELD(maniGraphAimNodeSource, offset.y),
    TEST_FIELD(maniGraphAimNodeSource, offset.z),
    TEST_FIELD(maniGraphAimNodeSource, up.x),
    TEST_FIELD(maniGraphAimNodeSource, up.y),
    TEST_FIELD(maniGraphAimNodeSource, up.z),
    TEST_FIELD(maniGraphAimNodeSource, twist.parameter),
    TEST_FIELD(maniGraphAimNodeSource, twist.value),
    TEST_FIELD(maniGraphAimNodeSource, cone.parameter),
    TEST_FIELD(maniGraphAimNodeSource, cone.value),
    TEST_FIELD(maniGraphAimNodeSource, weight.parameter),
    TEST_FIELD(maniGraphAimNodeSource, weight.value),
};

static const TestField kFieldsGraphBandsNodeSource[] = {
    TEST_FIELD(maniGraphBandsNodeSource, position.vector),
    TEST_FIELD(maniGraphBandsNodeSource, position.x.parameter),
    TEST_FIELD(maniGraphBandsNodeSource, position.x.value),
    TEST_FIELD(maniGraphBandsNodeSource, position.y.parameter),
    TEST_FIELD(maniGraphBandsNodeSource, position.y.value),
    TEST_FIELD(maniGraphBandsNodeSource, points),
    TEST_FIELD(maniGraphBandsNodeSource, polar),
};

static const TestField kFieldsGraphBlendNodeSource[] = {
    TEST_FIELD(maniGraphBlendNodeSource, weights),
    TEST_FIELD(maniGraphBlendNodeSource, direct),
};

static const TestField kFieldsGraphBlendSpace1DNodeSource[] = {
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, position.vector),
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, position.x.parameter),
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, position.x.value),
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, position.y.parameter),
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, position.y.value),
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, points),
    TEST_FIELD(maniGraphBlendSpace1DNodeSource, period),
};

static const TestField kFieldsGraphBodyNodeSource[] = {
    TEST_FIELD(maniGraphBodyNodeSource, top),
    TEST_FIELD(maniGraphBodyNodeSource, topMode),
    TEST_FIELD(maniGraphBodyNodeSource, effectors),
    TEST_FIELD(maniGraphBodyNodeSource, bones),
    TEST_FIELD(maniGraphBodyNodeSource, effectorCount),
    TEST_FIELD(maniGraphBodyNodeSource, boneCount),
    TEST_FIELD(maniGraphBodyNodeSource, iterations),
    TEST_FIELD(maniGraphBodyNodeSource, stretch),
    TEST_FIELD(maniGraphBodyNodeSource, tolerance),
    TEST_FIELD(maniGraphBodyNodeSource, weight.parameter),
    TEST_FIELD(maniGraphBodyNodeSource, weight.value),
};

static const TestField kFieldsGraphChainNodeSource[] = {
    TEST_FIELD(maniGraphChainNodeSource, root),
    TEST_FIELD(maniGraphChainNodeSource, end),
    TEST_FIELD(maniGraphChainNodeSource, goal),
    TEST_FIELD(maniGraphChainNodeSource, pole),
    TEST_FIELD(maniGraphChainNodeSource, rotation),
    TEST_FIELD(maniGraphChainNodeSource, iterations),
    TEST_FIELD(maniGraphChainNodeSource, limits),
    TEST_FIELD(maniGraphChainNodeSource, tolerance),
    TEST_FIELD(maniGraphChainNodeSource, step),
    TEST_FIELD(maniGraphChainNodeSource, rotationWeight.parameter),
    TEST_FIELD(maniGraphChainNodeSource, rotationWeight.value),
    TEST_FIELD(maniGraphChainNodeSource, weight.parameter),
    TEST_FIELD(maniGraphChainNodeSource, weight.value),
};

static const TestField kFieldsGraphClipNodeSource[] = {
    TEST_FIELD(maniGraphClipNodeSource, clip),
    TEST_FIELD(maniGraphClipNodeSource, syncGroup),
    TEST_FIELD(maniGraphClipNodeSource, speed.parameter),
    TEST_FIELD(maniGraphClipNodeSource, speed.value),
    TEST_FIELD(maniGraphClipNodeSource, loop),
    TEST_FIELD(maniGraphClipNodeSource, flags),
};

static const TestField kFieldsGraphClipSource[] = {
    TEST_FIELD(maniGraphClipSource, clip),
    TEST_FIELD(maniGraphClipSource, name),
    TEST_FIELD(maniGraphClipSource, nameLength),
    TEST_FIELD(maniGraphClipSource, map),
};

static const TestField kFieldsGraphConditionSource_b[] = {
    TEST_FIELD(maniGraphConditionSource, kind),
    TEST_FIELD(maniGraphConditionSource, parameter),
    TEST_FIELD(maniGraphConditionSource, compare),
    TEST_FIELD(maniGraphConditionSource, eventLength),
    TEST_FIELD(maniGraphConditionSource, value.b),
    TEST_FIELD(maniGraphConditionSource, event),
};

static const TestField kFieldsGraphConditionSource_f[] = {
    TEST_FIELD(maniGraphConditionSource, kind),
    TEST_FIELD(maniGraphConditionSource, parameter),
    TEST_FIELD(maniGraphConditionSource, compare),
    TEST_FIELD(maniGraphConditionSource, eventLength),
    TEST_FIELD(maniGraphConditionSource, value.f),
    TEST_FIELD(maniGraphConditionSource, event),
};

static const TestField kFieldsGraphConditionSource_i[] = {
    TEST_FIELD(maniGraphConditionSource, kind),
    TEST_FIELD(maniGraphConditionSource, parameter),
    TEST_FIELD(maniGraphConditionSource, compare),
    TEST_FIELD(maniGraphConditionSource, eventLength),
    TEST_FIELD(maniGraphConditionSource, value.i),
    TEST_FIELD(maniGraphConditionSource, event),
};

static const TestField kFieldsGraphDatabaseSource[] = {
    TEST_FIELD(maniGraphDatabaseSource, database),
    TEST_FIELD(maniGraphDatabaseSource, name),
    TEST_FIELD(maniGraphDatabaseSource, nameLength),
};

static const TestField kFieldsGraphEffectorSource[] = {
    TEST_FIELD(maniGraphEffectorSource, bone),
    TEST_FIELD(maniGraphEffectorSource, position),
    TEST_FIELD(maniGraphEffectorSource, rotation),
    TEST_FIELD(maniGraphEffectorSource, positionWeight.parameter),
    TEST_FIELD(maniGraphEffectorSource, positionWeight.value),
    TEST_FIELD(maniGraphEffectorSource, rotationWeight.parameter),
    TEST_FIELD(maniGraphEffectorSource, rotationWeight.value),
    TEST_FIELD(maniGraphEffectorSource, strength),
    TEST_FIELD(maniGraphEffectorSource, pullChain),
};

static const TestField kFieldsGraphInertializeNodeSource[] = {
    TEST_FIELD(maniGraphInertializeNodeSource, trigger),
    TEST_FIELD(maniGraphInertializeNodeSource, method),
    TEST_FIELD(maniGraphInertializeNodeSource, duration.parameter),
    TEST_FIELD(maniGraphInertializeNodeSource, duration.value),
    TEST_FIELD(maniGraphInertializeNodeSource, halfLifeScale),
    TEST_FIELD(maniGraphInertializeNodeSource, halfLifeMin),
    TEST_FIELD(maniGraphInertializeNodeSource, halfLifeMax),
};

static const TestField kFieldsGraphLearnedNodeSource[] = {
    TEST_FIELD(maniGraphLearnedNodeSource, model),
    TEST_FIELD(maniGraphLearnedNodeSource, trigger),
    TEST_FIELD(maniGraphLearnedNodeSource, query),
    TEST_FIELD(maniGraphLearnedNodeSource, queryCount),
    TEST_FIELD(maniGraphLearnedNodeSource, period),
    TEST_FIELD(maniGraphLearnedNodeSource, transitionCost),
    TEST_FIELD(maniGraphLearnedNodeSource, requiredTags.parameter),
    TEST_FIELD(maniGraphLearnedNodeSource, requiredTags.value),
    TEST_FIELD(maniGraphLearnedNodeSource, excludedTags.parameter),
    TEST_FIELD(maniGraphLearnedNodeSource, excludedTags.value),
    TEST_FIELD(maniGraphLearnedNodeSource, duration.parameter),
    TEST_FIELD(maniGraphLearnedNodeSource, duration.value),
};

static const TestField kFieldsGraphMachineNodeSource[] = {
    TEST_FIELD(maniGraphMachineNodeSource, transitions),
    TEST_FIELD(maniGraphMachineNodeSource, transitionCount),
    TEST_FIELD(maniGraphMachineNodeSource, entry),
    TEST_FIELD(maniGraphMachineNodeSource, chainLimit),
    TEST_FIELD(maniGraphMachineNodeSource, finalCount),
    TEST_FIELD(maniGraphMachineNodeSource, finals),
};

static const TestField kFieldsGraphMaskNodeSource[] = {
    TEST_FIELD(maniGraphMaskNodeSource, chains),
    TEST_FIELD(maniGraphMaskNodeSource, chainCount),
    TEST_FIELD(maniGraphMaskNodeSource, defaultWeight),
};

static const TestField kFieldsGraphMatchNodeSource[] = {
    TEST_FIELD(maniGraphMatchNodeSource, database),
    TEST_FIELD(maniGraphMatchNodeSource, trigger),
    TEST_FIELD(maniGraphMatchNodeSource, query),
    TEST_FIELD(maniGraphMatchNodeSource, queryCount),
    TEST_FIELD(maniGraphMatchNodeSource, historyCount),
    TEST_FIELD(maniGraphMatchNodeSource, period),
    TEST_FIELD(maniGraphMatchNodeSource, jumpThreshold),
    TEST_FIELD(maniGraphMatchNodeSource, endExclusion),
    TEST_FIELD(maniGraphMatchNodeSource, historyTime),
    TEST_FIELD(maniGraphMatchNodeSource, continuingBias),
    TEST_FIELD(maniGraphMatchNodeSource, transitionCost),
    TEST_FIELD(maniGraphMatchNodeSource, requiredTags.parameter),
    TEST_FIELD(maniGraphMatchNodeSource, requiredTags.value),
    TEST_FIELD(maniGraphMatchNodeSource, excludedTags.parameter),
    TEST_FIELD(maniGraphMatchNodeSource, excludedTags.value),
    TEST_FIELD(maniGraphMatchNodeSource, duration.parameter),
    TEST_FIELD(maniGraphMatchNodeSource, duration.value),
};

static const TestField kFieldsGraphModelSource[] = {
    TEST_FIELD(maniGraphModelSource, model),
    TEST_FIELD(maniGraphModelSource, name),
    TEST_FIELD(maniGraphModelSource, nameLength),
};

static const TestField kFieldsGraphNamedAssets[] = {
    TEST_FIELD(maniGraphNamedAssets, skeleton),       TEST_FIELD(maniGraphNamedAssets, clips),
    TEST_FIELD(maniGraphNamedAssets, databases),      TEST_FIELD(maniGraphNamedAssets, models),
    TEST_FIELD(maniGraphNamedAssets, springSets),     TEST_FIELD(maniGraphNamedAssets, clipCount),
    TEST_FIELD(maniGraphNamedAssets, databaseCount),  TEST_FIELD(maniGraphNamedAssets, modelCount),
    TEST_FIELD(maniGraphNamedAssets, springSetCount), TEST_FIELD(maniGraphNamedAssets, mirrorMap),
};

static const TestField kFieldsGraphNodeSource_additive[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, additive.weights),
};

static const TestField kFieldsGraphNodeSource_aim[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, aim.bones),
    TEST_FIELD(maniGraphNodeSource, aim.shares),
    TEST_FIELD(maniGraphNodeSource, aim.count),
    TEST_FIELD(maniGraphNodeSource, aim.goal),
    TEST_FIELD(maniGraphNodeSource, aim.pole),
    TEST_FIELD(maniGraphNodeSource, aim.forward.x),
    TEST_FIELD(maniGraphNodeSource, aim.forward.y),
    TEST_FIELD(maniGraphNodeSource, aim.forward.z),
    TEST_FIELD(maniGraphNodeSource, aim.offset.x),
    TEST_FIELD(maniGraphNodeSource, aim.offset.y),
    TEST_FIELD(maniGraphNodeSource, aim.offset.z),
    TEST_FIELD(maniGraphNodeSource, aim.up.x),
    TEST_FIELD(maniGraphNodeSource, aim.up.y),
    TEST_FIELD(maniGraphNodeSource, aim.up.z),
    TEST_FIELD(maniGraphNodeSource, aim.twist.parameter),
    TEST_FIELD(maniGraphNodeSource, aim.twist.value),
    TEST_FIELD(maniGraphNodeSource, aim.cone.parameter),
    TEST_FIELD(maniGraphNodeSource, aim.cone.value),
    TEST_FIELD(maniGraphNodeSource, aim.weight.parameter),
    TEST_FIELD(maniGraphNodeSource, aim.weight.value),
};

static const TestField kFieldsGraphNodeSource_bands[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, bands.position.vector),
    TEST_FIELD(maniGraphNodeSource, bands.position.x.parameter),
    TEST_FIELD(maniGraphNodeSource, bands.position.x.value),
    TEST_FIELD(maniGraphNodeSource, bands.position.y.parameter),
    TEST_FIELD(maniGraphNodeSource, bands.position.y.value),
    TEST_FIELD(maniGraphNodeSource, bands.points),
    TEST_FIELD(maniGraphNodeSource, bands.polar),
};

static const TestField kFieldsGraphNodeSource_blend[] = {
    TEST_FIELD(maniGraphNodeSource, kind),         TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),       TEST_FIELD(maniGraphNodeSource, blend.weights),
    TEST_FIELD(maniGraphNodeSource, blend.direct),
};

static const TestField kFieldsGraphNodeSource_blendSpace1D[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.position.vector),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.position.x.parameter),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.position.x.value),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.position.y.parameter),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.position.y.value),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.points),
    TEST_FIELD(maniGraphNodeSource, blendSpace1D.period),
};

static const TestField kFieldsGraphNodeSource_body[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, body.top),
    TEST_FIELD(maniGraphNodeSource, body.topMode),
    TEST_FIELD(maniGraphNodeSource, body.effectors),
    TEST_FIELD(maniGraphNodeSource, body.bones),
    TEST_FIELD(maniGraphNodeSource, body.effectorCount),
    TEST_FIELD(maniGraphNodeSource, body.boneCount),
    TEST_FIELD(maniGraphNodeSource, body.iterations),
    TEST_FIELD(maniGraphNodeSource, body.stretch),
    TEST_FIELD(maniGraphNodeSource, body.tolerance),
    TEST_FIELD(maniGraphNodeSource, body.weight.parameter),
    TEST_FIELD(maniGraphNodeSource, body.weight.value),
};

static const TestField kFieldsGraphNodeSource_chain[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, chain.root),
    TEST_FIELD(maniGraphNodeSource, chain.end),
    TEST_FIELD(maniGraphNodeSource, chain.goal),
    TEST_FIELD(maniGraphNodeSource, chain.pole),
    TEST_FIELD(maniGraphNodeSource, chain.rotation),
    TEST_FIELD(maniGraphNodeSource, chain.iterations),
    TEST_FIELD(maniGraphNodeSource, chain.limits),
    TEST_FIELD(maniGraphNodeSource, chain.tolerance),
    TEST_FIELD(maniGraphNodeSource, chain.step),
    TEST_FIELD(maniGraphNodeSource, chain.rotationWeight.parameter),
    TEST_FIELD(maniGraphNodeSource, chain.rotationWeight.value),
    TEST_FIELD(maniGraphNodeSource, chain.weight.parameter),
    TEST_FIELD(maniGraphNodeSource, chain.weight.value),
};

static const TestField kFieldsGraphNodeSource_clip[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, clip.clip),
    TEST_FIELD(maniGraphNodeSource, clip.syncGroup),
    TEST_FIELD(maniGraphNodeSource, clip.speed.parameter),
    TEST_FIELD(maniGraphNodeSource, clip.speed.value),
    TEST_FIELD(maniGraphNodeSource, clip.loop),
    TEST_FIELD(maniGraphNodeSource, clip.flags),
};

static const TestField kFieldsGraphNodeSource_inertialize[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, inertialize.trigger),
    TEST_FIELD(maniGraphNodeSource, inertialize.method),
    TEST_FIELD(maniGraphNodeSource, inertialize.duration.parameter),
    TEST_FIELD(maniGraphNodeSource, inertialize.duration.value),
    TEST_FIELD(maniGraphNodeSource, inertialize.halfLifeScale),
    TEST_FIELD(maniGraphNodeSource, inertialize.halfLifeMin),
    TEST_FIELD(maniGraphNodeSource, inertialize.halfLifeMax),
};

static const TestField kFieldsGraphNodeSource_learned[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, learned.model),
    TEST_FIELD(maniGraphNodeSource, learned.trigger),
    TEST_FIELD(maniGraphNodeSource, learned.query),
    TEST_FIELD(maniGraphNodeSource, learned.queryCount),
    TEST_FIELD(maniGraphNodeSource, learned.period),
    TEST_FIELD(maniGraphNodeSource, learned.transitionCost),
    TEST_FIELD(maniGraphNodeSource, learned.requiredTags.parameter),
    TEST_FIELD(maniGraphNodeSource, learned.requiredTags.value),
    TEST_FIELD(maniGraphNodeSource, learned.excludedTags.parameter),
    TEST_FIELD(maniGraphNodeSource, learned.excludedTags.value),
    TEST_FIELD(maniGraphNodeSource, learned.duration.parameter),
    TEST_FIELD(maniGraphNodeSource, learned.duration.value),
};

static const TestField kFieldsGraphNodeSource_machine[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, machine.transitions),
    TEST_FIELD(maniGraphNodeSource, machine.transitionCount),
    TEST_FIELD(maniGraphNodeSource, machine.entry),
    TEST_FIELD(maniGraphNodeSource, machine.chainLimit),
    TEST_FIELD(maniGraphNodeSource, machine.finalCount),
    TEST_FIELD(maniGraphNodeSource, machine.finals),
};

static const TestField kFieldsGraphNodeSource_mask[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, mask.chains),
    TEST_FIELD(maniGraphNodeSource, mask.chainCount),
    TEST_FIELD(maniGraphNodeSource, mask.defaultWeight),
};

static const TestField kFieldsGraphNodeSource_match[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, match.database),
    TEST_FIELD(maniGraphNodeSource, match.trigger),
    TEST_FIELD(maniGraphNodeSource, match.query),
    TEST_FIELD(maniGraphNodeSource, match.queryCount),
    TEST_FIELD(maniGraphNodeSource, match.historyCount),
    TEST_FIELD(maniGraphNodeSource, match.period),
    TEST_FIELD(maniGraphNodeSource, match.jumpThreshold),
    TEST_FIELD(maniGraphNodeSource, match.endExclusion),
    TEST_FIELD(maniGraphNodeSource, match.historyTime),
    TEST_FIELD(maniGraphNodeSource, match.continuingBias),
    TEST_FIELD(maniGraphNodeSource, match.transitionCost),
    TEST_FIELD(maniGraphNodeSource, match.requiredTags.parameter),
    TEST_FIELD(maniGraphNodeSource, match.requiredTags.value),
    TEST_FIELD(maniGraphNodeSource, match.excludedTags.parameter),
    TEST_FIELD(maniGraphNodeSource, match.excludedTags.value),
    TEST_FIELD(maniGraphNodeSource, match.duration.parameter),
    TEST_FIELD(maniGraphNodeSource, match.duration.value),
};

static const TestField kFieldsGraphNodeSource_oneShot[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, oneShot.fire),
    TEST_FIELD(maniGraphNodeSource, oneShot.fadeOutTrigger),
    TEST_FIELD(maniGraphNodeSource, oneShot.abort),
    TEST_FIELD(maniGraphNodeSource, oneShot.curve),
    TEST_FIELD(maniGraphNodeSource, oneShot.fadeIn),
    TEST_FIELD(maniGraphNodeSource, oneShot.fadeOut),
    TEST_FIELD(maniGraphNodeSource, oneShot.mask),
};

static const TestField kFieldsGraphNodeSource_pose[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, pose.parameter),
};

static const TestField kFieldsGraphNodeSource_seek[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, seek.trigger),
    TEST_FIELD(maniGraphNodeSource, seek.time.parameter),
    TEST_FIELD(maniGraphNodeSource, seek.time.value),
};

static const TestField kFieldsGraphNodeSource_select[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, select.parameter),
};

static const TestField kFieldsGraphNodeSource_spring[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, spring.springSet),
    TEST_FIELD(maniGraphNodeSource, spring.position),
    TEST_FIELD(maniGraphNodeSource, spring.rotation),
    TEST_FIELD(maniGraphNodeSource, spring.reset),
    TEST_FIELD(maniGraphNodeSource, spring.force),
};

static const TestField kFieldsGraphNodeSource_subgraph[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, subgraph.graph),
    TEST_FIELD(maniGraphNodeSource, subgraph.parameters),
};

static const TestField kFieldsGraphNodeSource_timeScale[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, timeScale.scale.parameter),
    TEST_FIELD(maniGraphNodeSource, timeScale.scale.value),
};

static const TestField kFieldsGraphNodeSource_triangulated[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, triangulated.position.vector),
    TEST_FIELD(maniGraphNodeSource, triangulated.position.x.parameter),
    TEST_FIELD(maniGraphNodeSource, triangulated.position.x.value),
    TEST_FIELD(maniGraphNodeSource, triangulated.position.y.parameter),
    TEST_FIELD(maniGraphNodeSource, triangulated.position.y.value),
    TEST_FIELD(maniGraphNodeSource, triangulated.points),
    TEST_FIELD(maniGraphNodeSource, triangulated.triangles),
    TEST_FIELD(maniGraphNodeSource, triangulated.triangleCount),
};

static const TestField kFieldsGraphNodeSource_turnLimit[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, turnLimit.speed.parameter),
    TEST_FIELD(maniGraphNodeSource, turnLimit.speed.value),
    TEST_FIELD(maniGraphNodeSource, turnLimit.reset),
    TEST_FIELD(maniGraphNodeSource, turnLimit.mask),
};

static const TestField kFieldsGraphNodeSource_twoBone[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
    TEST_FIELD(maniGraphNodeSource, twoBone.start),
    TEST_FIELD(maniGraphNodeSource, twoBone.middle),
    TEST_FIELD(maniGraphNodeSource, twoBone.end),
    TEST_FIELD(maniGraphNodeSource, twoBone.goal),
    TEST_FIELD(maniGraphNodeSource, twoBone.pole),
    TEST_FIELD(maniGraphNodeSource, twoBone.hinge.x),
    TEST_FIELD(maniGraphNodeSource, twoBone.hinge.y),
    TEST_FIELD(maniGraphNodeSource, twoBone.hinge.z),
    TEST_FIELD(maniGraphNodeSource, twoBone.twist.parameter),
    TEST_FIELD(maniGraphNodeSource, twoBone.twist.value),
    TEST_FIELD(maniGraphNodeSource, twoBone.soften.parameter),
    TEST_FIELD(maniGraphNodeSource, twoBone.soften.value),
    TEST_FIELD(maniGraphNodeSource, twoBone.weight.parameter),
    TEST_FIELD(maniGraphNodeSource, twoBone.weight.value),
};

static const TestField kFieldsGraphOneShotNodeSource[] = {
    TEST_FIELD(maniGraphOneShotNodeSource, fire),
    TEST_FIELD(maniGraphOneShotNodeSource, fadeOutTrigger),
    TEST_FIELD(maniGraphOneShotNodeSource, abort),
    TEST_FIELD(maniGraphOneShotNodeSource, curve),
    TEST_FIELD(maniGraphOneShotNodeSource, fadeIn),
    TEST_FIELD(maniGraphOneShotNodeSource, fadeOut),
    TEST_FIELD(maniGraphOneShotNodeSource, mask),
};

static const TestField kFieldsGraphParameterSource_b[] = {
    TEST_FIELD(maniGraphParameterSource, name),
    TEST_FIELD(maniGraphParameterSource, nameLength),
    TEST_FIELD(maniGraphParameterSource, type),
    TEST_FIELD(maniGraphParameterSource, defaultValue.b),
    TEST_FIELD(maniGraphParameterSource, minimum.b),
    TEST_FIELD(maniGraphParameterSource, maximum.b),
    TEST_FIELD(maniGraphParameterSource, bounded),
};

static const TestField kFieldsGraphParameterSource_f[] = {
    TEST_FIELD(maniGraphParameterSource, name),
    TEST_FIELD(maniGraphParameterSource, nameLength),
    TEST_FIELD(maniGraphParameterSource, type),
    TEST_FIELD(maniGraphParameterSource, defaultValue.f),
    TEST_FIELD(maniGraphParameterSource, minimum.f),
    TEST_FIELD(maniGraphParameterSource, maximum.f),
    TEST_FIELD(maniGraphParameterSource, bounded),
};

static const TestField kFieldsGraphParameterSource_i[] = {
    TEST_FIELD(maniGraphParameterSource, name),
    TEST_FIELD(maniGraphParameterSource, nameLength),
    TEST_FIELD(maniGraphParameterSource, type),
    TEST_FIELD(maniGraphParameterSource, defaultValue.i),
    TEST_FIELD(maniGraphParameterSource, minimum.i),
    TEST_FIELD(maniGraphParameterSource, maximum.i),
    TEST_FIELD(maniGraphParameterSource, bounded),
};

static const TestField kFieldsGraphPoseNodeSource[] = {
    TEST_FIELD(maniGraphPoseNodeSource, parameter),
};

static const TestField kFieldsGraphPosition[] = {
    TEST_FIELD(maniGraphPosition, vector),  TEST_FIELD(maniGraphPosition, x.parameter),
    TEST_FIELD(maniGraphPosition, x.value), TEST_FIELD(maniGraphPosition, y.parameter),
    TEST_FIELD(maniGraphPosition, y.value),
};

static const TestField kFieldsGraphScalar[] = {
    TEST_FIELD(maniGraphScalar, parameter),
    TEST_FIELD(maniGraphScalar, value),
};

static const TestField kFieldsGraphSeekNodeSource[] = {
    TEST_FIELD(maniGraphSeekNodeSource, trigger),
    TEST_FIELD(maniGraphSeekNodeSource, time.parameter),
    TEST_FIELD(maniGraphSeekNodeSource, time.value),
};

static const TestField kFieldsGraphSelectNodeSource[] = {
    TEST_FIELD(maniGraphSelectNodeSource, parameter),
};

static const TestField kFieldsGraphSource[] = {
    TEST_FIELD(maniGraphSource, skeleton),      TEST_FIELD(maniGraphSource, parameters),
    TEST_FIELD(maniGraphSource, clips),         TEST_FIELD(maniGraphSource, syncGroups),
    TEST_FIELD(maniGraphSource, nodes),         TEST_FIELD(maniGraphSource, parameterCount),
    TEST_FIELD(maniGraphSource, clipCount),     TEST_FIELD(maniGraphSource, syncGroupCount),
    TEST_FIELD(maniGraphSource, nodeCount),     TEST_FIELD(maniGraphSource, output),
    TEST_FIELD(maniGraphSource, databaseCount), TEST_FIELD(maniGraphSource, databases),
    TEST_FIELD(maniGraphSource, modelCount),    TEST_FIELD(maniGraphSource, springSetCount),
    TEST_FIELD(maniGraphSource, models),        TEST_FIELD(maniGraphSource, springSets),
    TEST_FIELD(maniGraphSource, mirrorMap),
};

static const TestField kFieldsGraphSpringNodeSource[] = {
    TEST_FIELD(maniGraphSpringNodeSource, springSet),
    TEST_FIELD(maniGraphSpringNodeSource, position),
    TEST_FIELD(maniGraphSpringNodeSource, rotation),
    TEST_FIELD(maniGraphSpringNodeSource, reset),
    TEST_FIELD(maniGraphSpringNodeSource, force),
};

static const TestField kFieldsGraphSpringSetSource[] = {
    TEST_FIELD(maniGraphSpringSetSource, set),
    TEST_FIELD(maniGraphSpringSetSource, name),
    TEST_FIELD(maniGraphSpringSetSource, nameLength),
};

static const TestField kFieldsGraphSubgraphNodeSource[] = {
    TEST_FIELD(maniGraphSubgraphNodeSource, graph),
    TEST_FIELD(maniGraphSubgraphNodeSource, parameters),
};

static const TestField kFieldsGraphSyncGroupSource[] = {
    TEST_FIELD(maniGraphSyncGroupSource, name),
    TEST_FIELD(maniGraphSyncGroupSource, nameLength),
    TEST_FIELD(maniGraphSyncGroupSource, flags),
};

static const TestField kFieldsGraphTags[] = {
    TEST_FIELD(maniGraphTags, parameter),
    TEST_FIELD(maniGraphTags, value),
};

static const TestField kFieldsGraphTimeScaleNodeSource[] = {
    TEST_FIELD(maniGraphTimeScaleNodeSource, scale.parameter),
    TEST_FIELD(maniGraphTimeScaleNodeSource, scale.value),
};

static const TestField kFieldsGraphTransitionSource[] = {
    TEST_FIELD(maniGraphTransitionSource, from),
    TEST_FIELD(maniGraphTransitionSource, to),
    TEST_FIELD(maniGraphTransitionSource, priority),
    TEST_FIELD(maniGraphTransitionSource, interruption),
    TEST_FIELD(maniGraphTransitionSource, entry),
    TEST_FIELD(maniGraphTransitionSource, fade),
    TEST_FIELD(maniGraphTransitionSource, duration),
    TEST_FIELD(maniGraphTransitionSource, cooldown),
    TEST_FIELD(maniGraphTransitionSource, conditions),
    TEST_FIELD(maniGraphTransitionSource, conditionCount),
};

static const TestField kFieldsGraphTrianglesNodeSource[] = {
    TEST_FIELD(maniGraphTrianglesNodeSource, position.vector),
    TEST_FIELD(maniGraphTrianglesNodeSource, position.x.parameter),
    TEST_FIELD(maniGraphTrianglesNodeSource, position.x.value),
    TEST_FIELD(maniGraphTrianglesNodeSource, position.y.parameter),
    TEST_FIELD(maniGraphTrianglesNodeSource, position.y.value),
    TEST_FIELD(maniGraphTrianglesNodeSource, points),
    TEST_FIELD(maniGraphTrianglesNodeSource, triangles),
    TEST_FIELD(maniGraphTrianglesNodeSource, triangleCount),
};

static const TestField kFieldsGraphTurnLimitNodeSource[] = {
    TEST_FIELD(maniGraphTurnLimitNodeSource, speed.parameter),
    TEST_FIELD(maniGraphTurnLimitNodeSource, speed.value),
    TEST_FIELD(maniGraphTurnLimitNodeSource, reset),
    TEST_FIELD(maniGraphTurnLimitNodeSource, mask),
};

static const TestField kFieldsGraphTwoBoneNodeSource[] = {
    TEST_FIELD(maniGraphTwoBoneNodeSource, start),
    TEST_FIELD(maniGraphTwoBoneNodeSource, middle),
    TEST_FIELD(maniGraphTwoBoneNodeSource, end),
    TEST_FIELD(maniGraphTwoBoneNodeSource, goal),
    TEST_FIELD(maniGraphTwoBoneNodeSource, pole),
    TEST_FIELD(maniGraphTwoBoneNodeSource, hinge.x),
    TEST_FIELD(maniGraphTwoBoneNodeSource, hinge.y),
    TEST_FIELD(maniGraphTwoBoneNodeSource, hinge.z),
    TEST_FIELD(maniGraphTwoBoneNodeSource, twist.parameter),
    TEST_FIELD(maniGraphTwoBoneNodeSource, twist.value),
    TEST_FIELD(maniGraphTwoBoneNodeSource, soften.parameter),
    TEST_FIELD(maniGraphTwoBoneNodeSource, soften.value),
    TEST_FIELD(maniGraphTwoBoneNodeSource, weight.parameter),
    TEST_FIELD(maniGraphTwoBoneNodeSource, weight.value),
};

static const TestField kFieldsImportClipOptions[] = {
    TEST_FIELD(maniImportClipOptions, sampleRate),
    TEST_FIELD(maniImportClipOptions, loop),
    TEST_FIELD(maniImportClipOptions, additive),
    TEST_FIELD(maniImportClipOptions, rootMotion),
};

static const TestField kFieldsInfluenceSource[] = {
    TEST_FIELD(maniInfluenceSource, skin),
    TEST_FIELD(maniInfluenceSource, offsets),
    TEST_FIELD(maniInfluenceSource, joints),
    TEST_FIELD(maniInfluenceSource, weights),
    TEST_FIELD(maniInfluenceSource, dualQuaternionWeights),
    TEST_FIELD(maniInfluenceSource, vertexCount),
};

static const TestField kFieldsJointLimit[] = {
    TEST_FIELD(maniJointLimit, kind),      TEST_FIELD(maniJointLimit, axis.x),
    TEST_FIELD(maniJointLimit, axis.y),    TEST_FIELD(maniJointLimit, axis.z),
    TEST_FIELD(maniJointLimit, cone),      TEST_FIELD(maniJointLimit, low),
    TEST_FIELD(maniJointLimit, high),      TEST_FIELD(maniJointLimit, twistLow),
    TEST_FIELD(maniJointLimit, twistHigh),
};

static const TestField kFieldsLearnedLayerSource[] = {
    TEST_FIELD(maniLearnedLayerSource, weights),
    TEST_FIELD(maniLearnedLayerSource, biases),
    TEST_FIELD(maniLearnedLayerSource, inputCount),
    TEST_FIELD(maniLearnedLayerSource, outputCount),
};

static const TestField kFieldsLearnedModelSource[] = {
    TEST_FIELD(maniLearnedModelSource, skeleton),
    TEST_FIELD(maniLearnedModelSource, database),
    TEST_FIELD(maniLearnedModelSource, decompressor),
    TEST_FIELD(maniLearnedModelSource, stepper),
    TEST_FIELD(maniLearnedModelSource, projectors),
    TEST_FIELD(maniLearnedModelSource, scaledBones),
    TEST_FIELD(maniLearnedModelSource, scales),
    TEST_FIELD(maniLearnedModelSource, floatTracks),
    TEST_FIELD(maniLearnedModelSource, events),
    TEST_FIELD(maniLearnedModelSource, projectorCount),
    TEST_FIELD(maniLearnedModelSource, scaledBoneCount),
    TEST_FIELD(maniLearnedModelSource, floatTrackCount),
    TEST_FIELD(maniLearnedModelSource, eventCount),
    TEST_FIELD(maniLearnedModelSource, latentCount),
    TEST_FIELD(maniLearnedModelSource, stepRate),
    TEST_FIELD(maniLearnedModelSource, weightFormat),
};

static const TestField kFieldsLearnedNameSource[] = {
    TEST_FIELD(maniLearnedNameSource, name),
    TEST_FIELD(maniLearnedNameSource, length),
};

static const TestField kFieldsLearnedNetworkSource[] = {
    TEST_FIELD(maniLearnedNetworkSource, layers),
    TEST_FIELD(maniLearnedNetworkSource, inputOffsets),
    TEST_FIELD(maniLearnedNetworkSource, inputScales),
    TEST_FIELD(maniLearnedNetworkSource, outputOffsets),
    TEST_FIELD(maniLearnedNetworkSource, outputScales),
    TEST_FIELD(maniLearnedNetworkSource, layerCount),
};

static const TestField kFieldsLearnedProjectorSource[] = {
    TEST_FIELD(maniLearnedProjectorSource, network.layers),
    TEST_FIELD(maniLearnedProjectorSource, network.inputOffsets),
    TEST_FIELD(maniLearnedProjectorSource, network.inputScales),
    TEST_FIELD(maniLearnedProjectorSource, network.outputOffsets),
    TEST_FIELD(maniLearnedProjectorSource, network.outputScales),
    TEST_FIELD(maniLearnedProjectorSource, network.layerCount),
    TEST_FIELD(maniLearnedProjectorSource, requiredTags),
    TEST_FIELD(maniLearnedProjectorSource, excludedTags),
};

static const TestField kFieldsLearnedTagFilter[] = {
    TEST_FIELD(maniLearnedTagFilter, required),
    TEST_FIELD(maniLearnedTagFilter, excluded),
};

static const TestField kFieldsLearnedTrainerOptions[] = {
    TEST_FIELD(maniLearnedTrainerOptions, latentCount),
    TEST_FIELD(maniLearnedTrainerOptions, hiddenWidth),
    TEST_FIELD(maniLearnedTrainerOptions, compressorLayers),
    TEST_FIELD(maniLearnedTrainerOptions, decompressorLayers),
    TEST_FIELD(maniLearnedTrainerOptions, stepperLayers),
    TEST_FIELD(maniLearnedTrainerOptions, projectorLayers),
    TEST_FIELD(maniLearnedTrainerOptions, batchSize),
    TEST_FIELD(maniLearnedTrainerOptions, stepperWindow),
    TEST_FIELD(maniLearnedTrainerOptions, iterations),
    TEST_FIELD(maniLearnedTrainerOptions, seed),
    TEST_FIELD(maniLearnedTrainerOptions, learningRate),
    TEST_FIELD(maniLearnedTrainerOptions, eventSeconds),
};

static const TestField kFieldsLearnedTrainerSource[] = {
    TEST_FIELD(maniLearnedTrainerSource, skeleton),
    TEST_FIELD(maniLearnedTrainerSource, database),
    TEST_FIELD(maniLearnedTrainerSource, clips),
    TEST_FIELD(maniLearnedTrainerSource, projectors),
    TEST_FIELD(maniLearnedTrainerSource, floatTracks),
    TEST_FIELD(maniLearnedTrainerSource, events),
    TEST_FIELD(maniLearnedTrainerSource, clipCount),
    TEST_FIELD(maniLearnedTrainerSource, projectorCount),
    TEST_FIELD(maniLearnedTrainerSource, floatTrackCount),
    TEST_FIELD(maniLearnedTrainerSource, eventCount),
    TEST_FIELD(maniLearnedTrainerSource, options.latentCount),
    TEST_FIELD(maniLearnedTrainerSource, options.hiddenWidth),
    TEST_FIELD(maniLearnedTrainerSource, options.compressorLayers),
    TEST_FIELD(maniLearnedTrainerSource, options.decompressorLayers),
    TEST_FIELD(maniLearnedTrainerSource, options.stepperLayers),
    TEST_FIELD(maniLearnedTrainerSource, options.projectorLayers),
    TEST_FIELD(maniLearnedTrainerSource, options.batchSize),
    TEST_FIELD(maniLearnedTrainerSource, options.stepperWindow),
    TEST_FIELD(maniLearnedTrainerSource, options.iterations),
    TEST_FIELD(maniLearnedTrainerSource, options.seed),
    TEST_FIELD(maniLearnedTrainerSource, options.learningRate),
    TEST_FIELD(maniLearnedTrainerSource, options.eventSeconds),
};

static const TestField kFieldsMaskChain[] = {
    TEST_FIELD(maniMaskChain, bone),
    TEST_FIELD(maniMaskChain, weight),
    TEST_FIELD(maniMaskChain, subtree),
};

static const TestField kFieldsMatchChannelSource[] = {
    TEST_FIELD(maniMatchChannelSource, kind),        TEST_FIELD(maniMatchChannelSource, bone),
    TEST_FIELD(maniMatchChannelSource, axis),        TEST_FIELD(maniMatchChannelSource, timeCount),
    TEST_FIELD(maniMatchChannelSource, times),       TEST_FIELD(maniMatchChannelSource, track),
    TEST_FIELD(maniMatchChannelSource, trackLength), TEST_FIELD(maniMatchChannelSource, weight),
};

static const TestField kFieldsMatchEntrySource[] = {
    TEST_FIELD(maniMatchEntrySource, clip),     TEST_FIELD(maniMatchEntrySource, flags),
    TEST_FIELD(maniMatchEntrySource, start),    TEST_FIELD(maniMatchEntrySource, end),
    TEST_FIELD(maniMatchEntrySource, costBias), TEST_FIELD(maniMatchEntrySource, tags),
};

static const TestField kFieldsMirrorClipSource[] = {
    TEST_FIELD(maniMirrorClipSource, map),
    TEST_FIELD(maniMirrorClipSource, skeleton),
    TEST_FIELD(maniMirrorClipSource, clip),
};

static const TestField kFieldsMirrorMapSource[] = {
    TEST_FIELD(maniMirrorMapSource, skeleton),      TEST_FIELD(maniMirrorMapSource, reference),
    TEST_FIELD(maniMirrorMapSource, pairs),         TEST_FIELD(maniMirrorMapSource, rules),
    TEST_FIELD(maniMirrorMapSource, namePairs),     TEST_FIELD(maniMirrorMapSource, names),
    TEST_FIELD(maniMirrorMapSource, pairCount),     TEST_FIELD(maniMirrorMapSource, ruleCount),
    TEST_FIELD(maniMirrorMapSource, namePairCount), TEST_FIELD(maniMirrorMapSource, nameCount),
    TEST_FIELD(maniMirrorMapSource, axis),
};

static const TestField kFieldsMirrorNamePairSource[] = {
    TEST_FIELD(maniMirrorNamePairSource, first),
    TEST_FIELD(maniMirrorNamePairSource, firstLength),
    TEST_FIELD(maniMirrorNamePairSource, secondLength),
    TEST_FIELD(maniMirrorNamePairSource, second),
};

static const TestField kFieldsMirrorNameSource[] = {
    TEST_FIELD(maniMirrorNameSource, name),
    TEST_FIELD(maniMirrorNameSource, nameLength),
};

static const TestField kFieldsMirrorPairSource[] = {
    TEST_FIELD(maniMirrorPairSource, first),
    TEST_FIELD(maniMirrorPairSource, second),
};

static const TestField kFieldsMirrorRuleSource[] = {
    TEST_FIELD(maniMirrorRuleSource, find),          TEST_FIELD(maniMirrorRuleSource, findLength),
    TEST_FIELD(maniMirrorRuleSource, place),         TEST_FIELD(maniMirrorRuleSource, replace),
    TEST_FIELD(maniMirrorRuleSource, replaceLength),
};

static const TestField kFieldsMorphAttributeSource[] = {
    TEST_FIELD(maniMorphAttributeSource, semantic),
    TEST_FIELD(maniMorphAttributeSource, set),
};

static const TestField kFieldsMorphChannelSource[] = {
    TEST_FIELD(maniMorphChannelSource, name),
    TEST_FIELD(maniMorphChannelSource, nameLength),
    TEST_FIELD(maniMorphChannelSource, targetCount),
    TEST_FIELD(maniMorphChannelSource, targets),
};

static const TestField kFieldsMorphSource[] = {
    TEST_FIELD(maniMorphSource, channels),       TEST_FIELD(maniMorphSource, channelCount),
    TEST_FIELD(maniMorphSource, vertexCount),    TEST_FIELD(maniMorphSource, attributes),
    TEST_FIELD(maniMorphSource, attributeCount),
};

static const TestField kFieldsMorphTargetSource[] = {
    TEST_FIELD(maniMorphTargetSource, fullWeight), TEST_FIELD(maniMorphTargetSource, count),
    TEST_FIELD(maniMorphTargetSource, indices),    TEST_FIELD(maniMorphTargetSource, deltas),
    TEST_FIELD(maniMorphTargetSource, name),       TEST_FIELD(maniMorphTargetSource, nameLength),
};

static const TestField kFieldsMotionDatabaseSource[] = {
    TEST_FIELD(maniMotionDatabaseSource, skeleton),
    TEST_FIELD(maniMotionDatabaseSource, clips),
    TEST_FIELD(maniMotionDatabaseSource, channels),
    TEST_FIELD(maniMotionDatabaseSource, entries),
    TEST_FIELD(maniMotionDatabaseSource, clipCount),
    TEST_FIELD(maniMotionDatabaseSource, channelCount),
    TEST_FIELD(maniMotionDatabaseSource, entryCount),
    TEST_FIELD(maniMotionDatabaseSource, sampleRate),
    TEST_FIELD(maniMotionDatabaseSource, up),
    TEST_FIELD(maniMotionDatabaseSource, forward),
    TEST_FIELD(maniMotionDatabaseSource, mirrorMap),
    TEST_FIELD(maniMotionDatabaseSource, leafFrames),
};

static const TestField kFieldsRagdollBodySource[] = {
    TEST_FIELD(maniRagdollBodySource, name),
    TEST_FIELD(maniRagdollBodySource, nameLength),
    TEST_FIELD(maniRagdollBodySource, bone),
    TEST_FIELD(maniRagdollBodySource, frameTranslation.x),
    TEST_FIELD(maniRagdollBodySource, frameTranslation.y),
    TEST_FIELD(maniRagdollBodySource, frameTranslation.z),
    TEST_FIELD(maniRagdollBodySource, frameRotation.x),
    TEST_FIELD(maniRagdollBodySource, frameRotation.y),
    TEST_FIELD(maniRagdollBodySource, frameRotation.z),
    TEST_FIELD(maniRagdollBodySource, frameRotation.w),
    TEST_FIELD(maniRagdollBodySource, shape),
    TEST_FIELD(maniRagdollBodySource, size.x),
    TEST_FIELD(maniRagdollBodySource, size.y),
    TEST_FIELD(maniRagdollBodySource, size.z),
    TEST_FIELD(maniRagdollBodySource, mass),
    TEST_FIELD(maniRagdollBodySource, joint.translation.x),
    TEST_FIELD(maniRagdollBodySource, joint.translation.y),
    TEST_FIELD(maniRagdollBodySource, joint.translation.z),
    TEST_FIELD(maniRagdollBodySource, joint.rotation.x),
    TEST_FIELD(maniRagdollBodySource, joint.rotation.y),
    TEST_FIELD(maniRagdollBodySource, joint.rotation.z),
    TEST_FIELD(maniRagdollBodySource, joint.rotation.w),
    TEST_FIELD(maniRagdollBodySource, joint.twistMin),
    TEST_FIELD(maniRagdollBodySource, joint.twistMax),
    TEST_FIELD(maniRagdollBodySource, joint.swingY),
    TEST_FIELD(maniRagdollBodySource, joint.swingZ),
    TEST_FIELD(maniRagdollBodySource, joint.driveStiffness),
    TEST_FIELD(maniRagdollBodySource, joint.driveDamping),
    TEST_FIELD(maniRagdollBodySource, joint.driveMaxTorque),
};

static const TestField kFieldsRagdollJointSource[] = {
    TEST_FIELD(maniRagdollJointSource, translation.x),
    TEST_FIELD(maniRagdollJointSource, translation.y),
    TEST_FIELD(maniRagdollJointSource, translation.z),
    TEST_FIELD(maniRagdollJointSource, rotation.x),
    TEST_FIELD(maniRagdollJointSource, rotation.y),
    TEST_FIELD(maniRagdollJointSource, rotation.z),
    TEST_FIELD(maniRagdollJointSource, rotation.w),
    TEST_FIELD(maniRagdollJointSource, twistMin),
    TEST_FIELD(maniRagdollJointSource, twistMax),
    TEST_FIELD(maniRagdollJointSource, swingY),
    TEST_FIELD(maniRagdollJointSource, swingZ),
    TEST_FIELD(maniRagdollJointSource, driveStiffness),
    TEST_FIELD(maniRagdollJointSource, driveDamping),
    TEST_FIELD(maniRagdollJointSource, driveMaxTorque),
};

static const TestField kFieldsRagdollSource[] = {
    TEST_FIELD(maniRagdollSource, skeleton),  TEST_FIELD(maniRagdollSource, bodies),
    TEST_FIELD(maniRagdollSource, bodyCount), TEST_FIELD(maniRagdollSource, facing.x),
    TEST_FIELD(maniRagdollSource, facing.y),  TEST_FIELD(maniRagdollSource, facing.z),
    TEST_FIELD(maniRagdollSource, up.x),      TEST_FIELD(maniRagdollSource, up.y),
    TEST_FIELD(maniRagdollSource, up.z),
};

static const TestField kFieldsRetargetChainSource[] = {
    TEST_FIELD(maniRetargetChainSource, sourceStart),
    TEST_FIELD(maniRetargetChainSource, sourceEnd),
    TEST_FIELD(maniRetargetChainSource, targetStart),
    TEST_FIELD(maniRetargetChainSource, targetEnd),
    TEST_FIELD(maniRetargetChainSource, rotation),
    TEST_FIELD(maniRetargetChainSource, rotationWeight),
    TEST_FIELD(maniRetargetChainSource, translation),
    TEST_FIELD(maniRetargetChainSource, scale),
};

static const TestField kFieldsRetargetClipSource[] = {
    TEST_FIELD(maniRetargetClipSource, map),
    TEST_FIELD(maniRetargetClipSource, source),
    TEST_FIELD(maniRetargetClipSource, target),
    TEST_FIELD(maniRetargetClipSource, clip),
    TEST_FIELD(maniRetargetClipSource, rootMotionBone),
};

static const TestField kFieldsRetargetMapSource[] = {
    TEST_FIELD(maniRetargetMapSource, source),
    TEST_FIELD(maniRetargetMapSource, target),
    TEST_FIELD(maniRetargetMapSource, sourceReference),
    TEST_FIELD(maniRetargetMapSource, targetReference),
    TEST_FIELD(maniRetargetMapSource, pairs),
    TEST_FIELD(maniRetargetMapSource, chains),
    TEST_FIELD(maniRetargetMapSource, pelvis),
    TEST_FIELD(maniRetargetMapSource, pairCount),
    TEST_FIELD(maniRetargetMapSource, chainCount),
    TEST_FIELD(maniRetargetMapSource, matchNames),
};

static const TestField kFieldsRetargetPairSource[] = {
    TEST_FIELD(maniRetargetPairSource, target),
    TEST_FIELD(maniRetargetPairSource, source),
    TEST_FIELD(maniRetargetPairSource, rotationWeight),
    TEST_FIELD(maniRetargetPairSource, translation),
    TEST_FIELD(maniRetargetPairSource, scale),
};

static const TestField kFieldsRetargetPelvisSource[] = {
    TEST_FIELD(maniRetargetPelvisSource, source),
    TEST_FIELD(maniRetargetPelvisSource, target),
    TEST_FIELD(maniRetargetPelvisSource, up.x),
    TEST_FIELD(maniRetargetPelvisSource, up.y),
    TEST_FIELD(maniRetargetPelvisSource, up.z),
    TEST_FIELD(maniRetargetPelvisSource, horizontal),
    TEST_FIELD(maniRetargetPelvisSource, vertical),
    TEST_FIELD(maniRetargetPelvisSource, absolute.x),
    TEST_FIELD(maniRetargetPelvisSource, absolute.y),
    TEST_FIELD(maniRetargetPelvisSource, absolute.z),
    TEST_FIELD(maniRetargetPelvisSource, toSource.x),
    TEST_FIELD(maniRetargetPelvisSource, toSource.y),
    TEST_FIELD(maniRetargetPelvisSource, toSource.z),
    TEST_FIELD(maniRetargetPelvisSource, translationWeight),
    TEST_FIELD(maniRetargetPelvisSource, rotationWeight),
};

static const TestField kFieldsRootMotionSource[] = {
    TEST_FIELD(maniRootMotionSource, bone),
    TEST_FIELD(maniRootMotionSource, translationAxes),
    TEST_FIELD(maniRootMotionSource, rotation),
    TEST_FIELD(maniRootMotionSource, twistAxis),
    TEST_FIELD(maniRootMotionSource, translationReference),
    TEST_FIELD(maniRootMotionSource, rotationReference),
    TEST_FIELD(maniRootMotionSource, loop),
};

static const TestField kFieldsSkeletonSource[] = {
    TEST_FIELD(maniSkeletonSource, bones),
    TEST_FIELD(maniSkeletonSource, boneCount),
};

static const TestField kFieldsSkinSource[] = {
    TEST_FIELD(maniSkinSource, skeleton),
    TEST_FIELD(maniSkinSource, joints),
    TEST_FIELD(maniSkinSource, inverseBinds),
    TEST_FIELD(maniSkinSource, jointCount),
};

static const TestField kFieldsSpringJointSource[] = {
    TEST_FIELD(maniSpringJointSource, bone),
    TEST_FIELD(maniSpringJointSource, hitRadius),
    TEST_FIELD(maniSpringJointSource, stiffness),
    TEST_FIELD(maniSpringJointSource, gravityPower),
    TEST_FIELD(maniSpringJointSource, gravityDirection.x),
    TEST_FIELD(maniSpringJointSource, gravityDirection.y),
    TEST_FIELD(maniSpringJointSource, gravityDirection.z),
    TEST_FIELD(maniSpringJointSource, drag),
    TEST_FIELD(maniSpringJointSource, limit),
    TEST_FIELD(maniSpringJointSource, limitAngle),
    TEST_FIELD(maniSpringJointSource, limitYaw),
    TEST_FIELD(maniSpringJointSource, limitRotation.x),
    TEST_FIELD(maniSpringJointSource, limitRotation.y),
    TEST_FIELD(maniSpringJointSource, limitRotation.z),
    TEST_FIELD(maniSpringJointSource, limitRotation.w),
};

static const TestField kFieldsSpringSetSource[] = {
    TEST_FIELD(maniSpringSetSource, skeleton),
    TEST_FIELD(maniSpringSetSource, colliders),
    TEST_FIELD(maniSpringSetSource, colliderGroups),
    TEST_FIELD(maniSpringSetSource, springs),
    TEST_FIELD(maniSpringSetSource, colliderCount),
    TEST_FIELD(maniSpringSetSource, colliderGroupCount),
    TEST_FIELD(maniSpringSetSource, springCount),
    TEST_FIELD(maniSpringSetSource, referenceRate),
    TEST_FIELD(maniSpringSetSource, stepRate),
};

static const TestField kFieldsSpringSource[] = {
    TEST_FIELD(maniSpringSource, name),           TEST_FIELD(maniSpringSource, nameLength),
    TEST_FIELD(maniSpringSource, jointCount),     TEST_FIELD(maniSpringSource, joints),
    TEST_FIELD(maniSpringSource, colliderGroups), TEST_FIELD(maniSpringSource, colliderGroupCount),
    TEST_FIELD(maniSpringSource, center),
};

#endif // MAUL_ANIM_TEST_SOURCE_PADDING_H
