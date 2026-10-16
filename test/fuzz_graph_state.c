// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restoring a graph instance's saved state (mani-0023) from any bytes,
// into an instance of a small graph made once: parameters of every type
// and a select over a clip and a time scale of it. The bytes are tried as
// they are, then with the header made right so they reach the checks
// behind it, then with the quaternion parameter made a unit one; a
// restored state saves back to the same bytes. Run as
// fuzz_graph_state <corpus dir>.

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static _Alignas(16) uint8_t s_skeletonFile[1024];
static _Alignas(16) uint8_t s_clipFile[4096];
static _Alignas(16) uint8_t s_graphFile[4096];
static _Alignas(8) uint8_t s_binding[512];
static _Alignas(8) uint8_t s_instance[2048];
static uint8_t s_saved[1024];
static maniSkeleton s_skeleton;
static maniClip s_clip;
static maniGraph s_graph;
static maniGraphInstance* s_live;
static size_t s_stateSize;

static bool Built(maniBuilder* builder, uint8_t* out, size_t capacity, size_t* size)
{
    return maniCopyBuilt(builder, out, capacity, size) == mani_success;
}

static bool MakeClip(maniBuilder* builder)
{
    static const maniBoneSource kBone = {
        "root", 4, -1, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kValues[6] = {0, 0, 0, 1, 0, 0};
    maniSkeletonSource skeleton = {&kBone, 1, 0};
    size_t size = 0;
    if (maniBuildSkeleton(builder, &skeleton).result != mani_success ||
        !Built(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ||
        maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result != mani_success)
    {
        return false;
    }
    const maniChannelSource none = {NULL, NULL, 0, maniInterpolationLinear};
    const maniBoneTrackSource track = {
        0, 0, {kTimes, kValues, 2, maniInterpolationLinear}, none, none};
    maniClipSource clip = {.skeleton = &s_skeleton, .boneTracks = &track, .boneTrackCount = 1};
    return maniBuildClip(builder, &clip).result == mani_success &&
           Built(builder, s_clipFile, sizeof s_clipFile, &size) &&
           maniOpenClip(s_clipFile, size, &s_clip).result == mani_success;
}

static maniGraphParameterSource Parameter(const char* name, maniGraphParameterType type)
{
    maniGraphParameterSource p = {name, (uint32_t)strlen(name), type, {{0}}, {{0}}, {{0}}, 0, 0};
    p.defaultValue.f[3] = type == maniGraphParameterQuat ? 1.0f : 0.0f;
    p.bounded = type == maniGraphParameterInt || type == maniGraphParameterFloat ? 1u : 0u;
    p.maximum.i = type == maniGraphParameterInt ? 3 : 0;
    p.maximum.f[0] = type == maniGraphParameterFloat ? 2.0f : p.maximum.f[0];
    return p;
}

static bool MakeGraph(maniBuilder* builder)
{
    const maniGraphParameterSource parameters[7] = {
        Parameter("b", maniGraphParameterBool),   Parameter("i", maniGraphParameterInt),
        Parameter("f", maniGraphParameterFloat),  Parameter("v2", maniGraphParameterVec2),
        Parameter("v3", maniGraphParameterVec3),  Parameter("q", maniGraphParameterQuat),
        Parameter("t", maniGraphParameterTrigger)};
    const maniGraphClipSource clip = {&s_clip, "c", 1, 0};
    static const uint32_t kClip[1] = {0};
    static const uint32_t kChoices[2] = {0, 1};
    const maniGraphNodeSource nodes[3] = {
        {maniGraphNodeClip, 0, NULL,
         .clip = {0, MANI_NO_SYNC_GROUP, {2, 0.0f}, maniGraphLoopClip, 0}},
        {maniGraphNodeTimeScale, 1, kClip, .timeScale = {{MANI_NO_PARAMETER, 2.0f}}},
        {maniGraphNodeSelect, 2, kChoices, .select = {1}},
    };
    maniGraphSource source = {&s_skeleton, parameters, &clip, NULL, nodes, 7, 1, 0, 3, 2, 0};
    size_t size = 0;
    return maniBuildGraph(builder, &source).result == mani_success &&
           Built(builder, s_graphFile, sizeof s_graphFile, &size) &&
           maniOpenGraph(s_graphFile, size, &s_graph).result == mani_success;
}

static void Setup(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    const maniClip* clips[1] = {&s_clip};
    maniGraphBinding* binding = NULL;
    bool ok =
        maniCreateBuilder(&def, &builder) == mani_success && MakeClip(builder) &&
        MakeGraph(builder) &&
        maniBindGraph(&s_graph, &s_skeleton, clips, 1, s_binding, sizeof s_binding, &binding,
                      NULL) == mani_success &&
        maniInitGraphInstance(binding, s_instance, sizeof s_instance, &s_live) == mani_success;
    maniDestroyBuilder(builder);
    s_stateSize = ok ? maniGraphStateSize(binding) : 0;
    if (!ok || s_stateSize > sizeof s_saved)
    {
        abort();
    }
}

// Restores the bytes; a restored state saves back to them.
static void Exercise(const uint8_t* bytes, size_t size)
{
    if (maniRestoreGraphInstance(s_live, bytes, size).result != mani_success)
    {
        return;
    }
    if (maniSaveGraphInstance(s_live, s_saved, sizeof s_saved) != mani_success ||
        memcmp(s_saved, bytes, s_stateSize) != 0)
    {
        abort();
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_live == NULL)
    {
        Setup();
    }
    uint8_t* copy = malloc(size > s_stateSize ? size : s_stateSize);
    if (copy == NULL)
    {
        return 0;
    }
    memset(copy, 0, size > s_stateSize ? size : s_stateSize);
    if (size > 0)
    {
        memcpy(copy, data, size);
    }
    Exercise(copy, size);
    // The header made right: magic, zero, the graph's identity, the size.
    const uint32_t magic = 0x5453474Du;
    const uint32_t zero = 0;
    const uint64_t identity = maniGraphIdentity(&s_graph);
    const uint64_t stateSize = s_stateSize;
    memcpy(copy, &magic, 4);
    memcpy(copy + 4, &zero, 4);
    memcpy(copy + 8, &identity, 8);
    memcpy(copy + 16, &stateSize, 8);
    Exercise(copy, s_stateSize);
    // And the quaternion made the identity, which random bytes seldom are,
    // so the rest reaches a whole restore.
    const float identityQuat[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    memcpy(copy + 24 + 5 * 16, identityQuat, sizeof identityQuat);
    Exercise(copy, s_stateSize);
    free(copy);
    return 0;
}
