// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restoring a graph instance's saved state (mani-0023) from any bytes,
// into an instance of a small graph made once: parameters of every type,
// a select over a clip and a time scale of it, blended with an
// inertialize node over a motion matching node (mani-0024) searching a
// database of the clip (a looping and a clamped entry, tags from the int
// parameter) and another over a learned node (mani-0025) playing a
// model of the database; the select under a spring node (mani-0026) of
// a spring from the root to its child, placed by the vec3 and quat
// parameters and reset by the trigger; the pose parameter is saved
// after the machines' blocks. The bytes are tried as they are,
// then with the
// header made right so they reach the checks behind it, then with the
// quaternion parameter made a unit one, then with the inertialize
// nodes' blocks as a fresh instance has them, so the searching nodes'
// reach their checks; a restored state saves back to the same bytes,
// updates and evaluates. Run as fuzz_graph_state <corpus dir>.

#include "bytes.h"
#include "fuzz_model.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_instance.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static _Alignas(16) uint8_t s_skeletonFile[1024];
static _Alignas(16) uint8_t s_clipFile[4096];
static _Alignas(16) uint8_t s_graphFile[4096];
static _Alignas(16) uint8_t s_databaseFile[8192];
static _Alignas(8) uint8_t s_binding[512];
static _Alignas(8) uint8_t s_instance[4096];
static uint8_t s_saved[4096];
static uint8_t s_fresh[4096];
static _Alignas(16) uint8_t s_modelFile[8192];
static _Alignas(16) uint8_t s_setFile[1024];
static maniSpringSet s_set;
static maniMotionDatabase s_database;
static maniLearnedModel s_model;
// The inertialize nodes' blocks in a state: their offsets and size.
static size_t s_eased[2];
static size_t s_easedSize;
static _Alignas(32) uint8_t s_scratch[16384];
static _Alignas(32) uint8_t s_poseMemory[1024];
static maniPose* s_pose;
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
    static const maniBoneSource kBones[2] = {
        {"root", 4, -1, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"tip", 3, 0, {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}}};
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kValues[6] = {0, 0, 0, 1, 0, 0};
    maniSkeletonSource skeleton = {kBones, 2, 0};
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

// The clip's frames: looping (tags 1) and clamped (tags 2), the root's
// position now their channel.
static bool MakeDatabase(maniBuilder* builder)
{
    static const float kNow[1] = {0.0f};
    const maniClip* clips[1] = {&s_clip};
    const maniMatchChannelSource channel = {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f};
    const maniMatchEntrySource entries[2] = {{0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u},
                                             {0, 0, 0.0f, 1.0f, 0.5f, 2u}};
    maniMotionDatabaseSource source = {&s_skeleton, clips, &channel,       entries,       1, 1,
                                       2,           10.0f, maniMatchAxisY, maniMatchAxisZ};
    size_t size = 0;
    return maniBuildMotionDatabase(builder, &source).result == mani_success &&
           Built(builder, s_databaseFile, sizeof s_databaseFile, &size) &&
           maniOpenMotionDatabase(s_databaseFile, size, &s_database).result == mani_success;
}

// A spring from the root to its tip.
static bool MakeSet(maniBuilder* builder)
{
    maniSpringJointSource joints[2];
    for (uint32_t j = 0; j < 2; ++j)
    {
        joints[j] = (maniSpringJointSource){.bone = j,
                                            .stiffness = 0.5f,
                                            .gravityPower = 1.0f,
                                            .gravityDirection = {0.0f, -1.0f, 0.0f},
                                            .drag = 0.3f,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    maniSpringSource spring = {"s", 1, 2, joints, NULL, 0, MANI_NO_BONE};
    maniSpringSetSource source = {&s_skeleton, NULL, NULL, &spring, 0, 0, 1, 0.0f, 0.0f, 0};
    size_t size = 0;
    return maniBuildSpringSet(builder, &source).result == mani_success &&
           Built(builder, s_setFile, sizeof s_setFile, &size) &&
           maniOpenSpringSet(s_setFile, size, &s_set).result == mani_success;
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
    const maniGraphParameterSource parameters[8] = {
        Parameter("b", maniGraphParameterBool),    Parameter("i", maniGraphParameterInt),
        Parameter("f", maniGraphParameterFloat),   Parameter("v2", maniGraphParameterVec2),
        Parameter("v3", maniGraphParameterVec3),   Parameter("q", maniGraphParameterQuat),
        Parameter("t", maniGraphParameterTrigger), Parameter("p", maniGraphParameterPose)};
    const maniGraphClipSource clip = {&s_clip, "c", 1, 0, NULL};
    const maniGraphDatabaseSource database = {&s_database, "d", 1, 0};
    const maniGraphModelSource model = {&s_model, "m", 1, 0};
    static const uint32_t kClip[1] = {0};
    static const uint32_t kChoices[2] = {0, 1};
    static const uint32_t kMatch[1] = {3};
    static const uint32_t kLearned[1] = {6};
    static const uint32_t kSelect[1] = {2};
    static const uint32_t kBlend[3] = {8, 4, 7};
    const maniGraphSpringSetSource set = {&s_set, "s", 1, 0};
    static const maniGraphScalar kThirds[3] = {
        {MANI_NO_PARAMETER, 1.0f}, {MANI_NO_PARAMETER, 1.0f}, {MANI_NO_PARAMETER, 1.0f}};
    const maniGraphMatchNodeSource match = {.database = 0,
                                            .trigger = 6,
                                            .historyCount = 3,
                                            .period = 0.2f,
                                            .jumpThreshold = 0.1f,
                                            .endExclusion = 0.1f,
                                            .historyTime = 0.3f,
                                            .requiredTags = {1, 0},
                                            .excludedTags = {MANI_NO_PARAMETER, 0},
                                            .duration = {2, 0.0f}};
    const maniGraphLearnedNodeSource learned = {.trigger = 6,
                                                .period = 0.3f,
                                                .transitionCost = 0.1f,
                                                .requiredTags = {MANI_NO_PARAMETER, 0},
                                                .excludedTags = {MANI_NO_PARAMETER, 0},
                                                .duration = {2, 0.0f}};
    const maniGraphNodeSource nodes[9] = {
        {maniGraphNodeClip, 0, NULL,
         .clip = {0, MANI_NO_SYNC_GROUP, {2, 0.0f}, maniGraphLoopClip, 0}},
        {maniGraphNodeTimeScale, 1, kClip, .timeScale = {{MANI_NO_PARAMETER, 2.0f}}},
        {maniGraphNodeSelect, 2, kChoices, .select = {1}},
        {maniGraphNodeMatch, 0, NULL, .match = match},
        {maniGraphNodeInertialize, 1, kMatch,
         .inertialize = {MANI_NO_PARAMETER, maniGraphInertialization, {2, 0.0f}, 0, 0, 0}},
        {maniGraphNodeBlend, 3, kBlend, .blend = {kThirds, 0, 0}},
        {maniGraphNodeLearned, 0, NULL, .learned = learned},
        {maniGraphNodeInertialize, 1, kLearned,
         .inertialize = {MANI_NO_PARAMETER, maniGraphInertialization, {2, 0.0f}, 0, 0, 0}},
        {maniGraphNodeSpring, 1, kSelect, .spring = {0, 4, 5, 6, 4}},
    };
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = parameters,
                              .clips = &clip,
                              .nodes = nodes,
                              .parameterCount = 8,
                              .clipCount = 1,
                              .nodeCount = 9,
                              .output = 5,
                              .databaseCount = 1,
                              .databases = &database,
                              .modelCount = 1,
                              .models = &model,
                              .springSetCount = 1,
                              .springSets = &set};
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
    const maniMotionDatabase* databases[1] = {&s_database};
    const maniLearnedModel* models[1] = {&s_model};
    const maniSpringSet* sets[1] = {&s_set};
    maniGraphBinding* binding = NULL;
    bool ok =
        maniCreateBuilder(&def, &builder) == mani_success && MakeClip(builder) &&
        MakeDatabase(builder) &&
        MakeFuzzModel(builder, &s_skeleton, &s_database, s_modelFile, sizeof s_modelFile,
                      &s_model) &&
        MakeSet(builder) && MakeGraph(builder) &&
        maniBindGraph(&s_graph,
                      &(maniGraphAssets){&s_skeleton, clips, databases, models, sets, 1, 1, 1, 1,
                                         NULL, NULL, 0, 0, NULL},
                      s_binding, sizeof s_binding, &binding, NULL) == mani_success &&
        maniInitGraphInstance(binding, s_instance, sizeof s_instance, &s_live) == mani_success;
    maniDestroyBuilder(builder);
    s_stateSize = ok ? maniGraphStateSize(binding) : 0;
    ok = ok && maniGraphScratchSize(binding) <= sizeof s_scratch &&
         maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success;
    if (!ok || s_stateSize > sizeof s_saved ||
        maniSaveGraphInstance(s_live, s_fresh, sizeof s_fresh) != mani_success)
    {
        abort();
    }
    uint32_t eased = 0;
    for (uint32_t p = 0; p < s_graph.nodeCount; ++p)
    {
        const uint8_t* e = s_graph.nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        if (maniLoadU16(e) == maniGraphNodeInertialize && eased < 2)
        {
            uint32_t block = maniLoadU32(s_graph.data +
                                         4 * (size_t)(maniLoadU32(e + 12) + maniGraphInertiaBlock));
            s_eased[eased++] = MANI_GRAPH_STATE_HEADER_SIZE +
                               (size_t)(s_graph.parameterCount + s_graph.nodeCount) * 16 +
                               4 * (size_t)block;
            s_easedSize = 4 * (size_t)MANI_GRAPH_INERTIA_BLOCK(s_graph.skeletonBoneCount, 0);
        }
    }
    if (eased != 2)
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
    // Updated from any restored state, forward and back.
    maniGraphEvent events[4];
    uint32_t count = 0;
    maniRootMove move;
    if (maniEvaluateGraph(s_live, s_scratch, sizeof s_scratch, s_pose) != mani_success ||
        maniUpdateGraph(s_live, 0.37f, events, 4, &count, &move) != mani_success ||
        maniUpdateGraph(s_live, -5.5f, events, 4, &count, &move) != mani_success ||
        maniEvaluateGraph(s_live, s_scratch, sizeof s_scratch, s_pose) != mani_success)
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
    memcpy(copy + s_eased[0], s_fresh + s_eased[0], s_easedSize);
    memcpy(copy + s_eased[1], s_fresh + s_eased[1], s_easedSize);
    Exercise(copy, s_stateSize);
    free(copy);
    return 0;
}
