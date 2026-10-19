// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Subset plans and their evaluation (mani-0035) on a rig of 20 bones (three
// pose groups): a plan holds the listed bones and their ancestors; two
// instances updated alike, one evaluated whole and one through a plan,
// agree bit for bit on the listed bones of a graph of clips, blends, an
// additive layer, a mask, a select, a machine and a one-shot, update after
// update, and every other bone keeps what it had; a graph with an IK node
// plans every bone and still agrees; every refusal.

#include "bytes.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_instance.h"
#include "graph_subset.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum
{
    kBones = 20,
    kClips = 3
};

static _Alignas(16) uint8_t s_skeletonFile[8192];
static _Alignas(16) uint8_t s_clipFiles[kClips][65536];
static _Alignas(16) uint8_t s_graphFile[65536];
static _Alignas(16) uint8_t s_binding[1u << 16];
static _Alignas(16) uint8_t s_instances[2][1u << 18];
static _Alignas(64) uint8_t s_scratch[1u << 18];
// The subset's own scratch: what a whole evaluation leaves in a shared
// one would hide a group the subset skipped wrongly.
static _Alignas(64) uint8_t s_subsetScratch[1u << 18];
static _Alignas(64) uint8_t s_poses[2][8192];
static _Alignas(8) uint8_t s_plan[4096];
static char s_names[kBones][8];
static maniSkeleton s_skeleton;
static maniClip s_clips[kClips];
static maniGraph s_graph;
static maniGraphBinding* s_bound;
static maniGraphInstance* s_live[2];
static _Alignas(16) uint8_t s_mirrorFile[8192];
static maniMirrorMap s_mirrorMap;
// The mirror map a graph opened next names, or NULL.
static const maniMirrorMap* s_mirror;
static _Alignas(16) uint8_t s_springFile[8192];
static maniSpringSet s_springSet;
static const maniGraphSpringSetSource kHostSets[1] = {{&s_springSet, "tail", 4, 0}};
static maniPose* s_pose[2];

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

// A tree in its depth-first order (the builder's, so names are indices):
// a spine b0 to b3, an arm b4 to b9 from b3, the spine on from b3 as b10
// to b13, an arm b14 to b19 from b11.
static int32_t Parent(uint32_t b)
{
    return b == 0 ? -1 : b == 4 || b == 10 ? 3 : b == 14 ? 11 : (int32_t)b - 1;
}

static void Rig(void)
{
    maniBoneSource bones[kBones];
    for (uint32_t b = 0; b < kBones; ++b)
    {
        snprintf(s_names[b], sizeof s_names[b], "b%u", b);
        bones[b] = (maniBoneSource){s_names[b],
                                    (uint32_t)strlen(s_names[b]),
                                    Parent(b),
                                    {{0.1f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1, 1, 1}}};
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, kBones, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "the rig");
    // Clips turning and moving every bone, each its own way; the third
    // additive.
    static float times[3] = {0.0f, 0.5f, 1.0f};
    static float rotations[kClips][kBones][12];
    static float translations[kClips][kBones][9];
    maniBoneTrackSource tracks[kBones];
    for (uint32_t c = 0; c < kClips; ++c)
    {
        for (uint32_t b = 0; b < kBones; ++b)
        {
            for (uint32_t k = 0; k < 3; ++k)
            {
                float h = 0.05f * (float)(k + 1) * (float)(c + 1) + 0.013f * (float)b;
                rotations[c][b][4 * k] = sinf(h) * 0.48f;
                rotations[c][b][4 * k + 1] = sinf(h) * 0.6f;
                rotations[c][b][4 * k + 2] = sinf(h) * 0.64f;
                rotations[c][b][4 * k + 3] = cosf(h);
                translations[c][b][3 * k] = 0.1f + 0.01f * (float)(k + c);
                translations[c][b][3 * k + 1] = 0.2f - 0.02f * (float)k;
                translations[c][b][3 * k + 2] = 0.003f * (float)(b + c);
            }
            tracks[b] =
                (maniBoneTrackSource){b,
                                      0,
                                      {times, translations[c][b], 3, maniInterpolationLinear},
                                      {times, rotations[c][b], 3, maniInterpolationLinear},
                                      {NULL, NULL, 0, maniInterpolationLinear}};
        }
        maniClipSource clip = {.skeleton = &s_skeleton,
                               .boneTracks = tracks,
                               .boneTrackCount = kBones,
                               .loop = maniLoopRepeat,
                               .sampleRate = 30.0f,
                               .duration = 1.0f,
                               .additive = c == 2 ? maniAdditiveRest : maniAdditiveNone};
        CHECK(maniBuildClip(builder, &clip).result == mani_success &&
                  maniCopyBuilt(builder, s_clipFiles[c], sizeof s_clipFiles[c], &size) ==
                      mani_success &&
                  maniOpenClip(s_clipFiles[c], size, &s_clips[c]).result == mani_success,
              "a clip");
    }
    // The arms paired, b4 to b9 with b14 to b19.
    maniMirrorPairSource pairs[6];
    for (uint32_t k = 0; k < 6; ++k)
    {
        pairs[k] = (maniMirrorPairSource){4 + k, 14 + k};
    }
    maniMirrorMapSource mirror = {
        .skeleton = &s_skeleton, .pairs = pairs, .pairCount = 6, .axis = maniMirrorAxisX};
    CHECK(maniBuildMirrorMap(builder, &mirror).result == mani_success &&
              maniCopyBuilt(builder, s_mirrorFile, sizeof s_mirrorFile, &size) == mani_success &&
              maniOpenMirrorMap(s_mirrorFile, size, &s_mirrorMap).result == mani_success,
          "the mirror map");
    // A spring down the second arm's end, colliding with a sphere on the
    // spine's end.
    maniSpringJointSource joints[4];
    for (uint32_t k = 0; k < 4; ++k)
    {
        joints[k] = (maniSpringJointSource){.bone = 16 + k,
                                            .hitRadius = 0.02f,
                                            .stiffness = 0.5f,
                                            .gravityPower = 0.2f,
                                            .gravityDirection = {0.0f, -1.0f, 0.0f},
                                            .drag = 0.4f,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    static const uint32_t kGroup[1] = {0};
    maniColliderSource collider = {.bone = 13, .radius = 0.1f};
    maniColliderGroupSource group = {"spine", 5, 1, kGroup};
    static const uint32_t kGroups[1] = {0};
    maniSpringSource spring = {"tail", 4, 4, joints, kGroups, 1, MANI_NO_BONE};
    maniSpringSetSource set = {&s_skeleton, &collider, &group, &spring, 1, 1, 1, 0.0f, 0.0f, 0};
    CHECK(maniBuildSpringSet(builder, &set).result == mani_success &&
              maniCopyBuilt(builder, s_springFile, sizeof s_springFile, &size) == mani_success &&
              maniOpenSpringSet(s_springFile, size, &s_springSet).result == mani_success,
          "the spring set");
    maniDestroyBuilder(builder);
}

static const maniGraphClipSource kHost[kClips] = {{&s_clips[0], "walk", 4, 0, NULL},
                                                  {&s_clips[1], "run", 3, 0, NULL},
                                                  {&s_clips[2], "lean", 4, 0, NULL}};

// A graph from its text, bound, two instances and two poses.
static bool Open(const char* text)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    const maniClip* clips[kClips] = {&s_clips[0], &s_clips[1], &s_clips[2]};
    const maniSpringSet* sets[1] = {&s_springSet};
    maniBuildResult built =
        maniBuildGraphJson(builder, text, strlen(text),
                           &(maniGraphNamedAssets){&s_skeleton, kHost, NULL, NULL, kHostSets,
                                                   kClips, 0, 0, 1, s_mirror});
    if (built.result != mani_success)
    {
        printf("refused: reason %u item %u other %u\n", built.reason, built.item, built.other);
    }
    bool ok = built.result == mani_success &&
              maniCopyBuilt(builder, s_graphFile, sizeof s_graphFile, &size) == mani_success &&
              maniOpenGraph(s_graphFile, size, &s_graph).result == mani_success &&
              maniBindGraph(&s_graph,
                            &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, sets, kClips, 0, 0,
                                               1, NULL, NULL, 0, 0, s_mirror},
                            s_binding, sizeof s_binding, &s_bound, NULL) == mani_success &&
              maniGraphScratchSize(s_bound) <= sizeof s_scratch;
    maniDestroyBuilder(builder);
    for (int i = 0; i < 2 && ok; ++i)
    {
        ok = maniInitGraphInstance(s_bound, s_instances[i], sizeof s_instances[i], &s_live[i]) ==
                 mani_success &&
             maniInitPose(&s_skeleton, s_poses[i], sizeof s_poses[i], &s_pose[i]) == mani_success;
    }
    return ok;
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": \"pick\", "     \
    "\"type\": \"int\"}, {\"name\": \"jump\", \"type\": \"trigger\"}, {\"name\": \"fire\", "       \
    "\"type\": \"trigger\"}, {\"name\": \"mix\", \"type\": \"float\", \"default\": 0.3}], "        \
    "\"clips\": [\"walk\", \"run\", \"lean\"], "

// Every node writing a bone from the same bone of its inputs, an
// inertialization over them eased by the machine's transition.
static const char kLocal[] = HEAD
    "\"nodes\": [{\"kind\": \"inertialize\", \"inputs\": [1], \"trigger\": \"jump\", "
    "\"duration\": 0.3}, "
    "{\"kind\": \"additive\", \"inputs\": [2, 10], \"weights\": [0.8]}, {\"kind\": \"oneShot\", "
    "\"inputs\": [3, 9], \"fire\": \"fire\", \"fadeIn\": 0.2, \"fadeOut\": 0.2}, {\"kind\": "
    "\"mask\", \"inputs\": [4, 5], \"chains\": [{\"bone\": \"b8\", \"weight\": 0.75}]}, "
    "{\"kind\": \"machine\", \"inputs\": [6, 7], \"transitions\": [{\"from\": 0, \"to\": 1, "
    "\"duration\": 0.5, \"conditions\": [{\"trigger\": \"jump\"}]}]}, {\"kind\": \"select\", "
    "\"inputs\": [8, 11], \"parameter\": \"pick\"}, {\"kind\": \"clip\", \"clip\": \"walk\"}, "
    "{\"kind\": \"blend\", \"inputs\": [12, 13], \"weights\": [{\"parameter\": \"mix\"}, 1]}, "
    "{\"kind\": \"clip\", \"clip\": \"run\"}, {\"kind\": \"clip\", \"clip\": \"walk\", "
    "\"speed\": 1.5}, {\"kind\": \"clip\", \"clip\": \"lean\"}, {\"kind\": \"clip\", \"clip\": "
    "\"run\", \"speed\": 0.5}, {\"kind\": \"clip\", \"clip\": \"walk\", \"speed\": 0.7}, "
    "{\"kind\": \"clip\", \"clip\": \"run\", \"speed\": 1.3}], \"output\": 0}";

// One IK node over a clip, the node at the file's end.
#define IK(node)                                                                                   \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": \"goal\", "     \
    "\"type\": \"vec3\", \"default\": [0.3, 0.9, 0.2]}], \"clips\": [\"walk\"], \"nodes\": [" node \
    ", {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}"

static void Set(uint32_t parameter, maniGraphParameterType type, int32_t value)
{
    maniGraphValue v = {.i = value};
    for (int i = 0; i < 2; ++i)
    {
        CHECK(maniSetGraphParameter(s_live[i], parameter, type, v) == mani_success, "set");
    }
}

static bool Same(maniTransform a, maniTransform b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

// A sentinel no evaluation writes.
static const maniTransform kMark = {{7.0f, 7.0f, 7.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {2, 2, 2}};

// Both instances updated by delta; the first evaluated whole, the second
// through the plan over a pose of marks; whether the plan's bones agree
// bit for bit and the rest keep their marks.
static bool Step(const maniGraphSubset* plan, float delta)
{
    uint32_t count = 0;
    const uint32_t* listed = maniGraphSubsetBones(plan, &count);
    bool ok = true;
    for (int i = 0; i < 2; ++i)
    {
        uint32_t events = 0;
        maniRootMove move;
        ok = ok && maniUpdateGraph(s_live[i], delta, NULL, 0, &events, &move) == mani_success;
    }
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniPoseSetLocal(s_pose[1], b, kMark);
    }
    ok = ok &&
         maniEvaluateGraph(s_live[0], s_scratch, sizeof s_scratch, s_pose[0]) == mani_success &&
         maniEvaluateGraphSubset(s_live[1], plan, s_subsetScratch, sizeof s_subsetScratch,
                                 s_pose[1]) == mani_success;
    uint32_t next = 0;
    for (uint32_t b = 0; b < kBones && ok; ++b)
    {
        bool isListed = next < count && listed[next] == b;
        next += isListed ? 1u : 0u;
        maniTransform got = maniPoseLocal(s_pose[1], b);
        ok = isListed ? Same(got, maniPoseLocal(s_pose[0], b)) : Same(got, kMark);
        if (!ok)
        {
            printf("bone %u differs\n", b);
        }
    }
    return ok && maniPoseIsDifference(s_pose[0]) == maniPoseIsDifference(s_pose[1]);
}

// A plan in memory filled with 0xFF first: no byte of it may be left
// unwritten, and none past maniGraphSubsetSize written.
static maniGraphSubset* Plan(const uint32_t* bones, uint32_t count)
{
    maniGraphSubset* plan = NULL;
    size_t size = maniGraphSubsetSize(s_bound);
    memset(s_plan, 0xFF, sizeof s_plan);
    CHECK(size + 64 <= sizeof s_plan &&
              maniPlanGraphSubset(s_bound, bones, count, s_plan, size, &plan) == mani_success,
          "planned");
    bool kept = true;
    for (size_t i = size; i < size + 64; ++i)
    {
        kept = kept && s_plan[i] == 0xFF;
    }
    CHECK(kept, "nothing written past the plan's size");
    return plan;
}

// The words of the second instance's output inertialization outside
// the plan's groups (the last group, bones 16 to 19): both kept outputs
// and the bones' captures.
enum
{
    kKept = 2 * MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS + 4 * 18
};

static void Outside(uint32_t* words)
{
    const uint8_t* record =
        s_graph.nodes + (size_t)maniGraphOutputNode(&s_graph) * MANI_GRAPH_NODE_SIZE;
    uint32_t data = maniLoadU32(record + 12);
    const uint32_t* block = (const uint32_t*)((const uint8_t*)s_live[1] + s_live[1]->machinesAt) +
                            maniLoadU32(s_graph.data + 4 * ((size_t)data + maniGraphInertiaBlock));
    maniGraphInertia node = maniGraphInertiaOf(&s_graph, data);
    uint32_t n = 0;
    uint32_t group = 2 * MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS;
    for (int last = 0; last < 2; ++last)
    {
        uint32_t at = maniGraphInertiaHistory(block, kBones, last != 0) + group;
        for (uint32_t k = 0; k < MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS; ++k)
        {
            words[n++] = block[at + k];
        }
    }
    for (uint32_t b = 16; b < kBones; ++b)
    {
        for (uint32_t k = 0; k < 18; ++k)
        {
            words[n++] = block[maniGraphInertiaCapture(&node, b) + k];
        }
    }
}

static void TestLocal(void)
{
    CHECK(Open(kLocal), "the graph of local nodes");
    // The arm's end (ancestors 0 to 3, then 8 to 13) and a spine bone.
    static const uint32_t kListed[2] = {6, 13};
    maniGraphSubset* plan = Plan(kListed, 2);
    static const uint32_t kPlanned[11] = {0, 1, 2, 3, 4, 5, 6, 10, 11, 12, 13};
    uint32_t count = 0;
    const uint32_t* planned = maniGraphSubsetBones(plan, &count);
    CHECK(count == 11 && memcmp(planned, kPlanned, sizeof kPlanned) == 0,
          "the listed bones and their ancestors");
    const uint8_t* groups = maniGraphSubsetGroups(plan);
    CHECK(plan->groups == 3 && groups[0] == 1 && groups[1] == 1 && groups[2] == 0,
          "the groups holding them, the last left out");
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniPoseSetLocal(s_pose[1], b, kMark);
    }
    bool rest = maniEvaluateGraphSubset(s_live[1], plan, s_subsetScratch, sizeof s_subsetScratch,
                                        s_pose[1]) == mani_success;
    for (uint32_t b = 0; b < kBones && rest; ++b)
    {
        bool isPlanned = (b <= 6) || (b >= 10 && b <= 13);
        rest =
            Same(maniPoseLocal(s_pose[1], b), isPlanned ? maniSkeletonRest(&s_skeleton, b) : kMark);
    }
    CHECK(rest, "before the first update, the rest at the plan's bones alone");
    // The second instance, evaluated only through the plan, never writes
    // its inertialization outside the plan's groups: as made, all zero.
    static uint32_t before[kKept];
    static uint32_t after[kKept];
    Outside(before);
    bool ok = true;
    ok = ok && Step(plan, 0.0f);
    ok = ok && Step(plan, 0.1f);
    Set(1, maniGraphParameterTrigger, 1);
    ok = ok && Step(plan, 0.15f) && Step(plan, 0.15f);
    Set(2, maniGraphParameterTrigger, 1);
    ok = ok && Step(plan, 0.1f) && Step(plan, 0.1f);
    Set(0, maniGraphParameterInt, 1);
    ok = ok && Step(plan, 0.3f) && Step(plan, 0.7f);
    CHECK(ok, "the listed bones bit for bit, the rest untouched, update after update");
    Outside(after);
    CHECK(memcmp(before, after, sizeof before) == 0,
          "an inertialization's outputs and captures outside the plan kept");
    // One bone of the last group alone, then every bone.
    static const uint32_t kLast[1] = {19};
    plan = Plan(kLast, 1);
    CHECK(Step(plan, 0.2f), "a bone of the last group");
    static uint32_t all[kBones];
    for (uint32_t b = 0; b < kBones; ++b)
    {
        all[b] = b;
    }
    plan = Plan(all, kBones);
    CHECK(Step(plan, 0.2f), "every bone");
    plan = Plan(NULL, 0);
    CHECK(Step(plan, 0.2f), "no bone");
}

// The bones a plan of listed bones holds, whether its last node runs,
// and agreement with a full evaluation over a few updates.
static bool Planned(const uint32_t* listed, uint32_t count, const uint32_t* want, uint32_t wanted,
                    bool runs)
{
    maniGraphSubset* plan = Plan(listed, count);
    uint32_t n = 0;
    const uint32_t* bones = maniGraphSubsetBones(plan, &n);
    bool ok = n == wanted && memcmp(bones, want, 4 * (size_t)n) == 0 &&
              maniGraphSubsetRuns(plan, maniGraphOutputNode(&s_graph)) == runs;
    if (!ok)
    {
        printf("planned:");
        for (uint32_t i = 0; i < n; ++i)
        {
            printf(" %u", bones[i]);
        }
        printf("\n");
    }
    return ok && Step(plan, 0.1f) && Step(plan, 0.25f) && Step(plan, 0.4f);
}

// The first arm's b8 and its ancestors.
static const uint32_t kArm[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

static void TestIk(void)
{
    static const uint32_t kEnd[1] = {8};
    static const uint32_t kBelow[1] = {9};
    static const uint32_t kBelowPlan[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static const uint32_t kSpine[1] = {12};
    static const uint32_t kSpinePlan[7] = {0, 1, 2, 3, 10, 11, 12};
    static const char* const kNodes[4] = {
        IK("{\"kind\": \"twoBone\", \"inputs\": [1], \"start\": \"b6\", \"middle\": \"b7\", "
           "\"end\": \"b8\", \"goal\": \"goal\"}"),
        IK("{\"kind\": \"aim\", \"inputs\": [1], \"bones\": [\"b7\", \"b8\"], \"goal\": "
           "\"goal\", \"forward\": [1, 0, 0]}"),
        IK("{\"kind\": \"ccd\", \"inputs\": [1], \"root\": \"b5\", \"end\": \"b8\", \"goal\": "
           "\"goal\", \"iterations\": 4}"),
        IK("{\"kind\": \"fabrik\", \"inputs\": [1], \"root\": \"b5\", \"end\": \"b8\", "
           "\"goal\": \"goal\", \"iterations\": 4}")};
    for (uint32_t k = 0; k < 4; ++k)
    {
        CHECK(Open(kNodes[k]), "a graph with an IK node");
        CHECK(Planned(kEnd, 1, kArm, 9, true), "its chain's end: the chain, which runs");
        CHECK(Planned(kBelow, 1, kBelowPlan, 10, true), "a bone below the chain: it runs");
        CHECK(Planned(kSpine, 1, kSpinePlan, 7, false), "a bone apart: it passes its input on");
        // A two-bone node's middle or an aim's first bone alone: the rest
        // of the chain is read.
        static const uint32_t kMiddle[1] = {7};
        CHECK(k > 1 || Planned(kMiddle, 1, kArm, 9, true), "a chain's first bones bring its end");
    }
    // An aim over the arm's last two bones, one above them listed: it
    // writes no planned bone.
    static const uint32_t kFore[1] = {7};
    CHECK(Open(IK("{\"kind\": \"aim\", \"inputs\": [1], \"bones\": [\"b8\", \"b9\"], "
                  "\"goal\": \"goal\", \"forward\": [1, 0, 0]}")) &&
              Planned(kFore, 1, kArm, 8, false),
          "a chain below the listed bones does not run");
    // A chain IK along the spine (b2, b3, b10, b11), an arm bone from b3
    // listed: the chain writes b2 and b3, so its whole path is planned.
    static const uint32_t kSpineChain[9] = {0, 1, 2, 3, 4, 5, 6, 10, 11};
    static const uint32_t kNine[1] = {6};
    CHECK(Open(IK("{\"kind\": \"ccd\", \"inputs\": [1], \"root\": \"b2\", \"end\": \"b11\", "
                  "\"goal\": \"goal\", \"iterations\": 4}")) &&
              Planned(kNine, 1, kSpineChain, 9, true),
          "a chain one of whose bones is planned brings its whole path");
}

// A mirror node: each planned bone brings its partner.
static void TestMirror(void)
{
    s_mirror = &s_mirrorMap;
    static const uint32_t kEnd[1] = {8};
    static const uint32_t kBoth[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 14, 15, 16, 17, 18};
    static const uint32_t kSpine[1] = {12};
    static const uint32_t kSpinePlan[7] = {0, 1, 2, 3, 10, 11, 12};
    CHECK(Open("{\"format\": \"maul-anim graph\", \"version\": 1, \"clips\": [\"walk\"], "
               "\"nodes\": [{\"kind\": \"mirror\", \"inputs\": [1]}, {\"kind\": \"clip\", "
               "\"clip\": \"walk\"}], \"output\": 0}") &&
              Planned(kEnd, 1, kBoth, 16, true) && Planned(kSpine, 1, kSpinePlan, 7, true),
          "a bone of an arm brings its partner and the partner's ancestors");
    s_mirror = NULL;
}

static const char kSpringGraph[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"clips\": [\"walk\"], \"springSets\": "
    "[\"tail\"], \"nodes\": [{\"kind\": \"spring\", \"inputs\": [1], \"springSet\": "
    "\"tail\"}, {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}";

// A spring node writing a planned joint brings its joints, tails,
// colliders' bones and their ancestors; one writing none does not run.
static void TestSprings(void)
{
    static const uint32_t kJoint[1] = {17};
    static const uint32_t kSet[14] = {0, 1, 2, 3, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    static const uint32_t kArmA[1] = {6};
    static const uint32_t kArmAPlan[7] = {0, 1, 2, 3, 4, 5, 6};
    CHECK(Open(kSpringGraph) && Planned(kJoint, 1, kSet, 14, true) &&
              Planned(kArmA, 1, kArmAPlan, 7, false),
          "a spring's joints, tails and colliders");
    // Passing its input on, the spring is not stepped: whole again, the
    // instance matches one only updated meanwhile.
    static _Alignas(16) uint8_t memory[1u << 18];
    maniGraphInstance* reference = NULL;
    CHECK(Open(kSpringGraph) &&
              maniInitGraphInstance(s_bound, memory, sizeof memory, &reference) == mani_success,
          "a reference instance");
    maniGraphSubset* plan = Plan(kArmA, 1);
    maniGraphInstance* both[2] = {s_live[1], reference};
    bool same = true;
    for (int frame = 0; frame < 5; ++frame)
    {
        for (int i = 0; i < 2; ++i)
        {
            uint32_t events = 0;
            maniRootMove move;
            same = same && maniUpdateGraph(both[i], 0.05f, NULL, 0, &events, &move) == mani_success;
        }
        bool whole = frame == 0 || frame == 4;
        same = same &&
               (whole ? maniEvaluateGraph(s_live[1], s_subsetScratch, sizeof s_subsetScratch,
                                          s_pose[1])
                      : maniEvaluateGraphSubset(s_live[1], plan, s_subsetScratch,
                                                sizeof s_subsetScratch, s_pose[1])) == mani_success;
        same = same && (!whole || maniEvaluateGraph(reference, s_scratch, sizeof s_scratch,
                                                    s_pose[0]) == mani_success);
    }
    for (uint32_t b = 0; b < kBones && same; ++b)
    {
        same = Same(maniPoseLocal(s_pose[0], b), maniPoseLocal(s_pose[1], b));
    }
    CHECK(same, "a spring passing its input on is not stepped");
}

// A full-body node writing a planned bone brings its paths from its top
// to its effectors.
static void TestBody(void)
{
    static const uint32_t kSpine[1] = {12};
    static const uint32_t kPaths[17] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 14, 15, 16, 17, 18};
    static const uint32_t kRoot[1] = {1};
    static const uint32_t kRootPlan[2] = {0, 1};
    for (int sdls = 0; sdls < 2; ++sdls)
    {
        char text[1024];
        snprintf(text, sizeof text,
                 "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": "
                 "[{\"name\": \"left\", \"type\": \"vec3\", \"default\": [0.5, 1, 0]}, "
                 "{\"name\": \"right\", \"type\": \"vec3\", \"default\": [-0.5, 1.2, 0]}], "
                 "\"clips\": [\"walk\"], \"nodes\": [{\"kind\": \"%s\", \"inputs\": [1], "
                 "\"top\": \"b3\", \"effectors\": [{\"bone\": \"b8\", \"position\": "
                 "\"left\"}, {\"bone\": \"b18\", \"position\": \"right\"}], \"iterations\": "
                 "4}, {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}",
                 sdls != 0 ? "bodySdls" : "bodyXpbd");
        CHECK(Open(text) && Planned(kSpine, 1, kPaths, 17, true) &&
                  Planned(kRoot, 1, kRootPlan, 2, false),
              "a full-body node's paths");
    }
}

// A closure needing several passes: the listed arm's partners reach an
// aim chain, whose last bone's partner is in the first arm.
static void TestFixedPoint(void)
{
    s_mirror = &s_mirrorMap;
    static const uint32_t kListed[1] = {8};
    static const uint32_t kPlan[18] = {0, 1,  2,  3,  4,  5,  6,  7,  8,
                                       9, 10, 11, 14, 15, 16, 17, 18, 19};
    CHECK(Open("{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "
               "\"goal\", \"type\": \"vec3\", \"default\": [0.3, 0.9, 0.2]}], \"clips\": "
               "[\"walk\"], \"nodes\": [{\"kind\": \"mirror\", \"inputs\": [1]}, {\"kind\": "
               "\"aim\", \"inputs\": [2], \"bones\": [\"b18\", \"b19\"], \"goal\": \"goal\", "
               "\"forward\": [1, 0, 0]}, {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": "
               "0}") &&
              Planned(kListed, 1, kPlan, 18, true),
          "partners, then the aim's chain, then its partners");
    s_mirror = NULL;
}

static void TestRefusals(void)
{
    CHECK(Open(kLocal), "the graph");
    maniGraphSubset* plan = NULL;
    static const uint32_t kDown[2] = {5, 3};
    static const uint32_t kTwice[2] = {3, 3};
    static const uint32_t kPast[1] = {kBones};
    CHECK(maniPlanGraphSubset(NULL, NULL, 0, s_plan, sizeof s_plan, &plan) == mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, NULL, 1, s_plan, sizeof s_plan, &plan) ==
                  mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, kDown, 2, s_plan, sizeof s_plan, &plan) ==
                  mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, kTwice, 2, s_plan, sizeof s_plan, &plan) ==
                  mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, kPast, 1, s_plan, sizeof s_plan, &plan) ==
                  mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, NULL, 0, s_plan + 4, sizeof s_plan - 4, &plan) ==
                  mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, NULL, 0, NULL, sizeof s_plan, &plan) ==
                  mani_errorInvalid &&
              maniPlanGraphSubset(s_bound, NULL, 0, s_plan, sizeof s_plan, NULL) ==
                  mani_errorInvalid,
          "plans refused");
    CHECK(maniPlanGraphSubset(s_bound, NULL, 0, s_plan, maniGraphSubsetSize(s_bound) - 1, &plan) ==
                  mani_errorCapacity &&
              maniGraphSubsetSize(NULL) == 0,
          "too little memory");
    uint32_t count = 7;
    CHECK(maniGraphSubsetBones(NULL, &count) == NULL && count == 0 &&
              maniGraphSubsetBones((const maniGraphSubset*)s_scratch, NULL) == NULL,
          "no plan, no bones");
    count = 7;
    CHECK(maniGraphSubsetBones((const maniGraphSubset*)s_scratch, &count) == NULL && count == 0,
          "memory that is no plan, no bones");
    plan = Plan(NULL, 0);
    // A plan whose stamp is gone.
    static _Alignas(8) uint8_t unstamped[4096];
    memcpy(unstamped, s_plan, sizeof unstamped);
    memset(unstamped, 0, 4);
    CHECK(maniEvaluateGraphSubset(s_live[0], (const maniGraphSubset*)unstamped, s_scratch,
                                  sizeof s_scratch, s_pose[0]) == mani_errorInvalid,
          "a plan without its stamp");
    static _Alignas(8) uint8_t kNever[64];
    CHECK(maniEvaluateGraphSubset(s_live[0], NULL, s_scratch, sizeof s_scratch, s_pose[0]) ==
                  mani_errorInvalid &&
              maniEvaluateGraphSubset(s_live[0], (const maniGraphSubset*)kNever, s_scratch,
                                      sizeof s_scratch, s_pose[0]) == mani_errorInvalid &&
              maniEvaluateGraphSubset(NULL, plan, s_scratch, sizeof s_scratch, s_pose[0]) ==
                  mani_errorInvalid,
          "evaluations refused");
    // A plan of another binding of the same graph.
    static _Alignas(16) uint8_t other[1u << 16];
    const maniClip* clips[kClips] = {&s_clips[0], &s_clips[1], &s_clips[2]};
    maniGraphBinding* binding = NULL;
    maniGraphSubset* foreign = NULL;
    static _Alignas(8) uint8_t memory[4096];
    CHECK(maniBindGraph(&s_graph,
                        &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, kClips, 0, 0, 0,
                                           NULL, NULL, 0, 0, NULL},
                        other, sizeof other, &binding, NULL) == mani_success &&
              maniPlanGraphSubset(binding, NULL, 0, memory, sizeof memory, &foreign) ==
                  mani_success &&
              maniEvaluateGraphSubset(s_live[0], foreign, s_scratch, sizeof s_scratch, s_pose[0]) ==
                  mani_errorInvalid,
          "a plan of another binding");
}

int main(void)
{
    Rig();
    TestLocal();
    TestIk();
    TestMirror();
    TestSprings();
    TestBody();
    TestFixedPoint();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
