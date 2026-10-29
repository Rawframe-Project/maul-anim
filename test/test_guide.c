// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The C snippets of docs/user/, each as written there, run in order:
// tools/check_guide.py checks that every snippet of those pages appears
// here or in test_guide_import.c unchanged, so the documentation cannot
// drift from the API. Each function gives a snippet what it uses and
// checks what it makes.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_clipFile[8192];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[3][2048];
static _Alignas(16) uint8_t s_skinFile[2048];
static _Alignas(16) uint8_t s_influencesFile[2048];
static _Alignas(16) uint8_t s_graphFile[4096];
static _Alignas(MANI_GRAPH_BINDING_ALIGNMENT) uint8_t s_bindingMemory[8192];
static _Alignas(MANI_GRAPH_INSTANCE_ALIGNMENT) uint8_t s_instanceMemory[8192];
static _Alignas(MANI_GRAPH_SCRATCH_ALIGNMENT) uint8_t s_scratch[16384];
static _Alignas(16) uint8_t s_tallFile[4096];
static _Alignas(16) uint8_t s_mapFile[8192];
static _Alignas(16) uint8_t s_mirrorFile[4096];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_tallMemory[2048];
static maniBuilder* s_builder;
static maniSkeleton s_skeleton;
static maniClip s_clip;
static maniPose* s_pose;

static void GuideCpu(void)
{
    // clang-format off
    const char* backend = NULL;
    int32_t runs = maniCheckCpu(&backend);   // before any other call
    // runs == 0: this CPU cannot run the build's backend (backend names it,
    // "avx2" say): refuse to start, or load a build made with
    // -DMAUL_ANIM_SIMD=scalar, which runs on any CPU with the same results.
    // clang-format on
    CHECK(runs == 1 && backend == maniGetSimdBackend(), "the CPU runs the build");
}

static void GuideSkeleton(void)
{
    uint8_t* file = s_skeletonFile;
    size_t capacity = sizeof s_skeletonFile;
    // clang-format off
    maniBuilderDef def = maniDefaultBuilderDef();   // the C library's allocator
    maniBuilder* builder = NULL;
    maniResult made = maniCreateBuilder(&def, &builder);

    const maniBoneSource bones[3] = {
        {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"spine", 5, 0, {{0.0f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"head", 4, 1, {{0.0f, 0.4f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}}};
    maniSkeletonSource source = {bones, 3};
    maniBuildResult built = maniBuildSkeleton(builder, &source);
    // built.result; when refused, built.reason, built.item and built.other.

    size_t size = 0;
    maniResult copied = maniBuilder_CopyBuilt(builder, file, capacity, &size);   // file 16-byte aligned
    maniSkeleton skeleton;
    maniFileResult opened = maniOpenSkeleton(file, size, &skeleton);
    // opened.result; on mani_errorFormat, opened.offset is the first bad byte.
    // clang-format on
    CHECK(made == mani_success && built.result == mani_success && copied == mani_success &&
              opened.result == mani_success && maniSkeleton_GetBoneCount(&skeleton) == 3 &&
              maniSkeleton_FindBone(&skeleton, "head", 4) == 2,
          "the skeleton snippet builds and opens three bones");
    s_builder = builder;
    s_skeleton = skeleton;
}

static void GuideClip(void)
{
    maniBuilder* builder = s_builder;
    maniSkeleton skeleton = s_skeleton;
    uint8_t* clipFile = s_clipFile;
    size_t clipCapacity = sizeof s_clipFile;
    // clang-format off
    static const float times[2] = {0.0f, 1.0f};
    static const float turns[8] = {0.0f, 0.0f, 0.0f, 1.0f,             // at 0 s
                                   0.0f, 0.38268343f, 0.0f, 0.92387953f}; // 45 degrees about y
    const maniBoneTrackSource tracks[1] = {
        {1, {NULL, NULL, 0, 0}, {times, turns, 2, maniInterpolationLinear}, {NULL, NULL, 0, 0}}};
    maniClipSource clipSource = {.skeleton = &skeleton,
                                 .boneTracks = tracks,
                                 .boneTrackCount = 1,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 30.0f,
                                 .duration = 1.0f};
    maniBuildResult clipBuilt = maniBuildClip(builder, &clipSource);
    size_t clipSize = 0;
    maniResult clipCopied = maniBuilder_CopyBuilt(builder, clipFile, clipCapacity, &clipSize);
    maniClip clip;
    maniFileResult clipOpened = maniOpenClip(clipFile, clipSize, &clip);
    // clang-format on
    CHECK(clipBuilt.result == mani_success && clipCopied == mani_success &&
              clipOpened.result == mani_success && maniClip_GetDuration(&clip) == 1.0f,
          "the clip snippet builds and opens a second's turn");
    s_clip = clip;
}

static void GuideSample(void)
{
    maniSkeleton skeleton = s_skeleton;
    maniClip clip = s_clip;
    uint8_t* poseMemory = s_poseMemory[0];
    // clang-format off
    maniPose* pose = NULL;
    maniResult ready = maniInitPose(&skeleton, poseMemory, maniSkeleton_GetPoseSize(&skeleton), &pose);
    maniResult sampled = maniClip_Sample(&clip, &skeleton, 0.5f, pose);   // half a second in
    maniTransform spine = maniPose_GetLocal(pose, 1);
    // clang-format on
    // Halfway between no turn and 45 degrees about y, by normalized
    // linear interpolation: within a few thousandths of 22.5 degrees.
    CHECK(maniSkeleton_GetPoseSize(&skeleton) <= sizeof s_poseMemory[0] && ready == mani_success &&
              sampled == mani_success && fabsf(spine.rotation.y - 0.19509032f) < 2e-3f &&
              spine.rotation.x == 0.0f,
          "the sampling snippet turns the spine half way");
    s_pose = pose;
}

static void GuideBlend(void)
{
    maniSkeleton skeleton = s_skeleton;
    maniPose* pose = s_pose;
    maniPose* walking = NULL;
    maniPose* running = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[1], sizeof s_poseMemory[1], &walking) ==
                  mani_success &&
              maniInitPose(&s_skeleton, s_poseMemory[2], sizeof s_poseMemory[2], &running) ==
                  mani_success &&
              maniClip_Sample(&s_clip, &s_skeleton, 0.0f, walking) == mani_success &&
              maniClip_Sample(&s_clip, &s_skeleton, 1.0f, running) == mani_success,
          "two poses to blend");
    // clang-format off
    const maniBlendLayer layers[2] = {{walking, 0.25f, NULL}, {running, 0.75f, NULL}};
    maniResult blended = maniSkeleton_BlendPoses(&skeleton, layers, 2, 0.1f, pose);
    // clang-format on
    maniQuat q = maniPose_GetLocal(pose, 1).rotation;
    CHECK(blended == mani_success && q.y > 0.19509032f && q.y < 0.38268343f,
          "the blend snippet leans toward the running pose");
}

static void GuideModel(void)
{
    maniSkeleton skeleton = s_skeleton;
    maniPose* pose = s_pose;
    // clang-format off
    maniResult composed = maniPose_ComputeModel(pose, &skeleton);
    maniAffine head = maniPose_GetModel(pose, 2);   // the head in model space
    // clang-format on
    // The head 0.4 above a spine 1.2 high: 1.6 up whatever its turn
    // about y.
    CHECK(composed == mani_success && maniPose_IsModelCurrent(pose) &&
              fabsf(head.m[10] - 1.6f) < 1e-6f,
          "the model snippet places the head");
}

static void GuideSkin(void)
{
    maniBuilder* builder = s_builder;
    maniSkeleton skeleton = s_skeleton;
    maniPose* pose = s_pose;
    uint8_t* skinFile = s_skinFile;
    size_t skinCapacity = sizeof s_skinFile;
    uint8_t* influencesFile = s_influencesFile;
    size_t influencesCapacity = sizeof s_influencesFile;
    // clang-format off
    const uint32_t joints[2] = {1, 2};   // the spine and the head
    const maniAffine inverseBinds[2] = {
        {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0.0f, -1.2f, 0.0f}},   // undoing each joint's rest
        {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0.0f, -1.6f, 0.0f}}};
    maniSkinSource skinSource = {&skeleton, joints, inverseBinds, 2};
    maniBuildResult skinBuilt = maniBuildSkin(builder, &skinSource);
    size_t skinSize = 0;
    maniResult skinCopied = maniBuilder_CopyBuilt(builder, skinFile, skinCapacity, &skinSize);
    maniSkin skin;
    maniFileResult skinOpened = maniOpenSkin(skinFile, skinSize, &skin);

    // Two vertices: one on the spine, one shared by the spine and the head.
    const uint32_t offsets[3] = {0, 1, 3};
    const uint32_t vertexJoints[3] = {0, 0, 1};
    const float weights[3] = {1.0f, 0.5f, 0.5f};
    maniInfluenceSource influenceSource = {&skin, offsets, vertexJoints, weights, NULL, 2};
    maniBuildResult influencesBuilt = maniBuildInfluences(builder, &influenceSource);
    size_t influencesSize = 0;
    maniResult influencesCopied =
        maniBuilder_CopyBuilt(builder, influencesFile, influencesCapacity, &influencesSize);
    maniInfluences influences;
    maniFileResult influencesOpened = maniOpenInfluences(influencesFile, influencesSize, &influences);
    // clang-format on
    CHECK(skinBuilt.result == mani_success && skinCopied == mani_success &&
              skinOpened.result == mani_success && influencesBuilt.result == mani_success &&
              influencesCopied == mani_success && influencesOpened.result == mani_success &&
              maniInfluences_GetVertexCount(&influences) == 2,
          "the skin snippet builds a skin and its influences");
    // clang-format off
    maniAffine palette[2];
    maniResult paletted = maniPose_ComputePalette(pose, &skeleton, &skin, palette, 2, NULL);
    const float positions[6] = {0.1f, 1.3f, 0.0f, 0.1f, 1.5f, 0.0f};
    float moved[6];
    const maniSkinStream stream = {positions, moved, 12, 12, maniStreamPoint};
    const maniSkinJob job = {&influences, palette, NULL, 2, 0, 2, 1, &stream};
    maniResult skinned = maniSkinLinear(&job);
    // clang-format on
    // The pose turns the spine about y only: heights stay, and the
    // vertex on the spine keeps its distance from the spine's axis.
    CHECK(paletted == mani_success && skinned == mani_success && fabsf(moved[1] - 1.3f) < 1e-6f &&
              fabsf(moved[4] - 1.5f) < 1e-6f &&
              fabsf(moved[0] * moved[0] + moved[2] * moved[2] - 0.01f) < 1e-6f && moved[0] < 0.1f,
          "the skinning snippet turns the vertices with the spine");
}

static void GuideGraph(void)
{
    maniBuilder* builder = s_builder;
    maniSkeleton skeleton = s_skeleton;
    maniClip clip = s_clip;
    maniPose* pose = s_pose;
    uint8_t* graphFile = s_graphFile;
    size_t graphCapacity = sizeof s_graphFile;
    uint8_t* bindingMemory = s_bindingMemory;
    uint8_t* instanceMemory = s_instanceMemory;
    uint8_t* scratch = s_scratch;
    // clang-format off
    static const char graphText[] =
        "{\"format\": \"maul-anim graph\", \"version\": 1,"
        " \"parameters\": [{\"name\": \"go\", \"type\": \"trigger\"}],"
        " \"clips\": [\"turn\"],"
        " \"nodes\": [{\"kind\": \"machine\", \"inputs\": [1, 2], \"transitions\":"
        " [{\"from\": 0, \"to\": 1, \"duration\": 0.25, \"conditions\": [{\"trigger\": \"go\"}]}]},"
        " {\"kind\": \"clip\", \"clip\": \"turn\"}, {\"kind\": \"clip\", \"clip\": \"turn\"}],"
        " \"output\": 0}";
    const maniGraphClipSource graphClips[1] = {{&clip, "turn", 4, NULL}};
    maniGraphNamedAssets named = {&skeleton, graphClips, NULL, NULL, NULL, 1, 0, 0, 0, NULL};
    maniBuildResult graphBuilt = maniBuildGraphJson(builder, graphText, sizeof graphText - 1, &named);
    size_t graphSize = 0;
    maniResult graphCopied = maniBuilder_CopyBuilt(builder, graphFile, graphCapacity, &graphSize);
    maniGraph graph;
    maniFileResult graphOpened = maniOpenGraph(graphFile, graphSize, &graph);
    // clang-format on
    CHECK(graphBuilt.result == mani_success && graphCopied == mani_success &&
              graphOpened.result == mani_success && maniGraph_GetNodeCount(&graph) == 3,
          "the graph snippet builds a machine of two clips");
    // clang-format off
    const maniClip* clips[1] = {&clip};
    maniGraphAssets assets = {.skeleton = &skeleton, .clips = clips, .clipCount = 1};
    maniGraphBinding* binding = NULL;
    maniResult bound =
        maniBindGraph(&graph, &assets, bindingMemory, maniGraph_GetBindingSize(&graph), &binding, NULL);
    maniGraphInstance* instance = NULL;
    maniResult started =
        maniInitGraphInstance(binding, instanceMemory, maniGraphBinding_GetInstanceSize(binding), &instance);
    // clang-format on
    CHECK(maniGraph_GetBindingSize(&graph) <= sizeof s_bindingMemory && bound == mani_success &&
              maniGraphBinding_GetInstanceSize(binding) <= sizeof s_instanceMemory &&
              started == mani_success,
          "the binding snippet makes an instance");
    // clang-format off
    uint32_t go = maniGraph_FindParameter(&graph, "go", 2);
    maniResult fired =
        maniGraphInstance_SetParameter(instance, go, maniGraphParameterTrigger, (maniGraphValue){.b = 1});
    maniGraphEvent events[8];
    uint32_t eventCount = 0;
    maniRootMove move;
    maniResult updated = maniGraphInstance_Update(instance, 1.0f / 60.0f, events, 8, &eventCount, &move);
    maniResult evaluated = maniGraphInstance_Evaluate(instance, scratch, maniGraphBinding_GetScratchSize(binding), pose);
    // clang-format on
    maniGraphNodeView view;
    CHECK(maniGraphBinding_GetScratchSize(binding) <= sizeof s_scratch && fired == mani_success &&
              updated == mani_success && evaluated == mani_success && eventCount == 0 &&
              maniGraphInstance_InspectNode(instance, maniGraph_GetOutputNode(&graph), &view) ==
                  mani_success &&
              view.current == 1 && view.transition == 0,
          "the frame snippet fires the transition and evaluates");
    (void)move;
}

static void GuideIk(void)
{
    maniSkeleton skeleton = s_skeleton;
    maniPose* pose = s_pose;
    // clang-format off
    // The hips, spine and head as a chain bending about the spine's z axis,
    // its end onto a goal in model space.
    const maniTwoBoneSolve reach = {.start = 0,
                                    .middle = 1,
                                    .end = 2,
                                    .goal = {0.3f, 1.5f, 0.0f},
                                    .hinge = {0.0f, 0.0f, 1.0f},
                                    .soften = 1.0f,
                                    .weight = 1.0f};
    bool reached = false;
    maniResult solved = maniPose_SolveTwoBone(pose, &skeleton, &reach, &reached);
    // clang-format on
    maniAffine head = maniPose_ComputeModel(pose, &skeleton) == mani_success
                          ? maniPose_GetModel(pose, 2)
                          : (maniAffine){{0}};
    CHECK(solved == mani_success && reached && fabsf(head.m[9] - 0.3f) < 1e-5f &&
              fabsf(head.m[10] - 1.5f) < 1e-5f && fabsf(head.m[11]) < 1e-5f,
          "the IK snippet puts the head on its goal");
}

// A rig like the guide's at twice its size, to retarget to.
static void Tall(maniSkeleton* tall)
{
    maniBoneSource bones[3];
    for (uint32_t b = 0; b < 3; ++b)
    {
        size_t length = 0;
        const char* name = maniSkeleton_GetBoneName(&s_skeleton, b, &length);
        maniTransform rest = maniSkeleton_GetRest(&s_skeleton, b);
        rest.translation.y *= 2.0f;
        bones[b] = (maniBoneSource){name, length, b == 0 ? -1 : (int32_t)b - 1, rest};
    }
    maniSkeletonSource source = {bones, 3};
    size_t size = 0;
    CHECK(maniBuildSkeleton(s_builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(s_builder, s_tallFile, sizeof s_tallFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_tallFile, size, tall).result == mani_success,
          "a tall rig");
}

static void GuideRetarget(void)
{
    maniBuilder* builder = s_builder;
    maniSkeleton skeleton = s_skeleton;
    maniPose* pose = s_pose;
    maniSkeleton tall;
    Tall(&tall);
    maniPose* tallPose = NULL;
    CHECK(maniInitPose(&tall, s_tallMemory, sizeof s_tallMemory, &tallPose) == mani_success,
          "a tall pose");
    uint8_t* mapFile = s_mapFile;
    size_t mapCapacity = sizeof s_mapFile;
    uint8_t* retargetScratch = s_scratch;
    uint8_t* mirrorFile = s_mirrorFile;
    size_t mirrorCapacity = sizeof s_mirrorFile;
    maniQuat before = maniPose_GetLocal(pose, 1).rotation;
    // clang-format off
    maniRetargetMapSource retargetSource = {.source = &skeleton, .target = &tall, .matchNames = true};
    maniBuildResult mapBuilt = maniBuildRetargetMap(builder, &retargetSource);
    size_t mapSize = 0;
    maniResult mapCopied = maniBuilder_CopyBuilt(builder, mapFile, mapCapacity, &mapSize);
    maniRetargetMap map;
    maniFileResult mapOpened = maniOpenRetargetMap(mapFile, mapSize, &map);
    maniResult carried = maniRetargetMap_RetargetPose(&map, &skeleton, pose, &tall, tallPose, retargetScratch,
                                          maniRetargetMap_GetScratchSize(&map));
    // clang-format on
    maniQuat after = maniPose_GetLocal(tallPose, 1).rotation;
    CHECK(mapBuilt.result == mani_success && mapCopied == mani_success &&
              mapOpened.result == mani_success &&
              maniRetargetMap_GetScratchSize(&map) <= sizeof s_scratch && carried == mani_success &&
              fabsf(before.x * after.x + before.y * after.y + before.z * after.z +
                    before.w * after.w) > 1.0f - 1e-6f,
          "the retargeting snippet carries the spine's turn to the tall rig");
    // clang-format off
    const maniMirrorRuleSource sides[2] = {{"L", 1, maniMirrorSuffix, "R", 1},
                                           {"R", 1, maniMirrorSuffix, "L", 1}};
    maniMirrorMapSource mirrorSource = {
        .skeleton = &skeleton, .rules = sides, .ruleCount = 2, .axis = maniMirrorAxisX};
    maniBuildResult mirrorBuilt = maniBuildMirrorMap(builder, &mirrorSource);
    size_t mirrorSize = 0;
    maniResult mirrorCopied = maniBuilder_CopyBuilt(builder, mirrorFile, mirrorCapacity, &mirrorSize);
    maniMirrorMap mirror;
    maniFileResult mirrorOpened = maniOpenMirrorMap(mirrorFile, mirrorSize, &mirror);
    maniResult mirrored = maniMirrorMap_MirrorPose(&mirror, &skeleton, pose, pose);   // in place
    // clang-format on
    // No bone has a partner: each mirrors itself across x, so the spine's
    // turn about y reverses.
    maniQuat back = maniPose_GetLocal(pose, 1).rotation;
    CHECK(mirrorBuilt.result == mani_success && mirrorCopied == mani_success &&
              mirrorOpened.result == mani_success && mirrored == mani_success &&
              maniMirrorMap_GetPartner(&mirror, 1) == 1 && fabsf(back.y + before.y) < 1e-6f,
          "the mirror snippet reflects the pose");
}

int main(void)
{
    GuideCpu();
    GuideSkeleton();
    GuideClip();
    GuideSample();
    GuideBlend();
    GuideModel();
    GuideSkin();
    GuideGraph();
    GuideIk();
    GuideRetarget();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
