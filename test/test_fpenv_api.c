// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The public calls under a host's floating-point environment: one
// scene run twice, every library call made with the host's control
// register set, once to the library's own setting and once to
// flush-to-zero, denormals-are-zero and round toward zero; every output
// (built files, poses, matrices, vertices, moves, search results) is the
// same bytes both times. The scene touches each module's computing
// calls: the builders and openers, sampling full and compressed clips
// and float tracks, blending, differences, the model pass, root motion,
// palettes and skinning, morph targets, the IK solvers, springs,
// mirroring, trajectories, motion matching and a graph. Its inputs are
// made once beforehand, in the library's setting, and include a
// subnormal so denormals-are-zero would show. tools/check_float_env.py
// checks that every computing call enters the environment; this suite
// checks that entering it holds.

#include "fpenv.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/morph.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"
#include "maul-anim/spring.h"
#include "maul-anim/trajectory.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__x86_64__) || defined(_M_X64)
// Flush-to-zero, denormals-are-zero, round toward zero, exceptions masked.
static const uint64_t s_hostSetting = 0x1f80u | (1u << 15) | (1u << 6) | (3u << 13);
static const int s_hostCanChange = 1;
#elif defined(__aarch64__) || defined(_M_ARM64)
// Flush-to-zero and round toward zero.
static const uint64_t s_hostSetting = (1ull << 24) | (3ull << 22);
static const int s_hostCanChange = 1;
#else
static const uint64_t s_hostSetting = 0;
static const int s_hostCanChange = 0;
#endif

#define BONES    12u
#define SAMPLES  31u
#define VERTICES 10u

// The register the calls run under, and the test's own, kept around
// each call.
static uint64_t s_host;
static uint64_t s_own;
static bool s_inHost;

static void Host(void)
{
    s_own = maniFloatControlRead();
    maniFloatControlWrite(s_host);
    s_inHost = true;
}

static void Back(void)
{
    s_inHost = false;
    maniFloatControlWrite(s_own);
}

#if defined(__x86_64__) || defined(_M_X64)
// MXCSR's sticky exception flags, which arithmetic sets on its own.
static const uint64_t s_flags = 0x3fu;
#else
static const uint64_t s_flags = 0;
#endif

// The builder's allocator, host code: it must run in the host's setting,
// not the library's. Counts its calls under Host() and those that found
// another setting; aligns by hand (no aligned_alloc on every target).
static uint32_t s_hostAllocations;
static uint32_t s_hostAllocationsElsewhere;

static void* Allocate(size_t size, size_t alignment, void* context)
{
    (void)context;
    if (s_inHost)
    {
        s_hostAllocations += 1;
        if ((maniFloatControlRead() & ~s_flags) != (s_host & ~s_flags))
        {
            s_hostAllocationsElsewhere += 1;
        }
    }
    uint8_t* raw = malloc(size + alignment + sizeof(void*));
    if (raw == NULL)
    {
        return NULL;
    }
    uintptr_t at = ((uintptr_t)(raw + sizeof(void*)) + alignment - 1) & ~(uintptr_t)(alignment - 1);
    memcpy((void*)(at - sizeof(void*)), &raw, sizeof raw);
    return (void*)at;
}

static void Free(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    void* raw = NULL;
    memcpy(&raw, (uint8_t*)memory - sizeof(void*), sizeof raw);
    free(raw);
}

// Each run's outputs, end to end.
static uint8_t s_out[2][1 << 18];
static size_t s_length[2];
static int s_run;

static void Record(const void* bytes, size_t size)
{
    size_t room = sizeof s_out[0] - s_length[s_run];
    size = size < room ? size : room;
    memcpy(s_out[s_run] + s_length[s_run], bytes, size);
    s_length[s_run] += size;
}

// A call's result: success, recorded.
static void RecordResult(int32_t result)
{
    CHECK(result == mani_success, "a call of the scene succeeds");
    Record(&result, sizeof result);
}

// The inputs, made once in the library's setting.
static maniBoneSource s_bones[BONES];
static float s_times[SAMPLES];
static float s_turns[BONES][SAMPLES * 4];
static float s_walk[SAMPLES * 3];
static float s_speed[SAMPLES];
static uint32_t s_offsets[VERTICES + 1];
static uint32_t s_joints[VERTICES * 4];
static float s_weights[VERTICES * 4];
static float s_dualWeights[VERTICES];
static float s_positions[VERTICES * 3];
static float s_normals[VERTICES * 3];
static float s_deltas[2][VERTICES * 3];
static maniAffine s_inverseBinds[BONES];

static void Rotation(double angle, double x, double y, double z, float* q)
{
    double length = sqrt(x * x + y * y + z * z);
    double s = sin(angle / 2.0) / length;
    q[0] = (float)(x * s);
    q[1] = (float)(y * s);
    q[2] = (float)(z * s);
    q[3] = (float)cos(angle / 2.0);
}

static void MakeInputs(void)
{
    static const char* const kNames[BONES] = {"hips",   "spine", "head",   "upperL",
                                              "lowerL", "handL", "upperR", "lowerR",
                                              "handR",  "tail0", "tail1",  "tail2"};
    static const int32_t kParents[BONES] = {-1, 0, 1, 1, 3, 4, 1, 6, 7, 0, 9, 10};
    static const float kOffsets[BONES][3] = {
        {0.0f, 1.0f, 0.0f},   {0.0f, 0.3f, 0.0f},  {0.0f, 0.3f, 0.02f}, {0.2f, 0.2f, 0.0f},
        {0.3f, 0.0f, 0.01f},  {0.25f, 0.0f, 0.0f}, {-0.2f, 0.2f, 0.0f}, {-0.3f, 0.0f, 0.01f},
        {-0.25f, 0.0f, 0.0f}, {0.0f, 0.0f, -0.1f}, {0.0f, 0.0f, -0.1f}, {0.0f, 0.0f, -0.1f}};
    float model[BONES][3];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        float rest[4];
        Rotation(0.05 * (double)b, 1.0, 0.5, 0.25, rest);
        s_bones[b] = (maniBoneSource){kNames[b],
                                      (uint32_t)strlen(kNames[b]),
                                      kParents[b],
                                      {{kOffsets[b][0], kOffsets[b][1], kOffsets[b][2]},
                                       {rest[0], rest[1], rest[2], rest[3]},
                                       {1.0f, 1.0f, 1.0f}}};
        for (uint32_t k = 0; k < 3; ++k)
        {
            model[b][k] = kOffsets[b][k] + (kParents[b] >= 0 ? model[kParents[b]][k] : 0.0f);
        }
        s_inverseBinds[b] =
            (maniAffine){{1, 0, 0, 0, 1, 0, 0, 0, 1, -model[b][0], -model[b][1], -model[b][2]}};
    }
    for (uint32_t s = 0; s < SAMPLES; ++s)
    {
        double t = (double)s / 30.0;
        s_times[s] = (float)t;
        for (uint32_t b = 0; b < BONES; ++b)
        {
            Rotation(0.4 * sin(6.283185307179586 * t + 0.7 * (double)b), 0.3, 1.0, 0.2 * (double)b,
                     &s_turns[b][4 * s]);
        }
        s_walk[3 * s] = (float)(0.1 * sin(3.0 * t));
        // A subnormal height at the first sample.
        s_walk[3 * s + 1] = s == 0 ? 1e-39f : (float)(1.0 + 0.02 * sin(9.0 * t));
        s_walk[3 * s + 2] = (float)(1.3 * t);
        s_speed[s] = (float)(1.3 + 0.1 * cos(5.0 * t));
    }
    for (uint32_t v = 0; v < VERTICES; ++v)
    {
        s_offsets[v] = 4 * v;
        const float weights[4] = {0.4f, 0.3f, 0.2f, 0.1f};
        for (uint32_t k = 0; k < 4; ++k)
        {
            s_joints[4 * v + k] = (v + 3 * k) % BONES;
            s_weights[4 * v + k] = weights[k];
        }
        s_dualWeights[v] = (float)v / (float)VERTICES;
        for (uint32_t k = 0; k < 3; ++k)
        {
            s_positions[3 * v + k] = model[v % BONES][k] + 0.01f * (float)(v + k);
            s_normals[3 * v + k] = k == 1 ? 0.8f : 0.6f / (float)(1 + k);
            s_deltas[0][3 * v + k] = 0.003f * (float)(v + 2 * k + 1);
            s_deltas[1][3 * v + k] = -0.002f * (float)(v + k + 1);
        }
    }
    s_offsets[VERTICES] = 4 * VERTICES;
}

// The run's memory: built files, poses, scratch.
// A slot a file: each view reads its file's bytes for the run.
static _Alignas(16) uint8_t s_files[10][1 << 15];
static _Alignas(32) uint8_t s_poseMemory[6][4096];
static _Alignas(8) uint8_t s_cursorMemory[1 << 14];
static _Alignas(8) uint8_t s_springMemory[4096];
static _Alignas(8) uint8_t s_bindingMemory[4096];
static _Alignas(8) uint8_t s_instanceMemory[1 << 14];
static _Alignas(32) uint8_t s_scratch[1 << 15];

// Copies the builder's file into slot and records its bytes.
static size_t Keep(maniBuilder* builder, int slot)
{
    size_t size = 0;
    CHECK(maniBuilder_CopyBuilt(builder, s_files[slot], sizeof s_files[slot], &size) ==
              mani_success,
          "a built file fits");
    Record(s_files[slot], size);
    return size;
}

static void RecordLocals(const maniPose* pose)
{
    maniTransform locals[BONES];
    CHECK(maniPose_ReadLocals(pose, 0, BONES, locals) == mani_success, "locals read");
    Record(locals, sizeof locals);
}

static maniBuilder* s_builder;
static maniSkeleton s_skeleton;
static maniClip s_clip;
static maniClip s_compressed;
static maniPose* s_pose[6];

// The skeleton and clip, built, compressed and opened.
static void Assets(void)
{
    maniSkeletonSource skeleton = {s_bones, BONES};
    Host();
    maniBuildResult built = maniBuildSkeleton(s_builder, &skeleton);
    Back();
    RecordResult(built.result);
    size_t size = Keep(s_builder, 0);
    Host();
    maniFileResult opened = maniOpenSkeleton(s_files[0], size, &s_skeleton);
    Back();
    RecordResult(opened.result);
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    maniBoneTrackSource tracks[BONES];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        tracks[b] = (maniBoneTrackSource){
            b, kNone, {s_times, s_turns[b], SAMPLES, maniInterpolationLinear}, kNone};
    }
    tracks[0].translation = (maniChannelSource){s_times, s_walk, SAMPLES, maniInterpolationLinear};
    const maniFloatTrackSource speed = {
        "speed", 5, {s_times, s_speed, SAMPLES, maniInterpolationLinear}};
    const maniRootMotionSource root = {
        0, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist, 1, 0, 0, 0};
    maniClipSource clip = {.skeleton = &s_skeleton,
                           .boneTracks = tracks,
                           .boneTrackCount = BONES,
                           .floatTracks = &speed,
                           .floatTrackCount = 1,
                           .loop = maniLoopRepeat,
                           .sampleRate = 30.0f,
                           .duration = 1.0f,
                           .rootMotion = &root};
    Host();
    built = maniBuildClip(s_builder, &clip);
    Back();
    RecordResult(built.result);
    size = Keep(s_builder, 1);
    Host();
    opened = maniOpenClip(s_files[1], size, &s_clip);
    Back();
    RecordResult(opened.result);
    maniCompressOptions options = maniDefaultCompressOptions();
    maniCompressReport report;
    Host();
    maniResult compressed = maniCompressClip(s_builder, &s_clip, &s_skeleton, &options, &report);
    Back();
    RecordResult(compressed);
    Record(&report, sizeof report);
    size = Keep(s_builder, 2);
    Host();
    opened = maniOpenClip(s_files[2], size, &s_compressed);
    Back();
    RecordResult(opened.result);
}

// Sampling, blending, differences, the model pass, root motion.
static void Poses(void)
{
    size_t poseSize = maniSkeleton_GetPoseSize(&s_skeleton);
    for (int k = 0; k < 6; ++k)
    {
        Host();
        maniResult made = maniInitPose(&s_skeleton, s_poseMemory[k], poseSize, &s_pose[k]);
        Back();
        RecordResult(made);
    }
    maniClipCursor* cursor = NULL;
    static const uint32_t kSubset[3] = {0, 3, 9};
    float floats[1];
    uint32_t floatCount = 0;
    maniResult r[8];
    Host();
    r[0] = maniClip_Sample(&s_clip, &s_skeleton, 0.37f, s_pose[0]);
    r[1] = maniClip_Sample(&s_compressed, &s_skeleton, 0.61f, s_pose[1]);
    r[2] = maniInitClipCursor(s_cursorMemory, sizeof s_cursorMemory, &cursor);
    r[3] = maniClip_SampleBones(&s_clip, &s_skeleton, 0.83f, kSubset, 3, s_pose[3]);
    r[4] = maniClip_SampleFloats(&s_clip, 0.29f, floats, 1, &floatCount);
    r[5] = maniClipCursor_Sample(cursor, &s_compressed, &s_skeleton, 0.53f, s_pose[2]);
    Back();
    for (int k = 0; k < 6; ++k)
    {
        RecordResult(r[k]);
    }
    Record(floats, sizeof floats);
    for (int k = 0; k < 4; ++k)
    {
        RecordLocals(s_pose[k]);
    }
    const maniBlendLayer layers[2] = {{s_pose[0], 0.3f, NULL}, {s_pose[1], 0.7f, NULL}};
    Host();
    r[0] = maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_pose[4]);
    r[1] = maniSkeleton_ComputePoseDifference(&s_skeleton, s_pose[4], s_pose[0], s_pose[5]);
    Back();
    const maniBlendLayer difference[1] = {{s_pose[5], 0.6f, NULL}};
    Host();
    r[2] = maniPose_AddLayers(s_pose[2], &s_skeleton, difference, 1);
    r[3] = maniPose_ComputeModel(s_pose[4], &s_skeleton);
    maniTransform head = maniPose_GetModelTransform(s_pose[4], 2);
    Back();
    for (int k = 0; k < 4; ++k)
    {
        RecordResult(r[k]);
    }
    Record(&head, sizeof head);
    RecordLocals(s_pose[2]);
    RecordLocals(s_pose[4]);
    maniAffine models[BONES];
    CHECK(maniPose_ReadModels(s_pose[4], 0, BONES, models) == mani_success, "models read");
    Record(models, sizeof models);
    maniTransform decomposed;
    maniRootMove moves[3];
    const float weights[2] = {0.25f, 0.75f};
    Host();
    r[0] = maniDecomposeAffine(&models[5], &decomposed);
    r[1] = maniClip_GetRootMotion(&s_clip, 0.2f, 0.45f, &moves[0]);
    r[2] = maniClip_GetRootMotion(&s_clip, 0.9f, 0.3f, &moves[1]);
    moves[2] = maniComposeRootMotion(moves[0], moves[1]);
    Back();
    maniRootMove blended;
    Host();
    r[3] = maniBlendRootMotion(moves, weights, 2, &blended);
    Back();
    for (int k = 0; k < 4; ++k)
    {
        RecordResult(r[k]);
    }
    Record(&decomposed, sizeof decomposed);
    Record(moves, sizeof moves);
    Record(&blended, sizeof blended);
}

// A skin of every bone, influences, palettes and the three jobs; morph
// targets of two channels.
static void Meshes(void)
{
    uint32_t joints[BONES];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        joints[b] = b;
    }
    maniSkinSource skin = {&s_skeleton, joints, s_inverseBinds, BONES};
    Host();
    maniBuildResult built = maniBuildSkin(s_builder, &skin);
    Back();
    RecordResult(built.result);
    size_t size = Keep(s_builder, 3);
    maniSkin opened;
    Host();
    RecordResult(maniOpenSkin(s_files[3], size, &opened).result);
    Back();
    maniInfluenceSource source = {&opened, s_offsets, s_joints, s_weights, s_dualWeights, VERTICES};
    Host();
    built = maniBuildInfluences(s_builder, &source);
    Back();
    RecordResult(built.result);
    size = Keep(s_builder, 4);
    maniInfluences influences;
    Host();
    RecordResult(maniOpenInfluences(s_files[4], size, &influences).result);
    Back();
    maniAffine palette[BONES];
    float dual[BONES * 20];
    float rows[BONES * 12];
    float moved[VERTICES * 3];
    float turned[VERTICES * 3];
    const maniSkinStream streams[2] = {{s_positions, moved, 12, 12, maniStreamPoint},
                                       {s_normals, turned, 12, 12, maniStreamNormal}};
    const maniSkinJob job = {&influences, palette, dual, BONES, 0, VERTICES, 2, streams};
    maniResult (*const jobs[3])(const maniSkinJob*) = {maniSkinLinear, maniSkinDualQuaternion,
                                                       maniSkinBlended};
    Host();
    maniResult r[3] = {
        maniPose_ComputePalette(s_pose[4], &s_skeleton, &opened, palette, BONES, NULL),
        maniConvertPaletteToDualQuaternions(palette, BONES, dual),
        maniConvertPaletteToRows(palette, BONES, rows)};
    Back();
    for (int k = 0; k < 3; ++k)
    {
        RecordResult(r[k]);
    }
    Record(palette, sizeof palette);
    Record(dual, sizeof dual);
    Record(rows, sizeof rows);
    for (int k = 0; k < 3; ++k)
    {
        Host();
        maniResult skinned = jobs[k](&job);
        Back();
        RecordResult(skinned);
        Record(moved, sizeof moved);
        Record(turned, sizeof turned);
    }
    const float* deltas[2][1] = {{s_deltas[0]}, {s_deltas[1]}};
    const maniMorphTargetSource targets[2] = {{1.0f, VERTICES, NULL, deltas[0], "a", 1},
                                              {1.0f, VERTICES, NULL, deltas[1], "b", 1}};
    const maniMorphChannelSource channels[2] = {{"a", 1, 1, &targets[0]}, {"b", 1, 1, &targets[1]}};
    const maniMorphAttributeSource attribute = {maniMorphPosition, 0};
    maniMorphSource morph = {channels, 2, VERTICES, &attribute, 1};
    Host();
    built = maniBuildMorphTargets(s_builder, &morph);
    Back();
    RecordResult(built.result);
    size = Keep(s_builder, 5);
    maniMorphTargets morphs;
    Host();
    RecordResult(maniOpenMorphTargets(s_files[5], size, &morphs).result);
    Back();
    const float channelWeights[2] = {0.35f, 0.9f};
    float targetWeights[2];
    const maniMorphStream stream = {s_positions, moved, 12, 12, maniMorphPosition, 0, 3};
    const maniMorphJob morphJob = {&morphs, targetWeights, 0, VERTICES, 1, &stream};
    Host();
    r[0] = maniMorphTargets_ComputeTargetWeights(&morphs, channelWeights, targetWeights,
                                                 maniMorphTargets_GetTargetCount(&morphs), NULL);
    r[1] = maniApplyMorphTargets(&morphJob);
    Back();
    RecordResult(r[0]);
    RecordResult(r[1]);
    Record(targetWeights, sizeof targetWeights);
    Record(moved, sizeof moved);
}

// The IK solvers, each from the blend; springs on the tail; the mirror.
static void Solvers(void)
{
    maniTransform start[BONES];
    CHECK(maniPose_ReadLocals(s_pose[4], 0, BONES, start) == mani_success, "the start");
    const maniTwoBoneSolve arm = {.start = 3,
                                  .middle = 4,
                                  .end = 5,
                                  .goal = {0.55f, 1.45f, 0.2f},
                                  .pole = {0.0f, 0.0f, 1.0f},
                                  .soften = 0.97f,
                                  .weight = 0.8f};
    const maniAimSolve look = {.bones = {1, 2},
                               .shares = {0.4f, 1.0f},
                               .count = 2,
                               .goal = {0.7f, 1.8f, 1.1f},
                               .pole = {0.0f, 1.0f, 0.0f},
                               .forward = {0.0f, 0.0f, 1.0f},
                               .up = {0.0f, 1.0f, 0.0f},
                               .weight = 1.0f};
    const maniChainSolve tail = {.root = 9,
                                 .end = 11,
                                 .goal = {0.12f, 1.1f, -0.25f},
                                 .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                                 .iterations = 8,
                                 .tolerance = 1e-4f,
                                 .weight = 1.0f};
    for (int k = 0; k < 4; ++k)
    {
        CHECK(maniPose_WriteLocals(s_pose[3], 0, BONES, start) == mani_success, "restored");
        bool reached = false;
        Host();
        maniResult solved = k == 0   ? maniPose_SolveTwoBone(s_pose[3], &s_skeleton, &arm, &reached)
                            : k == 1 ? maniPose_SolveAim(s_pose[3], &s_skeleton, &look, &reached)
                            : k == 2
                                ? maniPose_SolveCcd(s_pose[3], &s_skeleton, &tail, &reached)
                                : maniPose_SolveFabrik(s_pose[3], &s_skeleton, &tail, &reached);
        Back();
        RecordResult(solved);
        Record(&reached, sizeof reached);
        RecordLocals(s_pose[3]);
    }
    maniSpringJointSource joints[3];
    for (uint32_t k = 0; k < 3; ++k)
    {
        joints[k] = (maniSpringJointSource){.bone = 9 + k,
                                            .hitRadius = 0.02f,
                                            .stiffness = 0.7f,
                                            .gravityPower = 0.4f,
                                            .gravityDirection = {0.0f, -1.0f, 0.0f},
                                            .drag = 0.3f,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    const maniColliderSource ball = {0, maniColliderSphere, {0.0f, 0.0f, -0.15f}, {0, 0, 0}, 0.08f,
                                     0};
    static const uint32_t kMember[1] = {0};
    const maniColliderGroupSource group = {"hips", 4, 1, kMember};
    const maniSpringSource spring = {"tail", 4, 3, joints, kMember, 1, MANI_NO_BONE};
    const maniSpringSetSource set = {&s_skeleton, &ball, &group, &spring, 1, 1, 1, 0.0f, 0.0f};
    Host();
    maniBuildResult built = maniBuildSpringSet(s_builder, &set);
    Back();
    RecordResult(built.result);
    size_t size = Keep(s_builder, 6);
    maniSpringSet springs;
    maniSpringState* state = NULL;
    Host();
    RecordResult(maniOpenSpringSet(s_files[6], size, &springs).result);
    RecordResult(
        maniInitSpringState(&springs, &s_skeleton, s_springMemory, sizeof s_springMemory, &state));
    Back();
    const maniVec3 wind = {0.3f, 0.0f, 0.1f};
    for (int k = 0; k < 3; ++k)
    {
        CHECK(maniPose_WriteLocals(s_pose[3], 0, BONES, start) == mani_success, "restored");
        Host();
        maniResult stepped = maniSpringState_Step(state, s_pose[3], NULL, &wind, 1.0f / 60.0f);
        maniResult applied = maniSpringState_Apply(state, s_pose[3]);
        Back();
        RecordResult(stepped);
        RecordResult(applied);
        RecordLocals(s_pose[3]);
    }
    const maniMirrorPairSource pairs[3] = {{3, 6}, {4, 7}, {5, 8}};
    maniMirrorMapSource mirror = {
        .skeleton = &s_skeleton, .pairs = pairs, .pairCount = 3, .axis = maniMirrorAxisX};
    Host();
    built = maniBuildMirrorMap(s_builder, &mirror);
    Back();
    RecordResult(built.result);
    size = Keep(s_builder, 7);
    maniMirrorMap map;
    Host();
    RecordResult(maniOpenMirrorMap(s_files[7], size, &map).result);
    RecordResult(maniMirrorMap_MirrorPose(&map, &s_skeleton, s_pose[4], s_pose[3]));
    Back();
    RecordLocals(s_pose[3]);
}

// A trajectory, a motion database over the clip and a search; a graph
// blending the full clip and the compressed one.
static void Matching(void)
{
    maniTrajectory trajectory = {.rotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    const maniTrajectoryGoal goal = {
        {0.9f, 0.0f, 1.2f}, {0.0f, 0.38268343f, 0.0f, 0.92387953f}, 0.2f, 0.25f};
    maniTrajectory later = trajectory;
    Host();
    maniResult advanced = maniAdvanceTrajectory(&later, &goal, 0.4f);
    Back();
    RecordResult(advanced);
    Record(&later, sizeof later);
    static const float kNow[1] = {0.0f};
    static const float kAhead[2] = {0.2f, 0.4f};
    const maniMatchChannelSource channels[3] = {
        {maniMatchBonePosition, 5, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchBoneVelocity, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 2, kAhead, NULL, 0, 1.5f}};
    const maniClip* clips[1] = {&s_clip};
    const maniMatchEntrySource entry = {0, maniMatchEntryLoop, 0.0f, 1.0f, 0.0f, 1u};
    const maniMotionDatabaseSource source = {.skeleton = &s_skeleton,
                                             .clips = clips,
                                             .channels = channels,
                                             .entries = &entry,
                                             .clipCount = 1,
                                             .channelCount = 3,
                                             .entryCount = 1,
                                             .sampleRate = 30.0f,
                                             .up = maniMatchAxisY,
                                             .forward = maniMatchAxisZ};
    Host();
    maniBuildResult built = maniBuildMotionDatabase(s_builder, &source);
    Back();
    RecordResult(built.result);
    size_t size = Keep(s_builder, 8);
    maniMotionDatabase database;
    Host();
    RecordResult(maniOpenMotionDatabase(s_files[8], size, &database).result);
    Back();
    float features[11];
    float position[4];
    float facing[4];
    CHECK(maniMotionDatabase_GetFeatures(&database, 7, features, database.featureCount, NULL) ==
              mani_success,
          "features");
    Host();
    RecordResult(maniMotionDatabase_ComputeTrajectoryFeatures(&database, &trajectory, &later,
                                                              position, facing));
    Back();
    Record(position, 2 * sizeof(float));
    for (uint32_t j = 0; j < 7; ++j)
    {
        features[j] += 0.01f * (float)j;
    }
    const maniMatchQuery query = {
        .features = features, .current = 3, .jumpThreshold = 0.1f, .continuingBias = -0.01f};
    maniMatchResult found;
    Host();
    RecordResult(maniMotionDatabase_Search(&database, &query, &found));
    Back();
    Record(&found, sizeof found);
    static const char kGraph[] =
        "{\"format\": \"maul-anim graph\", \"version\": 1,"
        " \"parameters\": [{\"name\": \"mix\", \"type\": \"float\"}],"
        " \"clips\": [\"full\", \"small\"],"
        " \"nodes\": [{\"kind\": \"blend\", \"inputs\": [1, 2], \"weights\": [{\"parameter\": "
        "\"mix\"}, 1]},"
        " {\"kind\": \"clip\", \"clip\": \"full\"}, {\"kind\": \"clip\", \"clip\": \"small\"}],"
        " \"output\": 0}";
    const maniGraphClipSource sources[2] = {{&s_clip, "full", 4, NULL},
                                            {&s_compressed, "small", 5, NULL}};
    maniGraphNamedAssets named = {&s_skeleton, sources, NULL, NULL, NULL, 2, 0, 0, 0, NULL};
    Host();
    built = maniBuildGraphJson(s_builder, kGraph, sizeof kGraph - 1, &named);
    Back();
    RecordResult(built.result);
    size = Keep(s_builder, 9);
    maniGraph graph;
    const maniClip* bound[2] = {&s_clip, &s_compressed};
    maniGraphAssets assets = {.skeleton = &s_skeleton, .clips = bound, .clipCount = 2};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    maniRootMove move;
    uint32_t events = 0;
    maniResult r[6];
    Host();
    r[0] = maniOpenGraph(s_files[9], size, &graph).result;
    r[1] = maniBindGraph(&graph, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL);
    r[2] = maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory, &instance);
    r[3] = maniGraphInstance_SetParameter(instance, maniGraph_FindParameter(&graph, "mix", 3),
                                          maniGraphParameterFloat, (maniGraphValue){.f = {0.45f}});
    r[4] = maniGraphInstance_Update(instance, 0.37f, NULL, 0, &events, &move);
    r[5] = maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose[3]);
    Back();
    for (int k = 0; k < 6; ++k)
    {
        RecordResult(r[k]);
    }
    Record(&move, sizeof move);
    RecordLocals(s_pose[3]);
}

static void Scene(int run, uint64_t host)
{
    s_run = run;
    s_length[run] = 0;
    s_host = host;
    maniBuilderDef def = maniDefaultBuilderDef();
    def.allocator = (maniAllocator){Allocate, Free, NULL};
    s_hostAllocations = 0;
    s_hostAllocationsElsewhere = 0;
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success, "a builder");
    Assets();
    Poses();
    Meshes();
    Solvers();
    Matching();
    maniDestroyBuilder(s_builder);
    CHECK(s_hostAllocations > 0 && s_hostAllocationsElsewhere == 0,
          "the host's allocator runs in the host's setting");
}

static void TestTheSceneUnderTheHost(void)
{
    uint64_t own = maniFloatControlRead();
    Scene(0, own);
    Scene(1, s_hostSetting);
    CHECK(maniFloatControlRead() == own, "the test's setting back");
    CHECK(s_length[0] > 20000 && s_length[0] < sizeof s_out[0], "the scene's outputs recorded");
    size_t first = 0;
    while (first < s_length[0] && first < s_length[1] && s_out[0][first] == s_out[1][first])
    {
        ++first;
    }
    if (s_hostCanChange && (first != s_length[0] || s_length[0] != s_length[1]))
    {
        printf("  the outputs differ from byte %zu of %zu\n", first, s_length[0]);
    }
    CHECK(!s_hostCanChange || (s_length[0] == s_length[1] && first == s_length[0]),
          "every output the same bytes under the host's setting");
}

int main(void)
{
    MakeInputs();
    TestTheSceneUnderTheHost();
    return s_failures == 0 ? 0 : 1;
}
