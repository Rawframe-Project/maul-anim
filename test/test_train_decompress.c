// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's first training stage: the training data
// of a database of the walk and the run (counts, layout, a frame's pose
// and root velocity as the clip gives them, the events' states, the
// pairs); the Compressor and Decompressor trained, the loss falling; the
// same bits whatever order a host runs the chunks in; a chunk's loss
// against Holden's terms computed apart (character space composed
// through a pose the library builds), and its gradient against central
// differences of it; every frame's latent values. A database of the walk
// and a sway gives scaled bones, a float track, an event across a loop's
// wrap, a turning root, local velocities and character space.

#include "graph_match_fixture.h"
#include "train_data.h"
#include "train_decompress.h"
#include "train_decompress_loss.h"
#include "train_kinematics.h"
#include "train_net.h"
#include "train_random.h"

#include "maul-anim/learnedbuild.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_boneFile[1 << 15];
static maniMotionDatabase s_bones;
static const maniLearnedNameSource kEvents[2] = {{"left", 4, 0}, {"right", 5, 0}};
static const maniAllocator kAllocator = {NULL, NULL, NULL};

// The walk (looping, 10 frames) and the run (clamped, 11): the arm's
// position now and the root half a second ahead.
static void OpenBones(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 2, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    BuildDatabase(channels, 2, s_boneFile, sizeof s_boneFile, &s_bones);
}

static bool Data(maniTrainData* d)
{
    static const maniClip* clips[2];
    clips[0] = &s_clips[0];
    clips[1] = &s_clips[1];
    const maniTrainDataSource source = {&s_skeleton, &s_bones, clips, NULL, kEvents, 0, 2, 0.1f, 0};
    return maniTrainDataBuild(&kAllocator, &source, d);
}

static void TestData(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    CHECK(d.frameCount == 21 && d.featureCount == 5 && d.boneCount == 3 && d.scaledBoneCount == 0 &&
              d.outputCount == 27 + 6 + 2 &&
              d.inputCount == 3 * (3 + 6 + 3 + 3 + 3 + 6 + 3 + 6) + 6 + 2 && d.pairCount == 19 &&
              d.frameSeconds == 0.1f && d.groupStarts[maniTrainGroupScale] == 99 &&
              d.groupStarts[maniTrainGroupExtra] == 105,
          "the counts and layout");
    // Frame 3: the walk at 0.3 s.
    static _Alignas(32) uint8_t memory[4096];
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, memory, sizeof memory, &pose) == mani_success &&
              maniSampleClip(&s_clips[0], &s_skeleton, 0.3f, pose) == mani_success,
          "the walk at 0.3 s");
    bool same = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        const float* o = d.outputs + 3 * (size_t)d.outputCount + 9 * b;
        maniVec3 x = maniQuatRotate(t.rotation, maniVec3Make(1, 0, 0));
        maniVec3 y = maniQuatRotate(t.rotation, maniVec3Make(0, 1, 0));
        same = same && o[0] == t.translation.x && o[1] == t.translation.y &&
               o[2] == t.translation.z && o[3] == x.x && o[4] == x.y && o[5] == x.z &&
               o[6] == y.x && o[7] == y.y && o[8] == y.z;
    }
    CHECK(same, "a frame's local pose as the clip's");
    const float* root = d.outputs + 3 * (size_t)d.outputCount + 27;
    CHECK(fabsf(root[0] - 1.0f) <= 1e-4f && fabsf(root[1]) <= 1e-6f && fabsf(root[2]) <= 1e-6f &&
              fabsf(root[3]) <= 1e-6f && fabsf(root[4]) <= 1e-6f && fabsf(root[5]) <= 1e-6f,
          "the walk's root at 1 m/s, not turning");
    float features[5];
    CHECK(maniMotionDatabaseFeatures(&s_bones, 3, features) == mani_success &&
              memcmp(features, d.features + 15, sizeof features) == 0,
          "the database's features");
    // The walk's left at 0 s and right at 0.5 s; nothing in the run.
    bool events = true;
    for (uint32_t f = 0; f < 21; ++f)
    {
        const float* e = d.outputs + (size_t)f * d.outputCount + 33;
        events = events && e[0] == (f == 0 ? 1.0f : 0.0f) && e[1] == (f == 5 ? 1.0f : 0.0f);
        events = events && memcmp(e, d.inputs + (size_t)f * d.inputCount + 105, 8) == 0;
    }
    CHECK(events, "the events' states, for 0.1 s after each");
    CHECK(d.pairs[0] == 0 && d.pairs[8] == 8 && d.pairs[9] == 10 && d.pairs[18] == 19,
          "pairs within each entry");
    maniTrainDataRelease(&kAllocator, &d);
}

// The sway: bone 0 along x at 1 m/s and turning about a tilted axis
// (x and y), its twist about y taken as root motion; bone 1 a quarter
// turn about z and stretching along y; bone 2 growing from scale 1 to 2
// over its second, looping; a float track speed from 0 to 1, a left at
// 0.95 s.
static _Alignas(16) uint8_t s_swayFile[8192];
static _Alignas(16) uint8_t s_swayDatabaseFile[1 << 15];
static maniClip s_sway;
static maniMotionDatabase s_swayDatabase;

static void OpenSway(void)
{
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kAlong[6] = {0, 1, 0, 1, 1, 0};
    // A sixth of a turn about (1, 1, 0) / sqrt(2): sin 30 deg / sqrt(2).
    static const float kTilt[8] = {0, 0, 0, 1, 0.35355339f, 0.35355339f, 0, 0.8660254f};
    static const float kTurn[8] = {0, 0, 0, 1, 0, 0, 0.70710678f, 0.70710678f};
    static const float kStretch[6] = {1, 1, 1, 1, 2, 1};
    static const float kGrow[6] = {1, 1, 1, 2, 2, 2};
    static const float kSpeed[2] = {0.0f, 1.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const maniRootMotionSource kTwist = {0, 0, maniRootRotationTwist, 1, 0, 0, 0, 0};
    const maniBoneTrackSource tracks[3] = {
        {0,
         0,
         {kTimes, kAlong, 2, maniInterpolationLinear},
         {kTimes, kTilt, 2, maniInterpolationLinear},
         kNone},
        {1,
         0,
         kNone,
         {kTimes, kTurn, 2, maniInterpolationLinear},
         {kTimes, kStretch, 2, maniInterpolationLinear}},
        {2, 0, kNone, kNone, {kTimes, kGrow, 2, maniInterpolationLinear}}};
    const maniFloatTrackSource speed = {
        "speed", 5, 0, {kTimes, kSpeed, 2, maniInterpolationLinear}};
    static const maniEventSource kLate = {0.95f, 0, "left", 4, 0};
    maniClipSource source = {.skeleton = &s_skeleton,
                             .boneTracks = tracks,
                             .boneTrackCount = 3,
                             .floatTracks = &speed,
                             .floatTrackCount = 1,
                             .events = &kLate,
                             .eventCount = 1,
                             .loop = maniLoopRepeat,
                             .sampleRate = 10.0f,
                             .rootMotion = &kTwist};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_swayFile, sizeof s_swayFile, &size) == mani_success &&
              maniOpenClip(s_swayFile, size, &s_sway).result == mani_success,
          "the sway");
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channel = {maniMatchBonePosition, 2, 0, 1, kNow, NULL, 0, 1.0f};
    const maniClip* clips[2] = {&s_clips[0], &s_sway};
    const maniMatchEntrySource entries[2] = {{0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 0u},
                                             {1, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 0u}};
    maniMotionDatabaseSource database = {&s_skeleton, clips, &channel,       entries,        2,   1,
                                         2,           10.0f, maniMatchAxisY, maniMatchAxisZ, NULL};
    CHECK(maniBuildMotionDatabase(builder, &database).result == mani_success &&
              maniCopyBuilt(builder, s_swayDatabaseFile, sizeof s_swayDatabaseFile, &size) ==
                  mani_success &&
              maniOpenMotionDatabase(s_swayDatabaseFile, size, &s_swayDatabase).result ==
                  mani_success,
          "the walk and the sway");
    maniDestroyBuilder(builder);
}

static bool Near3(const float* v, float x, float y, float z)
{
    return fabsf(v[0] - x) <= 1e-4f && fabsf(v[1] - y) <= 1e-4f && fabsf(v[2] - z) <= 1e-4f;
}

static void TestSway(void)
{
    static const maniLearnedNameSource kSpeed[1] = {{"speed", 5, 0}};
    const maniClip* clips[2] = {&s_clips[0], &s_sway};
    const maniTrainDataSource source = {
        &s_skeleton, &s_swayDatabase, clips, kSpeed, kEvents, 1, 2, 0.1f, 0};
    maniTrainData d;
    CHECK(maniTrainDataBuild(&kAllocator, &source, &d), "the data");
    CHECK(d.frameCount == 20 && d.scaledBoneCount == 2 && d.scaledBones[0] == 1 &&
              d.scaledBones[1] == 2 && d.outputCount == 27 + 6 + 6 + 1 + 2 &&
              Near3(d.scales, 1, 1, 1),
          "bones 1 (along y only) and 2 scaled; bone 0's scale kept");
    // The sway at 0.3 s (frame 13): scales 1.3, speed 0.3; bone 0 at 1
    // m/s, bone 1 turning at a quarter turn a second, locally.
    const float* o = d.outputs + 13 * (size_t)d.outputCount;
    const float* in = d.inputs + 13 * (size_t)d.inputCount;
    const uint32_t* g = d.groupStarts;
    CHECK(Near3(o + 27, 1, 1.3f, 1) && Near3(o + 30, 1.3f, 1.3f, 1.3f) &&
              fabsf(o[39] - 0.3f) <= 1e-6f &&
              Near3(in + g[maniTrainGroupScale] + 3, 1.3f, 1.3f, 1.3f),
          "the scales and the float track");
    static _Alignas(32) uint8_t sides[2][4096];
    maniPose* side[2] = {NULL, NULL};
    for (uint32_t k = 0; k < 2; ++k)
    {
        CHECK(maniInitPose(&s_skeleton, sides[k], sizeof sides[k], &side[k]) == mani_success &&
                  maniSampleClip(&s_sway, &s_skeleton, 0.2f + 0.2f * (float)k, side[k]) ==
                      mani_success,
              "the sway either side");
    }
    maniVec3 moved =
        maniVec3Sub(maniPoseLocal(side[1], 0).translation, maniPoseLocal(side[0], 0).translation);
    CHECK(Near3(in + g[maniTrainGroupVelocity], moved.x / 0.2f, moved.y / 0.2f, moved.z / 0.2f) &&
              Near3(in + g[maniTrainGroupSpin] + 3, 0, 0, 1.5707964f),
          "local velocities");
    // The root's turn over the frame's two neighbours, as the clip's
    // root motion gives it: about y only, so differences of rotation
    // vectors are the move's.
    maniRootMove move;
    CHECK(maniClipRootMotion(&s_sway, 0.2f, 0.2f, &move) == mani_success, "the root's move");
    maniVec3 turn = maniQuatToRotationVector(move.rotation);
    CHECK(Near3(o + 36, turn.x / 0.2f, turn.y / 0.2f, turn.z / 0.2f) && fabsf(o[37]) > 0.1f,
          "the root turning");
    // Character space: bone 2 (the arm under the spine under the hips)
    // from bone 0, as its model matrix gives it.
    static _Alignas(32) uint8_t memory[4096];
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, memory, sizeof memory, &pose) == mani_success &&
              maniSampleClip(&s_sway, &s_skeleton, 0.3f, pose) == mani_success &&
              maniComputeModel(pose, &s_skeleton) == mani_success,
          "the sway at 0.3 s");
    maniAffine root = maniPoseModel(pose, 0);
    maniAffine arm = maniPoseModel(pose, 2);
    maniQuat undo = maniQuatConjugate(maniPoseLocal(pose, 0).rotation);
    CHECK(fabsf(undo.w) < 0.9999f, "bone 0 turned");
    maniVec3 at = maniQuatRotate(
        undo, maniVec3Make(arm.m[9] - root.m[9], arm.m[10] - root.m[10], arm.m[11] - root.m[11]));
    maniVec3 x = maniQuatRotate(undo, maniVec3Make(arm.m[0], arm.m[1], arm.m[2]));
    CHECK(Near3(in + g[maniTrainGroupCharacterPosition] + 6, at.x, at.y, at.z) &&
              Near3(in + g[maniTrainGroupCharacterColumns] + 12, x.x, x.y, x.z),
          "character space");
    // A looping entry's event near its end: the left at 0.95 s lasts
    // past the wrap into the sway's first frame; the walk has no speed.
    const float* first = d.outputs + 10 * (size_t)d.outputCount + 40;
    CHECK(first[0] == 1.0f && first[1] == 0.0f && d.outputs[39] == 0.0f,
          "an event across the wrap; a track a clip lacks is 0");
    maniTrainDataRelease(&kAllocator, &d);
}

static const maniTrainDecompressShape kShape = {4, 16, 1, 1, 4, 0};

// A host that runs ranges of one item, last first.
static void* Backwards(maniTaskFn* task, int32_t count, int32_t minRange, void* context, void* user)
{
    (void)minRange;
    (void)user;
    for (int32_t k = count; k-- > 0;)
    {
        task(k, k + 1, context);
    }
    return NULL;
}

// The loss of chunk 0 (pairs 0 and 12) at the stage's parameters.
static float ChunkLoss(maniTrainDecompress* t)
{
    uint64_t own = t->compressor.parameterCount;
    maniTrainTranspose(&t->compressor, t->parameters, t->transposed);
    maniTrainTranspose(&t->decompressor, t->parameters + own, t->transposed + own);
    t->chunks[0].pairs[0] = 0;
    t->chunks[0].pairs[1] = 12;
    maniTrainDecompressRun(t, &t->chunks[0], MANI_TRAIN_PAIRS);
    return t->chunks[0].loss;
}

static void TestTrain(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 1234);
    maniTrainDecompress t;
    CHECK(maniTrainDecompressInit(&kAllocator, &d, &kShape, &r, &t), "the stage");
    double first = 0.0;
    double last = 0.0;
    for (uint64_t step = 1; step <= 600; ++step)
    {
        float loss = maniTrainDecompressStep(&t, &d, &r, NULL, step, 0.002f);
        first += step <= 30 ? (double)loss : 0.0;
        last += step > 570 ? (double)loss : 0.0;
    }
    CHECK(last < 0.3 * first, "the loss falls to under 0.3 of its start");
    static float latents[21 * 4];
    maniTrainLatents(&t, &d, latents);
    bool finite = true;
    for (uint32_t k = 0; k < 21 * 4; ++k)
    {
        finite = finite && fabsf(latents[k]) < 1e30f;
    }
    CHECK(finite, "every frame's latent values");
    (void)ChunkLoss(&t);
    const float* z =
        t.chunks[0].compressorValues + maniTrainLayerAt(&t.compressor, t.compressor.layerCount, 4);
    CHECK(memcmp(z, latents, 4 * sizeof(float)) == 0, "frame 0's latent values as a chunk's");
    maniTrainDecompressRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

static void TestHosts(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainDecompress t[2];
    const maniTaskHooks hooks[2] = {{NULL, NULL, NULL}, {Backwards, NULL, NULL}};
    for (uint32_t h = 0; h < 2; ++h)
    {
        maniTrainRandom r;
        maniTrainSeed(&r, 77);
        CHECK(maniTrainDecompressInit(&kAllocator, &d, &kShape, &r, &t[h]), "the stage");
        for (uint64_t step = 1; step <= 20; ++step)
        {
            (void)maniTrainDecompressStep(&t[h], &d, &r, &hooks[h], step, 0.001f);
        }
    }
    CHECK(memcmp(t[0].parameters, t[1].parameters, sizeof(float) * t[0].count) == 0 &&
              memcmp(t[0].m, t[1].m, sizeof(float) * t[0].count) == 0,
          "the same bits on any host");
    maniTrainDecompressRelease(&kAllocator, &t[0]);
    maniTrainDecompressRelease(&kAllocator, &t[1]);
    maniTrainDataRelease(&kAllocator, &d);
}

// A rotation from two columns, made orthonormal from the first, in
// binary64: Shepperd's method.
static maniQuat FromColumns(const float* x, const float* y)
{
    double c[3][3];
    double xd[3] = {(double)x[0], (double)x[1], (double)x[2]};
    double yd[3] = {(double)y[0], (double)y[1], (double)y[2]};
    double lx = sqrt(xd[0] * xd[0] + xd[1] * xd[1] + xd[2] * xd[2]);
    for (uint32_t i = 0; i < 3; ++i)
    {
        c[0][i] = xd[i] / lx;
    }
    double u[3] = {c[0][1] * yd[2] - c[0][2] * yd[1], c[0][2] * yd[0] - c[0][0] * yd[2],
                   c[0][0] * yd[1] - c[0][1] * yd[0]};
    double lu = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    for (uint32_t i = 0; i < 3; ++i)
    {
        c[2][i] = u[i] / lu;
    }
    c[1][0] = c[2][1] * c[0][2] - c[2][2] * c[0][1];
    c[1][1] = c[2][2] * c[0][0] - c[2][0] * c[0][2];
    c[1][2] = c[2][0] * c[0][1] - c[2][1] * c[0][0];
    double trace = c[0][0] + c[1][1] + c[2][2];
    double q[4];
    if (trace > 0.0)
    {
        q[0] = c[1][2] - c[2][1];
        q[1] = c[2][0] - c[0][2];
        q[2] = c[0][1] - c[1][0];
        q[3] = 1.0 + trace;
    }
    else if (c[0][0] >= c[1][1] && c[0][0] >= c[2][2])
    {
        q[0] = 1.0 + c[0][0] - c[1][1] - c[2][2];
        q[1] = c[0][1] + c[1][0];
        q[2] = c[2][0] + c[0][2];
        q[3] = c[1][2] - c[2][1];
    }
    else if (c[1][1] >= c[2][2])
    {
        q[0] = c[0][1] + c[1][0];
        q[1] = 1.0 + c[1][1] - c[0][0] - c[2][2];
        q[2] = c[1][2] + c[2][1];
        q[3] = c[2][0] - c[0][2];
    }
    else
    {
        q[0] = c[2][0] + c[0][2];
        q[1] = c[1][2] + c[2][1];
        q[2] = 1.0 + c[2][2] - c[0][0] - c[1][1];
        q[3] = c[0][1] - c[1][0];
    }
    double l = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return (maniQuat){(float)(q[0] / l), (float)(q[1] / l), (float)(q[2] / l), (float)(q[3] / l)};
}

// A frame's character values from outputs y, through a pose the library
// composes: per bone the translation from bone 0, then two columns.
static void Character(const maniTrainData* d, const float* y, double* out)
{
    static _Alignas(32) uint8_t memory[4096];
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, memory, sizeof memory, &pose) == mani_success, "a pose");
    uint32_t scaled = 0;
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        const float* o = y + 9 * b;
        bool given = scaled < d->scaledBoneCount && d->scaledBones[scaled] == b;
        const float* scale = given ? y + 9 * d->boneCount + 3 * scaled++ : d->scales + 3 * b;
        maniPoseSetLocal(pose, b,
                         (maniTransform){maniVec3Make(o[0], o[1], o[2]), FromColumns(o + 3, o + 6),
                                         maniVec3Make(scale[0], scale[1], scale[2])});
    }
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "composed");
    maniQuat undo = maniQuatConjugate(maniPoseLocal(pose, 0).rotation);
    maniAffine root = maniPoseModel(pose, 0);
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        maniAffine m = maniPoseModel(pose, b);
        maniVec3 v[3] = {
            maniVec3Make(m.m[9] - root.m[9], m.m[10] - root.m[10], m.m[11] - root.m[11]),
            maniVec3Make(m.m[0], m.m[1], m.m[2]), maniVec3Make(m.m[3], m.m[4], m.m[5])};
        for (uint32_t k = 0; k < 3; ++k)
        {
            maniVec3 r = maniQuatRotate(undo, v[k]);
            out[9 * b + 3 * k] = (double)r.x;
            out[9 * b + 3 * k + 1] = (double)r.y;
            out[9 * b + 3 * k + 2] = (double)r.z;
        }
    }
}

// The character terms of chunk 0: translations (15) and columns (5),
// and their velocities across each pair (2, 0.75).
static double CharacterTerms(const maniTrainData* d, const uint32_t* f, double y[4][64])
{
    double q[4][27];
    double bones = (double)d->boneCount;
    for (uint32_t s = 0; s < 4; ++s)
    {
        float row[64];
        for (uint32_t j = 0; j < d->outputCount; ++j)
        {
            row[j] = (float)y[s][j];
        }
        Character(d, row, q[s]);
    }
    double loss = 0.0;
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        for (uint32_t i = 0; i < 9; ++i)
        {
            bool position = i < 3;
            uint32_t at = d->groupStarts[position ? maniTrainGroupCharacterPosition
                                                  : maniTrainGroupCharacterColumns] +
                          (position ? 3 * b + i : 6 * b + i - 3);
            double w = position ? 15.0 / (4 * 3 * bones) : 5.0 / (4 * 6 * bones);
            double v = position ? 2.0 / (2 * 3 * bones) : 0.75 / (2 * 6 * bones);
            double target[4];
            for (uint32_t s = 0; s < 4; ++s)
            {
                target[s] = (double)d->inputs[(size_t)f[s] * d->inputCount + at];
                loss += w * fabs(q[s][9 * b + i] - target[s]);
            }
            for (uint32_t p = 0; p < 2; ++p)
            {
                double moved = q[2 * p + 1][9 * b + i] - q[2 * p][9 * b + i];
                double want = target[2 * p + 1] - target[2 * p];
                loss += v * fabs((moved - want) / (double)d->frameSeconds);
            }
        }
    }
    return loss;
}

// Holden's loss of chunk 0, in binary64 from the chunk's values: each
// term the mean over its values times its weight.
static double Reference(const maniTrainDecompress* t, const maniTrainData* d)
{
    const maniTrainDecompressChunk* c = &t->chunks[0];
    uint32_t f[4] = {d->pairs[0], d->pairs[0] + 1, d->pairs[12], d->pairs[12] + 1};
    uint32_t o = d->outputCount;
    uint32_t latent = 4;
    double bones = 3.0;
    double dt = (double)d->frameSeconds;
    const float* yn =
        c->decompressorValues + maniTrainLayerAt(&t->decompressor, t->decompressor.layerCount, 4);
    const float* z =
        c->compressorValues + maniTrainLayerAt(&t->compressor, t->compressor.layerCount, 4);
    double y[4][64];
    double loss = 0.0;
    for (uint32_t s = 0; s < 4; ++s)
    {
        for (uint32_t j = 0; j < o; ++j)
        {
            y[s][j] = (double)(yn[s * o + j] * t->outputScales[j] + t->outputOffsets[j]);
            double diff = fabs(y[s][j] - (double)d->outputs[(size_t)f[s] * o + j]);
            uint32_t scaled = 3 * d->scaledBoneCount;
            uint32_t rest = j - 27 - scaled;
            double w = j < 27 ? (j % 9 < 3 ? 75.0 / (4 * 3 * bones) : 10.0 / (4 * 6 * bones))
                       : j < 27 + scaled ? 10.0 / (4.0 * scaled)
                       : rest < 6        ? 2.0 / (4 * 3)
                                         : 2.0 / (4.0 * (d->floatTrackCount + d->eventCount));
            loss += w * diff;
        }
        for (uint32_t k = 0; k < latent; ++k)
        {
            double x = (double)z[s * latent + k];
            loss += 0.1 / (4 * latent) * fabs(x) + 0.1 / (4 * latent) * x * x;
        }
    }
    for (uint32_t p = 0; p < 2; ++p)
    {
        for (uint32_t j = 0; j < 27; ++j)
        {
            double want = (double)d->outputs[(size_t)f[2 * p + 1] * o + j] -
                          (double)d->outputs[(size_t)f[2 * p] * o + j];
            double diff = fabs((y[2 * p + 1][j] - y[2 * p][j] - want) / dt);
            loss += (j % 9 < 3 ? 10.0 / (2 * 3 * bones) : 1.75 / (2 * 6 * bones)) * diff;
        }
        for (uint32_t k = 0; k < latent; ++k)
        {
            double change =
                ((double)z[(2 * p + 1) * latent + k] - (double)z[2 * p * latent + k]) / dt;
            loss += 0.01 / (2 * latent) * fabs(change);
        }
    }
    return loss + CharacterTerms(d, f, y);
}

// The latent values' own gradient (the chunk's gradient by them less the
// Decompressor's by its latent inputs) as their terms' formula.
static bool LatentTermsMatch(const maniTrainDecompress* t, const maniTrainData* d)
{
    const maniTrainDecompressChunk* c = &t->chunks[0];
    const float* z =
        c->compressorValues + maniTrainLayerAt(&t->compressor, t->compressor.layerCount, 4);
    uint32_t n = d->featureCount;
    double dt = (double)d->frameSeconds;
    bool ok = true;
    for (uint32_t s = 0; s < 4; ++s)
    {
        for (uint32_t k = 0; k < 4; ++k)
        {
            double x = (double)z[s * 4 + k];
            double want = 0.1 / 16.0 * (x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0)) + 0.2 / 16.0 * x;
            uint32_t pair = s / 2;
            double change = ((double)z[(2 * pair + 1) * 4 + k] - (double)z[2 * pair * 4 + k]) / dt;
            double sign = change > 0.0 ? 1.0 : (change < 0.0 ? -1.0 : 0.0);
            want += (s % 2 == 1 ? 1.0 : -1.0) * 0.01 / 8.0 * sign / dt;
            double got = (double)c->latentGradient[s * 4 + k] -
                         (double)c->inputGradient[s * (n + 4) + n + k];
            ok = ok && fabs(got - want) <= 1e-6;
        }
    }
    return ok;
}

// The Decompressor's outputs' offsets and scales are each value's mean
// and deviation; the Compressor's inputs' scales their group's.
static bool NormalizationMatches(const maniTrainDecompress* t, const maniTrainData* d)
{
    bool ok = true;
    for (uint32_t j = 0; j < d->outputCount; ++j)
    {
        double sum = 0.0;
        double squares = 0.0;
        for (uint32_t f = 0; f < d->frameCount; ++f)
        {
            double x = (double)d->outputs[(size_t)f * d->outputCount + j];
            sum += x;
            squares += x * x;
        }
        double mean = sum / d->frameCount;
        double deviation = sqrt(fmax(squares / d->frameCount - mean * mean, 0.0));
        ok = ok && fabs((double)t->outputOffsets[j] - mean) <= 1e-5 &&
             fabs((double)t->outputScales[j] - deviation) <= 1e-5;
    }
    uint32_t first = d->groupStarts[maniTrainGroupPosition];
    uint32_t end = d->groupStarts[maniTrainGroupPosition + 1];
    double sum = 0.0;
    double squares = 0.0;
    for (uint32_t f = 0; f < d->frameCount; ++f)
    {
        for (uint32_t j = first; j < end; ++j)
        {
            double x = (double)d->inputs[(size_t)f * d->inputCount + j];
            sum += x;
            squares += x * x;
        }
    }
    double count = (double)d->frameCount * (end - first);
    double deviation = sqrt(squares / count - (sum / count) * (sum / count));
    for (uint32_t j = first; j < end; ++j)
    {
        ok = ok && fabs((double)t->inputScales[j] - deviation) <= 1e-5;
    }
    return ok;
}

static void CheckChunk(const maniTrainData* d, const char* name)
{
    maniTrainRandom r;
    maniTrainSeed(&r, 5);
    maniTrainDecompress t;
    CHECK(maniTrainDecompressInit(&kAllocator, d, &kShape, &r, &t), "the stage");
    CHECK(NormalizationMatches(&t, d), "the normalization");
    float loss = ChunkLoss(&t);
    double want = Reference(&t, d);
    if (!(fabs((double)loss - want) <= 1e-4 * want))
    {
        printf("  loss %.9g, Holden's terms %.9g\n", (double)loss, want);
    }
    CHECK(fabs((double)loss - want) <= 1e-4 * want, name);
    CHECK(LatentTermsMatch(&t, d), "the latent terms' gradient");
    static float gradient[8192];
    CHECK(t.count <= 8192, "room");
    memcpy(gradient, t.chunks[0].gradients, sizeof(float) * t.count);
    uint32_t checked = 0;
    uint32_t near = 0;
    for (uint64_t k = 0; k < t.count; k += 7)
    {
        if (fabsf(gradient[k]) < 0.05f)
        {
            continue;
        }
        float kept = t.parameters[k];
        float h = 2e-3f;
        t.parameters[k] = kept + h;
        double up = (double)ChunkLoss(&t);
        t.parameters[k] = kept - h;
        double down = (double)ChunkLoss(&t);
        t.parameters[k] = kept;
        double slope = (up - down) / (2.0 * (double)h);
        checked += 1;
        near += fabs((double)gradient[k] - slope) <= 0.02 + 0.05 * fabs(slope) ? 1u : 0u;
    }
    CHECK(checked >= 20 && near == checked,
          "a chunk's gradient as central differences of its loss");
    maniTrainDecompressRelease(&kAllocator, &t);
}

static void TestGradient(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    CheckChunk(&d, "a chunk's loss as Holden's terms");
    maniTrainDataRelease(&kAllocator, &d);
    static const maniLearnedNameSource kSpeed[1] = {{"speed", 5, 0}};
    const maniClip* clips[2] = {&s_clips[0], &s_sway};
    const maniTrainDataSource source = {
        &s_skeleton, &s_swayDatabase, clips, kSpeed, kEvents, 1, 2, 0.1f, 0};
    CHECK(maniTrainDataBuild(&kAllocator, &source, &d), "the sway's data");
    CheckChunk(&d, "a chunk's loss as Holden's terms, scales and a track among them");
    maniTrainDataRelease(&kAllocator, &d);
    maniTrainDecompressShape odd = kShape;
    maniTrainDecompress t;
    maniTrainRandom r;
    maniTrainSeed(&r, 5);
    CHECK(Data(&d), "the data");
    odd.batchPairs = 3;
    CHECK(!maniTrainDecompressInit(&kAllocator, &d, &odd, &r, &t), "a batch of half a chunk");
    odd.batchPairs = 0;
    CHECK(!maniTrainDecompressInit(&kAllocator, &d, &odd, &r, &t), "an empty batch");
    maniTrainDataRelease(&kAllocator, &d);
}

// The character pass's gradient by its outputs against central
// differences: three bones in a chain, bones 1 and 2 scaled, random
// outputs and a random linear loss of the character values.
static void TestKinematics(void)
{
    static uint32_t parents[3] = {MANI_NO_BONE, 0, 1};
    static uint32_t scaled[2] = {1, 2};
    static float scales[9] = {1.5f, 0.75f, 1.25f, 1, 1, 1, 1, 1, 1};
    maniTrainData d = {.boneCount = 3,
                       .scaledBoneCount = 2,
                       .scaledBones = scaled,
                       .scales = scales,
                       .parents = parents};
    maniTrainRandom r;
    maniTrainSeed(&r, 21);
    float y[33];
    float c[27];
    for (uint32_t k = 0; k < 33; ++k)
    {
        y[k] = 2.0f * maniTrainUniform(&r) - 1.0f;
    }
    for (uint32_t k = 27; k < 33; ++k)
    {
        y[k] += 1.5f;
    }
    for (uint32_t k = 0; k < 27; ++k)
    {
        c[k] = 2.0f * maniTrainUniform(&r) - 1.0f;
    }
    float work[3 * MANI_TRAIN_KINEMATIC_FLOATS];
    float character[27];
    float gradient[33];
    float scratch[36];
    memset(gradient, 0, sizeof gradient);
    maniTrainKinematics(&d, y, work, character);
    maniTrainKinematicsBackward(&d, work, c, gradient, scratch);
    bool near = true;
    for (uint32_t k = 0; k < 33; ++k)
    {
        float kept = y[k];
        double sums[2];
        for (uint32_t side = 0; side < 2; ++side)
        {
            y[k] = kept + (side == 0 ? 3e-3f : -3e-3f);
            maniTrainKinematics(&d, y, work, character);
            sums[side] = 0.0;
            for (uint32_t i = 0; i < 27; ++i)
            {
                sums[side] += (double)c[i] * (double)character[i];
            }
        }
        y[k] = kept;
        double step = (double)(kept + 3e-3f) - (double)(kept - 3e-3f);
        double slope = (sums[0] - sums[1]) / step;
        near = near && fabs((double)gradient[k] - slope) <= 2e-3 * (1.0 + fabs(slope));
    }
    CHECK(near, "the character pass's gradient as central differences");
}

int main(void)
{
    Open();
    OpenDatabase();
    OpenBones();
    OpenSway();
    TestData();
    TestSway();
    TestTrain();
    TestHosts();
    TestGradient();
    TestKinematics();
    return s_failures == 0 ? 0 : 1;
}
