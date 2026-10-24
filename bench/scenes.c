// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark's scenes (scenes.h): each built by its builder, copied
// into memory of its own and opened there, as a host loads its files.
// The memory is never given back: the scenes last the program.

#include "scenes.h"

#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/matchbuild.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A clip's samples: 10 s at 30 a second, both ends.
#define RATE    30.0f
#define SAMPLES 301u

// Bones by place in the rig, depth first.
enum
{
    boneHips = 0,
    boneSpine1 = 2,
    boneChest = 3,
    boneNeck = 4,
    boneHead = 5,
    boneHairLeft = 9,
    boneHairRight = 15,
    boneHairLength = 6,
    boneFingersLeft = 25,
    boneFingersRight = 38,
    boneFingerCount = 9,
    boneThigh = 47,
    boneShin = 48,
    boneFoot = 49,
    boneFootRight = 53,
    boneTail = 55,
    boneTailEnd = 63,
};

static void* s_blocks[64];
static uint32_t s_blockCount;

// size bytes aligned to 64, kept for the program's life.
static void* Take(size_t size)
{
    if (s_blockCount == sizeof s_blocks / sizeof s_blocks[0])
    {
        return NULL;
    }
    unsigned char* block = malloc(size + 64);
    if (block == NULL)
    {
        return NULL;
    }
    s_blocks[s_blockCount++] = block;
    return block + (64 - (uintptr_t)block % 64);
}

static bool Fail(const char* what)
{
    fprintf(stderr, "bench: %s failed\n", what);
    return false;
}

// Copies a builder's file into memory of its own: false when the build
// was refused or there is no memory.
static bool Keep(maniBuilder* builder, maniBuildResult built, const char* what, void** file,
                 size_t* size)
{
    if (built.result != mani_success)
    {
        fprintf(stderr, "bench: %s refused: result %d, reason %u, item %u\n", what,
                (int)built.result, (unsigned)built.reason, (unsigned)built.item);
        return false;
    }
    if (maniCopyBuilt(builder, NULL, 0, size) == mani_errorInvalid)
    {
        return Fail(what);
    }
    *file = Take(*size);
    if (*file == NULL || maniCopyBuilt(builder, *file, *size, NULL) != mani_success)
    {
        return Fail(what);
    }
    return true;
}

static bool Opened(maniFileResult opened, const char* what)
{
    return opened.result == mani_success || Fail(what);
}

static bool IsFinger(uint32_t bone)
{
    return (bone >= boneFingersLeft && bone < boneFingersLeft + boneFingerCount) ||
           (bone >= boneFingersRight && bone < boneFingersRight + boneFingerCount);
}

// The rig, depth first, its bones' rest offsets in meters.
typedef struct Rig
{
    maniBoneSource bones[BENCH_BONES];
    char names[BENCH_BONES][48];
    maniVec3 model[BENCH_BONES];
    uint32_t count;
} Rig;

static uint32_t Add(Rig* rig, const char* name, int32_t parent, float x, float y, float z)
{
    uint32_t bone = rig->count++;
    snprintf(rig->names[bone], sizeof rig->names[bone], "%s", name);
    rig->bones[bone] = (maniBoneSource){rig->names[bone],
                                        (uint32_t)strlen(rig->names[bone]),
                                        parent,
                                        {{x, y, z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    maniVec3 base = parent < 0 ? (maniVec3){0.0f, 0.0f, 0.0f} : rig->model[parent];
    rig->model[bone] = (maniVec3){base.x + x, base.y + y, base.z + z};
    return bone;
}

// A chain of count bones under parent, the first at first from it and
// each next at step from the one before.
static void Chain(Rig* rig, const char* name, uint32_t parent, uint32_t count, maniVec3 first,
                  maniVec3 step)
{
    char each[48];
    for (uint32_t k = 0; k < count; ++k)
    {
        snprintf(each, sizeof each, "%s%u", name, (unsigned)k);
        maniVec3 at = k == 0 ? first : step;
        parent = Add(rig, each, (int32_t)parent, at.x, at.y, at.z);
    }
}

static void Arm(Rig* rig, const char* side, float sign)
{
    char name[32];
    snprintf(name, sizeof name, "shoulder%s", side);
    uint32_t shoulder = Add(rig, name, boneChest, 0.05f * sign, 0.12f, 0.0f);
    snprintf(name, sizeof name, "upperArm%s", side);
    uint32_t upper = Add(rig, name, (int32_t)shoulder, 0.12f * sign, 0.0f, 0.0f);
    snprintf(name, sizeof name, "forearm%s", side);
    uint32_t fore = Add(rig, name, (int32_t)upper, 0.28f * sign, 0.0f, 0.0f);
    snprintf(name, sizeof name, "hand%s", side);
    uint32_t hand = Add(rig, name, (int32_t)fore, 0.25f * sign, 0.0f, 0.0f);
    for (uint32_t f = 0; f < 3; ++f)
    {
        snprintf(name, sizeof name, "finger%s%u_", side, (unsigned)f);
        Chain(rig, name, hand, 3, (maniVec3){0.08f * sign, 0.0f, 0.03f * ((float)f - 1.0f)},
              (maniVec3){0.035f * sign, 0.0f, 0.0f});
    }
}

static void Leg(Rig* rig, const char* side, float sign)
{
    char name[32];
    snprintf(name, sizeof name, "thigh%s", side);
    uint32_t thigh = Add(rig, name, boneHips, 0.1f * sign, -0.05f, 0.0f);
    snprintf(name, sizeof name, "shin%s", side);
    uint32_t shin = Add(rig, name, (int32_t)thigh, 0.0f, -0.45f, 0.01f);
    snprintf(name, sizeof name, "foot%s", side);
    uint32_t foot = Add(rig, name, (int32_t)shin, 0.0f, -0.42f, -0.01f);
    snprintf(name, sizeof name, "toe%s", side);
    Add(rig, name, (int32_t)foot, 0.0f, -0.06f, 0.12f);
}

static void MakeRig(Rig* rig)
{
    rig->count = 0;
    Add(rig, "hips", -1, 0.0f, 1.0f, 0.0f);
    Add(rig, "spine", boneHips, 0.0f, 0.1f, 0.0f);
    Add(rig, "spine1", 1, 0.0f, 0.15f, 0.0f);
    Add(rig, "chest", boneSpine1, 0.0f, 0.15f, 0.0f);
    Add(rig, "neck", boneChest, 0.0f, 0.2f, 0.0f);
    Add(rig, "head", boneNeck, 0.0f, 0.1f, 0.0f);
    Add(rig, "jaw", boneHead, 0.0f, 0.02f, 0.06f);
    Add(rig, "eyeL", boneHead, 0.03f, 0.08f, 0.08f);
    Add(rig, "eyeR", boneHead, -0.03f, 0.08f, 0.08f);
    Chain(rig, "hairL", boneHead, boneHairLength, (maniVec3){0.06f, 0.05f, -0.08f},
          (maniVec3){0.0f, -0.06f, -0.01f});
    Chain(rig, "hairR", boneHead, boneHairLength, (maniVec3){-0.06f, 0.05f, -0.08f},
          (maniVec3){0.0f, -0.06f, -0.01f});
    Arm(rig, "L", 1.0f);
    Arm(rig, "R", -1.0f);
    Leg(rig, "L", 1.0f);
    Leg(rig, "R", -1.0f);
    Chain(rig, "tail", boneHips, boneTailEnd - boneTail + 1, (maniVec3){0.0f, 0.0f, -0.1f},
          (maniVec3){0.0f, 0.0f, -0.08f});
}

static bool BuildSkeleton(maniBuilder* builder, const Rig* rig, maniSkeleton* skeleton)
{
    maniSkeletonSource source = {rig->bones, rig->count, 0};
    void* file = NULL;
    size_t size = 0;
    if (!Keep(builder, maniBuildSkeleton(builder, &source), "the skeleton", &file, &size) ||
        !Opened(maniOpenSkeleton(file, size, skeleton), "the skeleton"))
    {
        return false;
    }
    // The builder keeps a depth-first order as it is.
    return (maniSkeletonBoneCount(skeleton) == BENCH_BONES &&
            maniSkeletonFindBone(skeleton, "tail8", 5) == boneTailEnd &&
            maniSkeletonFindBone(skeleton, "footL", 5) == boneFoot) ||
           Fail("the rig's order");
}

// A turn of a about x then b about z (y for the hips, its heading).
static void Turn(double a, double b, bool heading, float* out)
{
    double s1 = sin(a / 2.0);
    double c1 = cos(a / 2.0);
    double s2 = sin(b / 2.0);
    double c2 = cos(b / 2.0);
    out[0] = (float)(s1 * c2);
    out[1] = (float)(heading ? c1 * s2 : -s1 * s2);
    out[2] = (float)(heading ? s1 * s2 : c1 * s2);
    out[3] = (float)(c1 * c2);
}

// Clip c: each bone but the fingers turns at its own frequencies, all
// whole numbers of tenths of a hertz so the clip loops cleanly; the
// fingers hold a curl; the hips walk ahead along z at a speed of the
// clip's, swaying and bobbing, and turn their heading, which root motion
// takes out.
static bool BuildClip(maniBuilder* builder, const maniSkeleton* skeleton, uint32_t c, float* keys,
                      maniClip* clip, void** fileOut, size_t* sizeOut)
{
    static float times[SAMPLES];
    static const float kHold[2] = {0.0f, BENCH_DURATION};
    static const float kCurl[8] = {0.0f, 0.0f, 0.3f, 0.95393920f, 0.0f, 0.0f, 0.3f, 0.95393920f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    const double tau = 6.283185307179586;
    maniBoneTrackSource tracks[BENCH_BONES];
    float* walk = keys + (size_t)BENCH_BONES * SAMPLES * 4;
    for (uint32_t s = 0; s < SAMPLES; ++s)
    {
        times[s] = (float)s / RATE;
    }
    for (uint32_t b = 0; b < BENCH_BONES; ++b)
    {
        tracks[b] = (maniBoneTrackSource){b, 0, kNone, kNone, kNone};
        if (IsFinger(b))
        {
            tracks[b].rotation = (maniChannelSource){kHold, kCurl, 2, maniInterpolationLinear};
            continue;
        }
        double f = 0.1 * (double)(5 + (c + b) % 7);
        double g = 0.1 * (double)(3 + (3 * c + b) % 5);
        double phase = 0.37 * (double)b + 0.11 * (double)c;
        float* turns = keys + (size_t)b * SAMPLES * 4;
        for (uint32_t s = 0; s < SAMPLES; ++s)
        {
            double t = (double)s / (double)RATE;
            Turn(0.3 * sin(tau * f * t + phase), 0.2 * sin(tau * g * t + 2.0 * phase),
                 b == boneHips, turns + 4 * s);
        }
        tracks[b].rotation = (maniChannelSource){times, turns, SAMPLES, maniInterpolationLinear};
    }
    double speed = 0.8 + 0.2 * (double)c;
    for (uint32_t s = 0; s < SAMPLES; ++s)
    {
        double t = (double)s / (double)RATE;
        walk[3 * s] = (float)(0.05 * sin(tau * 0.5 * t));
        walk[3 * s + 1] = (float)(1.0 + 0.03 * sin(tau * 2.0 * t));
        walk[3 * s + 2] = (float)(speed * t);
    }
    tracks[boneHips].translation =
        (maniChannelSource){times, walk, SAMPLES, maniInterpolationLinear};
    const maniRootMotionSource root = {boneHips,
                                       maniRootAxisX | maniRootAxisZ,
                                       maniRootRotationTwist,
                                       1,
                                       maniRootReferenceRest,
                                       maniRootReferenceRest,
                                       0,
                                       0};
    maniClipSource source = {.skeleton = skeleton,
                             .boneTracks = tracks,
                             .boneTrackCount = BENCH_BONES,
                             .loop = maniLoopRepeat,
                             .sampleRate = RATE,
                             .duration = BENCH_DURATION,
                             .rootMotion = &root};
    return Keep(builder, maniBuildClip(builder, &source), "a clip", fileOut, sizeOut) &&
           Opened(maniOpenClip(*fileOut, *sizeOut, clip), "a clip");
}

static bool BuildClips(maniBuilder* builder, BenchScenes* scenes)
{
    float* keys = malloc(((size_t)BENCH_BONES * 4 + 3) * SAMPLES * sizeof(float));
    if (keys == NULL)
    {
        return Fail("the clips' keys");
    }
    bool built = true;
    maniCompressOptions options = maniDefaultCompressOptions();
    for (uint32_t c = 0; c < BENCH_CLIPS && built; ++c)
    {
        void* fullFile = NULL;
        size_t fullSize = 0;
        void* file = NULL;
        size_t size = 0;
        built = BuildClip(builder, &scenes->skeleton, c, keys, &scenes->full[c], &fullFile,
                          &fullSize) &&
                Keep(builder,
                     (maniBuildResult){maniCompressClip(builder, &scenes->full[c],
                                                        &scenes->skeleton, &options, NULL),
                                       0, 0, 0},
                     "a compression", &file, &size) &&
                Opened(maniOpenClip(file, size, &scenes->compressed[c]), "a compressed clip");
        if (built && c == 0)
        {
            scenes->clipFile = fullFile;
            scenes->clipFileSize = fullSize;
            scenes->compressedFile = file;
            scenes->compressedFileSize = size;
        }
    }
    free(keys);
    return built;
}

// Blends the first two compressed clips by "mix", a half each.
static bool BuildGraph(maniBuilder* builder, BenchScenes* scenes)
{
    static const char kText[] =
        "{\"format\": \"maul-anim graph\", \"version\": 1,"
        " \"parameters\": [{\"name\": \"mix\", \"type\": \"float\"}],"
        " \"clips\": [\"walk\", \"run\"],"
        " \"nodes\": [{\"kind\": \"blend\", \"inputs\": [1, 2], \"weights\": [{\"parameter\": "
        "\"mix\"}, 1]},"
        " {\"kind\": \"clip\", \"clip\": \"walk\"}, {\"kind\": \"clip\", \"clip\": \"run\"}],"
        " \"output\": 0}";
    const maniGraphClipSource clips[2] = {{&scenes->compressed[0], "walk", 4, 0, NULL},
                                          {&scenes->compressed[1], "run", 3, 0, NULL}};
    maniGraphNamedAssets named = {&scenes->skeleton, clips, NULL, NULL, NULL, 2, 0, 0, 0, NULL};
    void* file = NULL;
    size_t size = 0;
    static maniGraph graph;
    if (!Keep(builder, maniBuildGraphJson(builder, kText, sizeof kText - 1, &named), "the graph",
              &file, &size) ||
        !Opened(maniOpenGraph(file, size, &graph), "the graph"))
    {
        return false;
    }
    const maniClip* bound[2] = {&scenes->compressed[0], &scenes->compressed[1]};
    maniGraphAssets assets = {.skeleton = &scenes->skeleton, .clips = bound, .clipCount = 2};
    size_t bindingSize = maniGraphBindingSize(&graph);
    void* bindingMemory = Take(bindingSize);
    maniGraphBinding* binding = NULL;
    if (bindingMemory == NULL ||
        maniBindGraph(&graph, &assets, bindingMemory, bindingSize, &binding, NULL) != mani_success)
    {
        return Fail("the graph's binding");
    }
    size_t instanceSize = maniGraphInstanceSize(binding);
    void* instanceMemory = Take(instanceSize);
    scenes->scratchSize = maniGraphScratchSize(binding);
    scenes->scratch = Take(scenes->scratchSize);
    scenes->mix = maniGraphFindParameter(&graph, "mix", 3);
    return (instanceMemory != NULL && scenes->scratch != NULL &&
            maniInitGraphInstance(binding, instanceMemory, instanceSize, &scenes->instance) ==
                mani_success &&
            maniSetGraphParameter(scenes->instance, scenes->mix, maniGraphParameterFloat,
                                  (maniGraphValue){.f = {0.5f}}) == mani_success) ||
           Fail("the graph's instance");
}

// Every clip's frames, looping: both feet's positions and the hips'
// velocity now, the root's path and facing 0.2, 0.4 and 0.6 s ahead.
static bool BuildDatabase(maniBuilder* builder, BenchScenes* scenes)
{
    static const float kNow[1] = {0.0f};
    static const float kAhead[3] = {0.2f, 0.4f, 0.6f};
    const maniMatchChannelSource channels[5] = {
        {maniMatchBonePosition, boneFoot, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchBonePosition, boneFootRight, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchBoneVelocity, boneHips, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, boneHips, 0, 3, kAhead, NULL, 0, 1.0f},
        {maniMatchTrajectoryFacing, boneHips, 0, 3, kAhead, NULL, 0, 1.5f}};
    maniMatchEntrySource entries[BENCH_CLIPS];
    const maniClip* clips[BENCH_CLIPS];
    for (uint32_t c = 0; c < BENCH_CLIPS; ++c)
    {
        entries[c] = (maniMatchEntrySource){c, MANI_MATCH_LOOP, 0.0f, BENCH_DURATION, 0.0f, 1u};
        clips[c] = &scenes->full[c];
    }
    const maniMotionDatabaseSource source = {&scenes->skeleton,
                                             clips,
                                             channels,
                                             entries,
                                             BENCH_CLIPS,
                                             5,
                                             BENCH_CLIPS,
                                             RATE,
                                             maniMatchAxisY,
                                             maniMatchAxisZ,
                                             NULL,
                                             0,
                                             0};
    void* file = NULL;
    size_t size = 0;
    if (!Keep(builder, maniBuildMotionDatabase(builder, &source), "the motion database", &file,
              &size) ||
        !Opened(maniOpenMotionDatabase(file, size, &scenes->database), "the motion database"))
    {
        return false;
    }
    scenes->databaseFile = file;
    scenes->databaseFileSize = size;
    // Each query is a frame's raw features, nudged, continuing the frame
    // before it: frames spread over the database.
    const maniMotionDatabase* d = &scenes->database;
    uint32_t n = d->featureCount;
    float* offsets = Take((size_t)n * sizeof(float));
    float* scales = Take((size_t)n * sizeof(float));
    scenes->queryFeatures = Take((size_t)n * BENCH_QUERIES * sizeof(float));
    if (offsets == NULL || scales == NULL || scenes->queryFeatures == NULL ||
        maniMotionDatabaseNormalization(d, offsets, scales) != mani_success)
    {
        return Fail("the queries");
    }
    for (uint32_t q = 0; q < BENCH_QUERIES; ++q)
    {
        uint32_t frame = 1 + (uint32_t)((uint64_t)q * 7919u % (d->frameCount - 1));
        float* features = scenes->queryFeatures + (size_t)q * n;
        if (maniMotionDatabaseFeatures(d, frame, features) != mani_success)
        {
            return Fail("the queries");
        }
        for (uint32_t j = 0; j < n; ++j)
        {
            float nudge = 0.05f * (float)((q * 31u + j * 17u) % 11u) - 0.25f;
            features[j] = (features[j] + nudge) * scales[j] + offsets[j];
        }
        scenes->queries[q] = (maniMatchQuery){.features = features,
                                              .current = frame - 1,
                                              .continuingBias = -0.01f,
                                              .transitionCost = 0.0f,
                                              .jumpThreshold = 0.2f,
                                              .endExclusion = 0.0f,
                                              .historyTime = 0.0f};
    }
    return true;
}

// The hair chains and the tail, against spheres on the head, chest and
// hips.
static bool BuildSprings(maniBuilder* builder, BenchScenes* scenes)
{
    const maniColliderSource colliders[3] = {
        {boneHead, maniColliderSphere, {0.0f, 0.08f, 0.0f}, {0, 0, 0}, 0.1f, 0},
        {boneChest, maniColliderSphere, {0.0f, 0.1f, 0.0f}, {0, 0, 0}, 0.14f, 0},
        {boneHips, maniColliderSphere, {0.0f, 0.0f, 0.0f}, {0, 0, 0}, 0.15f, 0}};
    static const uint32_t kMembers[3] = {0, 1, 2};
    const maniColliderGroupSource groups[1] = {{"body", 4, 3, kMembers}};
    static const uint32_t kGroups[1] = {0};
    maniSpringJointSource joints[2 * boneHairLength + boneTailEnd - boneTail + 1];
    static const uint32_t kFirst[3] = {boneHairLeft, boneHairRight, boneTail};
    static const uint32_t kCount[3] = {boneHairLength, boneHairLength, boneTailEnd - boneTail + 1};
    maniSpringSource springs[3];
    static const char* const kNames[3] = {"hairL", "hairR", "tail"};
    uint32_t at = 0;
    for (uint32_t s = 0; s < 3; ++s)
    {
        springs[s] = (maniSpringSource){
            kNames[s],   (uint32_t)strlen(kNames[s]), kCount[s], joints + at, kGroups, 1,
            MANI_NO_BONE};
        for (uint32_t k = 0; k < kCount[s]; ++k)
        {
            joints[at++] = (maniSpringJointSource){.bone = kFirst[s] + k,
                                                   .hitRadius = 0.02f,
                                                   .stiffness = 1.0f,
                                                   .gravityPower = 0.5f,
                                                   .gravityDirection = {0.0f, -1.0f, 0.0f},
                                                   .drag = 0.4f,
                                                   .limit = maniSpringLimitNone,
                                                   .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
        }
    }
    const maniSpringSetSource source = {
        &scenes->skeleton, colliders, groups, springs, 3, 1, 3, 0.0f, 0.0f, 0};
    void* file = NULL;
    size_t size = 0;
    static maniSpringSet set;
    if (!Keep(builder, maniBuildSpringSet(builder, &source), "the springs", &file, &size) ||
        !Opened(maniOpenSpringSet(file, size, &set), "the springs"))
    {
        return false;
    }
    size_t stateSize = maniSpringStateSize(&set);
    void* memory = Take(stateSize);
    return (memory != NULL && maniInitSpringState(&set, &scenes->skeleton, memory, stateSize,
                                                  &scenes->springs) == mani_success) ||
           Fail("the spring state");
}

// The mesh: vertex v by bone v % 64 and three more 13 apart, weights a
// half, a quarter, 0.15 and 0.1, near the first bone's rest place.
static bool BuildSkin(maniBuilder* builder, const Rig* rig, BenchScenes* scenes)
{
    uint32_t joints[BENCH_BONES];
    maniAffine inverseBinds[BENCH_BONES];
    for (uint32_t b = 0; b < BENCH_BONES; ++b)
    {
        joints[b] = b;
        maniVec3 p = rig->model[b];
        inverseBinds[b] = (maniAffine){{1, 0, 0, 0, 1, 0, 0, 0, 1, -p.x, -p.y, -p.z}};
    }
    maniSkinSource skinSource = {&scenes->skeleton, joints, inverseBinds, BENCH_BONES, 0};
    void* file = NULL;
    size_t size = 0;
    if (!Keep(builder, maniBuildSkin(builder, &skinSource), "the skin", &file, &size) ||
        !Opened(maniOpenSkin(file, size, &scenes->skin), "the skin"))
    {
        return false;
    }
    uint32_t* offsets = malloc((BENCH_VERTICES + 1) * sizeof(uint32_t));
    uint32_t* vertexJoints = malloc(BENCH_VERTICES * 4 * sizeof(uint32_t));
    float* weights = malloc(BENCH_VERTICES * 4 * sizeof(float));
    size_t stream = BENCH_VERTICES * 3 * sizeof(float);
    scenes->positions = Take(stream);
    scenes->normals = Take(stream);
    scenes->movedPositions = Take(stream);
    scenes->movedNormals = Take(stream);
    bool made = offsets != NULL && vertexJoints != NULL && weights != NULL &&
                scenes->positions != NULL && scenes->normals != NULL &&
                scenes->movedPositions != NULL && scenes->movedNormals != NULL;
    static const float kWeights[4] = {0.5f, 0.25f, 0.15f, 0.1f};
    for (uint32_t v = 0; made && v < BENCH_VERTICES; ++v)
    {
        offsets[v] = 4 * v;
        for (uint32_t k = 0; k < 4; ++k)
        {
            vertexJoints[4 * v + k] = (v + 13 * k) % BENCH_BONES;
            weights[4 * v + k] = kWeights[k];
        }
        maniVec3 p = rig->model[v % BENCH_BONES];
        float u = 0.01f * (float)(v % 97);
        float w = 0.01f * (float)(v % 89);
        float* at = scenes->positions + 3 * v;
        at[0] = p.x + 0.05f - u * 0.1f;
        at[1] = p.y + 0.05f - w * 0.1f;
        at[2] = p.z + 0.03f;
        float* normal = scenes->normals + 3 * v;
        float length = sqrtf(u * u + w * w + 1.0f);
        normal[0] = u / length;
        normal[1] = w / length;
        normal[2] = 1.0f / length;
    }
    if (made)
    {
        offsets[BENCH_VERTICES] = 4 * BENCH_VERTICES;
        maniInfluenceSource source = {&scenes->skin,  offsets, vertexJoints, weights, NULL,
                                      BENCH_VERTICES, 0};
        made =
            Keep(builder, maniBuildInfluences(builder, &source), "the influences", &file, &size) &&
            Opened(maniOpenInfluences(file, size, &scenes->influences), "the influences");
    }
    free(offsets);
    free(vertexJoints);
    free(weights);
    return made;
}

// Target t moves every tenth vertex from t % 10, positions and normals,
// alone in its channel.
static bool BuildMorphs(maniBuilder* builder, BenchScenes* scenes)
{
    enum
    {
        moved = BENCH_VERTICES / 10
    };
    static char names[BENCH_TARGETS][8];
    uint32_t* indices = malloc((size_t)BENCH_TARGETS * moved * sizeof(uint32_t));
    float* deltas = malloc((size_t)BENCH_TARGETS * moved * 6 * sizeof(float));
    const float* pointers[BENCH_TARGETS][2];
    maniMorphTargetSource targets[BENCH_TARGETS];
    maniMorphChannelSource channels[BENCH_TARGETS];
    if (indices == NULL || deltas == NULL)
    {
        free(indices);
        free(deltas);
        return Fail("the morph targets' deltas");
    }
    for (uint32_t t = 0; t < BENCH_TARGETS; ++t)
    {
        uint32_t* own = indices + (size_t)t * moved;
        float* positions = deltas + (size_t)t * moved * 6;
        float* normals = positions + (size_t)moved * 3;
        for (uint32_t i = 0; i < moved; ++i)
        {
            own[i] = 10 * i + t % 10;
            for (uint32_t k = 0; k < 3; ++k)
            {
                positions[3 * i + k] = 0.001f * (float)((i + 3 * k + t) % 17) - 0.008f;
                normals[3 * i + k] = 0.01f * (float)((i + 5 * k + t) % 13) - 0.06f;
            }
        }
        snprintf(names[t], sizeof names[t], "t%02u", (unsigned)t);
        pointers[t][0] = positions;
        pointers[t][1] = normals;
        targets[t] = (maniMorphTargetSource){1.0f, moved, own, pointers[t], names[t], 3, 0};
        channels[t] = (maniMorphChannelSource){names[t], 3, 1, &targets[t]};
        scenes->channelWeights[t] = 0.3f + 0.02f * (float)t;
    }
    static const maniMorphAttributeSource kAttributes[2] = {{maniMorphPosition, 0},
                                                            {maniMorphNormal, 0}};
    maniMorphSource source = {channels, BENCH_TARGETS, BENCH_VERTICES, kAttributes, 2, 0};
    void* file = NULL;
    size_t size = 0;
    bool made =
        Keep(builder, maniBuildMorphTargets(builder, &source), "the morph targets", &file, &size) &&
        Opened(maniOpenMorphTargets(file, size, &scenes->morphs), "the morph targets");
    free(indices);
    free(deltas);
    scenes->targetWeights = Take(maniMorphTargetCount(&scenes->morphs) * sizeof(float));
    return made && (scenes->targetWeights != NULL || Fail("the target weights"));
}

static maniVec3 Place(const maniPose* pose, uint32_t bone, float x, float y, float z)
{
    maniAffine m = maniPoseModel(pose, bone);
    return (maniVec3){m.m[9] + x, m.m[10] + y, m.m[11] + z};
}

// The poses, the cursor, the solves and the palette.
static bool Prepare(BenchScenes* scenes)
{
    const maniSkeleton* skeleton = &scenes->skeleton;
    size_t poseSize = maniPoseSize(skeleton);
    maniPose** poses[3] = {&scenes->pose, &scenes->layers[0], &scenes->layers[1]};
    for (uint32_t k = 0; k < 3; ++k)
    {
        void* memory = Take(poseSize);
        if (memory == NULL || maniInitPose(skeleton, memory, poseSize, poses[k]) != mani_success)
        {
            return Fail("a pose");
        }
    }
    size_t cursorSize = maniClipCursorSize(&scenes->compressed[0]);
    for (uint32_t c = 1; c < BENCH_CLIPS; ++c)
    {
        size_t each = maniClipCursorSize(&scenes->compressed[c]);
        cursorSize = each > cursorSize ? each : cursorSize;
    }
    void* cursorMemory = Take(cursorSize);
    scenes->locals = Take(BENCH_BONES * sizeof(maniTransform));
    if (cursorMemory == NULL || scenes->locals == NULL ||
        maniInitClipCursor(cursorMemory, cursorSize, &scenes->cursor) != mani_success ||
        maniSampleClip(&scenes->full[0], skeleton, 1.0f, scenes->layers[0]) != mani_success ||
        maniSampleClip(&scenes->full[1], skeleton, 1.0f, scenes->layers[1]) != mani_success ||
        maniPoseReadLocals(scenes->layers[0], 0, BENCH_BONES, scenes->locals) != mani_success ||
        maniComputeModel(scenes->layers[0], skeleton) != mani_success)
    {
        return Fail("the first poses");
    }
    const maniPose* first = scenes->layers[0];
    scenes->leg = (maniTwoBoneSolve){.start = boneThigh,
                                     .middle = boneShin,
                                     .end = boneFoot,
                                     .goal = Place(first, boneFoot, 0.05f, 0.1f, 0.08f),
                                     .pole = {0.0f, 0.0f, 1.0f},
                                     .hinge = {1.0f, 0.0f, 0.0f},
                                     .soften = 1.0f,
                                     .weight = 1.0f};
    scenes->look = (maniAimSolve){.bones = {boneSpine1, boneChest, boneNeck, boneHead},
                                  .shares = {0.25f, 0.33f, 0.5f, 1.0f},
                                  .count = 4,
                                  .goal = Place(first, boneHead, 0.5f, 0.2f, 1.0f),
                                  .pole = {0.0f, 1.0f, 0.0f},
                                  .forward = {0.0f, 0.0f, 1.0f},
                                  .offset = {0.0f, 0.08f, 0.08f},
                                  .up = {0.0f, 1.0f, 0.0f},
                                  .weight = 1.0f};
    scenes->tail = (maniChainSolve){.root = boneTail,
                                    .end = boneTailEnd,
                                    .goal = Place(first, boneTailEnd, 0.2f, 0.3f, 0.0f),
                                    .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                                    .iterations = 10,
                                    .tolerance = 1e-3f,
                                    .weight = 1.0f};
    scenes->palette = Take(BENCH_BONES * sizeof(maniAffine));
    scenes->dualQuaternions = Take(BENCH_BONES * 20 * sizeof(float));
    return (scenes->palette != NULL && scenes->dualQuaternions != NULL &&
            maniComputePalette(first, skeleton, &scenes->skin, scenes->palette, BENCH_BONES,
                               NULL) == mani_success &&
            maniPaletteDualQuaternions(scenes->palette, BENCH_BONES, scenes->dualQuaternions) ==
                mani_success) ||
           Fail("the palette");
}

bool BenchBuild(BenchScenes* scenes)
{
    memset(scenes, 0, sizeof *scenes);
    static Rig rig;
    MakeRig(&rig);
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        return Fail("the builder");
    }
    bool built = BuildSkeleton(builder, &rig, &scenes->skeleton) && BuildClips(builder, scenes) &&
                 BuildGraph(builder, scenes) && BuildDatabase(builder, scenes) &&
                 BuildSprings(builder, scenes) && BuildSkin(builder, &rig, scenes) &&
                 BuildMorphs(builder, scenes) && Prepare(scenes);
    maniDestroyBuilder(builder);
    return built;
}

void BenchRestore(BenchScenes* scenes)
{
    (void)maniPoseWriteLocals(scenes->pose, 0, BENCH_BONES, scenes->locals);
}
