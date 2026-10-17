// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned models (mani-0025): a model of random networks built and
// opened, its records read back; every evaluation equal bit for bit to a
// plain evaluation in the same order (normalized inputs, biases, each
// nonzero input times its weights, ReLU, denormalized outputs); the
// Stepper's step; the Decompressor's pose (rotations from two columns,
// made orthonormal, parallel and zero columns included, near half turns
// to 1e-6), scales (1 when none given), root motion, float tracks and
// events; 16-bit weights rounded to nearest even input by input, near
// the binary32 model; the builder's refusals; opening's checks of every
// record against a file changed at its byte.
// Run as test_learned --seed <file> it writes the model, fuzz_learned's
// seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "graph_match_fixture.h"
#include "learned_file.h"

#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// The fixture's database: two features (the walk's root half a second
// ahead); three latent values, a state of five. Outputs: nine a bone
// (three), three for the scaled bone 2, six for the root, a float track
// and two events: 39.
#define FEATURES 2u
#define LATENT   3u
#define STATE    5u
#define OUTPUTS  39u

static float s_pool[16384];
static size_t s_used;
static uint64_t s_random = 0x9E3779B97F4A7C15ull;

static float Random(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (float)(s_random >> 40) / 16777216.0f - 0.5f;
}

static float* Pool(size_t count, float low, float spread)
{
    float* p = s_pool + s_used;
    s_used += count;
    for (size_t k = 0; k < count; ++k)
    {
        p[k] = low + spread * Random();
    }
    return p;
}

// A network of random values through the sizes given (count + 1 of them).
static maniLearnedNetworkSource Network(maniLearnedLayerSource* layers, const uint32_t* sizes,
                                        uint32_t count)
{
    for (uint32_t l = 0; l < count; ++l)
    {
        layers[l] =
            (maniLearnedLayerSource){Pool((size_t)sizes[l] * sizes[l + 1], 0.0f, 1.0f),
                                     Pool(sizes[l + 1], 0.0f, 0.4f), sizes[l], sizes[l + 1]};
    }
    return (maniLearnedNetworkSource){layers,
                                      Pool(sizes[0], 0.0f, 1.0f),
                                      Pool(sizes[0], 1.25f, 1.5f),
                                      Pool(sizes[count], 0.0f, 1.0f),
                                      Pool(sizes[count], 1.25f, 1.5f),
                                      count,
                                      0};
}

static maniLearnedLayerSource s_layers[5][3];
static maniLearnedNetworkSource s_decompressor;
static maniLearnedNetworkSource s_stepper;
static maniLearnedProjectorSource s_projectors[2];
static const uint32_t kScaled[1] = {2};
static float s_scales[9] = {1, 1, 1, 2, 2, 2, 1, 1, 1};
static const maniLearnedNameSource kTracks[1] = {{"speed", 5, 0}};
static const maniLearnedNameSource kEvents[2] = {{"left", 4, 0}, {"right", 5, 0}};

static void MakeNetworks(void)
{
    static const uint32_t kDecompressor[3] = {STATE, 8, OUTPUTS};
    static const uint32_t kStepper[4] = {STATE, 8, 8, STATE};
    static const uint32_t kFirst[3] = {FEATURES, 7, STATE};
    static const uint32_t kSecond[3] = {FEATURES, 8, STATE};
    s_decompressor = Network(s_layers[0], kDecompressor, 2);
    s_stepper = Network(s_layers[1], kStepper, 3);
    s_projectors[0] = (maniLearnedProjectorSource){Network(s_layers[2], kFirst, 2), 1, 0};
    s_projectors[1] = (maniLearnedProjectorSource){Network(s_layers[3], kSecond, 2), 2, 1};
}

static maniLearnedModelSource Source(void)
{
    return (maniLearnedModelSource){.skeleton = &s_skeleton,
                                    .database = &s_database,
                                    .decompressor = &s_decompressor,
                                    .stepper = &s_stepper,
                                    .projectors = s_projectors,
                                    .scaledBones = kScaled,
                                    .scales = s_scales,
                                    .floatTracks = kTracks,
                                    .events = kEvents,
                                    .projectorCount = 2,
                                    .scaledBoneCount = 1,
                                    .floatTrackCount = 1,
                                    .eventCount = 2,
                                    .latentCount = LATENT,
                                    .stepRate = 60.0f,
                                    .weightFormat = maniLearnedWeightsFloat32};
}

static _Alignas(16) uint8_t s_model[65536];
static size_t s_modelSize;

static maniBuildResult Build(const maniLearnedModelSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildLearnedModel(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_model, sizeof s_model, &s_modelSize) == mani_success,
              "copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static bool Opened(const maniLearnedModelSource* source, maniLearnedModel* model)
{
    return Build(source).result == mani_success &&
           maniOpenLearnedModel(s_model, s_modelSize, model).result == mani_success;
}

// The plain evaluation, in the library's order.
static void Plain(const maniLearnedNetworkSource* n, const float* input, float* output)
{
    float a[64];
    float b[64];
    uint32_t in = n->layers[0].inputCount;
    for (uint32_t i = 0; i < in; ++i)
    {
        a[i] = (input[i] - n->inputOffsets[i]) / n->inputScales[i];
    }
    for (uint32_t l = 0; l < n->layerCount; ++l)
    {
        const maniLearnedLayerSource* layer = &n->layers[l];
        for (uint32_t o = 0; o < layer->outputCount; ++o)
        {
            b[o] = layer->biases[o];
        }
        for (uint32_t i = 0; i < layer->inputCount; ++i)
        {
            if (a[i] == 0.0f)
            {
                continue;
            }
            for (uint32_t o = 0; o < layer->outputCount; ++o)
            {
                b[o] += a[i] * layer->weights[(size_t)i * layer->outputCount + o];
            }
        }
        for (uint32_t o = 0; o < layer->outputCount; ++o)
        {
            a[o] = l + 1 < n->layerCount && !(b[o] > 0.0f) ? 0.0f : b[o];
        }
    }
    uint32_t out = n->layers[n->layerCount - 1].outputCount;
    for (uint32_t o = 0; o < out; ++o)
    {
        output[o] = a[o] * n->outputScales[o] + n->outputOffsets[o];
    }
}

static bool SameBits(const float* a, const float* b, uint32_t count)
{
    return memcmp(a, b, sizeof(float) * count) == 0;
}

static _Alignas(16) uint8_t s_scratch[4096];
static _Alignas(32) uint8_t s_poseMemory[4096];

static void TestRoundTrip(void)
{
    maniLearnedModelSource source = Source();
    maniLearnedModel m;
    CHECK(Opened(&source, &m), "built and opened");
    CHECK(m.boneCount == 3 && m.featureCount == FEATURES && m.latentCount == LATENT &&
              m.outputCount == OUTPUTS && m.projectorCount == 2 && m.layerCount == 9 &&
              m.widest == OUTPUTS && m.stepRate == 60.0f && m.up == s_database.up &&
              m.forward == s_database.forward,
          "the counts");
    CHECK(maniLearnedModelIdentity(&m) != 0 &&
              maniLearnedModelSkeletonIdentity(&m) == maniSkeletonIdentity(&s_skeleton) &&
              maniLearnedModelDatabaseIdentity(&m) == maniMotionDatabaseIdentity(&s_database) &&
              maniLearnedModelIdentity(NULL) == 0 && maniLearnedModelSkeletonIdentity(NULL) == 0 &&
              maniLearnedModelDatabaseIdentity(NULL) == 0,
          "the identities");
    maniMatchChannel mine;
    maniMatchChannel theirs;
    CHECK(maniLearnedModelChannel(&m, 0, &mine) == mani_success &&
              maniMotionDatabaseChannel(&s_database, 0, &theirs) == mani_success &&
              memcmp(&mine, &theirs, sizeof mine) == 0 &&
              maniLearnedModelChannel(&m, 1, &mine) == mani_errorInvalid &&
              maniLearnedModelChannel(NULL, 0, &mine) == mani_errorInvalid &&
              maniLearnedModelChannel(&m, 0, NULL) == mani_errorInvalid,
          "the database's channel");
    float offsets[2][FEATURES];
    float scales[2][FEATURES];
    CHECK(maniLearnedModelNormalization(&m, offsets[0], scales[0]) == mani_success &&
              maniMotionDatabaseNormalization(&s_database, offsets[1], scales[1]) == mani_success &&
              SameBits(offsets[0], offsets[1], FEATURES) &&
              SameBits(scales[0], scales[1], FEATURES) &&
              maniLearnedModelNormalization(&m, NULL, NULL) == mani_success &&
              maniLearnedModelNormalization(NULL, offsets[0], NULL) == mani_errorInvalid,
          "the database's normalization");
    uint32_t required = 0;
    uint32_t excluded = 0;
    CHECK(maniLearnedModelProjector(&m, 1, &required, &excluded) == mani_success && required == 2 &&
              excluded == 1 && maniLearnedModelProjector(&m, 0, &required, NULL) == mani_success &&
              required == 1 && maniLearnedModelProjector(&m, 2, NULL, NULL) == mani_errorInvalid &&
              maniLearnedModelProjector(NULL, 0, NULL, NULL) == mani_errorInvalid,
          "the Projectors' filters");
    uint32_t length = 0;
    const char* track = maniLearnedModelFloatTrackName(&m, 0, &length);
    CHECK(track != NULL && length == 5 && memcmp(track, "speed", 5) == 0 &&
              maniLearnedModelFloatTrackName(&m, 1, &length) == NULL &&
              maniLearnedModelFloatTrackName(NULL, 0, NULL) == NULL,
          "the float track's name");
    const char* right = maniLearnedModelEventName(&m, 1, &length);
    CHECK(right != NULL && length == 5 && memcmp(right, "right", 5) == 0 &&
              maniLearnedModelEventName(&m, 0, NULL) != NULL &&
              maniLearnedModelEventName(&m, 2, &length) == NULL &&
              maniLearnedModelEventName(NULL, 0, NULL) == NULL,
          "the events' names");
    CHECK(maniLearnedScratchSize(&m) == 2 * sizeof(float) * OUTPUTS &&
              maniLearnedScratchSize(NULL) == 0,
          "the scratch");
}

// The Decompressor's outputs against the pose and the rest.
static bool Decompressed(const maniLearnedModel* m, const float* state, const float* outputs)
{
    maniPose* pose = NULL;
    float tracks[1];
    float events[2];
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "a pose");
    maniLearnedOutput out = {pose, tracks, events, {0, 0, 0}, {0, 0, 0}};
    if (maniLearnedDecompress(m, state, &out, s_scratch, sizeof s_scratch) != mani_success)
    {
        return false;
    }
    bool ok = SameBits(&out.rootVelocity.x, outputs + 30, 3) &&
              SameBits(&out.rootAngularVelocity.x, outputs + 33, 3) &&
              SameBits(tracks, outputs + 36, 1) && SameBits(events, outputs + 37, 2);
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        const float* o = outputs + 9 * b;
        // The first column turned back by the rotation, and the normal
        // of the first two.
        maniVec3 x = maniVec3Normalize(maniVec3Make(o[3], o[4], o[5]), maniVec3Make(1, 0, 0));
        maniVec3 z = maniVec3Normalize(maniVec3Cross(x, maniVec3Make(o[6], o[7], o[8])),
                                       maniVec3Make(0, 0, 1));
        maniVec3 rx = maniQuatRotate(t.rotation, maniVec3Make(1, 0, 0));
        maniVec3 rz = maniQuatRotate(t.rotation, maniVec3Make(0, 0, 1));
        const float* scale = b == 2 ? outputs + 27 : s_scales + 3 * b;
        ok = ok && SameBits(&t.translation.x, o, 3) && SameBits(&t.scale.x, scale, 3) &&
             maniVec3Dot(rx, x) > 1.0f - 1e-5f && maniVec3Dot(rz, z) > 1.0f - 1e-5f &&
             t.rotation.w >= 0.0f;
    }
    return ok;
}

static void TestEvaluate(void)
{
    maniLearnedModelSource source = Source();
    maniLearnedModel m;
    CHECK(Opened(&source, &m), "built and opened");
    bool projected = true;
    bool stepped = true;
    bool decompressed = true;
    for (int k = 0; k < 50; ++k)
    {
        float query[FEATURES] = {2.0f * Random(), 2.0f * Random()};
        uint32_t p = (uint32_t)k % 2;
        float state[STATE];
        float want[OUTPUTS];
        projected = projected && maniLearnedProject(&m, p, query, state, s_scratch,
                                                    sizeof s_scratch) == mani_success;
        Plain(&s_projectors[p].network, query, want);
        projected = projected && SameBits(state, want, STATE);
        float rate[STATE];
        Plain(&s_stepper, state, rate);
        for (uint32_t i = 0; i < STATE; ++i)
        {
            want[i] = state[i] + (1.0f / 60.0f) * rate[i];
        }
        stepped = stepped &&
                  maniLearnedStep(&m, state, s_scratch, sizeof s_scratch) == mani_success &&
                  SameBits(state, want, STATE);
        Plain(&s_decompressor, state, want);
        decompressed = decompressed && Decompressed(&m, state, want);
    }
    CHECK(projected, "projections as the plain evaluation");
    CHECK(stepped, "steps as the plain evaluation");
    CHECK(decompressed, "decompressions as the plain evaluation");
}

static void TestEvaluateRefusals(void)
{
    maniLearnedModelSource source = Source();
    maniLearnedModel m;
    CHECK(Opened(&source, &m), "built and opened");
    float query[FEATURES] = {0.1f, 0.2f};
    float state[STATE] = {0, 0, 0, 0, 0};
    float bad[STATE] = {0, NAN, 0, 0, 0};
    size_t need = maniLearnedScratchSize(&m);
    CHECK(maniLearnedProject(&m, 2, query, state, s_scratch, need) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, bad, state, s_scratch, need) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, query, state, s_scratch, need - 1) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, query, state, s_scratch + 1, need) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, query, state, NULL, need) == mani_errorInvalid &&
              maniLearnedProject(NULL, 0, query, state, s_scratch, need) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, NULL, state, s_scratch, need) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, query, NULL, s_scratch, need) == mani_errorInvalid &&
              maniLearnedProject(&m, 0, query, state, s_scratch, need) == mani_success,
          "projection's refusals");
    float kept[STATE];
    memcpy(kept, bad, sizeof kept);
    CHECK(maniLearnedStep(&m, bad, s_scratch, need) == mani_errorInvalid &&
              SameBits(kept, bad, STATE) &&
              maniLearnedStep(&m, state, s_scratch, need - 1) == mani_errorInvalid &&
              maniLearnedStep(NULL, state, s_scratch, need) == mani_errorInvalid &&
              maniLearnedStep(&m, NULL, s_scratch, need) == mani_errorInvalid,
          "a step's refusals, the state kept");
    maniPose* pose = NULL;
    static _Alignas(32) uint8_t other[4096];
    maniPose* difference = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success &&
              maniInitPose(&s_skeleton, other, sizeof other, &difference) == mani_success &&
              maniPoseDifference(&s_skeleton, pose, pose, difference) == mani_success,
          "poses");
    // A pose of a one-bone skeleton.
    static _Alignas(16) uint8_t oneFile[1024];
    static _Alignas(32) uint8_t onePose[1024];
    static const maniBoneSource kBone = {"hips", 4, -1, {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}};
    const maniSkeletonSource oneSource = {&kBone, 1, 0};
    maniSkeleton one;
    maniPose* small = NULL;
    size_t oneSize = 0;
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildSkeleton(builder, &oneSource).result == mani_success &&
              maniCopyBuilt(builder, oneFile, sizeof oneFile, &oneSize) == mani_success &&
              maniOpenSkeleton(oneFile, oneSize, &one).result == mani_success &&
              maniInitPose(&one, onePose, sizeof onePose, &small) == mani_success,
          "a one-bone pose");
    maniDestroyBuilder(builder);
    maniLearnedOutput mismatched = {small, NULL, NULL, {0, 0, 0}, {0, 0, 0}};
    CHECK(maniLearnedDecompress(&m, state, &mismatched, s_scratch, need) == mani_errorInvalid,
          "a pose of another bone count");
    maniLearnedOutput out = {difference, NULL, NULL, {0, 0, 0}, {0, 0, 0}};
    maniLearnedOutput plain = {pose, NULL, NULL, {0, 0, 0}, {0, 0, 0}};
    CHECK(maniLearnedDecompress(&m, state, &out, s_scratch, need) == mani_errorInvalid &&
              maniLearnedDecompress(&m, bad, &plain, s_scratch, need) == mani_errorInvalid &&
              maniLearnedDecompress(&m, state, NULL, s_scratch, need) == mani_errorInvalid &&
              maniLearnedDecompress(&m, state, &plain, s_scratch, need - 1) == mani_errorInvalid &&
              maniLearnedDecompress(NULL, state, &plain, s_scratch, need) == mani_errorInvalid &&
              maniLearnedDecompress(&m, state, &plain, s_scratch, need) == mani_success,
          "decompression's refusals");
}

// A Decompressor whose outputs are its biases: no weights that count,
// its normalization none.
static float s_biases[48];
static float s_zeros[STATE * 48];
static const float kOnes[48] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

static maniTransform Local(const maniLearnedModel* m, uint32_t bone)
{
    maniPose* pose = NULL;
    float state[STATE] = {1, 1, 1, 1, 1};
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "a pose");
    maniLearnedOutput out = {pose, NULL, NULL, {0, 0, 0}, {0, 0, 0}};
    CHECK(maniLearnedDecompress(m, state, &out, s_scratch, sizeof s_scratch) == mani_success,
          "decompressed");
    return maniPoseLocal(pose, bone);
}

static maniQuat Rotation(const maniLearnedModel* m, uint32_t bone)
{
    return Local(m, bone).rotation;
}

// Turns of 179.9 degrees about x, y and z on bones 0 to 2: each near a
// half turn, where only the largest of Shepperd's four candidates keeps
// the precision.
static void TestHalfTurns(const maniLearnedModelSource* source)
{
    double angle = 179.9 * 3.14159265358979323846 / 180.0;
    float c = (float)cos(angle);
    float sn = (float)sin(angle);
    const float columns[3][6] = {{1, 0, 0, 0, c, sn}, {c, 0, -sn, 0, 1, 0}, {c, sn, 0, -sn, c, 0}};
    for (uint32_t b = 0; b < 3; ++b)
    {
        memcpy(s_biases + 9 * b + 3, columns[b], sizeof columns[b]);
    }
    maniLearnedModel m;
    CHECK(Opened(source, &m), "built and opened");
    double half = sin(angle / 2.0);
    double w = cos(angle / 2.0);
    bool near = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniQuat q = Rotation(&m, b);
        const float axis[3] = {q.x, q.y, q.z};
        for (uint32_t k = 0; k < 3; ++k)
        {
            near = near && fabs((double)axis[k] - (k == b ? half : 0.0)) <= 1e-6;
        }
        near = near && fabs((double)q.w - w) <= 1e-6;
    }
    CHECK(near, "near half turns about x, y and z to 1e-6");
}

static void TestColumns(void)
{
    const maniLearnedLayerSource layer = {s_zeros, s_biases, STATE, OUTPUTS};
    maniLearnedNetworkSource fixed = {&layer, s_zeros, kOnes, s_zeros, kOnes, 1, 0};
    maniLearnedModelSource source = Source();
    source.decompressor = &fixed;
    // Bone 0 a quarter turn about z; bone 1 columns neither unit nor
    // square; bone 2 parallel columns.
    const float columns[3][6] = {{0, 1, 0, -1, 0, 0}, {2, 0, 0, 1, 1, 0}, {1, 0, 0, 3, 0, 0}};
    for (uint32_t b = 0; b < 3; ++b)
    {
        memcpy(s_biases + 9 * b + 3, columns[b], sizeof columns[b]);
    }
    maniLearnedModel m;
    CHECK(Opened(&source, &m), "built and opened");
    maniQuat q = Rotation(&m, 0);
    CHECK(fabsf(q.x) <= 1e-7f && fabsf(q.y) <= 1e-7f && fabsf(q.z - 0.70710678f) <= 1e-7f &&
              fabsf(q.w - 0.70710678f) <= 1e-7f,
          "a quarter turn about z");
    q = Rotation(&m, 1);
    CHECK(q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 1.0f,
          "columns made orthonormal, the first kept");
    q = Rotation(&m, 2);
    CHECK(q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 1.0f, "parallel columns");
    const float turned[3][6] = {{0, 0, 0, 0, 1, 0}, {0, -1, 0, 1, 0, 0}, {0, 0, 1, 0, 1, 0}};
    for (uint32_t b = 0; b < 3; ++b)
    {
        memcpy(s_biases + 9 * b + 3, turned[b], sizeof turned[b]);
    }
    CHECK(Opened(&source, &m), "built and opened");
    q = Rotation(&m, 0);
    CHECK(q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 1.0f, "a zero first column");
    q = Rotation(&m, 1);
    maniVec3 x = maniQuatRotate(q, maniVec3Make(1, 0, 0));
    maniVec3 y = maniQuatRotate(q, maniVec3Make(0, 1, 0));
    CHECK(fabsf(x.y + 1.0f) <= 1e-6f && fabsf(y.x - 1.0f) <= 1e-6f && q.w >= 0.0f,
          "a half turn's neighbour: x to -y, y to x");
    q = Rotation(&m, 2);
    x = maniQuatRotate(q, maniVec3Make(1, 0, 0));
    y = maniQuatRotate(q, maniVec3Make(0, 1, 0));
    CHECK(fabsf(x.z - 1.0f) <= 1e-6f && fabsf(y.y - 1.0f) <= 1e-6f, "x to z, y kept");
    // Parallel columns whose least axis is z: the third column is the
    // first crossed with z.
    const float parallel[6] = {0.6f, 0.8f, 0, 1.2f, 1.6f, 0};
    memcpy(s_biases + 3, parallel, sizeof parallel);
    CHECK(Opened(&source, &m), "built and opened");
    maniVec3 z = maniQuatRotate(Rotation(&m, 0), maniVec3Make(0, 0, 1));
    CHECK(fabsf(z.x - 0.8f) <= 1e-6f && fabsf(z.y + 0.6f) <= 1e-6f && fabsf(z.z) <= 1e-6f,
          "parallel columns: the least axis crossed");
    TestHalfTurns(&source);
    source.scales = NULL;
    CHECK(Opened(&source, &m), "built and opened");
    maniTransform t = Local(&m, 0);
    CHECK(t.scale.x == 1.0f && t.scale.y == 1.0f && t.scale.z == 1.0f, "no scales given: 1");
    memset(s_biases, 0, sizeof s_biases);
}

// 16-bit weights: each input's weights over their largest magnitude /
// 32767, rounded to nearest even; ties exact where the scale is 1.
static void TestInt16(void)
{
    float* row = (float*)s_stepper.layers[0].weights;
    float kept[4];
    memcpy(kept, row, sizeof kept);
    const float ties[4] = {32767.0f, 2.5f, 3.5f, -2.5f};
    memcpy(row, ties, sizeof ties);
    for (uint32_t o = 4; o < 8; ++o)
    {
        row[o] = 0.25f * row[o];
    }
    maniLearnedModelSource source = Source();
    maniLearnedModel wide;
    CHECK(Opened(&source, &wide), "binary32");
    size_t wideSize = s_modelSize;
    static _Alignas(16) uint8_t wideFile[65536];
    memcpy(wideFile, s_model, s_modelSize);
    CHECK(maniOpenLearnedModel(wideFile, wideSize, &wide).result == mani_success, "kept");
    source.weightFormat = maniLearnedWeightsInt16;
    maniLearnedModel narrow;
    CHECK(Opened(&source, &narrow) && narrow.weightFormat == maniLearnedWeightsInt16 &&
              s_modelSize < wideSize,
          "16-bit, smaller");
    // The Stepper's first layer: after the Decompressor's 5 * 8 + 8 * 39.
    const uint8_t* q = narrow.weights + 2 * (5 * 8 + 8 * 39);
    CHECK((int16_t)maniLoadU16(q) == 32767 && (int16_t)maniLoadU16(q + 2) == 2 &&
              (int16_t)maniLoadU16(q + 4) == 4 && (int16_t)maniLoadU16(q + 6) == -2,
          "ties to even");
    bool rounded = true;
    const uint8_t* w = narrow.weights;
    const maniLearnedNetworkSource* networks[4] = {
        &s_decompressor, &s_stepper, &s_projectors[0].network, &s_projectors[1].network};
    for (uint32_t n = 0; n < 4; ++n)
    {
        for (uint32_t l = 0; l < networks[n]->layerCount; ++l)
        {
            const maniLearnedLayerSource* layer = &networks[n]->layers[l];
            for (uint32_t i = 0; i < layer->inputCount; ++i)
            {
                const float* r = layer->weights + (size_t)i * layer->outputCount;
                float largest = 0.0f;
                for (uint32_t o = 0; o < layer->outputCount; ++o)
                {
                    largest = fmaxf(largest, fabsf(r[o]));
                }
                float scale = largest / 32767.0f;
                for (uint32_t o = 0; o < layer->outputCount; ++o, w += 2)
                {
                    rounded = rounded && (int16_t)maniLoadU16(w) == (int16_t)rintf(r[o] / scale);
                }
            }
        }
    }
    CHECK(rounded, "every weight rounded to nearest even");
    bool near = true;
    for (int k = 0; k < 50; ++k)
    {
        float query[FEATURES] = {2.0f * Random(), 2.0f * Random()};
        float a[STATE];
        float b[STATE];
        near =
            near &&
            maniLearnedProject(&wide, 0, query, a, s_scratch, sizeof s_scratch) == mani_success &&
            maniLearnedProject(&narrow, 0, query, b, s_scratch, sizeof s_scratch) == mani_success;
        for (uint32_t i = 0; i < STATE; ++i)
        {
            near = near && fabsf(a[i] - b[i]) <= 1e-3f * (1.0f + fabsf(a[i]));
        }
    }
    CHECK(near, "16-bit projections near binary32's");
    memcpy(row, kept, sizeof kept);
    for (uint32_t o = 4; o < 8; ++o)
    {
        row[o] = 4.0f * row[o];
    }
}

static bool RefusedAs(const maniLearnedModelSource* s, maniBuildReason reason, uint32_t item,
                      uint32_t other)
{
    maniBuildResult r = Build(s);
    bool ok =
        r.result == mani_errorInvalid && r.reason == reason && r.item == item && r.other == other;
    if (!ok)
    {
        printf("  refused %d: reason %u item %u other %u\n", (int)r.result, r.reason, r.item,
               r.other);
    }
    return ok;
}

static void TestModelRefusals(void)
{
    maniLearnedModelSource s = Source();
    s.stepRate = 0.0f;
    CHECK(RefusedAs(&s, maniBuildReasonTiming, 0, 0), "no step rate");
    s = Source();
    s.stepRate = 1001.0f;
    CHECK(RefusedAs(&s, maniBuildReasonTiming, 0, 0), "a step rate past the limit");
    s = Source();
    s.latentCount = 0;
    CHECK(RefusedAs(&s, maniBuildReasonOption, 0, 0), "no latent values");
    s.latentCount = MANI_LEARNED_MAX_LATENT + 1;
    CHECK(RefusedAs(&s, maniBuildReasonOption, 0, 0), "latent values past the limit");
    s = Source();
    s.weightFormat = 2;
    CHECK(RefusedAs(&s, maniBuildReasonOption, 1, 0), "an unknown weight format");
    s = Source();
    s.projectorCount = 0;
    CHECK(RefusedAs(&s, maniBuildReasonCount, 0, 0), "no Projector");
    s.projectorCount = MANI_LEARNED_MAX_PROJECTORS + 1;
    CHECK(RefusedAs(&s, maniBuildReasonCount, 0, 0), "Projectors past the limit");
    s = Source();
    s.scaledBoneCount = 4;
    CHECK(RefusedAs(&s, maniBuildReasonCount, 0, 0), "more scaled bones than bones");
    s = Source();
    s.floatTrackCount = MANI_LEARNED_MAX_FLOAT_TRACKS + 1;
    CHECK(RefusedAs(&s, maniBuildReasonCount, 0, 0), "float tracks past the limit");
    s = Source();
    s.eventCount = MANI_LEARNED_MAX_EVENTS + 1;
    CHECK(RefusedAs(&s, maniBuildReasonCount, 0, 0), "events past the limit");
    static const uint32_t kPast[1] = {3};
    static const uint32_t kBack[2] = {1, 1};
    s = Source();
    s.scaledBones = kPast;
    CHECK(RefusedAs(&s, maniBuildReasonJoint, 0, 0), "a scaled bone past the skeleton");
    s.scaledBones = kBack;
    s.scaledBoneCount = 2;
    CHECK(RefusedAs(&s, maniBuildReasonJoint, 1, 0), "scaled bones not increasing");
    s = Source();
    s_scales[4] = INFINITY;
    CHECK(RefusedAs(&s, maniBuildReasonNotFinite, 1, 0), "a scale not finite");
    s_scales[4] = 2.0f;
    static const maniLearnedNameSource kBad[2] = {{"a", 1, 0}, {"\xff", 1, 0}};
    static const maniLearnedNameSource kTwice[2] = {{"a", 1, 0}, {"a", 1, 0}};
    static const maniLearnedNameSource kEmpty[1] = {{"", 0, 0}};
    s = Source();
    s.floatTracks = kBad;
    s.floatTrackCount = 2;
    s.decompressor = NULL;
    CHECK(Build(&s).result == mani_errorInvalid && Build(&s).reason == maniBuildReasonNone,
          "a misuse: no Decompressor");
    s = Source();
    s.events = kBad;
    CHECK(RefusedAs(&s, maniBuildReasonEvent | maniBuildReasonName, 1, 0), "an event not UTF-8");
    s.events = kTwice;
    CHECK(RefusedAs(&s, maniBuildReasonEvent | maniBuildReasonDuplicateName, 1, 0),
          "an event twice");
    s = Source();
    s.floatTracks = kEmpty;
    CHECK(RefusedAs(&s, maniBuildReasonFloatTrack | maniBuildReasonName, 0, 0),
          "a float track without a name");
    s = Source();
    s.floatTracks = kEvents;
    CHECK(Build(&s).result == mani_success, "a float track and an event of one name");
    s.floatTrackCount = 2;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 0, 1), "the Decompressor's outputs not the count");
}

static void TestNetworkRefusals(void)
{
    maniLearnedModelSource s = Source();
    maniLearnedNetworkSource broken = s_stepper;
    s.stepper = &broken;
    broken.layerCount = 0;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 0), "no layers");
    broken.layerCount = MANI_LEARNED_MAX_LAYERS + 1;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 0), "layers past the limit");
    broken.layerCount = 2;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 1), "a last output not the state");
    broken = s_stepper;
    maniLearnedLayerSource layers[3];
    memcpy(layers, s_stepper.layers, sizeof layers);
    broken.layers = layers;
    layers[1].inputCount = 7;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 1), "layers not chained");
    memcpy(layers, s_stepper.layers, sizeof layers);
    layers[0].inputCount = 4;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 0), "a first input not the state");
    memcpy(layers, s_stepper.layers, sizeof layers);
    layers[1].weights = NULL;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 1), "no weights");
    memcpy(layers, s_stepper.layers, sizeof layers);
    float* weight = (float*)s_stepper.layers[2].weights + 3;
    float kept = *weight;
    *weight = NAN;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 2), "a weight not finite");
    *weight = kept;
    float* bias = (float*)s_stepper.layers[0].biases + 2;
    kept = *bias;
    *bias = -INFINITY;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 0), "a bias not finite");
    *bias = kept;
    float* scale = (float*)s_stepper.inputScales + 4;
    kept = *scale;
    *scale = 0.0f;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 3), "an input scale of 0");
    *scale = kept;
    float* offset = (float*)s_stepper.outputOffsets + 1;
    kept = *offset;
    *offset = NAN;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 3), "an output offset not finite");
    *offset = kept;
    broken = s_stepper;
    broken.outputScales = NULL;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 1, 3), "no output scales");
    s = Source();
    maniLearnedProjectorSource projectors[2] = {s_projectors[0], s_projectors[1]};
    s.projectors = projectors;
    memcpy(layers, s_projectors[1].network.layers, 2 * sizeof layers[0]);
    projectors[1].network.layers = layers;
    layers[0].inputCount = STATE;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 3, 0), "a Projector taking a state");
    static const maniLearnedLayerSource kWide = {s_pool, s_pool, FEATURES,
                                                 MANI_LEARNED_MAX_WIDTH + 1};
    layers[0] = kWide;
    CHECK(RefusedAs(&s, maniBuildReasonNetwork, 3, 0), "a layer past the width");
    s = Source();
    maniMotionDatabase other = s_database;
    other.skeletonIdentity ^= 1u;
    s.database = &other;
    CHECK(RefusedAs(&s, maniBuildReasonDatabase, 0, 0), "a database of another skeleton");
}

static _Alignas(16) uint8_t s_changed[65536];

// The model last built with size bytes at a section's offset replaced,
// its hashes made again (the identity too when rehash), opened.
static maniFileResult Changed(uint32_t tag, uint64_t offset, const void* bytes, size_t size,
                              bool rehash, uint64_t* sectionOut)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t length = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_model, s_modelSize, maniKindLearnedModel, MANI_LEARNED_FORMAT_VERSION,
                            &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &length, &at),
          "the section");
    memcpy(s_changed, s_model, s_modelSize);
    memcpy(s_changed + at + offset, bytes, size);
    maniStoreU64(s_changed + 32, maniHashBytes(MANI_HASH_INIT, s_changed + 64, s_modelSize - 64));
    if (rehash)
    {
        maniContainer d;
        CHECK(maniOpenContainer(s_changed, s_modelSize, maniKindLearnedModel,
                                MANI_LEARNED_FORMAT_VERSION, &d)
                      .result == mani_success,
              "the crafted container");
        const uint8_t* sections[MANI_LEARNED_SECTIONS];
        uint64_t sizes[MANI_LEARNED_SECTIONS];
        for (uint32_t t = 1; t <= MANI_LEARNED_SECTIONS; ++t)
        {
            uint64_t where = 0;
            sections[t - 1] = s_changed;
            sizes[t - 1] = 0;
            (void)maniContainerSection(&d, t, &sections[t - 1], &sizes[t - 1], &where);
        }
        maniStoreU64(s_changed + 40, maniLearnedComputeIdentity(sections, sizes));
        maniStoreU64(s_changed + 32,
                     maniHashBytes(MANI_HASH_INIT, s_changed + 64, s_modelSize - 64));
    }
    *sectionOut = at;
    maniLearnedModel m;
    return maniOpenLearnedModel(s_changed, s_modelSize, &m);
}

static uint64_t SectionOf(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t length = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_model, s_modelSize, maniKindLearnedModel, MANI_LEARNED_FORMAT_VERSION,
                            &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &length, &at),
          "the section");
    return at;
}

// Refused at want, a byte of section wantTag (UINT64_MAX: that section's
// size in the table, every section present).
static bool RefusedIn(uint32_t tag, uint64_t offset, const void* bytes, size_t size,
                      uint32_t wantTag, uint64_t want)
{
    uint64_t at = 0;
    maniFileResult r = Changed(tag, offset, bytes, size, true, &at);
    uint64_t expected =
        want != UINT64_MAX
            ? SectionOf(wantTag) + want
            : MANI_CONTAINER_HEADER_SIZE + (uint64_t)(wantTag - 1) * MANI_CONTAINER_ENTRY_SIZE + 16;
    if (r.result != mani_errorFormat || r.offset != expected)
    {
        printf("  tag %u offset %llu: result %d at %llu, expected %llu\n", tag,
               (unsigned long long)offset, (int)r.result, (unsigned long long)r.offset,
               (unsigned long long)expected);
    }
    return r.result == mani_errorFormat && r.offset == expected;
}

static bool RefusedAt(uint32_t tag, uint64_t offset, const void* bytes, size_t size, uint64_t want)
{
    return RefusedIn(tag, offset, bytes, size, tag, want);
}

static bool WordRefusedIn(uint32_t tag, uint64_t offset, uint32_t value, uint32_t wantTag,
                          uint64_t want)
{
    uint8_t bytes[4];
    maniStoreU32(bytes, value);
    return RefusedIn(tag, offset, bytes, 4, wantTag, want);
}

static bool WordRefused(uint32_t tag, uint64_t offset, uint32_t value, uint64_t want)
{
    uint8_t bytes[4];
    maniStoreU32(bytes, value);
    return RefusedAt(tag, offset, bytes, 4, want);
}

static uint32_t BitsOf(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

static void TestOpenHeader(void)
{
    maniLearnedModelSource source = Source();
    CHECK(Build(&source).result == mani_success, "built");
    // Word, value, refused at: the counts, format, rate and axes.
    const uint32_t kWords[18][3] = {
        {16, 0, 16}, {20, 1025, 20}, {24, 0, 24},   {28, 0, 28}, {32, 257, 32},
        {36, 4, 36}, {40, 257, 40},  {44, 257, 44}, {48, 0, 48}, {48, 33, 48},
        {52, 3, 52}, {52, 65, 52},   {56, 2, 56},   {60, 0, 60}, {60, 0x447A4000u, 60},
        {64, 6, 64}, {68, 1, 68},    {68, 6, 68}};
    for (uint32_t k = 0; k < 18; ++k)
    {
        CHECK(WordRefused(maniLearnedTagHeader, kWords[k][0], kWords[k][1], kWords[k][2]),
              "a header word refused");
    }
    uint64_t at = 0;
    uint8_t two[4];
    maniStoreU32(two, 2);
    maniFileResult r = Changed(maniLearnedTagHeader, 24, two, 4, true, &at);
    CHECK(r.result == mani_errorFormat &&
              r.offset == MANI_CONTAINER_HEADER_SIZE + MANI_CONTAINER_ENTRY_SIZE + 16,
          "the channels' size checked against the count");
    CHECK(WordRefusedIn(maniLearnedTagHeader, 52, 10, maniLearnedTagLayers, UINT64_MAX),
          "the layers' size checked against the count");
    uint8_t kept[1] = {s_model[200]};
    uint8_t flipped[1] = {(uint8_t)(kept[0] ^ 1u)};
    r = Changed(maniLearnedTagHeader, 0, flipped, 0, false, &at);
    CHECK(r.result == mani_success, "unchanged, it opens");
    memcpy(s_changed, s_model, s_modelSize);
    s_changed[s_modelSize - 1] ^= 1u;
    maniStoreU64(s_changed + 32, maniHashBytes(MANI_HASH_INIT, s_changed + 64, s_modelSize - 64));
    maniLearnedModel m;
    r = maniOpenLearnedModel(s_changed, s_modelSize, &m);
    CHECK(r.result == mani_errorFormat && r.offset == 40, "the identity");
    CHECK(maniOpenLearnedModel(s_model, s_modelSize, NULL).result == mani_errorInvalid &&
              maniOpenLearnedModel(s_model + 1, s_modelSize, &m).result == mani_errorInvalid,
          "a NULL view, misaligned bytes");
}

static void TestOpenRecords(void)
{
    maniLearnedModelSource source = Source();
    CHECK(Build(&source).result == mani_success, "built");
    // The channel's words: kind, bone, axis, times, first time, first
    // feature, features, weight.
    const uint32_t kChannel[8] = {6, 1, 1, 0, 1, 1, 3, 0};
    for (uint32_t k = 0; k < 8; ++k)
    {
        CHECK(WordRefused(maniLearnedTagChannels, 4 * k, kChannel[k], 4 * k),
              "a channel word refused");
    }
    CHECK(WordRefused(maniLearnedTagTimes, 0, BitsOf(61.0f), 0), "a time past the reach");
    CHECK(WordRefused(maniLearnedTagNormalization, 4, BitsOf(NAN), 4), "an offset not finite");
    CHECK(WordRefused(maniLearnedTagNormalization, 8, 0, 8), "a scale of 0");
    CHECK(WordRefused(maniLearnedTagNormalization, 12, BitsOf(-1.0f), 12), "a negative scale");
    CHECK(WordRefused(maniLearnedTagBones, 0, 3, 0), "a scaled bone past the bones");
    CHECK(WordRefused(maniLearnedTagBones, 4 + 20, BitsOf(INFINITY), 4 + 20),
          "a bone's scale not finite");
    // Networks: the Decompressor's layer count; the Stepper's tags and
    // the reserved word; a Projector's tags are its own.
    CHECK(WordRefused(maniLearnedTagNetworks, 0, 0, 0), "a network of no layers");
    CHECK(WordRefused(maniLearnedTagNetworks, 0, 17, 0), "layers past the limit");
    CHECK(WordRefusedIn(maniLearnedTagNetworks, 0, 3, maniLearnedTagLayers, 16),
          "a layer past the network's role");
    CHECK(WordRefused(maniLearnedTagNetworks, 20, 1, 20), "the Stepper's required tags");
    CHECK(WordRefused(maniLearnedTagNetworks, 24, 1, 24), "the Stepper's excluded tags");
    CHECK(WordRefused(maniLearnedTagNetworks, 44, 1, 44), "the reserved word");
    uint64_t at = 0;
    uint8_t tags[4];
    maniStoreU32(tags, 7);
    CHECK(Changed(maniLearnedTagNetworks, 36, tags, 4, true, &at).result == mani_success,
          "a Projector's tags");
    // Layers: the Decompressor's first input, an output not chained, a
    // last output not its role's, an output of 0.
    CHECK(WordRefused(maniLearnedTagLayers, 0, 4, 0), "a first input not the state");
    CHECK(WordRefused(maniLearnedTagLayers, 4, 9, 8), "layers not chained");
    CHECK(WordRefused(maniLearnedTagLayers, 12, 38, 12), "a last output not the role's");
    CHECK(WordRefused(maniLearnedTagLayers, 4, 0, 4), "an output of 0");
    CHECK(WordRefused(maniLearnedTagValues, 0, BitsOf(NAN), 0), "a value not finite");
    CHECK(WordRefused(maniLearnedTagValues, 20, 0, 20), "an input scale of 0");
    CHECK(WordRefused(maniLearnedTagWeights, 8, BitsOf(INFINITY), 8), "a weight not finite");
}

static void TestOpenNames(void)
{
    static const maniLearnedNameSource kTrack[1] = {{"xy", 2, 0}};
    static const maniLearnedNameSource kTwo[2] = {{"ab", 2, 0}, {"cd", 2, 0}};
    maniLearnedModelSource source = Source();
    source.floatTracks = kTrack;
    source.events = kTwo;
    CHECK(Build(&source).result == mani_success, "built");
    // Records: xy (0, 2), ab (2, 2), cd (4, 2).
    CHECK(WordRefused(maniLearnedTagNames, 0, 1, 0), "a name not where the last ended");
    CHECK(WordRefused(maniLearnedTagNames, 4, 0, 4), "an empty name");
    CHECK(WordRefused(maniLearnedTagNames, 4, 100, 4), "a name past the bytes");
    CHECK(WordRefusedIn(maniLearnedTagNames, 20, 1, maniLearnedTagNameBytes, UINT64_MAX),
          "names short of the bytes");
    uint8_t bad[1] = {0xFF};
    CHECK(RefusedIn(maniLearnedTagNameBytes, 2, bad, 1, maniLearnedTagNames, 12),
          "a name not UTF-8");
    CHECK(RefusedIn(maniLearnedTagNameBytes, 4, "ab", 2, maniLearnedTagNames, 20),
          "an event's name twice");
    uint64_t at = 0;
    CHECK(Changed(maniLearnedTagNameBytes, 0, "ab", 2, true, &at).result == mani_success,
          "a float track's name an event's too");
}

static void TestOpenNarrow(void)
{
    maniLearnedModelSource source = Source();
    source.weightFormat = maniLearnedWeightsInt16;
    CHECK(Build(&source).result == mani_success, "built");
    // The Decompressor's values: 2 * 5 + 2 * 39, its first layer's 8
    // biases, then its 5 inputs' scales.
    CHECK(WordRefused(maniLearnedTagValues, 4 * 96, BitsOf(-1.0f), 4 * 96),
          "an input's scale below 0");
    // 601 weights: the last two bytes pad.
    uint8_t pad[2] = {1, 0};
    CHECK(RefusedAt(maniLearnedTagWeights, 1202, pad, 2, 1202), "padding not 0");
    uint8_t any[2] = {0xFF, 0x7F};
    uint64_t at = 0;
    CHECK(Changed(maniLearnedTagWeights, 0, any, 2, true, &at).result == mani_success,
          "any 16-bit weight");
}

// A model of two scaled bones (a Decompressor of its biases) whose
// second Projector's two layers each fit its role: a scaled bone not
// after the last, and a layer no network counts.
static void TestOpenShapes(void)
{
    static const uint32_t kTwoScaled[2] = {1, 2};
    static const uint32_t kSpare[3] = {FEATURES, STATE, STATE};
    const maniLearnedLayerSource layer = {s_zeros, s_biases, STATE, 42};
    maniLearnedNetworkSource fixed = {&layer, s_zeros, kOnes, s_zeros, kOnes, 1, 0};
    maniLearnedLayerSource spare[2];
    maniLearnedProjectorSource projectors[2] = {s_projectors[0], {Network(spare, kSpare, 2), 0, 0}};
    maniLearnedModelSource source = Source();
    source.decompressor = &fixed;
    source.scaledBones = kTwoScaled;
    source.scaledBoneCount = 2;
    source.projectors = projectors;
    CHECK(Build(&source).result == mani_success, "built");
    CHECK(WordRefused(maniLearnedTagBones, 4, 1, 4), "scaled bones not increasing");
    CHECK(WordRefusedIn(maniLearnedTagNetworks, 3 * MANI_LEARNED_NETWORK_SIZE, 1,
                        maniLearnedTagHeader, 52),
          "a layer no network counts");
}

static bool WriteFile(const char* path, const void* data, size_t size)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return false;
    }
    size_t written = fwrite(data, 1, size, out);
    return fclose(out) == 0 && written == size;
}

int main(int argc, char** argv)
{
    Open();
    OpenDatabase();
    MakeNetworks();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        maniLearnedModelSource source = Source();
        bool ok = Build(&source).result == mani_success && WriteFile(argv[2], s_model, s_modelSize);
        return ok && s_failures == 0 ? 0 : 1;
    }
    TestRoundTrip();
    TestEvaluate();
    TestEvaluateRefusals();
    TestColumns();
    TestInt16();
    TestModelRefusals();
    TestNetworkRefusals();
    TestOpenHeader();
    TestOpenRecords();
    TestOpenNames();
    TestOpenNarrow();
    TestOpenShapes();
    return s_failures == 0 ? 0 : 1;
}
