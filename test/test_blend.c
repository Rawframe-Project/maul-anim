// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending and masks: random blends against a binary64
// reference of the same rule, the rest's fill, kinds, refusals with the
// output unchanged, masks from chains, and the bits pinned.

#include "pose_memory.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The blend scene: a blend of three layers, that blend as a difference
// added back onto a layer, the result's model pass, and its mirror;
// every platform and backend must give this value.
#define PINNED_SCENE_HASH 0x7a088ec6687bcea3ull

enum
{
    // Two groups of eight and a partial third.
    kBones = 21,
    kLayers = 3,
    kFileCapacity = 8192,
    kPoseCapacity = 8192,
};

static _Alignas(16) uint8_t s_file[kFileCapacity];
static _Alignas(32) uint8_t s_poses[kLayers + 1][kPoseCapacity];
static _Alignas(32) uint8_t s_copy[kPoseCapacity];
static char s_names[kBones][4];
static maniSkeleton s_skeleton;
static maniPose* s_layer[kLayers];
static maniPose* s_out;

static uint64_t s_random = 0x9E3779B97F4A7C15ull;

static uint32_t Next(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (uint32_t)(s_random >> 32);
}

static float Uniform(float low, float high)
{
    return low + (high - low) * ((float)(Next() >> 8) / 16777216.0f);
}

static maniQuat RandomRotation(void)
{
    float x = Uniform(-1.0f, 1.0f);
    float y = Uniform(-1.0f, 1.0f);
    float z = Uniform(-1.0f, 1.0f);
    float w = Uniform(-1.0f, 1.0f);
    float n = sqrtf(x * x + y * y + z * z + w * w);
    return (maniQuat){x / n, y / n, z / n, w / n};
}

static maniTransform RandomTransform(void)
{
    maniTransform t;
    t.translation = (maniVec3){Uniform(-2.0f, 2.0f), Uniform(-2.0f, 2.0f), Uniform(-2.0f, 2.0f)};
    t.rotation = RandomRotation();
    t.scale = (maniVec3){Uniform(0.5f, 1.5f), Uniform(0.5f, 1.5f), Uniform(0.5f, 1.5f)};
    return t;
}

// A random hierarchy, each bone's parent any earlier bone.
static void BuildSkeleton(void)
{
    maniBoneSource bones[kBones];
    for (uint32_t i = 0; i < kBones; ++i)
    {
        s_names[i][0] = 'b';
        s_names[i][1] = (char)('a' + i / 26);
        s_names[i][2] = (char)('a' + i % 26);
        bones[i].name = s_names[i];
        bones[i].nameLength = 3;
        bones[i].parent = i == 0 ? -1 : (int32_t)(Next() % i);
        bones[i].rest = RandomTransform();
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    maniSkeletonSource source = {bones, kBones};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds");
    size_t size = 0;
    CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success, "copies");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_file, size, &s_skeleton).result == mani_success, "opens");
    for (int i = 0; i <= kLayers; ++i)
    {
        maniPose* pose = NULL;
        CHECK(maniInitPose(&s_skeleton, s_poses[i], sizeof s_poses[i], &pose) == mani_success,
              "pose");
        if (i < kLayers)
        {
            s_layer[i] = pose;
        }
        else
        {
            s_out = pose;
        }
    }
}

static void RandomizeLayers(void)
{
    for (int i = 0; i < kLayers; ++i)
    {
        maniPoseSetKind(s_layer[i], false);
        for (uint32_t b = 0; b < kBones; ++b)
        {
            maniPose_SetLocal(s_layer[i], b, RandomTransform());
        }
    }
}

static void Components(maniTransform t, double v[10])
{
    const float values[10] = {t.translation.x, t.translation.y, t.translation.z, t.rotation.x,
                              t.rotation.y,    t.rotation.z,    t.rotation.w,    t.scale.x,
                              t.scale.y,       t.scale.z};
    for (int c = 0; c < 10; ++c)
    {
        v[c] = (double)values[c];
    }
}

// The rule in binary64: sums with the running sum's sign, the rest at
// the shortfall (copied when nothing weighed), normalized.
static void Reference(const maniBlendLayer* layers, uint32_t count, float threshold, uint32_t bone,
                      bool difference, double out[10])
{
    double sum[10] = {0};
    double total = 0.0;
    for (uint32_t i = 0; i < count; ++i)
    {
        double w = (double)layers[i].weight *
                   (layers[i].boneWeights != NULL ? (double)layers[i].boneWeights[bone] : 1.0);
        if (layers[i].weight == 0.0f)
        {
            continue;
        }
        double v[10];
        Components(maniPose_GetLocal(layers[i].pose, bone), v);
        double dot = sum[3] * v[3] + sum[4] * v[4] + sum[5] * v[5] + sum[6] * v[6];
        for (int c = 0; c < 10; ++c)
        {
            sum[c] += v[c] * w * (c >= 3 && c < 7 && dot < 0.0 ? -1.0 : 1.0);
        }
        total += w;
    }
    maniTransform restT = difference ? (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}
                                     : maniSkeleton_GetRest(&s_skeleton, bone);
    double rest[10];
    Components(restT, rest);
    if (total == 0.0)
    {
        memcpy(out, rest, sizeof rest);
        return;
    }
    if (total < (double)threshold)
    {
        double f = (double)threshold - total;
        double dot = sum[3] * rest[3] + sum[4] * rest[4] + sum[5] * rest[5] + sum[6] * rest[6];
        for (int c = 0; c < 10; ++c)
        {
            sum[c] += rest[c] * f * (c >= 3 && c < 7 && dot < 0.0 ? -1.0 : 1.0);
        }
        total = (double)threshold;
    }
    double n = sqrt(sum[3] * sum[3] + sum[4] * sum[4] + sum[5] * sum[5] + sum[6] * sum[6]);
    for (int c = 0; c < 10; ++c)
    {
        out[c] = c >= 3 && c < 7 ? sum[c] / n : sum[c] / total;
    }
}

static double Apart(const maniBlendLayer* layers, uint32_t count, float threshold, bool difference)
{
    double worst = 0.0;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        double want[10];
        double got[10];
        Reference(layers, count, threshold, b, difference, want);
        Components(maniPose_GetLocal(s_out, b), got);
        for (int c = 0; c < 10; ++c)
        {
            double e = fabs(want[c] - got[c]);
            worst = e > worst ? e : worst;
        }
    }
    return worst;
}

static void TestAgainstReference(void)
{
    float boneWeights[kLayers][kBones];
    double worst = 0.0;
    for (int round = 0; round < 200; ++round)
    {
        RandomizeLayers();
        maniBlendLayer layers[kLayers];
        for (int i = 0; i < kLayers; ++i)
        {
            bool masked = Next() % 2 == 0;
            for (uint32_t b = 0; b < kBones; ++b)
            {
                boneWeights[i][b] = Next() % 4 == 0 ? 0.0f : Uniform(0.0f, 1.0f);
            }
            float weight = Next() % 5 == 0 ? 0.0f : Uniform(0.0f, 1.0f);
            layers[i] = (maniBlendLayer){s_layer[i], weight, masked ? boneWeights[i] : NULL};
        }
        uint32_t count = 1 + Next() % kLayers;
        float threshold = Next() % 2 == 0 ? MANI_BLEND_THRESHOLD : 0.5f;
        CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, count, threshold, s_out) == mani_success,
              "blends");
        double e = Apart(layers, count, threshold, false);
        worst = e > worst ? e : worst;
    }
    CHECK(worst < 2e-6, "within binary32 of the binary64 rule");
}

static bool SameAsRest(void)
{
    bool same = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform got = maniPose_GetLocal(s_out, b);
        maniTransform rest = maniSkeleton_GetRest(&s_skeleton, b);
        same = same && memcmp(&got, &rest, sizeof got) == 0;
    }
    return same;
}

static void TestRest(void)
{
    RandomizeLayers();
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, NULL, 0, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "no layers");
    CHECK(SameAsRest(), "no layers: the rest, exactly");
    maniBlendLayer zero[2] = {{s_layer[0], 0.0f, NULL}, {s_layer[1], 0.0f, NULL}};
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, zero, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "weights 0");
    CHECK(SameAsRest(), "nothing weighed: the rest, exactly");
    // Half the threshold: halfway to the rest.
    maniBlendLayer half = {s_layer[0], 0.25f, NULL};
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, &half, 1, 0.5f, s_out) == mani_success, "half");
    CHECK(Apart(&half, 1, 0.5f, false) < 2e-6, "the rest fills the shortfall");
    maniTransform t = maniPose_GetLocal(s_out, 3);
    maniTransform a = maniPose_GetLocal(s_layer[0], 3);
    maniTransform r = maniSkeleton_GetRest(&s_skeleton, 3);
    CHECK(fabsf(t.translation.x - 0.5f * (a.translation.x + r.translation.x)) < 1e-6f,
          "translation halfway");
    // One layer alone gives itself back.
    maniBlendLayer one = {s_layer[1], 0.7f, NULL};
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, &one, 1, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "one");
    bool itself = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        double want[10];
        double got[10];
        Components(maniPose_GetLocal(s_layer[1], b), want);
        Components(maniPose_GetLocal(s_out, b), got);
        for (int c = 0; c < 10; ++c)
        {
            itself = itself && fabs(want[c] - got[c]) < 1e-6;
        }
    }
    CHECK(itself, "one layer: itself");
}

// A rotation and its negative are one turn: the result does not change.
static void TestHemisphere(void)
{
    RandomizeLayers();
    maniBlendLayer layers[2] = {{s_layer[0], 0.4f, NULL}, {s_layer[1], 0.6f, NULL}};
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "blends");
    maniTransform before[kBones];
    CHECK(maniPose_ReadLocals(s_out, 0, kBones, before) == mani_success, "reads");
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_layer[1], b);
        t.rotation = (maniQuat){-t.rotation.x, -t.rotation.y, -t.rotation.z, -t.rotation.w};
        maniPose_SetLocal(s_layer[1], b, t);
    }
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "blends negated");
    bool same = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_out, b);
        same = same && memcmp(&t, &before[b], sizeof t) == 0;
    }
    CHECK(same, "a negated layer blends to the same bits");
}

// Padding bones past the count stay the identity; zero and overflowing
// rotation sums give the identity; the model is marked stale.
static void TestEdges(void)
{
    RandomizeLayers();
    // Weights whose shares do not sum to exactly 1: blended identities
    // would drift from 1 in the padding's scales.
    maniBlendLayer three[3] = {
        {s_layer[0], 0.1f, NULL}, {s_layer[1], 0.2f, NULL}, {s_layer[2], 0.7f, NULL}};
    bool padding = true;
    for (int round = 0; round < 16; ++round)
    {
        three[0].weight = Uniform(0.01f, 1.0f);
        three[1].weight = Uniform(0.01f, 1.0f);
        CHECK(maniSkeleton_BlendPoses(&s_skeleton, three, 3, MANI_BLEND_THRESHOLD, s_out) ==
                  mani_success,
              "blends");
        const float* locals = maniPoseLocalsRead(s_out);
        for (uint32_t b = kBones; b < 24; ++b)
        {
            const float* slot = locals + (size_t)(b / 8) * 80 + b % 8;
            for (uint32_t c = 0; c < 10; ++c)
            {
                float want = c == 6 || c >= 7 ? 1.0f : 0.0f;
                padding = padding && slot[c * 8] == want;
            }
        }
    }
    CHECK(padding, "padding bones stay the identity");
    maniBlendLayer layers[2] = {{s_layer[0], 0.4f, NULL}, {s_layer[1], 0.6f, NULL}};
    const maniQuat sums[2] = {{0.0f, 0.0f, 0.0f, 0.0f}, {3e19f, 3e19f, 0.0f, 0.0f}};
    for (int i = 0; i < 2; ++i)
    {
        for (uint32_t b = 0; b < kBones; ++b)
        {
            for (int l = 0; l < 2; ++l)
            {
                maniTransform t = maniPose_GetLocal(s_layer[l], b);
                t.rotation = sums[i];
                maniPose_SetLocal(s_layer[l], b, t);
            }
        }
        CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
                  mani_success,
              "blends");
        bool identity = true;
        for (uint32_t b = 0; b < kBones; ++b)
        {
            maniQuat q = maniPose_GetLocal(s_out, b).rotation;
            identity = identity && q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 1.0f;
        }
        CHECK(identity, "a zero or overflowing rotation sum: the identity");
    }
    RandomizeLayers();
    CHECK(maniPose_SetRest(s_out, &s_skeleton) == mani_success && maniPose_IsModelCurrent(s_out),
          "a current model");
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "blends");
    CHECK(!maniPose_IsModelCurrent(s_out), "a blend marks the model stale");
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[2]) ==
              mani_success,
          "difference");
    CHECK(maniPose_ComputeModel(s_out, &s_skeleton) == mani_success, "composes");
    maniBlendLayer add = {s_layer[2], 0.5f, NULL};
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &add, 1) == mani_success, "adds");
    CHECK(!maniPose_IsModelCurrent(s_out), "adding marks the model stale");
    // A difference whose turn overflows binary32: the identity turn.
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_layer[0], b);
        t.rotation = (maniQuat){3e19f, 3e19f, 0.0f, 0.0f};
        maniPose_SetLocal(s_layer[0], b, t);
    }
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[2]) ==
              mani_success,
          "difference");
    bool turn = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniQuat q = maniPose_GetLocal(s_layer[2], b).rotation;
        turn = turn && q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 1.0f;
    }
    CHECK(turn, "an overflowing difference turn: the identity");
}

static void TestKinds(void)
{
    RandomizeLayers();
    maniPoseSetKind(s_layer[0], true);
    maniPoseSetKind(s_layer[1], true);
    maniBlendLayer layers[2] = {{s_layer[0], 0.0f, NULL}, {s_layer[1], 0.0f, NULL}};
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "differences");
    CHECK(maniPose_IsDifference(s_out), "a difference out");
    bool identity = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_out, b);
        identity = identity && t.translation.x == 0.0f && t.rotation.w == 1.0f &&
                   t.rotation.x == 0.0f && t.scale.y == 1.0f;
    }
    CHECK(identity, "a difference's rest is the identity");
    CHECK(maniPose_ComputeModel(s_out, &s_skeleton) == mani_errorInvalid,
          "a difference has no model");
    layers[0].weight = 0.3f;
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, 0.5f, s_out) == mani_success, "weighed");
    CHECK(Apart(layers, 2, 0.5f, true) < 2e-6, "toward the identity");
    maniPoseSetKind(s_layer[1], false);
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_errorInvalid,
          "kinds mixed");
    CHECK(maniPose_SetRest(s_out, &s_skeleton) == mani_success && !maniPose_IsDifference(s_out),
          "the rest makes a pose");
}

static void ExpectRefused(const maniBlendLayer* layers, uint32_t count, float threshold,
                          const char* what)
{
    memcpy(s_copy, s_poses[kLayers], sizeof s_copy);
    maniResult r = maniSkeleton_BlendPoses(&s_skeleton, layers, count, threshold, s_out);
    CHECK(r == mani_errorInvalid, what);
    CHECK(memcmp(s_copy, s_poses[kLayers], sizeof s_copy) == 0, "out unchanged");
}

static void TestRefusals(void)
{
    RandomizeLayers();
    float boneWeights[kBones];
    for (uint32_t b = 0; b < kBones; ++b)
    {
        boneWeights[b] = 1.0f;
    }
    maniBlendLayer layers[2] = {{s_layer[0], 0.5f, NULL}, {s_layer[1], 0.5f, boneWeights}};
    const float wrong[4] = {-1.0f, -0.0001f, INFINITY, NAN};
    for (int i = 0; i < 4; ++i)
    {
        layers[0].weight = wrong[i];
        ExpectRefused(layers, 2, MANI_BLEND_THRESHOLD, "a layer weight");
        layers[0].weight = 0.5f;
        boneWeights[kBones - 1] = wrong[i];
        ExpectRefused(layers, 2, MANI_BLEND_THRESHOLD, "a bone weight");
        boneWeights[kBones - 1] = 1.0f;
    }
    ExpectRefused(layers, 2, 0.0f, "threshold 0");
    ExpectRefused(layers, 2, -1.0f, "threshold negative");
    ExpectRefused(layers, 2, NAN, "threshold NaN");
    ExpectRefused(layers, 2, INFINITY, "threshold infinite");
    ExpectRefused(NULL, 1, MANI_BLEND_THRESHOLD, "no layers given");
    layers[1].pose = s_out;
    ExpectRefused(layers, 2, MANI_BLEND_THRESHOLD, "out among the layers");
    layers[1].pose = NULL;
    ExpectRefused(layers, 2, MANI_BLEND_THRESHOLD, "a NULL pose");
    layers[1].pose = s_layer[1];
    CHECK(maniSkeleton_BlendPoses(NULL, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_errorInvalid,
          "no skeleton");
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, NULL) ==
              mani_errorInvalid,
          "no output");
    maniSkeleton other = s_skeleton;
    other.identity ^= 1;
    CHECK(maniSkeleton_BlendPoses(&other, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_errorInvalid,
          "another skeleton");
    // Weights run from 0 to 1, both ends included.
    layers[0].weight = nextafterf(1.0f, 2.0f);
    ExpectRefused(layers, 2, MANI_BLEND_THRESHOLD, "a layer weight past 1");
    layers[0].weight = 1.0f;
    boneWeights[0] = nextafterf(1.0f, 2.0f);
    ExpectRefused(layers, 2, MANI_BLEND_THRESHOLD, "a bone weight past 1");
    boneWeights[0] = 1.0f;
    ExpectRefused(layers, 2, nextafterf(1.0f, 2.0f), "a threshold past 1");
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, 1.0f, s_out) == mani_success,
          "weights of 1");
    // Inputs at 2^127, half binary32's range: the blend stays within
    // them (shares summing a rounding past 1 cannot reach infinity).
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_layer[0], b);
        t.translation = (maniVec3){0x1p127f, -0x1p127f, 0x1p126f};
        t.scale = (maniVec3){0x1p127f, 0x1p100f, -0x1p127f};
        maniPose_SetLocal(s_layer[0], b, t);
        maniPose_SetLocal(s_layer[1], b, t);
    }
    layers[0].weight = 0.7f;
    layers[1].weight = 0.9f;
    boneWeights[3] = 0.3f;
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "huge inputs");
    bool finite = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_out, b);
        finite = finite && isfinite(t.translation.x) && isfinite(t.translation.y) &&
                 isfinite(t.scale.x) && isfinite(t.scale.z) && fabsf(t.translation.x) <= FLT_MAX;
    }
    CHECK(finite, "a blend of finite inputs is finite");
}

static void TestMask(void)
{
    float weights[kBones];
    // Bone 0 is the root; each bone's chain is found by the parents.
    const maniMaskChain chains[4] = {
        {1, 0.5f, true}, {1, 0.75f, false}, {2, 0.25f, true}, {2, 0.125f, true}};
    CHECK(maniSkeleton_BuildMask(&s_skeleton, chains, 4, 0.9f, weights, kBones, NULL) ==
              mani_success,
          "builds");
    bool right = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        // The nearest of bones 1 and 2 above or at b, else the default.
        float want = 0.9f;
        bool settled = false;
        for (uint32_t a = b; !settled && a != MANI_NO_BONE;
             a = maniSkeleton_GetParent(&s_skeleton, a))
        {
            settled = a == 1 || a == 2;
            want = a == 2 ? 0.125f : (a == 1 ? 0.5f : want);
        }
        want = b == 1 ? 0.75f : want;
        right = right && weights[b] == want;
    }
    CHECK(right, "nearest chain, the bone's own first, the later of two");
    // A bone with children: its own chain, listed before its subtree's,
    // stays its own, and its children take the subtree's.
    uint32_t parent = maniSkeleton_GetParent(&s_skeleton, kBones - 1);
    const maniMaskChain mixed[2] = {{parent, 0.25f, false}, {parent, 0.5f, true}};
    CHECK(maniSkeleton_BuildMask(&s_skeleton, mixed, 2, 0.0f, weights, kBones, NULL) ==
              mani_success,
          "builds");
    CHECK(weights[parent] == 0.25f && weights[kBones - 1] == 0.5f,
          "own first for the bone, the subtree for its children");
    // A bone's own chain alone: its children take the default.
    CHECK(maniSkeleton_BuildMask(&s_skeleton, mixed, 1, 0.75f, weights, kBones, NULL) ==
              mani_success,
          "builds");
    CHECK(weights[parent] == 0.25f && weights[kBones - 1] == 0.75f,
          "an own chain is not inherited");
    float kept[kBones];
    memcpy(kept, weights, sizeof kept);
    const maniMaskChain past = {kBones, 0.5f, true};
    CHECK(maniSkeleton_BuildMask(&s_skeleton, &past, 1, 0.0f, weights, kBones, NULL) ==
              mani_errorInvalid,
          "a bone past the skeleton");
    const maniMaskChain negative = {0, -0.5f, true};
    CHECK(maniSkeleton_BuildMask(&s_skeleton, &negative, 1, 0.0f, weights, kBones, NULL) ==
              mani_errorInvalid,
          "a negative weight");
    CHECK(maniSkeleton_BuildMask(&s_skeleton, chains, 4, NAN, weights, kBones, NULL) ==
              mani_errorInvalid,
          "a NaN default");
    uint32_t measured = 0;
    uint32_t refused = 0;
    CHECK(maniSkeleton_BuildMask(&s_skeleton, chains, 4, 0.0f, weights, kBones - 1, &refused) ==
                  mani_errorCapacity &&
              refused == kBones &&
              maniSkeleton_BuildMask(&s_skeleton, chains, 4, 0.0f, NULL, 0, &measured) ==
                  mani_errorCapacity &&
              measured == kBones &&
              maniSkeleton_BuildMask(&s_skeleton, chains, 4, 0.0f, NULL, 1, NULL) ==
                  mani_errorInvalid,
          "too small, measured");
    CHECK(maniSkeleton_BuildMask(&s_skeleton, NULL, 1, 0.0f, weights, kBones, NULL) ==
              mani_errorInvalid,
          "no chains");
    CHECK(memcmp(kept, weights, sizeof kept) == 0, "weights unchanged");
    CHECK(maniSkeleton_BuildMask(&s_skeleton, NULL, 0, 0.0f, weights, kBones, NULL) ==
                  mani_success &&
              weights[kBones - 1] == 0.0f,
          "no chains: the default");
}

// Additive layers in binary64 for one bone: the same formulas.
static void AddReference(const maniTransform* base, const maniBlendLayer* layers, uint32_t count,
                         uint32_t bone, double out[10])
{
    Components(*base, out);
    for (uint32_t i = 0; i < count; ++i)
    {
        double w = (double)layers[i].weight *
                   (layers[i].boneWeights != NULL ? (double)layers[i].boneWeights[bone] : 1.0);
        if (w == 0.0)
        {
            continue;
        }
        double d[10];
        Components(maniPose_GetLocal(layers[i].pose, bone), d);
        double sign = d[6] < 0.0 ? -1.0 : 1.0;
        double q[4] = {sign * d[3] * w, sign * d[4] * w, sign * d[5] * w,
                       (sign * d[6] - 1.0) * w + 1.0};
        double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int k = 0; k < 4; ++k)
        {
            q[k] /= n;
        }
        double x = out[3], y = out[4], z = out[5], r = out[6];
        out[3] = r * q[0] + x * q[3] + y * q[2] - z * q[1];
        out[4] = r * q[1] - x * q[2] + y * q[3] + z * q[0];
        out[5] = r * q[2] + x * q[1] - y * q[0] + z * q[3];
        out[6] = r * q[3] - x * q[0] - y * q[1] - z * q[2];
        for (int c = 0; c < 3; ++c)
        {
            out[c] += w * d[c];
            out[7 + c] *= 1.0 + w * (d[7 + c] - 1.0);
        }
    }
}

// Differences of random poses from random bases.
static void MakeDifferences(uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[i], s_out, s_layer[i]) ==
                  mani_success,
              "a difference in place");
    }
}

static void TestAdditive(void)
{
    float boneWeights[kBones];
    double worst = 0.0;
    for (int round = 0; round < 200; ++round)
    {
        RandomizeLayers();
        for (uint32_t b = 0; b < kBones; ++b)
        {
            maniPose_SetLocal(s_out, b, RandomTransform());
            boneWeights[b] = Uniform(0.0f, 1.0f);
        }
        maniPoseSetKind(s_out, false);
        MakeDifferences(2);
        for (uint32_t b = 0; b < kBones; ++b)
        {
            maniPose_SetLocal(s_out, b, RandomTransform());
        }
        maniTransform base[kBones];
        CHECK(maniPose_ReadLocals(s_out, 0, kBones, base) == mani_success, "reads");
        maniBlendLayer layers[2] = {{s_layer[0], Uniform(0.0f, 1.0f), NULL},
                                    {s_layer[1], Uniform(0.0f, 1.0f), boneWeights}};
        CHECK(maniPose_AddLayers(s_out, &s_skeleton, layers, 2) == mani_success, "adds");
        for (uint32_t b = 0; b < kBones; ++b)
        {
            double want[10];
            double got[10];
            AddReference(&base[b], layers, 2, b, want);
            Components(maniPose_GetLocal(s_out, b), got);
            for (int c = 0; c < 10; ++c)
            {
                double e = fabs(want[c] - got[c]);
                worst = e > worst ? e : worst;
            }
        }
    }
    CHECK(worst < 4e-6, "additive within binary32 of the binary64 formulas");
}

// A difference applied at 1 to its basis gives the pose back; at 0
// nothing changes; out may be the pose or the basis.
static void TestRoundTrip(void)
{
    RandomizeLayers();
    // s_layer[0] the pose, s_layer[1] the basis, s_layer[2] the difference.
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[2]) ==
              mani_success,
          "difference");
    CHECK(maniPose_IsDifference(s_layer[2]), "a difference");
    memcpy(s_poses[kLayers], s_poses[1], sizeof s_poses[1]);
    maniBlendLayer layer = {s_layer[2], 1.0f, NULL};
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_success, "adds");
    double worst = 0.0;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        double want[10];
        double got[10];
        Components(maniPose_GetLocal(s_layer[0], b), want);
        Components(maniPose_GetLocal(s_out, b), got);
        double dot = want[3] * got[3] + want[4] * got[4] + want[5] * got[5] + want[6] * got[6];
        for (int c = 0; c < 10; ++c)
        {
            double e = fabs(want[c] - (c >= 3 && c < 7 && dot < 0.0 ? -got[c] : got[c]));
            worst = e > worst ? e : worst;
        }
    }
    CHECK(worst < 1e-5, "the basis plus the difference: the pose");
    memcpy(s_poses[kLayers], s_poses[1], sizeof s_poses[1]);
    layer.weight = 0.0f;
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_success, "adds at 0");
    CHECK(memcmp(s_poses[kLayers], s_poses[1], sizeof s_poses[1]) == 0, "at 0: unchanged");
    // In place, either side, against out of place.
    memcpy(s_copy, s_poses[2], sizeof s_copy);
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[0]) ==
              mani_success,
          "out is the pose");
    CHECK(memcmp(s_poses[0] + 64, s_copy + 64, sizeof s_copy - 64) == 0, "the same bits");
    RandomizeLayers();
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[2]) ==
              mani_success,
          "out of place");
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[1]) ==
              mani_success,
          "out is the basis");
    CHECK(memcmp(s_poses[1] + 64, s_poses[2] + 64, sizeof s_copy - 64) == 0, "the same bits");
}

// A difference's turn and its negative add the same.
static void TestAdditiveHemisphere(void)
{
    RandomizeLayers();
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[2]) ==
              mani_success,
          "difference");
    maniPoseSetKind(s_out, false);
    memcpy(s_poses[kLayers], s_poses[1], sizeof s_poses[1]);
    maniBlendLayer layer = {s_layer[2], 0.6f, NULL};
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_success, "adds");
    memcpy(s_copy, s_poses[kLayers], sizeof s_copy);
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_layer[2], b);
        t.rotation = (maniQuat){-t.rotation.x, -t.rotation.y, -t.rotation.z, -t.rotation.w};
        maniPose_SetLocal(s_layer[2], b, t);
    }
    memcpy(s_poses[kLayers], s_poses[1], sizeof s_poses[1]);
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_success, "adds negated");
    CHECK(memcmp(s_copy, s_poses[kLayers], sizeof s_copy) == 0, "the same bits");
}

// At the top weight a halving scale difference halves the scale: the
// factor stays between 1 and d_s, never negative.
static void TestAdditiveBounds(void)
{
    RandomizeLayers();
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniPose_SetLocal(s_layer[2], b,
                          (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {0.5f, 0.5f, 0.5f}});
    }
    maniPoseSetKind(s_layer[2], true);
    maniPoseSetKind(s_out, false);
    memcpy(s_poses[kLayers], s_poses[0], sizeof s_poses[0]);
    maniBlendLayer layer = {s_layer[2], 1.0f, NULL};
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_success, "adds at 1");
    bool halved = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        halved = halved && maniPose_GetLocal(s_out, b).scale.x ==
                               0.5f * maniPose_GetLocal(s_layer[0], b).scale.x;
    }
    CHECK(halved, "w 1, d_s 0.5: half the scale");
}

static void TestAdditiveRefusals(void)
{
    RandomizeLayers();
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_layer[2]) ==
              mani_success,
          "difference");
    maniPoseSetKind(s_out, false);
    float boneWeights[kBones];
    for (uint32_t b = 0; b < kBones; ++b)
    {
        boneWeights[b] = 1.0f;
    }
    maniBlendLayer layer = {s_layer[2], 0.5f, boneWeights};
    const float wrong[4] = {-0.5f, INFINITY, NAN, 0x1.000002p0f};
    for (int i = 0; i < 4; ++i)
    {
        memcpy(s_copy, s_poses[kLayers], sizeof s_copy);
        layer.weight = wrong[i];
        CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_errorInvalid,
              "a layer weight");
        layer.weight = 0.5f;
        boneWeights[3] = wrong[i];
        CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_errorInvalid,
              "a bone weight");
        boneWeights[3] = 1.0f;
        CHECK(memcmp(s_copy, s_poses[kLayers], sizeof s_copy) == 0, "the pose unchanged");
    }
    layer.pose = s_layer[0];
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, &layer, 1) == mani_errorInvalid,
          "a pose as a layer");
    layer.pose = s_layer[2];
    CHECK(maniPose_AddLayers(s_layer[2], &s_skeleton, &layer, 1) == mani_errorInvalid,
          "a difference as the pose");
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, NULL, 1) == mani_errorInvalid, "no layers given");
    CHECK(maniPose_AddLayers(s_out, &s_skeleton, NULL, 0) == mani_success, "none");
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[2], s_layer[1], s_out) ==
              mani_errorInvalid,
          "a difference as the pose");
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[2], s_out) ==
              mani_errorInvalid,
          "a difference as the basis");
    maniTransform t = maniPose_GetLocal(s_layer[1], kBones - 1);
    maniTransform kept = t;
    memcpy(s_copy, s_poses[kLayers], sizeof s_copy);
    const float scales[2] = {0.0f, 1e-39f};
    for (int i = 0; i < 2; ++i)
    {
        t.scale.z = scales[i];
        maniPose_SetLocal(s_layer[1], kBones - 1, t);
        CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_out) ==
                  mani_errorInvalid,
              "a basis scale 0 or near it");
    }
    t = kept;
    t.translation.x = -FLT_MAX;
    maniPose_SetLocal(s_layer[1], kBones - 1, t);
    maniTransform far = maniPose_GetLocal(s_layer[0], kBones - 1);
    far.translation.x = FLT_MAX;
    maniPose_SetLocal(s_layer[0], kBones - 1, far);
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_layer[0], s_layer[1], s_out) ==
              mani_errorInvalid,
          "translations past binary32's range apart");
    CHECK(memcmp(s_copy, s_poses[kLayers], sizeof s_copy) == 0, "out unchanged");
    maniPose_SetLocal(s_layer[1], kBones - 1, kept);
    CHECK(maniSkeleton_ComputePoseDifference(NULL, s_layer[0], s_layer[1], s_out) ==
              mani_errorInvalid,
          "no skeleton");
}

static void TestPinned(void)
{
    s_random = 0x0123456789ABCDEFull;
    RandomizeLayers();
    float boneWeights[kBones];
    for (uint32_t b = 0; b < kBones; ++b)
    {
        boneWeights[b] = Uniform(0.0f, 1.0f);
    }
    maniBlendLayer layers[3] = {
        {s_layer[0], 0.3f, NULL}, {s_layer[1], 0.6f, boneWeights}, {s_layer[2], 0.2f, NULL}};
    CHECK(maniSkeleton_BlendPoses(&s_skeleton, layers, 3, MANI_BLEND_THRESHOLD, s_out) ==
              mani_success,
          "blends");
    maniTransform locals[kBones];
    CHECK(maniPose_ReadLocals(s_out, 0, kBones, locals) == mani_success, "reads");
    uint64_t hash = maniHash64(MANI_HASH_INIT, locals, (int32_t)sizeof locals);
    // That blend as a difference from layer 2, added back onto layer 0.
    CHECK(maniSkeleton_ComputePoseDifference(&s_skeleton, s_out, s_layer[2], s_out) == mani_success,
          "difference");
    maniBlendLayer add = {s_out, 0.7f, boneWeights};
    CHECK(maniPose_AddLayers(s_layer[0], &s_skeleton, &add, 1) == mani_success, "adds");
    CHECK(maniPose_ReadLocals(s_layer[0], 0, kBones, locals) == mani_success, "reads");
    hash = maniHash64(hash, locals, (int32_t)sizeof locals);
    maniAffine models[kBones];
    CHECK(maniPose_ComputeModel(s_layer[0], &s_skeleton) == mani_success &&
              maniPose_ReadModels(s_layer[0], 0, kBones, models) == mani_success,
          "composes");
    hash = maniHash64(hash, models, (int32_t)sizeof models);
    // Mirrored across x, two pairs of bones swapping sides.
    const maniMirrorPairSource pairs[2] = {{1, 2}, {3, 4}};
    maniMirrorMapSource source = {&s_skeleton, NULL, pairs, NULL, NULL,           NULL,
                                  2,           0,    0,     0,    maniMirrorAxisX};
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    static _Alignas(16) uint8_t file[4096];
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success &&
              maniBuildMirrorMap(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success,
          "mirror map built");
    maniDestroyBuilder(builder);
    maniMirrorMap map;
    CHECK(maniOpenMirrorMap(file, size, &map).result == mani_success, "mirror map opens");
    CHECK(maniMirrorMap_MirrorPose(&map, &s_skeleton, s_layer[0], s_layer[1]) == mani_success,
          "mirrors");
    CHECK(maniPose_ReadLocals(s_layer[1], 0, kBones, locals) == mani_success, "reads");
    hash = maniHash64(hash, locals, (int32_t)sizeof locals);
    CHECK(hash == PINNED_SCENE_HASH, "the scene's bits the same on every platform");
    if (hash != PINNED_SCENE_HASH)
    {
        printf("scene hash 0x%016llx\n", (unsigned long long)hash);
    }
}

int main(void)
{
    BuildSkeleton();
    TestAgainstReference();
    TestRest();
    TestHemisphere();
    TestEdges();
    TestKinds();
    TestRefusals();
    TestMask();
    TestAdditive();
    TestRoundTrip();
    TestAdditiveHemisphere();
    TestAdditiveRefusals();
    TestAdditiveBounds();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
