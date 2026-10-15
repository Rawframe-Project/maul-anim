// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending and masks (mani-0020): random blends against a binary64
// reference of the same rule, the rest's fill, kinds, refusals with the
// output unchanged, masks from chains, and the bits pinned.

#include "pose_memory.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PINNED_BLEND_HASH 0x131c3b8fe854e21eull

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
    maniSkeletonSource source = {bones, kBones, 0};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success, "copies");
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
            maniPoseSetLocal(s_layer[i], b, RandomTransform());
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
        Components(maniPoseLocal(layers[i].pose, bone), v);
        double dot = sum[3] * v[3] + sum[4] * v[4] + sum[5] * v[5] + sum[6] * v[6];
        for (int c = 0; c < 10; ++c)
        {
            sum[c] += v[c] * w * (c >= 3 && c < 7 && dot < 0.0 ? -1.0 : 1.0);
        }
        total += w;
    }
    maniTransform restT = difference ? (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}
                                     : maniSkeletonRest(&s_skeleton, bone);
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
        Components(maniPoseLocal(s_out, b), got);
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
            float weight = Next() % 5 == 0 ? 0.0f : Uniform(0.0f, 2.0f);
            layers[i] = (maniBlendLayer){s_layer[i], weight, masked ? boneWeights[i] : NULL};
        }
        uint32_t count = 1 + Next() % kLayers;
        float threshold = Next() % 2 == 0 ? MANI_BLEND_THRESHOLD : 0.5f;
        CHECK(maniBlendPoses(&s_skeleton, layers, count, threshold, s_out) == mani_success,
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
        maniTransform got = maniPoseLocal(s_out, b);
        maniTransform rest = maniSkeletonRest(&s_skeleton, b);
        same = same && memcmp(&got, &rest, sizeof got) == 0;
    }
    return same;
}

static void TestRest(void)
{
    RandomizeLayers();
    CHECK(maniBlendPoses(&s_skeleton, NULL, 0, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "no layers");
    CHECK(SameAsRest(), "no layers: the rest, exactly");
    maniBlendLayer zero[2] = {{s_layer[0], 0.0f, NULL}, {s_layer[1], 0.0f, NULL}};
    CHECK(maniBlendPoses(&s_skeleton, zero, 2, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "weights 0");
    CHECK(SameAsRest(), "nothing weighed: the rest, exactly");
    // Half the threshold: halfway to the rest.
    maniBlendLayer half = {s_layer[0], 0.25f, NULL};
    CHECK(maniBlendPoses(&s_skeleton, &half, 1, 0.5f, s_out) == mani_success, "half");
    CHECK(Apart(&half, 1, 0.5f, false) < 2e-6, "the rest fills the shortfall");
    maniTransform t = maniPoseLocal(s_out, 3);
    maniTransform a = maniPoseLocal(s_layer[0], 3);
    maniTransform r = maniSkeletonRest(&s_skeleton, 3);
    CHECK(fabsf(t.translation.x - 0.5f * (a.translation.x + r.translation.x)) < 1e-6f,
          "translation halfway");
    // One layer alone gives itself back.
    maniBlendLayer one = {s_layer[1], 0.7f, NULL};
    CHECK(maniBlendPoses(&s_skeleton, &one, 1, MANI_BLEND_THRESHOLD, s_out) == mani_success, "one");
    bool itself = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        double want[10];
        double got[10];
        Components(maniPoseLocal(s_layer[1], b), want);
        Components(maniPoseLocal(s_out, b), got);
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
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "blends");
    maniTransform before[kBones];
    CHECK(maniPoseReadLocals(s_out, 0, kBones, before) == mani_success, "reads");
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPoseLocal(s_layer[1], b);
        t.rotation = (maniQuat){-t.rotation.x, -t.rotation.y, -t.rotation.z, -t.rotation.w};
        maniPoseSetLocal(s_layer[1], b, t);
    }
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "blends negated");
    bool same = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPoseLocal(s_out, b);
        same = same && memcmp(&t, &before[b], sizeof t) == 0;
    }
    CHECK(same, "a negated layer blends to the same bits");
}

static void TestKinds(void)
{
    RandomizeLayers();
    maniPoseSetKind(s_layer[0], true);
    maniPoseSetKind(s_layer[1], true);
    maniBlendLayer layers[2] = {{s_layer[0], 0.0f, NULL}, {s_layer[1], 0.0f, NULL}};
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "differences");
    CHECK(maniPoseIsDifference(s_out), "a difference out");
    bool identity = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniPoseLocal(s_out, b);
        identity = identity && t.translation.x == 0.0f && t.rotation.w == 1.0f &&
                   t.rotation.x == 0.0f && t.scale.y == 1.0f;
    }
    CHECK(identity, "a difference's rest is the identity");
    CHECK(maniComputeModel(s_out, &s_skeleton) == mani_errorInvalid, "a difference has no model");
    layers[0].weight = 0.3f;
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, 0.5f, s_out) == mani_success, "weighed");
    CHECK(Apart(layers, 2, 0.5f, true) < 2e-6, "toward the identity");
    maniPoseSetKind(s_layer[1], false);
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_errorInvalid,
          "kinds mixed");
    CHECK(maniPoseSetRest(s_out, &s_skeleton) == mani_success && !maniPoseIsDifference(s_out),
          "the rest makes a pose");
}

static void ExpectRefused(const maniBlendLayer* layers, uint32_t count, float threshold,
                          const char* what)
{
    memcpy(s_copy, s_poses[kLayers], sizeof s_copy);
    maniResult r = maniBlendPoses(&s_skeleton, layers, count, threshold, s_out);
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
    CHECK(maniBlendPoses(NULL, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_errorInvalid,
          "no skeleton");
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, NULL) == mani_errorInvalid,
          "no output");
    maniSkeleton other = s_skeleton;
    other.identity ^= 1;
    CHECK(maniBlendPoses(&other, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_errorInvalid,
          "another skeleton");
    // Weights above 1 are allowed.
    layers[0].weight = 3.0f;
    boneWeights[0] = 2.5f;
    CHECK(maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "weights past 1");
}

static void TestMask(void)
{
    float weights[kBones];
    // Bone 0 is the root; each bone's chain is found by the parents.
    const maniMaskChain chains[4] = {
        {1, 0.5f, true}, {1, 0.75f, false}, {2, 0.25f, true}, {2, 0.125f, true}};
    CHECK(maniBuildMask(&s_skeleton, chains, 4, 0.9f, weights, kBones) == mani_success, "builds");
    bool right = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        // The nearest of bones 1 and 2 above or at b, else the default.
        float want = 0.9f;
        bool settled = false;
        for (uint32_t a = b; !settled && a != MANI_NO_BONE; a = maniSkeletonParent(&s_skeleton, a))
        {
            settled = a == 1 || a == 2;
            want = a == 2 ? 0.125f : (a == 1 ? 0.5f : want);
        }
        want = b == 1 ? 0.75f : want;
        right = right && weights[b] == want;
    }
    CHECK(right, "nearest chain, the bone's own first, the later of two");
    float kept[kBones];
    memcpy(kept, weights, sizeof kept);
    const maniMaskChain past = {kBones, 0.5f, true};
    CHECK(maniBuildMask(&s_skeleton, &past, 1, 0.0f, weights, kBones) == mani_errorInvalid,
          "a bone past the skeleton");
    const maniMaskChain negative = {0, -0.5f, true};
    CHECK(maniBuildMask(&s_skeleton, &negative, 1, 0.0f, weights, kBones) == mani_errorInvalid,
          "a negative weight");
    CHECK(maniBuildMask(&s_skeleton, chains, 4, NAN, weights, kBones) == mani_errorInvalid,
          "a NaN default");
    CHECK(maniBuildMask(&s_skeleton, chains, 4, 0.0f, weights, kBones - 1) == mani_errorCapacity,
          "too small");
    CHECK(maniBuildMask(&s_skeleton, NULL, 1, 0.0f, weights, kBones) == mani_errorInvalid,
          "no chains");
    CHECK(memcmp(kept, weights, sizeof kept) == 0, "weights unchanged");
    CHECK(maniBuildMask(&s_skeleton, NULL, 0, 0.0f, weights, kBones) == mani_success &&
              weights[kBones - 1] == 0.0f,
          "no chains: the default");
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
    CHECK(maniBlendPoses(&s_skeleton, layers, 3, MANI_BLEND_THRESHOLD, s_out) == mani_success,
          "blends");
    maniTransform locals[kBones];
    CHECK(maniPoseReadLocals(s_out, 0, kBones, locals) == mani_success, "reads");
    uint64_t hash = maniHash64(MANI_HASH_INIT, locals, (int32_t)sizeof locals);
    CHECK(hash == PINNED_BLEND_HASH, "pinned blend");
    if (hash != PINNED_BLEND_HASH)
    {
        printf("blend hash 0x%016llx\n", (unsigned long long)hash);
    }
}

int main(void)
{
    BuildSkeleton();
    TestAgainstReference();
    TestRest();
    TestHemisphere();
    TestKinds();
    TestRefusals();
    TestMask();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
