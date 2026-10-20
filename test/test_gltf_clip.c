// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF animations as clips (mani-0010): a rig with an animated placement
// node, a slerped hip, a cubic spine, a stepped head, morph weights on a
// skinned mesh, a pointer channel, a channel for another rig and one
// with no target. The clip's samples are checked against glTF's Appendix
// C evaluated here in binary64, locals with placement reported, model
// transforms with it folded and the basis changed; then each refusal.

#include "test_gltf_sweep.h"
#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static char s_json[65536];
static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_clipFile[65536];
static _Alignas(32) uint8_t s_poseMemory[16384];

static const double kPi = 3.14159265358979323846;

// The keys, laid out as the buffer holds them.
//   times A: 0, 0.5, 1; times B: 0, 1; times C: 0, 0.4
//   hip rotation (linear, A): 0, 45, 90 degrees about y
//   spine translation (cubic, B): in, value, out per key
//   head scale (step, C): (1 1 1), (2 2 2)
//   scene translation (linear, B): (0 0 5), (1 0 5)
//   body weights (linear, B): (0 1), (1 0.5)
//   base color (linear, B): (1 0 0 1), (0 1 0 1)
//   prop translation (linear, B): (3 0 0), (4 0 0)
static float s_keys[96];

enum
{
    kTimesA = 0,
    kTimesB = 3,
    kTimesC = 5,
    kHip = 7,
    kSpine = 19,
    kHead = 37,
    kScene = 43,
    kWeights = 49,
    kColor = 53,
    kProp = 61,
    // Eight int16: a normalized rotation from identity to a half turn
    // about y; its first six as a normalized translation.
    kShorts = 67,
    kKeyFloats = 71,
};

static const float kSpineKeys[18] = {0, 0, 0, 0, 0.5f, 0, 1, 0, 0, 0, 1, -1, 0, 0.8f, 0, 0, 0, 0};

static void FillKeys(void)
{
    const float times[7] = {0, 0.5f, 1, 0, 1, 0, 0.4f};
    memcpy(s_keys, times, sizeof times);
    for (int k = 0; k < 3; ++k)
    {
        double half = 0.5 * (double)k * kPi / 4.0;
        float q[4] = {0, (float)sin(half), 0, (float)cos(half)};
        memcpy(s_keys + kHip + 4 * k, q, sizeof q);
    }
    memcpy(s_keys + kSpine, kSpineKeys, sizeof kSpineKeys);
    const float head[6] = {1, 1, 1, 2, 2, 2};
    memcpy(s_keys + kHead, head, sizeof head);
    const float scene[6] = {0, 0, 5, 1, 0, 5};
    memcpy(s_keys + kScene, scene, sizeof scene);
    const float weights[4] = {0, 1, 1, 0.5f};
    memcpy(s_keys + kWeights, weights, sizeof weights);
    const float color[8] = {1, 0, 0, 1, 0, 1, 0, 1};
    memcpy(s_keys + kColor, color, sizeof color);
    const float prop[6] = {3, 0, 0, 4, 0, 0};
    memcpy(s_keys + kProp, prop, sizeof prop);
    const int16_t shorts[8] = {0, 0, 0, 32767, 0, 32767, 0, 0};
    memcpy(s_keys + kShorts, shorts, sizeof shorts);
}

static void Base64(const uint8_t* bytes, size_t size, char* out)
{
    static const char kDigits[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < size; i += 3)
    {
        uint32_t group = (uint32_t)bytes[i] << 16;
        group |= i + 1 < size ? (uint32_t)bytes[i + 1] << 8 : 0;
        group |= i + 2 < size ? bytes[i + 2] : 0;
        out[o++] = kDigits[group >> 18];
        out[o++] = kDigits[(group >> 12) & 63];
        out[o++] = i + 1 < size ? kDigits[(group >> 6) & 63] : '=';
        out[o++] = i + 2 < size ? kDigits[group & 63] : '=';
    }
    out[o] = '\0';
}

// The document, with `channels` and `samplers` given (the default rig's
// when NULL) and `scene` the scene node's extra properties.
static void Document(const char* samplers, const char* channels, const char* sceneExtra)
{
    FillKeys();
    char digits[600];
    Base64((const uint8_t*)s_keys, kKeyFloats * sizeof(float), digits);
    static const char kSamplers[] =
        "{\"input\":0,\"output\":3},"
        "{\"input\":1,\"output\":4,\"interpolation\":\"CUBICSPLINE\"},"
        "{\"input\":2,\"output\":5,\"interpolation\":\"STEP\"},"
        "{\"input\":1,\"output\":6},{\"input\":1,\"output\":7},{\"input\":1,\"output\":8},"
        "{\"input\":1,\"output\":9}";
    static const char kChannels[] =
        "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"rotation\"}},"
        "{\"sampler\":1,\"target\":{\"node\":2,\"path\":\"translation\"}},"
        "{\"sampler\":2,\"target\":{\"node\":3,\"path\":\"scale\"}},"
        "{\"sampler\":3,\"target\":{\"node\":0,\"path\":\"translation\"}},"
        "{\"sampler\":4,\"target\":{\"node\":5,\"path\":\"weights\"}},"
        "{\"sampler\":5,\"target\":{\"path\":\"pointer\",\"extensions\":{\"KHR_animation_pointer\":"
        "{\"pointer\":\"/materials/0/pbrMetallicRoughness/baseColorFactor\"}}}},"
        "{\"sampler\":6,\"target\":{\"node\":6,\"path\":\"translation\"}},"
        "{\"sampler\":6,\"target\":{\"path\":\"translation\"}}";
    snprintf(
        s_json, sizeof s_json,
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"nodes\":[{\"name\":\"Scene\",\"translation\":[0,0,5],\"children\":[1,5]%s},"
        "{\"name\":\"hip\",\"translation\":[0,1,0],\"children\":[2,4]},"
        "{\"name\":\"spine\",\"translation\":[0,0.5,0],\"children\":[3]},"
        "{\"name\":\"head\",\"translation\":[0,0.3,0]},"
        "{\"name\":\"leg\",\"translation\":[0.2,-0.5,0]},"
        "{\"name\":\"body\",\"mesh\":0,\"skin\":0},{\"name\":\"prop\",\"translation\":[3,0,0]},"
        "{\"name\":\"still\",\"matrix\":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}],"
        "\"skins\":[{\"joints\":[1,3,4]},{\"joints\":[6]}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{},\"targets\":[{},{}]}],"
        "\"extras\":{\"targetNames\":[\"smile\",\"frown\"]}},"
        "{\"primitives\":[{\"attributes\":{}}]}],"
        "\"buffers\":[{\"byteLength\":%d,\"uri\":\"data:application/octet-stream;base64,%s\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteLength\":%d},"
        "{\"buffer\":0,\"byteOffset\":12,\"byteLength\":12,\"byteStride\":8},"
        "{\"buffer\":0,\"byteOffset\":12,\"byteLength\":8,\"byteStride\":4}],"
        "\"accessors\":["
        "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"SCALAR\",\"min\":[0],"
        "\"max\":[1]},"
        "{\"bufferView\":0,\"byteOffset\":12,\"componentType\":5126,\"count\":2,\"type\":"
        "\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":20,\"componentType\":5126,\"count\":2,\"type\":"
        "\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":28,\"componentType\":5126,\"count\":3,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":76,\"componentType\":5126,\"count\":6,\"type\":\"VEC3\"},"
        "{\"bufferView\":0,\"byteOffset\":148,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":172,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":196,\"componentType\":5126,\"count\":4,\"type\":"
        "\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":212,\"componentType\":5126,\"count\":2,\"type\":\"VEC4\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":244,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}"
        ","
        // 10: bytes for times; 11: times in a strided view; 12, 13: the
        // shorts; 14: one time.
        "{\"bufferView\":0,\"byteOffset\":12,\"componentType\":5121,\"count\":2,\"type\":"
        "\"SCALAR\"},"
        "{\"bufferView\":1,\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":268,\"componentType\":5122,\"normalized\":true,"
        "\"count\":2,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":268,\"componentType\":5123,\"normalized\":true,"
        "\"count\":2,\"type\":\"VEC3\"},"
        "{\"bufferView\":0,\"componentType\":5126,\"count\":1,\"type\":\"SCALAR\"},"
        // 15: two 4x4 matrices, for a pointer of sixteen components; 16:
        // accessor 1's times in a view of their own stride.
        "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"MAT4\"},"
        "{\"bufferView\":2,\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\"}"
        "],"
        "\"animations\":[{\"name\":\"walk\",\"samplers\":[%s],\"channels\":[%s]}]}",
        sceneExtra, (int)(kKeyFloats * sizeof(float)), digits, (int)(kKeyFloats * sizeof(float)),
        samplers != NULL ? samplers : kSamplers, channels != NULL ? channels : kChannels);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static maniGltf* Open(maniBuilder* builder)
{
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, s_json, strlen(s_json), &gltf);
    CHECK(r.result == mani_success, "opens");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu item %u near: %.50s\n", r.reason,
               (unsigned long long)r.offset, r.item, s_json + r.offset);
    }
    return gltf;
}

static bool BuildSkeleton(maniBuilder* builder, const maniGltf* gltf, uint32_t index,
                          const maniGltfOptions* options, maniSkeleton* skeleton)
{
    size_t size = 0;
    return maniGltfBuildSkeleton(builder, gltf, index, options).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, skeleton).result == mani_success;
}

static bool OpenClip(maniBuilder* builder, maniClip* clip)
{
    size_t size = 0;
    return maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
           maniOpenClip(s_clipFile, size, clip).result == mani_success;
}

// glTF's evaluation at a time, in glTF's basis.
static void HipRotation(double t, double q[4])
{
    double angle = 0.5 * (t <= 0 ? 0 : (t >= 1 ? 1 : t)) * kPi / 2.0;
    q[0] = 0;
    q[1] = sin(angle);
    q[2] = 0;
    q[3] = cos(angle);
}

static void SpineTranslation(double t, double v[3])
{
    double u = t <= 0 ? 0 : (t >= 1 ? 1 : t);
    double h00 = 2 * u * u * u - 3 * u * u + 1;
    double h10 = u * u * u - 2 * u * u + u;
    double h01 = -2 * u * u * u + 3 * u * u;
    double h11 = u * u * u - u * u;
    for (int c = 0; c < 3; ++c)
    {
        v[c] = h00 * (double)kSpineKeys[3 + c] + h10 * (double)kSpineKeys[6 + c] +
               h01 * (double)kSpineKeys[12 + c] + h11 * (double)kSpineKeys[9 + c];
    }
}

static double HeadScale(double t)
{
    return t < (double)0.4f ? 1.0 : 2.0;
}

static double SceneX(double t)
{
    return t <= 0 ? 0 : (t >= 1 ? 1 : t);
}

static bool Near(float a, double b, double tolerance)
{
    return fabs((double)a - b) <= tolerance;
}

static void TestLocals(void)
{
    Document(NULL, NULL, "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    CHECK(maniGltfAnimationCount(gltf) == 1, "one animation");
    uint32_t length = 0;
    const char* name = maniGltfAnimationName(gltf, 0, &length);
    CHECK(length == 4 && memcmp(name, "walk", 4) == 0, "its name");
    CHECK(maniGltfAnimationName(gltf, 1, &length) == NULL && length == 0, "past the count");
    uint32_t bound = 0;
    maniImportResult r = maniGltfBuildClip(
        builder, gltf, 0, &skeleton, &options,
        &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopRepeat}, &bound);
    CHECK(r.result == mani_success, "builds");
    if (r.result != mani_success)
    {
        printf("  reason %u item %u\n", r.reason, r.item);
    }
    CHECK(bound == 5, "hip, spine, head, weights, pointer; not the scene, prop or untargeted");
    maniClip clip;
    CHECK(OpenClip(builder, &clip), "opens");
    CHECK(maniClipDuration(&clip) == 1.0f && maniClipSampleCount(&clip) == 11, "a second");
    CHECK(maniClipFloatTrackCount(&clip) == 6, "two weights, four color components");
    uint32_t smile = maniClipFindFloatTrack(&clip, "body/smile", 10);
    uint32_t frown = maniClipFindFloatTrack(&clip, "body/frown", 10);
    const char kGreen[] = "/materials/0/pbrMetallicRoughness/baseColorFactor/1";
    uint32_t green = maniClipFindFloatTrack(&clip, kGreen, sizeof kGreen - 1);
    CHECK(smile != MANI_NO_TRACK && frown != MANI_NO_TRACK && green != MANI_NO_TRACK,
          "float tracks named");
    uint32_t hip = maniSkeletonFindBone(&skeleton, "hip", 3);
    uint32_t spine = maniSkeletonFindBone(&skeleton, "spine", 5);
    uint32_t head = maniSkeletonFindBone(&skeleton, "head", 4);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    for (int k = 0; k <= 10; ++k)
    {
        // The clip's sample times, k / 10 in binary64, as its builder takes them.
        double t = (double)k / 10.0;
        CHECK(maniSampleClip(&clip, &skeleton, (float)t, pose) == mani_success, "samples");
        double q[4];
        HipRotation(t, q);
        maniTransform h = maniPoseLocal(pose, hip);
        CHECK(Near(h.rotation.y, q[1], 1e-6) && Near(h.rotation.w, q[3], 1e-6), "the hip's slerp");
        CHECK(h.translation.y == 1.0f, "the hip's rest translation");
        double v[3];
        SpineTranslation(t, v);
        maniTransform s = maniPoseLocal(pose, spine);
        CHECK(Near(s.translation.x, v[0], 1e-6) && Near(s.translation.y, v[1], 1e-6) &&
                  Near(s.translation.z, v[2], 1e-6),
              "the spine's cubic");
        CHECK(maniPoseLocal(pose, head).scale.x == (float)HeadScale(t), "the head's step");
        float values[6];
        CHECK(maniSampleClipFloats(&clip, (float)t, values, 6, NULL) == mani_success, "floats");
        CHECK(Near(values[smile], t, 1e-6) && Near(values[frown], 1 - 0.5 * t, 1e-6), "weights");
        CHECK(Near(values[green], t, 1e-6), "a pointer component");
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static maniImportResult BuildWith(maniBuilder* builder, maniGltf* gltf,
                                  const maniSkeleton* skeleton, maniImportClipOptions clipOptions)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    return maniGltfBuildClip(builder, gltf, 0, skeleton, &options, &clipOptions, NULL);
}

// The hip's turn about y taken out as root motion and restored; the clip
// settings checked; an additive clip.
static void TestClipOptions(void)
{
    Document(NULL, NULL, "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    uint32_t hip = maniSkeletonFindBone(&skeleton, "hip", 3);
    maniRootMotionSource turn = {hip, 0, maniRootRotationTwist, 1, 0, 0, 0, 0};
    maniImportClipOptions settings = maniDefaultImportClipOptions();
    settings.sampleRate = 10.0f;
    settings.rootMotion = &turn;
    maniClip clip;
    CHECK(BuildWith(builder, gltf, &skeleton, settings).result == mani_success &&
              OpenClip(builder, &clip),
          "builds with root motion");
    maniRootMotionSource read;
    maniRootMove move;
    CHECK(maniClipRootMotionSource(&clip, &read) && read.bone == hip &&
              maniClipRootMotion(&clip, 0.0f, 1.0f, &move) == mani_success &&
              Near(move.rotation.y, sqrt(0.5), 1e-6) && Near(move.rotation.w, sqrt(0.5), 1e-6),
          "the quarter turn taken out");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    double q[4];
    HipRotation(0.6, q);
    CHECK(maniSampleClip(&clip, &skeleton, 0.6f, pose) == mani_success &&
              Near(maniPoseLocal(pose, hip).rotation.w, 1.0, 1e-6),
          "in place");
    CHECK(maniRestoreRootMotion(&clip, &skeleton, 0.6f, pose) == mani_success &&
              Near(maniPoseLocal(pose, hip).rotation.y, q[1], 1e-6) &&
              Near(maniPoseLocal(pose, hip).rotation.w, q[3], 1e-6),
          "restored");
    // The leg is not animated: the builder refuses it.
    turn.bone = maniSkeletonFindBone(&skeleton, "leg", 3);
    maniImportResult r = BuildWith(builder, gltf, &skeleton, settings);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBuild &&
              r.item == maniBuildReasonRootMotion,
          "a root motion bone without a track");
    turn.bone = hip;
    settings.additive = maniAdditiveRest;
    r = BuildWith(builder, gltf, &skeleton, settings);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBuild &&
              r.item == maniBuildReasonRootMotion,
          "root motion on an additive clip");
    settings.rootMotion = NULL;
    CHECK(BuildWith(builder, gltf, &skeleton, settings).result == mani_success &&
              OpenClip(builder, &clip) && maniClipAdditiveBasis(&clip) == maniAdditiveRest,
          "an additive clip");
    uint32_t misuses = maniBuilderMisuses(builder);
    maniImportClipOptions bad[8];
    for (int i = 0; i < 8; ++i)
    {
        bad[i] = maniDefaultImportClipOptions();
    }
    maniRootMotionSource past = {maniSkeletonBoneCount(&skeleton), 1, 0, 0, 0, 0, 0, 0};
    maniRootMotionSource none = {hip, 0, maniRootRotationNone, 0, 0, 0, 0, 0};
    bad[0].sampleRate = -1.0f;
    bad[1].sampleRate = NAN;
    bad[2].loop = 2;
    bad[3].additive = 3;
    bad[4].reserved = 1;
    bad[5].rootMotion = &past;
    bad[6].rootMotion = &none;
    bad[7].sampleRate = INFINITY;
    bool refused = true;
    for (int i = 0; i < 8; ++i)
    {
        refused =
            refused && BuildWith(builder, gltf, &skeleton, bad[i]).result == mani_errorInvalid;
    }
    CHECK(refused && maniBuilderMisuses(builder) == misuses + 8, "settings out of range");
    CHECK(maniGltfBuildClip(builder, gltf, 0, &skeleton, &options, NULL, NULL).result ==
              mani_errorInvalid,
          "no settings");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// B M B^-1 for the signed permutation x' = x, y' = -z, z' = y.
static maniAffine ZUp(maniAffine a)
{
    static const int p[3] = {0, 2, 1};
    static const float s[3] = {1, -1, 1};
    maniAffine out;
    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
        {
            out.m[j * 3 + i] = s[i] * s[j] * a.m[p[j] * 3 + p[i]];
        }
        out.m[9 + j] = s[j] * a.m[9 + p[j]];
    }
    return out;
}

static maniAffine Trs(const double t[3], const double q[4], const double s[3])
{
    return maniAffineFromTrs(maniVec3Make((float)t[0], (float)t[1], (float)t[2]),
                             (maniQuat){(float)q[0], (float)q[1], (float)q[2], (float)q[3]},
                             maniVec3Make((float)s[0], (float)s[1], (float)s[2]));
}

static void TestFoldedModels(void)
{
    Document(NULL, NULL, "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 1};
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    uint32_t bound = 0;
    maniImportResult r = maniGltfBuildClip(
        builder, gltf, 0, &skeleton, &options,
        &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, &bound);
    CHECK(r.result == mani_success && bound == 6, "the scene's channel composed in");
    maniClip clip;
    CHECK(OpenClip(builder, &clip), "opens");
    uint32_t head = maniSkeletonFindBone(&skeleton, "head", 4);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    double worst = 0;
    for (int k = 0; k <= 10; ++k)
    {
        // The clip's sample times, k / 10 in binary64, as its builder takes them.
        double t = (double)k / 10.0;
        CHECK(maniSampleClip(&clip, &skeleton, (float)t, pose) == mani_success, "samples");
        CHECK(maniComputeModel(pose, &skeleton) == mani_success, "model");
        // The head's world in glTF's basis: scene, hip, spine, head.
        const double one[3] = {1, 1, 1};
        const double none[4] = {0, 0, 0, 1};
        double sceneT[3] = {SceneX(t), 0, 5};
        double hipT[3] = {0, 1, 0};
        double hipQ[4];
        HipRotation(t, hipQ);
        double spineT[3];
        SpineTranslation(t, spineT);
        double headT[3] = {0, 0.3, 0};
        double headS[3] = {HeadScale(t), HeadScale(t), HeadScale(t)};
        maniAffine chain[4] = {Trs(sceneT, none, one), Trs(hipT, hipQ, one), Trs(spineT, none, one),
                               Trs(headT, none, headS)};
        maniAffine world = chain[0];
        for (int i = 1; i < 4; ++i)
        {
            world = maniAffineMul(&world, &chain[i]);
        }
        maniAffine expected = ZUp(world);
        maniAffine model = maniPoseModel(pose, head);
        for (int i = 0; i < 12; ++i)
        {
            worst = fmax(worst, fabs((double)model.m[i] - (double)expected.m[i]));
        }
    }
    CHECK(worst < 2e-5, "the head's world, placement folded and z up");
    if (worst >= 2e-5)
    {
        printf("  worst %g\n", worst);
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestAnotherRig(void)
{
    Document(NULL, NULL, "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 1, &options, &skeleton), "the prop");
    uint32_t bound = 0;
    CHECK(maniGltfBuildClip(builder, gltf, 0, &skeleton, &options,
                            &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                            &bound)
                  .result == mani_success,
          "builds");
    CHECK(bound == 2, "the prop's channel and the pointer");
    maniClip clip;
    CHECK(OpenClip(builder, &clip), "opens");
    CHECK(maniClipDuration(&clip) == 1.0f && maniClipSampleRate(&clip) == 30.0f,
          "the animation's duration, the default rate");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    CHECK(maniSampleClip(&clip, &skeleton, 0.5f, pose) == mani_success, "samples");
    CHECK(Near(maniPoseLocal(pose, 0).translation.x, 3.5, 1e-6), "the prop moves");
    // Misuse.
    CHECK(maniGltfBuildClip(builder, gltf, 1, &skeleton, &options,
                            &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                            NULL)
                  .result == mani_errorInvalid,
          "an animation past the count");
    CHECK(maniGltfBuildClip(builder, gltf, 0, NULL, &options,
                            &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                            NULL)
                  .result == mani_errorInvalid,
          "no skeleton");
    CHECK(maniBuilderMisuses(builder) == 2, "misuses");
    CHECK(maniGltfBuildClip(NULL, gltf, 0, &skeleton, &options,
                            &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                            NULL)
                  .result == mani_errorInvalid,
          "no builder");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void ExpectOpenRefusal(maniImportReason reason, const char* at, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, s_json, strlen(s_json), &gltf);
    const char* found = at != NULL ? strstr(s_json, at) : NULL;
    bool right = r.result == mani_errorFormat && r.reason == reason &&
                 (at == NULL || (found != NULL && r.offset == (uint64_t)(found - s_json)));
    CHECK(right, what);
    if (!right)
    {
        printf("  reason %u offset %llu near: %.50s\n", r.reason, (unsigned long long)r.offset,
               r.offset < strlen(s_json) ? s_json + r.offset : "");
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    static const char kOne[] = "{\"input\":1,\"output\":6}";
    Document(kOne, "{\"sampler\":0,\"target\":{\"node\":7,\"path\":\"translation\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, "{\"sampler\":0", "TRS on a matrix node");
    Document(kOne,
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}},"
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}}",
             "");
    ExpectOpenRefusal(maniImportReasonProperty,
                      "{\"sampler\":0,\"target\":{\"node\":1,\"path\":"
                      "\"translation\"}}]",
                      "a target twice");
    Document("{\"input\":1,\"output\":7}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"weights\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "weights without morph targets");
    Document("{\"input\":0,\"output\":6}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "three keys, two values");
    Document("{\"input\":1,\"output\":6,\"interpolation\":\"SMOOTH\"}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, "SMOOTH", "an interpolation glTF lacks");
    Document("{\"input\":6,\"output\":6}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "input not scalar");
    Document(kOne, "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"matrix\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, "matrix\"}", "a path glTF lacks");
    Document(kOne, "{\"sampler\":1,\"target\":{\"node\":1,\"path\":\"translation\"}}", "");
    ExpectOpenRefusal(maniImportReasonIndex, "1,\"target", "a sampler past the count");
    Document(kOne,
             "{\"sampler\":0,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"nodes/1/translation\"}}}}",
             "");
    ExpectOpenRefusal(maniImportReasonProperty, "nodes/1/translation", "a pointer without /");
    Document(kOne,
             "{\"sampler\":0,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/nodes/99/translation\"}}}}",
             "");
    ExpectOpenRefusal(maniImportReasonIndex, "/nodes/99", "a pointer past the nodes");
    Document(kOne,
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}},"
             "{\"sampler\":0,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/nodes/1/translation\"}}}}",
             "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "a pointer to a target already used");
    static const char kHipMove[] =
        "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"translation\"}}";
    Document("{\"input\":10,\"output\":6}", kHipMove, "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "times not floats");
    Document("{\"input\":11,\"output\":6}", kHipMove, "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "times strided");
    Document("{\"input\":16,\"output\":6}", kHipMove, "");
    {
        maniBuilder* builder = NewBuilder();
        maniGltf* gltf = NULL;
        CHECK(maniOpenGltf(builder, s_json, strlen(s_json), &gltf).result == mani_success,
              "times in a view of their own stride");
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
    }
    Document("{\"input\":1,\"output\":13}", kHipMove, "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "a normalized translation");
    Document(kOne, "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"rotation\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "a rotation of three");
    Document("{\"input\":14,\"output\":3,\"interpolation\":\"CUBICSPLINE\"}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"rotation\"}}", "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "a cubic spline of one key");
    Document(kOne, "", "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "no channels");
    // No channels member at all: refused, nothing read past the JSON.
    Document(kOne, "", "");
    char* channels = strstr(s_json, "\"channels\":[]");
    CHECK(channels != NULL, "the document has an empty channels array");
    if (channels != NULL)
    {
        channels[1] = 'x';
        ExpectOpenRefusal(maniImportReasonProperty, NULL, "no channels member");
    }
    Document("", kHipMove, "");
    ExpectOpenRefusal(maniImportReasonProperty, NULL, "no samplers");
    Document(kOne, kHipMove, "");
    char* name = strstr(s_json, "\"walk\"");
    memcpy(name, "7     ", 6);
    ExpectOpenRefusal(maniImportReasonProperty, "7     ", "a name not a string");
    // "/a" of three components and "/a/0" of one: the same float name.
    Document("{\"input\":1,\"output\":6},{\"input\":1,\"output\":1}",
             "{\"sampler\":0,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/a\"}}}},"
             "{\"sampler\":1,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/a/0\"}}}}",
             "");
    {
        maniBuilder* builder = NewBuilder();
        maniGltf* gltf = Open(builder);
        maniGltfOptions options = maniDefaultGltfOptions();
        maniSkeleton skeleton;
        CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
        maniImportResult r = maniGltfBuildClip(
            builder, gltf, 0, &skeleton, &options,
            &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, NULL);
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonName,
              "a float name twice");
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
    }
    // Folded under a scale of 2 along x, a hip turning about y shears:
    // refused, as a sheared rest is, rather than dropped.
    Document("{\"input\":0,\"output\":3}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"rotation\"}}", ",\"scale\":[2,1,1]");
    {
        maniBuilder* builder = NewBuilder();
        maniGltf* gltf = Open(builder);
        maniGltfOptions fold = {{maniAxisX, maniAxisY, maniAxisZ}, 1};
        maniSkeleton skeleton;
        CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &fold, &skeleton),
              "the rest has no shear");
        maniImportResult r = maniGltfBuildClip(
            builder, gltf, 0, &skeleton, &fold,
            &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, NULL);
        const char* channel = strstr(s_json, "{\"sampler\":0");
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonMatrix && r.item == 0 &&
                  r.offset == (uint64_t)(channel - s_json),
              "a folded shear");
        maniGltfOptions report = maniDefaultGltfOptions();
        CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &report, &skeleton) &&
                  maniGltfBuildClip(
                      builder, gltf, 0, &skeleton, &report,
                      &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, NULL)
                          .result == mani_success,
              "reported placement: the clip holds the hip alone");
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
    }
    // A key before 0 reaches the clip builder: Data on the channel.
    Document("{\"input\":1,\"output\":6}",
             "{\"sampler\":0,\"target\":{\"node\":2,\"path\":\"translation\"}}", "");
    s_keys[kTimesB] = -1.0f;
    char digits[600];
    Base64((const uint8_t*)s_keys, kKeyFloats * sizeof(float), digits);
    char* data = strstr(s_json, "base64,") + 7;
    memcpy(data, digits, strlen(digits));
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    maniImportResult r = maniGltfBuildClip(
        builder, gltf, 0, &skeleton, &options,
        &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, NULL);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData && r.item == 0,
          "a key before 0");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// What glTF allows beside the defaults: a normalized rotation, pointers
// shaped like a node's but naming none, a last channel shorter than an
// earlier one.
static void TestAllowed(void)
{
    Document("{\"input\":1,\"output\":12},{\"input\":1,\"output\":6},"
             "{\"input\":2,\"output\":5,\"interpolation\":\"STEP\"},"
             "{\"input\":1,\"output\":1},{\"input\":1,\"output\":15}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"rotation\"}},"
             "{\"sampler\":1,\"target\":{\"node\":1,\"path\":\"translation\"}},"
             "{\"sampler\":1,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/nodes/01/translation\"}}}},"
             "{\"sampler\":1,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/nodes/1/translationX\"}}}},"
             "{\"sampler\":3,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/materials/0/alphaCutoff\"}}}},"
             "{\"sampler\":4,\"target\":{\"path\":\"pointer\",\"extensions\":"
             "{\"KHR_animation_pointer\":{\"pointer\":\"/x/matrix\"}}}},"
             "{\"sampler\":2,\"target\":{\"node\":3,\"path\":\"scale\"}}",
             "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    uint32_t bound = 0;
    CHECK(gltf != NULL &&
              maniGltfBuildClip(
                  builder, gltf, 0, &skeleton, &options,
                  &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, &bound)
                      .result == mani_success &&
              bound == 7,
          "builds");
    maniClip clip;
    CHECK(OpenClip(builder, &clip), "opens");
    CHECK(maniClipDuration(&clip) == 1.0f, "the longest channel's end, not the last's");
    const char kScalar[] = "/materials/0/alphaCutoff";
    const char kSixteenth[] = "/x/matrix/15";
    CHECK(maniClipFindFloatTrack(&clip, kScalar, sizeof kScalar - 1) != MANI_NO_TRACK,
          "one component: the pointer alone");
    CHECK(maniClipFindFloatTrack(&clip, kSixteenth, sizeof kSixteenth - 1) != MANI_NO_TRACK,
          "two-digit components");
    const char kZero[] = "/nodes/01/translation/0";
    const char kLonger[] = "/nodes/1/translationX/2";
    CHECK(maniClipFindFloatTrack(&clip, kZero, sizeof kZero - 1) != MANI_NO_TRACK &&
              maniClipFindFloatTrack(&clip, kLonger, sizeof kLonger - 1) != MANI_NO_TRACK,
          "pointers, not node targets");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    uint32_t hip = maniSkeletonFindBone(&skeleton, "hip", 3);
    CHECK(maniSampleClip(&clip, &skeleton, 1.0f, pose) == mani_success, "samples");
    maniQuat q = maniPoseLocal(pose, hip).rotation;
    CHECK(Near(q.y, 1.0, 1e-6) && Near(q.w, 0.0, 1e-6), "a normalized short rotation");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// A clip's float track names when the mesh's target names cannot serve:
// too few, or one empty. The indices stand in.
static void ExpectIndexNames(const char* names, const char* what)
{
    Document(NULL, NULL, "");
    char* at = strstr(s_json, "\"smile\",\"frown\"");
    memcpy(at, names, strlen(names));
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    maniClip clip;
    CHECK(gltf != NULL &&
              maniGltfBuildClip(
                  builder, gltf, 0, &skeleton, &options,
                  &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, NULL)
                      .result == mani_success &&
              OpenClip(builder, &clip),
          "builds");
    CHECK(maniClipFindFloatTrack(&clip, "body/0", 6) != MANI_NO_TRACK &&
              maniClipFindFloatTrack(&clip, "body/1", 6) != MANI_NO_TRACK,
          what);
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// Folding a placement with no channel: the scene turned a quarter about
// y, only the hip animated. The hip's model is the scene's times its own.
static void TestFoldedStatic(void)
{
    Document("{\"input\":0,\"output\":3}",
             "{\"sampler\":0,\"target\":{\"node\":1,\"path\":\"rotation\"}}",
             ",\"rotation\":[0,0.70710677,0,0.70710677]");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 1};
    maniSkeleton skeleton;
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &options, &skeleton), "skeleton");
    maniClip clip;
    CHECK(gltf != NULL &&
              maniGltfBuildClip(
                  builder, gltf, 0, &skeleton, &options,
                  &(maniImportClipOptions){.sampleRate = 10.0f, .loop = maniLoopClamp}, NULL)
                      .result == mani_success &&
              OpenClip(builder, &clip),
          "builds");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    uint32_t hip = maniSkeletonFindBone(&skeleton, "hip", 3);
    CHECK(maniSampleClip(&clip, &skeleton, 1.0f, pose) == mani_success &&
              maniComputeModel(pose, &skeleton) == mani_success,
          "samples");
    const double one[3] = {1, 1, 1};
    const double sceneT[3] = {0, 0, 5};
    const double sceneQ[4] = {0, sqrt(0.5), 0, sqrt(0.5)};
    const double hipT[3] = {0, 1, 0};
    double hipQ[4];
    HipRotation(1.0, hipQ);
    maniAffine scene = Trs(sceneT, sceneQ, one);
    maniAffine local = Trs(hipT, hipQ, one);
    maniAffine world = maniAffineMul(&scene, &local);
    maniAffine expected = ZUp(world);
    maniAffine model = maniPoseModel(pose, hip);
    double worst = 0;
    for (int i = 0; i < 12; ++i)
    {
        worst = fmax(worst, fabs((double)model.m[i] - (double)expected.m[i]));
    }
    CHECK(worst < 2e-6, "the placement's turn composed in");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// Every allocation failing in turn while opening the document and
// building its skeleton and walk clip.
static void TestAllocationFailures(void)
{
    Document(NULL, NULL, "");
    GltfSweep sweep = {s_json, 0, UINT32_MAX, false};
    CHECK(TestSweep(GltfSweepWork, &sweep, 8192), "allocation failures handled, nothing kept");
}

int main(void)
{
    TestAllocationFailures();
    TestLocals();
    TestFoldedModels();
    TestAnotherRig();
    TestRefusals();
    TestAllowed();
    // One name, then a key: read on, the key would pass for a second.
    ExpectIndexNames("\"smile\"],\"x\":[0", "too few target names");
    ExpectIndexNames("\"smile\",\"\"     ", "an empty target name");
    TestFoldedStatic();
    TestClipOptions();
    return s_failures == 0 ? 0 : 1;
}
