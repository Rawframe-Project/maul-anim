// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF skeletons and skins: bones from skins' joints with
// the nodes between, skins sharing nodes grouped (checked against a
// brute force over random forests), placement reported or folded, the
// basis change, matrix nodes, bone names, and bind-pose palettes that
// come out as the identity, the end-to-end proof that rests, placement,
// basis and inverse binds agree.

#include "cleared.h"
#include "gltf_document.h"
#include "test_gltf_sweep.h"
#include "test_harness.h"
#include "test_text.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static char s_json[65536];
static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_skinFile[16384];
static _Alignas(32) uint8_t s_poseMemory[16384];

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
        printf("  reason %u offset %llu item %u near: %.40s\n", r.reason,
               (unsigned long long)r.offset, r.item, s_json + r.offset);
    }
    return gltf;
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

// The rig's nodes in glTF's basis:
//   0 Scene (placement): T(0 0 5) S(2), children 1, 5
//   1 hip (joint): T(0 1 0) R(30 degrees about y), children 2, 4
//   2 spine (between joints): T(0 0.5 0), child 3
//   3 head (joint): a matrix, 90 degrees about x and T(0 0.3 0)
//   4 leg (joint): T(0.2 -0.5 0) S(1 1.5 1)
//   5 mesh, 6 prop (its own tree, a joint of the third skin)
static const int kParents[7] = {-1, 0, 1, 2, 1, 0, -1};
static const float kHalf = 0.25881904510252074f; // sin 15 degrees
static const float kCos = 0.9659258262890683f;   // cos 15 degrees

static maniAffine Local(int node)
{
    switch (node)
    {
    case 0:
        return maniAffineFromTrs(maniVec3Make(0, 0, 5), maniQuatIdentity(), maniVec3Make(2, 2, 2));
    case 1:
        return maniAffineFromTrs(maniVec3Make(0, 1, 0), (maniQuat){0, kHalf, 0, kCos},
                                 maniVec3Make(1, 1, 1));
    case 2:
        return maniAffineFromTrs(maniVec3Make(0, 0.5f, 0), maniQuatIdentity(),
                                 maniVec3Make(1, 1, 1));
    case 3:
    {
        maniAffine a = {{1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0.3f, 0}};
        return a;
    }
    case 4:
        return maniAffineFromTrs(maniVec3Make(0.2f, -0.5f, 0), maniQuatIdentity(),
                                 maniVec3Make(1, 1.5f, 1));
    default:
        return maniAffineFromTrs(maniVec3Make(3, 0, 0), maniQuatIdentity(), maniVec3Make(1, 1, 1));
    }
}

static maniAffine World(int node)
{
    maniAffine w = Local(node);
    for (int p = kParents[node]; p >= 0; p = kParents[p])
    {
        maniAffine parent = Local(p);
        w = maniAffineMul(&parent, &w);
    }
    return w;
}

static void RigJson(void)
{
    // Inverse binds: skin 0's joints 1, 3, 4, then skin 1's joint 3.
    static const int kJoints[4] = {1, 3, 4, 3};
    float ibm[64];
    for (int j = 0; j < 4; ++j)
    {
        maniAffine world = World(kJoints[j]);
        maniAffine inverse;
        CHECK(maniAffineInverse(&world, &inverse), "invertible");
        for (int c = 0; c < 4; ++c)
        {
            for (int r = 0; r < 3; ++r)
            {
                ibm[16 * j + 4 * c + r] = inverse.m[3 * c + r];
            }
            ibm[16 * j + 4 * c + 3] = c == 3 ? 1.0f : 0.0f;
        }
    }
    char digits[400];
    Base64((const uint8_t*)ibm, sizeof ibm, digits);
    snprintf(
        s_json, sizeof s_json,
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"nodes\":["
        "{\"name\":\"Scene\",\"translation\":[0,0,5],\"scale\":[2,2,2],\"children\":[1,5]},"
        "{\"name\":\"hip\",\"translation\":[0,1,0],\"rotation\":[0,%.17g,0,%.17g],"
        "\"children\":[2,4]},"
        "{\"name\":\"spine\",\"translation\":[0,0.5,0],\"children\":[3]},"
        "{\"name\":\"head\",\"matrix\":[1,0,0,0, 0,0,1,0, 0,-1,0,0, 0,0.3,0,1]},"
        "{\"name\":\"leg\",\"translation\":[0.2,-0.5,0],\"scale\":[1,1.5,1]},"
        "{\"name\":\"mesh\"},{\"name\":\"prop\",\"translation\":[3,0,0]}],"
        "\"skins\":[{\"joints\":[1,3,4],\"inverseBindMatrices\":0,\"skeleton\":0},"
        "{\"joints\":[3],\"inverseBindMatrices\":1},{\"joints\":[6]}],"
        "\"buffers\":[{\"byteLength\":256,"
        "\"uri\":\"data:application/octet-stream;base64,%s\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteLength\":256}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"MAT4\"},"
        "{\"bufferView\":0,\"byteOffset\":192,\"componentType\":5126,\"count\":1,"
        "\"type\":\"MAT4\"}]}",
        (double)kHalf, (double)kCos, digits);
}

static bool NearAffine(const maniAffine* a, const maniAffine* b, float tolerance)
{
    for (int i = 0; i < 12; ++i)
    {
        if (!(fabsf(a->m[i] - b->m[i]) <= tolerance))
        {
            return false;
        }
    }
    return true;
}

static void OpenBuilt(maniBuilder* builder, uint8_t* file, size_t capacity, size_t* size)
{
    CHECK(maniBuilder_CopyBuilt(builder, file, capacity, size) == mani_success, "copies");
}

static void TestGroupsAndBones(void)
{
    RigJson();
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    if (gltf == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniGltf_GetNodeCount(gltf) == 7 && maniGltf_GetSkinCount(gltf) == 3, "counts");
    CHECK(maniGltf_GetSkeletonCount(gltf) == 2, "skins 0 and 1 share nodes");
    CHECK(maniGltf_GetSkinSkeleton(gltf, 0) == 0 && maniGltf_GetSkinSkeleton(gltf, 1) == 0 &&
              maniGltf_GetSkinSkeleton(gltf, 2) == 1 &&
              maniGltf_GetSkinSkeleton(gltf, 3) == UINT32_MAX,
          "skeletons by lowest skin");
    maniGltfOptions options = maniDefaultGltfOptions();
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &options).result == mani_success, "builds");
    size_t size = 0;
    OpenBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success, "opens");
    CHECK(maniSkeleton_GetBoneCount(&skeleton) == 4, "hip, spine, head, leg");
    uint32_t hip = maniSkeleton_FindBone(&skeleton, "hip", 3);
    uint32_t spine = maniSkeleton_FindBone(&skeleton, "spine", 5);
    uint32_t head = maniSkeleton_FindBone(&skeleton, "head", 4);
    uint32_t leg = maniSkeleton_FindBone(&skeleton, "leg", 3);
    CHECK(hip == 0 && maniSkeleton_GetParent(&skeleton, spine) == hip &&
              maniSkeleton_GetParent(&skeleton, head) == spine &&
              maniSkeleton_GetParent(&skeleton, leg) == hip,
          "the hierarchy");
    CHECK(maniSkeleton_FindBone(&skeleton, "Scene", 5) == MANI_NO_BONE, "the scene is placement");
    maniTransform headRest = maniSkeleton_GetRest(&skeleton, head);
    CHECK(fabsf(headRest.rotation.x - 0.70710677f) < 1e-6f &&
              fabsf(headRest.translation.y - 0.3f) < 1e-7f,
          "a matrix decomposed");
    maniAffine placement;
    CHECK(maniGltf_GetSkeletonPlacement(gltf, 0, &options, &placement).result == mani_success,
          "placement");
    maniAffine scene = Local(0);
    CHECK(NearAffine(&placement, &scene, 0), "the scene node's transform");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// Builds skeleton 0 and skin `skin` with the options, and checks that
// the bind pose's palette is the placement's inverse (the identity when
// folded).
static void CheckBindPalette(uint32_t skin, const maniGltfOptions* options, const char* what)
{
    RigJson();
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    if (gltf == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    size_t size = 0;
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, options).result == mani_success, what);
    OpenBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success, what);
    maniImportResult r = maniBuildGltfSkin(builder, gltf, skin, &skeleton, options);
    CHECK(r.result == mani_success, what);
    OpenBuilt(builder, s_skinFile, sizeof s_skinFile, &size);
    maniSkin built;
    CHECK(maniOpenSkin(s_skinFile, size, &built).result == mani_success, what);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success, what);
    maniAffine palette[4];
    uint32_t count = 0;
    CHECK(maniPose_ComputePalette(pose, &skeleton, &built, palette, 4, &count) == mani_success,
          what);
    maniAffine placement;
    CHECK(maniGltf_GetSkeletonPlacement(gltf, 0, options, &placement).result == mani_success, what);
    maniAffine identity = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    for (uint32_t j = 0; j < count; ++j)
    {
        maniAffine product = maniAffineMul(&placement, &palette[j]);
        CHECK(NearAffine(&product, &identity, 2e-5f), what);
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestBindPalettes(void)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    CheckBindPalette(0, &options, "placement reported");
    CheckBindPalette(1, &options, "a second skin on the skeleton");
    options.foldPlacement = 1;
    CheckBindPalette(0, &options, "placement folded");
    maniGltfOptions zUp = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 0};
    CheckBindPalette(0, &zUp, "Z up");
    zUp.foldPlacement = 1;
    CheckBindPalette(0, &zUp, "Z up, folded");
    maniGltfOptions mirrored = {{maniAxisNegativeX, maniAxisY, maniAxisZ}, 1};
    CheckBindPalette(0, &mirrored, "handedness flipped");
    maniGltfOptions rotated = {{maniAxisZ, maniAxisX, maniAxisY}, 0};
    CheckBindPalette(1, &rotated, "axes rotated");
}

static void TestBasisRests(void)
{
    RigJson();
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions zUp = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 0};
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &zUp).result == mani_success, "builds");
    size_t size = 0;
    OpenBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success, "opens");
    // hip: T(0 1 0) becomes (0 0 1); its turn about y becomes one about z.
    maniTransform hip = maniSkeleton_GetRest(&skeleton, 0);
    CHECK(hip.translation.x == 0 && hip.translation.y == 0 && hip.translation.z == 1,
          "y up becomes z up");
    CHECK(hip.rotation.z == kHalf && hip.rotation.y == 0 && hip.rotation.w == kCos,
          "a turn about y becomes one about z");
    maniTransform leg = maniSkeleton_GetRest(&skeleton, maniSkeleton_FindBone(&skeleton, "leg", 3));
    CHECK(leg.scale.x == 1 && leg.scale.y == 1 && leg.scale.z == 1.5f, "scales permuted");
    // Invalid options are a misuse.
    maniGltfOptions twice = {{maniAxisX, maniAxisNegativeX, maniAxisY}, 0};
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &twice).result == mani_errorInvalid, "x twice");
    maniGltfOptions far = {{maniAxisX, maniAxisY, 6}, 0};
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &far).result == mani_errorInvalid, "axis 6");
    maniGltfOptions fold = {{maniAxisX, maniAxisY, maniAxisZ}, 2};
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &fold).result == mani_errorInvalid, "fold 2");
    CHECK(maniBuildGltfSkeleton(builder, gltf, 2, &zUp).result == mani_errorInvalid, "past");
    CHECK(maniBuilder_GetMisuse(builder) == 4, "misuses");
    maniAffine placement;
    Fill(&placement, sizeof placement);
    CHECK(maniGltf_GetSkeletonPlacement(gltf, 0, &twice, &placement).result == mani_errorInvalid &&
              Cleared(&placement, sizeof placement),
          "placement, bad options: cleared");
    Fill(&placement, sizeof placement);
    CHECK(maniGltf_GetSkeletonPlacement(NULL, 0, &zUp, &placement).result == mani_errorInvalid &&
              Cleared(&placement, sizeof placement),
          "placement, no document: cleared");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestNodeSkeleton(void)
{
    RigJson();
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    CHECK(maniBuildGltfNodeSkeleton(builder, gltf, 0, &options).result == mani_success, "builds");
    size_t size = 0;
    OpenBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success, "opens");
    CHECK(maniSkeleton_GetBoneCount(&skeleton) == 6, "the scene's whole subtree");
    CHECK(maniSkeleton_FindBone(&skeleton, "mesh", 4) != MANI_NO_BONE, "every node");
    CHECK(maniBuildGltfNodeSkeleton(builder, gltf, 7, &options).result == mani_errorInvalid,
          "a node past the count");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void Names(const char* nodes, const char* joints)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[%s],\"skins\":[{\"joints\":[%s]}]}",
             nodes, joints);
}

static void TestNames(void)
{
    Names("{\"name\":\"root\",\"children\":[1,2,3]},{\"name\":\"bone\"},{\"name\":\"bone\"},{}",
          "1,2,3");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &options).result == mani_success, "builds");
    size_t size = 0;
    OpenBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success, "opens");
    CHECK(maniSkeleton_FindBone(&skeleton, "root", 4) == 0, "the common root, not a joint");
    CHECK(maniSkeleton_FindBone(&skeleton, "bone", 4) != MANI_NO_BONE, "the first keeps it");
    CHECK(maniSkeleton_FindBone(&skeleton, "bone#2", 6) != MANI_NO_BONE, "a repeat renamed");
    CHECK(maniSkeleton_FindBone(&skeleton, "#3", 2) != MANI_NO_BONE, "no name");
    CHECK(maniGltf_GetRenamedCount(gltf) == 2 && maniGltf_GetRenamedNode(gltf, 0) == 2 &&
              maniGltf_GetRenamedNode(gltf, 1) == 3 &&
              maniGltf_GetRenamedNode(gltf, 2) == UINT32_MAX,
          "the renamed nodes, in node order");
    size_t length = 0;
    const char* name = maniGltf_GetNodeBoneName(gltf, 2, &length);
    CHECK(length == 6 && memcmp(name, "bone#2", 6) == 0, "a node's bone name");
    CHECK(maniGltf_GetNodeBoneName(gltf, 4, &length) == NULL && length == 0, "past the nodes");
    maniCloseGltf(gltf);
    // A node really named as a renamed one would be.
    Names("{\"name\":\"root\",\"children\":[1,2,3]},{\"name\":\"bone\"},{\"name\":\"bone\"},"
          "{\"name\":\"bone#2\"}",
          "1,2,3");
    gltf = Open(builder);
    maniImportResult r = maniBuildGltfSkeleton(builder, gltf, 0, &options);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonName, "a collision");
    maniCloseGltf(gltf);
    // Names are fixed per document: the second rig's "hip" repeats the
    // first's, so it is renamed in its own skeleton too.
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"name\":\"hip\"},"
             "{\"name\":\"hip\"}],\"skins\":[{\"joints\":[0]},{\"joints\":[1]}]}");
    gltf = Open(builder);
    CHECK(maniGltf_GetSkeletonCount(gltf) == 2, "two rigs");
    CHECK(maniBuildGltfSkeleton(builder, gltf, 1, &options).result == mani_success, "builds");
    OpenBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success, "opens");
    CHECK(maniSkeleton_FindBone(&skeleton, "hip#1", 5) == 0, "renamed for the document");
    CHECK(maniBuildGltfSkin(builder, gltf, 1, &skeleton, &options).result == mani_success,
          "its skin binds by that name");
    maniCloseGltf(gltf);
    // A skin's joint missing from the skeleton given.
    Names("{\"name\":\"root\",\"children\":[1]},{\"name\":\"a\"}", "1");
    gltf = Open(builder);
    r = maniBuildGltfSkin(builder, gltf, 0, &skeleton, &options);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBone, "a joint not there");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// The bone names of one rig exported twice, its nodes in another order:
// every node with a unique given name keeps it; the unnamed one is the
// only rename, under its new index.
static void TestNamesAcrossExports(void)
{
    static const char* const kFirst =
        "{\"name\":\"hip\",\"children\":[1,3]},{\"name\":\"spine\",\"children\":[2]},"
        "{\"name\":\"head\"},{}";
    static const char* const kSecond =
        "{},{\"name\":\"head\"},{\"name\":\"spine\",\"children\":[1]},"
        "{\"name\":\"hip\",\"children\":[2,0]}";
    static const char* const kGiven[3] = {"hip", "spine", "head"};
    const char* exports[2] = {kFirst, kSecond};
    const uint32_t unnamed[2] = {3, 0};
    maniBuilder* builder = NewBuilder();
    for (int e = 0; e < 2; ++e)
    {
        Names(exports[e], e == 0 ? "0,1,2,3" : "3,2,1,0");
        maniGltf* gltf = Open(builder);
        bool same = gltf != NULL;
        for (uint32_t v = 0; same && v < maniGltf_GetNodeCount(gltf); ++v)
        {
            size_t length = 0;
            const char* name = maniGltf_GetNodeBoneName(gltf, v, &length);
            bool given = false;
            for (int k = 0; k < 3; ++k)
            {
                given =
                    given || (length == strlen(kGiven[k]) && memcmp(name, kGiven[k], length) == 0);
            }
            same = given || v == unnamed[e];
        }
        CHECK(same, "unique given names kept");
        CHECK(gltf != NULL && maniGltf_GetRenamedCount(gltf) == 1 &&
                  maniGltf_GetRenamedNode(gltf, 0) == unnamed[e],
              "one rename, the unnamed node");
        maniCloseGltf(gltf);
    }
    maniDestroyBuilder(builder);
}

static void ExpectOpens(const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, s_json, strlen(s_json), &gltf);
    CHECK(r.result == mani_success, what);
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(maniImportReason reason, const char* at, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, s_json, strlen(s_json), &gltf);
    bool right = r.result == mani_errorFormat && r.reason == reason &&
                 (at == NULL || r.offset == (uint64_t)(strstr(s_json, at) - s_json));
    CHECK(right, what);
    if (!right)
    {
        printf("  reason %u offset %llu\n", r.reason, (unsigned long long)r.offset);
    }
    maniDestroyBuilder(builder);
}

static void ExpectBuildRefusal(maniImportReason reason, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    if (gltf != NULL)
    {
        maniGltfOptions options = maniDefaultGltfOptions();
        maniImportResult r = maniBuildGltfSkeleton(builder, gltf, 0, &options);
        CHECK(r.result == mani_errorFormat && r.reason == reason, what);
        maniCloseGltf(gltf);
    }
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    Names("{\"children\":[2]},{\"children\":[2]},{}", "2");
    ExpectRefusal(maniImportReasonHierarchy, "2]},{}", "two parents");
    Names("{\"children\":[1]},{\"children\":[0]}", "0");
    ExpectRefusal(maniImportReasonHierarchy, NULL, "a cycle");
    Names("{\"children\":[0]}", "0");
    ExpectRefusal(maniImportReasonHierarchy, NULL, "its own child");
    Names("{},{}", "0,1");
    ExpectRefusal(maniImportReasonHierarchy, "[0,1]", "joints in two trees");
    Names("{\"children\":[1]},{}", "1,1");
    ExpectRefusal(maniImportReasonProperty, "1]}]}", "a joint twice");
    Names("{}", "");
    ExpectRefusal(maniImportReasonProperty, "[]}]}", "no joints");
    Names("{}", "1");
    ExpectRefusal(maniImportReasonIndex, "1]", "a joint past the nodes");
    Names("{\"children\":[3]}", "0");
    ExpectRefusal(maniImportReasonIndex, "3]", "a child past the nodes");
    Names("{\"children\":[1.5]}", "0");
    ExpectRefusal(maniImportReasonProperty, "1.5", "a child not an index");
    // Matrices.
    Names("{\"matrix\":[1,0,0,0, 0.5,1,0,0, 0,0,1,0, 0,0,0,1]}", "0");
    ExpectBuildRefusal(maniImportReasonMatrix, "a shear");
    Names("{\"matrix\":[1,0,0,0, 0,1,0,0, 0,0,1,0.5, 0,0,0,1]}", "0");
    ExpectBuildRefusal(maniImportReasonMatrix, "projective");
    Names("{\"matrix\":[1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],\"translation\":[1,0,0]}", "0");
    ExpectBuildRefusal(maniImportReasonProperty, "a matrix and TRS");
    Names("{\"matrix\":[1,0,0,0]}", "0");
    ExpectBuildRefusal(maniImportReasonProperty, "four numbers");
    Names("{\"translation\":[1,0,0,0]}", "0");
    ExpectBuildRefusal(maniImportReasonProperty, "four numbers for a translation");
    Names("{\"rotation\":[0,0,0,0]}", "0");
    ExpectBuildRefusal(maniImportReasonProperty, "a zero rotation");
    Names("{\"translation\":[1e39,0,0]}", "0");
    ExpectBuildRefusal(maniImportReasonProperty, "past binary32");
    Names("{\"name\":7}", "0");
    ExpectRefusal(maniImportReasonProperty, "7}", "a name not a string");
    // Inverse binds of the wrong shape.
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{}],\"skins\":[{\"joints\":[0],"
             "\"inverseBindMatrices\":0}],\"accessors\":[{\"componentType\":5126,\"count\":1,"
             "\"type\":\"MAT3\"}]}");
    ExpectRefusal(maniImportReasonProperty, "0}]", "inverse binds not MAT4");
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"children\":[1]},{}],\"skins\":"
             "[{\"joints\":[0,1],\"inverseBindMatrices\":0}],\"accessors\":[{\"componentType\":"
             "5126,\"count\":1,\"type\":\"MAT4\"}]}");
    ExpectRefusal(maniImportReasonProperty, "0}]", "fewer inverse binds than joints");
    // 144 zero bytes as base64; a stride of 64, the element's size, reads
    // as none, one of 80 is refused.
    char zeros[193];
    memset(zeros, 'A', 192);
    zeros[192] = 0;
    for (int stride = 64; stride <= 80; stride += 16)
    {
        snprintf(s_json, sizeof s_json,
                 "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{}],\"skins\":[{\"joints\":[0],"
                 "\"inverseBindMatrices\":0}],\"buffers\":[{\"byteLength\":144,\"uri\":"
                 "\"data:application/octet-stream;base64,%s\"}],\"bufferViews\":[{\"buffer\":0,"
                 "\"byteLength\":144,\"byteStride\":%d}],\"accessors\":[{\"bufferView\":0,"
                 "\"componentType\":5126,\"count\":1,\"type\":\"MAT4\"}]}",
                 zeros, stride);
        if (stride == 64)
        {
            ExpectOpens("inverse binds in a view of their own stride");
        }
        else
        {
            ExpectRefusal(maniImportReasonLayout, "0}]", "inverse binds in a strided view");
        }
    }
}

// A shear small beside the matrix's largest column is within the
// tolerance: 1e-3 under a column of 1000.
static void TestShearTolerance(void)
{
    Names("{\"matrix\":[1,0,0,0, 0.001,1,0,0, 0,0,1000,0, 0,0,0,1]}", "0");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    CHECK(gltf != NULL && maniBuildGltfSkeleton(builder, gltf, 0, &options).result == mani_success,
          "a shear within 1e-4 of the largest column");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// A rest folded under a non-uniform scale that its rotation shears is
// refused, as a sheared node matrix is.
static void TestFoldedRestShear(void)
{
    Names("{\"scale\":[2,1,1],\"children\":[1]},"
          "{\"name\":\"a\",\"rotation\":[0,0.38268343,0,0.92387953]}",
          "1");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions fold = {{maniAxisX, maniAxisY, maniAxisZ}, 1};
    maniImportResult r =
        gltf != NULL ? maniBuildGltfSkeleton(builder, gltf, 0, &fold) : (maniImportResult){0};
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonMatrix, "a folded shear");
    maniGltfOptions report = maniDefaultGltfOptions();
    CHECK(gltf != NULL && maniBuildGltfSkeleton(builder, gltf, 0, &report).result == mani_success,
          "reported, the rest is the node's own");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// Skins numbered by their lowest: a lone skin 0, then skin 3 joining
// skin 1 and, through it, skin 2. Skins 1 to 3 are one skeleton, 1.
static void TestLowestSkinNumbering(void)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"name\":\"L\"},"
             "{\"name\":\"P\",\"children\":[2,4]},{\"name\":\"A\",\"children\":[3]},"
             "{\"name\":\"A1\"},{\"name\":\"B\",\"children\":[5]},{\"name\":\"B1\"}],"
             "\"skins\":[{\"joints\":[0]},{\"joints\":[1,2]},{\"joints\":[5]},"
             "{\"joints\":[3,5]}]}");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    CHECK(gltf != NULL && maniGltf_GetSkeletonCount(gltf) == 2, "two skeletons");
    CHECK(gltf != NULL && maniGltf_GetSkinSkeleton(gltf, 0) == 0 &&
              maniGltf_GetSkinSkeleton(gltf, 1) == 1 && maniGltf_GetSkinSkeleton(gltf, 2) == 1 &&
              maniGltf_GetSkinSkeleton(gltf, 3) == 1,
          "numbered by their lowest skins");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// Grouping against a brute force: random forests of 24 nodes, random
// skins within one tree each.
static uint64_t s_random = 0x5EED5EED5EEDull;

static uint32_t Next(uint32_t bound)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (uint32_t)(s_random % bound);
}

enum
{
    kNodes = 24,
    kSkins = 5,
};

static int TreeRoot(const int* parents, int v)
{
    while (parents[v] >= 0)
    {
        v = parents[v];
    }
    return v;
}

static bool IsAncestor(const int* parents, int a, int v)
{
    for (; v >= 0; v = parents[v])
    {
        if (v == a)
        {
            return true;
        }
    }
    return false;
}

static int Common(const int* parents, int a, int b)
{
    for (; a >= 0; a = parents[a])
    {
        if (IsAncestor(parents, a, b))
        {
            return a;
        }
    }
    return -1;
}

static void RandomTrial(void)
{
    int parents[kNodes];
    for (int i = 0; i < kNodes; ++i)
    {
        parents[i] = i == 0 || Next(5) == 0 ? -1 : (int)Next((uint32_t)i);
    }
    int joints[kSkins][4];
    int jointCount[kSkins];
    int skinCount = 1 + (int)Next(kSkins);
    for (int s = 0; s < skinCount; ++s)
    {
        int first = (int)Next(kNodes);
        int tree = TreeRoot(parents, first);
        jointCount[s] = 0;
        for (int k = 0; k < 8 && jointCount[s] < 4; ++k)
        {
            int v = k == 0 ? first : (int)Next(kNodes);
            bool fresh = TreeRoot(parents, v) == tree;
            for (int e = 0; e < jointCount[s]; ++e)
            {
                fresh = fresh && joints[s][e] != v;
            }
            if (fresh)
            {
                joints[s][jointCount[s]++] = v;
            }
        }
    }
    // The document.
    TestText text = TestTextIn(s_json, sizeof s_json);
    TestTextAppend(&text, "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[");
    for (int i = 0; i < kNodes; ++i)
    {
        TestTextAppend(&text, "%s{\"children\":[", i ? "," : "");
        bool any = false;
        for (int c = 0; c < kNodes; ++c)
        {
            if (parents[c] == i)
            {
                TestTextAppend(&text, "%s%d", any ? "," : "", c);
                any = true;
            }
        }
        TestTextAppend(&text, "]}");
    }
    TestTextAppend(&text, "],\"skins\":[");
    for (int s = 0; s < skinCount; ++s)
    {
        TestTextAppend(&text, "%s{\"joints\":[", s ? "," : "");
        for (int j = 0; j < jointCount[s]; ++j)
        {
            TestTextAppend(&text, "%s%d", j ? "," : "", joints[s][j]);
        }
        TestTextAppend(&text, "]}");
    }
    TestTextAppend(&text, "]}");
    CHECK(text.fits, "the document fits its buffer");
    if (!text.fits)
    {
        return;
    }
    // The brute force: each skin's bones, sets joined while they meet.
    bool bones[kSkins][kNodes];
    int roots[kSkins];
    for (int s = 0; s < skinCount; ++s)
    {
        roots[s] = joints[s][0];
        for (int j = 1; j < jointCount[s]; ++j)
        {
            roots[s] = Common(parents, roots[s], joints[s][j]);
        }
        memset(bones[s], 0, sizeof bones[s]);
        for (int j = 0; j < jointCount[s]; ++j)
        {
            for (int v = joints[s][j]; v != parents[roots[s]]; v = parents[v])
            {
                bones[s][v] = true;
            }
        }
    }
    int group[kSkins];
    for (int s = 0; s < skinCount; ++s)
    {
        group[s] = s;
    }
    for (bool changed = true; changed;)
    {
        changed = false;
        for (int a = 0; a < skinCount; ++a)
        {
            for (int b = a + 1; b < skinCount; ++b)
            {
                bool meet = false;
                for (int v = 0; v < kNodes; ++v)
                {
                    meet = meet || (bones[a][v] && bones[b][v]);
                }
                int low = group[a] < group[b] ? group[a] : group[b];
                if (meet && group[a] != group[b])
                {
                    int high = group[a] + group[b] - low;
                    for (int s = 0; s < skinCount; ++s)
                    {
                        group[s] = group[s] == high ? low : group[s];
                    }
                    changed = true;
                }
            }
        }
    }
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    if (gltf != NULL)
    {
        for (int a = 0; a < skinCount; ++a)
        {
            for (int b = 0; b < skinCount; ++b)
            {
                bool same = maniGltf_GetSkinSkeleton(gltf, (uint32_t)a) ==
                            maniGltf_GetSkinSkeleton(gltf, (uint32_t)b);
                CHECK(same == (group[a] == group[b]), "grouping as the brute force");
            }
            // The skeleton's root: the common ancestor of its skins' roots.
            int root = -1;
            for (int s = 0; s < skinCount; ++s)
            {
                if (group[s] == group[a])
                {
                    root = root < 0 ? roots[s] : Common(parents, root, roots[s]);
                }
            }
            uint32_t k = maniGltf_GetSkinSkeleton(gltf, (uint32_t)a);
            CHECK(gltf->skeletonRoots[k] == (uint32_t)root, "the skeleton's root");
        }
        maniCloseGltf(gltf);
    }
    maniDestroyBuilder(builder);
}

static void TestGroupingAgainstBruteForce(void)
{
    for (int trial = 0; trial < 300; ++trial)
    {
        RandomTrial();
    }
}

// Every allocation failing in turn while opening the document,
// building its skeleton and first skin.
static void TestAllocationFailures(void)
{
    RigJson();
    GltfSweep sweep = {s_json, UINT32_MAX, 0, false};
    CHECK(TestSweep(GltfSweepWork, &sweep, 8192), "allocation failures handled, nothing kept");
}

int main(void)
{
    TestAllocationFailures();
    TestShearTolerance();
    TestFoldedRestShear();
    TestNamesAcrossExports();
    TestLowestSkinNumbering();
    TestGroupsAndBones();
    TestBindPalettes();
    TestBasisRests();
    TestNodeSkeleton();
    TestNames();
    TestRefusals();
    TestGroupingAgainstBruteForce();
    return s_failures == 0 ? 0 : 1;
}
