// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// `VRMC_springBone_extended_collider` and `VRMC_springBone_limit` read
// into spring bone sources: inside spheres and capsules and
// planes in place of their fallbacks, an extension of another version
// or without a shape leaving the fallback; cone, hinge and spherical
// limits with their angles and rotations, a last joint's limit unread;
// a limit's frame under a basis change, proper and mirrored, the one
// glTF gives it carried into the output's basis; the sources built; and
// each refusal at its value. Run as test_gltf_spring_extensions --seed
// <file> it writes the document read, a seed of the glTF fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "gltf_basis.h"
#include "spring_frame.h"
#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static char s_json[16384];
static _Alignas(16) uint8_t s_skeletonFiles[2][16384];
static _Alignas(16) uint8_t s_setFile[16384];

#define EXTENDED(shape) "\"extensions\":{\"VRMC_springBone_extended_collider\":" shape "}"
#define LIMIT(limit)    "\"extensions\":{\"VRMC_springBone_limit\":" limit "}"
#define FAR_SPHERE      "\"shape\":{\"sphere\":{\"offset\":[0,-10000,0],\"radius\":0}}"

// clang-format off
static const char kColliders[] =
    "[{\"node\":4," FAR_SPHERE ","
    EXTENDED("{\"specVersion\":\"1.0\",\"shape\":{\"sphere\":{\"offset\":[0,0.1,0],"
             "\"radius\":0.3,\"inside\":true}}}") "},"
    "{\"node\":0," FAR_SPHERE ","
    EXTENDED("{\"specVersion\":\"1.0\",\"shape\":{\"capsule\":{\"offset\":[0,0.1,0],"
             "\"radius\":0.2,\"tail\":[0,0.5,0],\"inside\":true}}}") "},"
    "{\"node\":0,\"shape\":{\"sphere\":{\"offset\":[0,-1000,0],\"radius\":1000}},"
    EXTENDED("{\"specVersion\":\"1.0\",\"shape\":{\"plane\":{\"offset\":[0,0.5,0],"
             "\"normal\":[0,1,0]}}}") "},"
    "{\"node\":4,\"shape\":{\"sphere\":{\"radius\":0.05}},"
    EXTENDED("{\"specVersion\":\"2.0\",\"shape\":{\"plane\":{}}}") "},"
    "{\"node\":4,\"shape\":{\"capsule\":{\"radius\":0.06,\"tail\":[0,0.1,0]}},"
    EXTENDED("{\"specVersion\":\"1.0\"}") "},"
    "{\"node\":4,\"shape\":{\"sphere\":{}},"
    EXTENDED("{\"specVersion\":\"1.0\",\"shape\":{\"plane\":{}}}") "}]";

static const char kSprings[] =
    "[{\"joints\":["
    "{\"node\":0,"
    LIMIT("{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{\"angle\":0.5,"
          "\"rotation\":[0,0,0.38268343,0.9238795]}}}") "},"
    "{\"node\":1,"
    LIMIT("{\"specVersion\":\"1.0\",\"limit\":{\"hinge\":{\"angle\":4}}}") "},"
    "{\"node\":2,"
    LIMIT("{\"specVersion\":\"1.0\",\"limit\":{\"spherical\":{\"pitch\":0.3,\"yaw\":0.2,"
          "\"rotation\":[0.25881905,0,0,0.9659258]}}}") "},"
    "{\"node\":3,"
    LIMIT("{\"specVersion\":\"1.0\",\"limit\":5}") "}]},"
    "{\"joints\":[{\"node\":4,"
    LIMIT("{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{\"angle\":0.7,"
          "\"rotation\":[0.25881905,0,0,0.9659258]}}}") "},"
    "{\"node\":5}]}]";

// A limit of another version on a spring's first joint.
static const char kOtherVersion[] =
    "[{\"joints\":[{\"node\":1,"
    LIMIT("{\"specVersion\":\"1.1\",\"limit\":{}}") "},"
    "{\"node\":2}]}]";
// clang-format on

static void Document(const char* colliders, const char* springs)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},"
             "\"nodes\":[{\"name\":\"hip\",\"children\":[1,4]},"
             "{\"name\":\"hair1\",\"translation\":[0,0.1,0],\"children\":[2]},"
             "{\"name\":\"hair2\",\"translation\":[0.05,0.1,0],\"children\":[3]},"
             "{\"name\":\"hair3\",\"translation\":[0,0.1,0.05]},"
             "{\"name\":\"head\",\"translation\":[0,0.3,0],\"children\":[5]},"
             "{\"name\":\"top\",\"translation\":[0,-0.2,0]}],"
             "\"skins\":[{\"joints\":[0,1,2,3,4,5]}],"
             "\"extensions\":{\"VRMC_springBone\":{\"specVersion\":\"1.0\",\"colliders\":%s,"
             "\"springs\":%s}}}",
             colliders, springs);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Builds skeleton 0 with the options into the k-th file.
static bool Skeleton(maniBuilder* builder, maniGltf* gltf, const maniGltfOptions* options, int k,
                     maniSkeleton* skeleton)
{
    size_t size = 0;
    return maniGltfBuildSkeleton(builder, gltf, 0, options).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFiles[k], sizeof s_skeletonFiles[k], &size) ==
               mani_success &&
           maniOpenSkeleton(s_skeletonFiles[k], size, skeleton).result == mani_success;
}

static maniGltf* Open(maniBuilder* builder)
{
    maniGltf* gltf = NULL;
    CHECK(maniOpenGltf(builder, s_json, strlen(s_json), &gltf).result == mani_success, "opens");
    return gltf;
}

static bool Builds(maniBuilder* builder, const maniSpringSetSource* set)
{
    size_t size = 0;
    maniSpringSet opened;
    return maniBuildSpringSet(builder, set).result == mani_success &&
           maniCopyBuilt(builder, s_setFile, sizeof s_setFile, &size) == mani_success &&
           maniOpenSpringSet(s_setFile, size, &opened).result == mani_success;
}

static void TestColliders(void)
{
    Document(kColliders, kSprings);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniSpringSetSource set;
    CHECK(Skeleton(builder, gltf, &options, 0, &skeleton) &&
              maniGltfReadSprings(gltf, &skeleton, &options, &set).result == mani_success &&
              set.colliderCount == 6,
          "read");
    const maniColliderSource* c = set.colliders;
    CHECK(c[0].shape == maniColliderSphere && c[0].inside == 1 && c[0].radius == 0.3f &&
              c[0].offset.y == 0.1f,
          "an inside sphere in place of its fallback");
    CHECK(c[1].shape == maniColliderCapsule && c[1].inside == 1 && c[1].radius == 0.2f &&
              c[1].offset.y == 0.1f && c[1].tail.y == 0.5f,
          "an inside capsule");
    CHECK(c[2].shape == maniColliderPlane && c[2].inside == 0 && c[2].radius == 0.0f &&
              c[2].offset.y == 0.5f && c[2].tail.x == 0.0f && c[2].tail.y == 1.0f &&
              c[2].tail.z == 0.0f,
          "a plane, its normal in tail");
    CHECK(c[3].shape == maniColliderSphere && c[3].radius == 0.05f && c[3].inside == 0,
          "another version: the fallback");
    CHECK(c[4].shape == maniColliderCapsule && c[4].radius == 0.06f && c[4].tail.y == 0.1f,
          "no shape: the fallback");
    CHECK(c[5].shape == maniColliderPlane && c[5].tail.z == 1.0f && c[5].offset.y == 0.0f,
          "a plane's defaults: +z through the bone");
    CHECK(Builds(builder, &set), "built");
    // Z up: the plane's normal along +z, its point too.
    maniGltfOptions zUp = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 0};
    CHECK(Skeleton(builder, gltf, &zUp, 0, &skeleton) &&
              maniGltfReadSprings(gltf, &skeleton, &zUp, &set).result == mani_success &&
              set.colliders[2].tail.z == 1.0f && set.colliders[2].offset.z == 0.5f &&
              set.colliders[5].tail.y == -1.0f && Builds(builder, &set),
          "z up");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static bool QuatIs(maniQuat q, float x, float y, float z, float w)
{
    return q.x == x && q.y == y && q.z == z && q.w == w;
}

static void TestLimits(void)
{
    Document(kColliders, kSprings);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniSpringSetSource set;
    CHECK(Skeleton(builder, gltf, &options, 0, &skeleton) &&
              maniGltfReadSprings(gltf, &skeleton, &options, &set).result == mani_success &&
              set.springCount == 2 && set.springs[0].jointCount == 4,
          "read");
    const maniSpringJointSource* j = set.springs[0].joints;
    CHECK(j[0].limit == maniSpringLimitCone && j[0].limitAngle == 0.5f &&
              QuatIs(j[0].limitRotation, 0.0f, 0.0f, 0.38268343f, 0.9238795f),
          "a cone and its rotation as written");
    CHECK(j[1].limit == maniSpringLimitHinge && j[1].limitAngle == 4.0f &&
              QuatIs(j[1].limitRotation, 0.0f, 0.0f, 0.0f, 1.0f),
          "a hinge past π kept for the builder, the identity by default");
    CHECK(j[2].limit == maniSpringLimitSpherical && j[2].limitAngle == 0.3f &&
              j[2].limitYaw == 0.2f &&
              QuatIs(j[2].limitRotation, 0.25881905f, 0.0f, 0.0f, 0.9659258f),
          "a spherical limit's pitch and yaw");
    CHECK(j[3].limit == maniSpringLimitNone, "the last joint's limit ignored, unread");
    CHECK(set.springs[1].joints[0].limit == maniSpringLimitCone, "a cone down -y");
    CHECK(Builds(builder, &set), "built");
    maniCloseGltf(gltf);
    Document("[]", kOtherVersion);
    gltf = Open(builder);
    CHECK(Skeleton(builder, gltf, &options, 0, &skeleton) &&
              maniGltfReadSprings(gltf, &skeleton, &options, &set).result == mani_success &&
              set.springs[0].joints[0].limit == maniSpringLimitNone,
          "another version: no limit, its body unread");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// The limit frame the builder makes from a source joint.
static maniQuat Frame(const maniSkeleton* skeleton, const maniSpringJointSource* joints, int k)
{
    maniVec3 a = maniSpringRestDirection(skeleton, joints[k].bone, joints[k + 1].bone);
    return maniQuatMul(maniSpringTurnFromY(a), maniQuatNormalize(joints[k].limitRotation));
}

static bool Near(maniVec3 a, maniVec3 b)
{
    return fabsf(a.x - b.x) < 1e-5f && fabsf(a.y - b.y) < 1e-5f && fabsf(a.z - b.z) < 1e-5f;
}

// Under a basis change B the output's frame maps the limit's axes as
// B F M does, F glTF's frame and M the mirror of x for det B < 0.
static void TestLimitBasis(const maniGltfOptions* basis, bool mirror, const char* what)
{
    Document(kColliders, kSprings);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton plain;
    maniSkeleton changed;
    maniSpringSetSource set;
    CHECK(Skeleton(builder, gltf, &options, 0, &plain) &&
              Skeleton(builder, gltf, basis, 1, &changed) &&
              maniGltfReadSprings(gltf, &plain, &options, &set).result == mani_success,
          "read");
    // Spring 0's three limits, then spring 1's, whose rest direction is
    // -y: half a turn from +y.
    maniSpringJointSource joints[6];
    memcpy(joints, set.springs[0].joints, 4 * sizeof *joints);
    memcpy(joints + 4, set.springs[1].joints, 2 * sizeof *joints);
    CHECK(maniGltfReadSprings(gltf, &changed, basis, &set).result == mani_success, "read again");
    maniSpringJointSource read[6];
    memcpy(read, set.springs[0].joints, 4 * sizeof *read);
    memcpy(read + 4, set.springs[1].joints, 2 * sizeof *read);
    bool same = true;
    const int limited[4] = {0, 1, 2, 4};
    for (int n = 0; n < 4; ++n)
    {
        int k = limited[n];
        maniQuat f = Frame(&plain, joints, k);
        maniQuat g = Frame(&changed, read, k);
        const maniVec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        for (int i = 0; i < 3; ++i)
        {
            maniVec3 u = axes[i];
            maniVec3 m = i == 0 && mirror ? maniVec3Scale(u, -1.0f) : u;
            maniVec3 v = maniQuatRotate(f, m);
            float b[3] = {v.x, v.y, v.z};
            maniGltfBasisVector(basis, b, true);
            same = same && Near(maniQuatRotate(g, u), maniVec3Make(b[0], b[1], b[2]));
        }
    }
    CHECK(same, what);
    CHECK(Builds(builder, &set), "built");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(const char* colliders, const char* springs, const char* at,
                          const char* what)
{
    Document(colliders, springs);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniSpringSetSource set;
    CHECK(Skeleton(builder, gltf, &options, 0, &skeleton), "a skeleton");
    maniImportResult r = maniGltfReadSprings(gltf, &skeleton, &options, &set);
    const char* found = at != NULL ? strstr(s_json, at) : NULL;
    bool right = r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
                 (at == NULL || (found != NULL && r.offset == (uint64_t)(found - s_json)));
    CHECK(right, what);
    if (!right)
    {
        printf("  reason %u offset %llu near: %.40s\n", r.reason, (unsigned long long)r.offset,
               r.offset < strlen(s_json) ? s_json + r.offset : "");
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static const char kPlain[] = "[{\"joints\":[{\"node\":1},{\"node\":2}]}]";

static void TestRefusals(void)
{
    char colliders[512];
    const char* shapes[7][3] = {
        {"{\"specVersion\":1}", "1}", "a version not a string"},
        {"{\"shape\":{\"plane\":{}}}", NULL, "no version"},
        {"{\"specVersion\":\"1.0\",\"shape\":{\"plane\":{\"normal\":[0,0.5,0]}}}", "[0,0.5,0]",
         "a normal too short"},
        {"{\"specVersion\":\"1.0\",\"shape\":{\"sphere\":{},\"plane\":{}}}", "{\"sphere\":{},",
         "two shapes"},
        {"{\"specVersion\":\"1.0\",\"shape\":{}}", "{}}}}]", "no shape in a shape"},
        {"{\"specVersion\":\"1.0\",\"shape\":{\"plane\":{\"normal\":[0,2,0]}}}", "[0,2,0]",
         "a normal not unit"},
        {"{\"specVersion\":\"1.0\",\"shape\":{\"sphere\":{\"inside\":1}}}", "1}}",
         "inside not a boolean"}};
    for (int k = 0; k < 7; ++k)
    {
        snprintf(colliders, sizeof colliders,
                 "[{\"node\":4,\"shape\":{\"sphere\":{}},\"extensions\":{"
                 "\"VRMC_springBone_extended_collider\":%s}}]",
                 shapes[k][0]);
        ExpectRefusal(colliders, kPlain, shapes[k][1], shapes[k][2]);
    }
    ExpectRefusal("[{\"node\":4,\"shape\":{\"sphere\":{}},\"extensions\":{"
                  "\"VRMC_springBone_extended_collider\":{\"specVersion\":\"1.0\",\"shape\":{"
                  "\"capsule\":{\"radius\":-1}}}}}]",
                  kPlain, "-1}", "a negative radius");
    char springs[512];
    const char* limits[8][3] = {
        {"{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{\"angle\":1,\"rotation\":[0,0,0,0.5]}}}",
         "[0,0,0,0.5]", "a rotation too short"},
        {"{\"specVersion\":\"1.0\"}", NULL, "no limit"},
        {"{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{\"angle\":1},\"hinge\":{\"angle\":1}}}",
         "{\"cone\"", "two limits"},
        {"{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{}}}", NULL, "a cone without its angle"},
        {"{\"specVersion\":\"1.0\",\"limit\":{\"spherical\":{\"pitch\":1}}}", NULL,
         "a spherical limit without its yaw"},
        {"{\"specVersion\":\"1.0\",\"limit\":{\"hinge\":{\"angle\":-0.1}}}", "-0.1",
         "an angle below 0"},
        {"{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{\"angle\":1,\"rotation\":[0,0,0,2]}}}",
         "[0,0,0,2]", "a rotation not unit"},
        {"{\"specVersion\":\"1.0\",\"limit\":{\"cone\":{\"angle\":1,\"rotation\":[0,0,1]}}}",
         "[0,0,1]", "three numbers for a rotation"}};
    for (int k = 0; k < 8; ++k)
    {
        snprintf(springs, sizeof springs,
                 "[{\"joints\":[{\"node\":1,\"extensions\":{\"VRMC_springBone_limit\":%s}},"
                 "{\"node\":2}]}]",
                 limits[k][0]);
        ExpectRefusal("[]", springs, limits[k][1], limits[k][2]);
    }
}

static bool Seed(const char* path)
{
    Document(kColliders, kSprings);
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return false;
    }
    size_t length = strlen(s_json);
    size_t written = fwrite(s_json, 1, length, out);
    return fclose(out) == 0 && written == length;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return Seed(argv[2]) ? 0 : 1;
    }
    TestColliders();
    TestLimits();
    maniGltfOptions zUp = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 0};
    TestLimitBasis(&zUp, false, "z up: the frame carried");
    maniGltfOptions mirrored = {{maniAxisNegativeX, maniAxisY, maniAxisZ}, 0};
    TestLimitBasis(&mirrored, true, "x mirrored: the frame carried, its x mirrored");
    maniGltfOptions turned = {{maniAxisZ, maniAxisX, maniAxisNegativeY}, 0};
    TestLimitBasis(&turned, true, "axes turned and mirrored");
    maniGltfOptions aboutX = {{maniAxisX, maniAxisNegativeY, maniAxisNegativeZ}, 0};
    TestLimitBasis(&aboutX, false, "half a turn about x");
    maniGltfOptions aboutY = {{maniAxisNegativeX, maniAxisY, maniAxisNegativeZ}, 0};
    TestLimitBasis(&aboutY, false, "half a turn about y");
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
