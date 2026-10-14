// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// `VRMC_springBone` as spring bone sources (mani-0011): colliders,
// groups and springs with their values and the schema's defaults, bones
// bound by name, the basis change on offsets and gravity, and every
// rule the specification sets on versions, ranges, chains and centers.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdio.h>
#include <string.h>

static char s_json[16384];
static _Alignas(16) uint8_t s_skeletonFile[16384];

static const char kColliders[] =
    "[{\"node\":4,\"shape\":{\"sphere\":{\"offset\":[0,0.1,0],\"radius\":0.08}}},"
    "{\"node\":0,\"shape\":{\"capsule\":{\"offset\":[0,0,0],\"radius\":0.05,\"tail\":[0,0.2,0]}}}]";
static const char kGroups[] = "[{\"name\":\"head\",\"colliders\":[0]},{\"colliders\":[0,1]}]";
static const char kSprings[] =
    "[{\"name\":\"hair\",\"joints\":[{\"node\":1,\"hitRadius\":0.02,\"stiffness\":1.5,"
    "\"gravityPower\":0.3,\"gravityDir\":[0,-1,0],\"dragForce\":0.4},{\"node\":2},{\"node\":3}],"
    "\"colliderGroups\":[0,1],\"center\":0}]";

static void Document(const char* version, const char* colliders, const char* groups,
                     const char* springs)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},"
             "\"nodes\":[{\"name\":\"hip\",\"children\":[1,4]},{\"name\":\"hair1\","
             "\"children\":[2]},{\"name\":\"hair2\",\"children\":[3]},{\"name\":\"hair3\"},"
             "{\"name\":\"head\"}],"
             "\"skins\":[{\"joints\":[0,1,2,3,4]}],"
             "\"extensions\":{\"VRMC_springBone\":{\"specVersion\":\"%s\",\"colliders\":%s,"
             "\"colliderGroups\":%s,\"springs\":%s}}}",
             version, colliders, groups, springs);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Opens the document and builds skeleton 0 with the options.
static maniGltf* Open(maniBuilder* builder, const maniGltfOptions* options, maniSkeleton* skeleton)
{
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, s_json, strlen(s_json), &gltf);
    CHECK(r.result == mani_success, "opens");
    size_t size = 0;
    bool built =
        r.result == mani_success &&
        maniGltfBuildSkeleton(builder, gltf, 0, options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, skeleton).result == mani_success;
    CHECK(built, "a skeleton");
    return gltf;
}

static void TestValues(void)
{
    Document("1.0", kColliders, kGroups, kSprings);
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniGltf* gltf = Open(builder, &options, &skeleton);
    maniSpringSetSource set;
    maniImportResult r = maniGltfReadSprings(gltf, &skeleton, &options, &set);
    CHECK(r.result == mani_success, "reads");
    CHECK(set.skeleton == &skeleton && set.colliderCount == 2 && set.colliderGroupCount == 2 &&
              set.springCount == 1,
          "counts");
    uint32_t head = maniSkeletonFindBone(&skeleton, "head", 4);
    uint32_t hip = maniSkeletonFindBone(&skeleton, "hip", 3);
    const maniColliderSource* sphere = &set.colliders[0];
    CHECK(sphere->bone == head && sphere->shape == maniColliderSphere && sphere->offset.y == 0.1f &&
              sphere->radius == 0.08f,
          "a sphere");
    const maniColliderSource* capsule = &set.colliders[1];
    CHECK(capsule->bone == hip && capsule->shape == maniColliderCapsule &&
              capsule->tail.y == 0.2f && capsule->radius == 0.05f,
          "a capsule");
    CHECK(set.colliderGroups[0].nameLength == 4 &&
              memcmp(set.colliderGroups[0].name, "head", 4) == 0 &&
              set.colliderGroups[0].colliderCount == 1,
          "a named group");
    CHECK(set.colliderGroups[1].name == NULL && set.colliderGroups[1].colliderCount == 2 &&
              set.colliderGroups[1].colliders[1] == 1,
          "an unnamed group");
    const maniSpringSource* hair = &set.springs[0];
    CHECK(hair->nameLength == 4 && hair->jointCount == 3 && hair->colliderGroupCount == 2 &&
              hair->center == hip,
          "the spring");
    const maniSpringJointSource* first = &hair->joints[0];
    CHECK(first->bone == maniSkeletonFindBone(&skeleton, "hair1", 5) && first->hitRadius == 0.02f &&
              first->stiffness == 1.5f && first->gravityPower == 0.3f && first->drag == 0.4f,
          "a joint's values");
    const maniSpringJointSource* tail = &hair->joints[2];
    CHECK(tail->stiffness == 1.0f && tail->drag == 0.5f && tail->hitRadius == 0.0f &&
              tail->gravityPower == 0.0f && tail->gravityDirection.y == -1.0f,
          "the schema's defaults");
    // A second read replaces the first; the set stays valid until then.
    maniGltfOptions zUp = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 0};
    maniSkeleton skeletonZ;
    size_t size = 0;
    CHECK(maniGltfBuildSkeleton(builder, gltf, 0, &zUp).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &skeletonZ).result == mani_success,
          "z up");
    CHECK(maniGltfReadSprings(gltf, &skeletonZ, &zUp, &set).result == mani_success, "reads");
    CHECK(set.colliders[0].offset.z == 0.1f && set.colliders[0].offset.y == 0.0f, "offset z up");
    CHECK(set.springs[0].joints[2].gravityDirection.z == -1.0f, "gravity down is -z");
    CHECK(maniGltfReadSprings(gltf, NULL, &zUp, &set).result == mani_errorInvalid, "no skeleton");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestNone(void)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"name\":\"a\"}],"
             "\"skins\":[{\"joints\":[0]}]}");
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniGltf* gltf = Open(builder, &options, &skeleton);
    maniSpringSetSource set;
    memset(&set, 0xFF, sizeof set);
    CHECK(maniGltfReadSprings(gltf, &skeleton, &options, &set).result == mani_success &&
              set.springCount == 0 && set.colliderCount == 0,
          "no extension, no springs");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(maniImportReason reason, const char* at, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniGltf* gltf = Open(builder, &options, &skeleton);
    maniSpringSetSource set;
    maniImportResult r = maniGltfReadSprings(gltf, &skeleton, &options, &set);
    const char* found = at != NULL ? strstr(s_json, at) : NULL;
    bool right = r.result == mani_errorFormat && r.reason == reason &&
                 (at == NULL || (found != NULL && r.offset == (uint64_t)(found - s_json)));
    CHECK(right, what);
    if (!right)
    {
        printf("  reason %u offset %llu near: %.40s\n", r.reason, (unsigned long long)r.offset,
               r.offset < strlen(s_json) ? s_json + r.offset : "");
    }
    CHECK(set.springCount == 0 && set.springs == NULL, "nothing on a refusal");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    Document("0.9", kColliders, kGroups, kSprings);
    ExpectRefusal(maniImportReasonVersion, "0.9", "another version");
    Document("1.0", kColliders, kGroups,
             "[{\"joints\":[{\"node\":1,\"dragForce\":1.5},{\"node\":2}]}]");
    ExpectRefusal(maniImportReasonProperty, "1.5", "drag past 1");
    Document("1.0", kColliders, kGroups,
             "[{\"joints\":[{\"node\":1,\"stiffness\":-1},{\"node\":2}]}]");
    ExpectRefusal(maniImportReasonProperty, "-1", "stiffness below 0");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[{\"node\":3},{\"node\":1}]}]");
    ExpectRefusal(maniImportReasonHierarchy, "{\"node\":1}", "a chain going up");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[{\"node\":1},{\"node\":4}]}]");
    ExpectRefusal(maniImportReasonHierarchy, "{\"node\":4}", "a chain across branches");
    Document("1.0", kColliders, kGroups,
             "[{\"joints\":[{\"node\":1},{\"node\":2}]},{\"joints\":[{\"node\":2},{\"node\":3}]}]");
    ExpectRefusal(maniImportReasonProperty, "{\"node\":2},{\"node\":3}", "a joint in two chains");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[{\"node\":1},{\"node\":2}],\"center\":4}]");
    ExpectRefusal(maniImportReasonHierarchy, "4}]", "a center off the chain");
    // The specification: a center is no joint of another chain, nor
    // below one; here the later chain's joint is above the center.
    Document("1.0", kColliders, kGroups,
             "[{\"joints\":[{\"node\":2},{\"node\":3}],\"center\":2},"
             "{\"joints\":[{\"node\":1}]}]");
    ExpectRefusal(maniImportReasonHierarchy, "2},{\"joints\":[{\"node\":1}",
                  "a center below another chain");
    Document("1.0", kColliders, kGroups,
             "[{\"joints\":[{\"node\":1}]},{\"joints\":[{\"node\":2},{\"node\":3}],"
             "\"center\":1}]");
    ExpectRefusal(maniImportReasonHierarchy, "1}]}}}", "a center on another chain");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[{\"node\":1}],\"colliderGroups\":[2]}]");
    ExpectRefusal(maniImportReasonIndex, "2]}]", "a group past the count");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[]}]");
    ExpectRefusal(maniImportReasonProperty, "[]}]", "no joints");
    Document("1.0", "[{\"node\":4,\"shape\":{\"sphere\":{},\"capsule\":{}}}]",
             "[{\"colliders\":[0]}]", kSprings);
    ExpectRefusal(maniImportReasonProperty, "{\"sphere\":{},", "a sphere and a capsule");
    Document("1.0", "[{\"node\":4,\"shape\":{}}]", "[{\"colliders\":[0]}]", kSprings);
    ExpectRefusal(maniImportReasonProperty, "{}}]", "neither");
    Document("1.0", "[{\"node\":4,\"shape\":{\"sphere\":{\"radius\":-0.1}}}]",
             "[{\"colliders\":[0]}]", kSprings);
    ExpectRefusal(maniImportReasonProperty, "-0.1", "a negative radius");
    Document("1.0", kColliders, "[{\"colliders\":[5]}]", kSprings);
    ExpectRefusal(maniImportReasonIndex, "5]", "a collider past the count");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[{\"node\":1,\"gravityDir\":[0,-1]}]}]");
    ExpectRefusal(maniImportReasonProperty, "[0,-1]", "two numbers for a direction");
    Document("1.0", kColliders, kGroups, "[{\"joints\":[{\"node\":1,\"gravityDir\":[0,-1,0,0]}]}]");
    ExpectRefusal(maniImportReasonProperty, "[0,-1,0,0]", "four numbers for a direction");
    Document("1.0", kColliders, "[{\"colliders\":[0.5]}]", kSprings);
    ExpectRefusal(maniImportReasonProperty, "0.5]", "an index not whole");
    Document("1.0", kColliders, kGroups,
             "[{\"joints\":[{\"node\":1},{\"node\":2},{\"node\":3}],\"center\":2}]");
    ExpectRefusal(maniImportReasonHierarchy, "2}]", "a center inside its chain");
    // A node outside the skeleton given: a skeleton of the head alone.
    Document("1.0", kColliders, kGroups, kSprings);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(maniOpenGltf(builder, s_json, strlen(s_json), &gltf).result == mani_success, "opens");
    maniGltfOptions options = maniDefaultGltfOptions();
    size_t size = 0;
    maniSkeleton head;
    CHECK(maniGltfBuildNodeSkeleton(builder, gltf, 4, &options).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &head).result == mani_success,
          "the head alone");
    maniSpringSetSource set;
    maniImportResult r = maniGltfReadSprings(gltf, &head, &options, &set);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBone, "a bone not there");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestValues();
    TestNone();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
