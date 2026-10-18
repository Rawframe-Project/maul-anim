// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The OMI physics extensions as a ragdoll source (mani-0027): a pelvis,
// spine, chest and head skeleton with bodies on the pelvis (a box on a
// turned child collider), the chest (a capsule on its own node) and the
// head (a sphere on a node turned a quarter about z and scaled by 2),
// joints from the chest to the
// pelvis (limited, driven) and from the head to the chest (a pin), and a
// floor and a crate no bone holds, left out; frames, sizes, limits,
// drives and the root's axes read; the source built; every refusal at
// its value; a document without bodies. Run as test_gltf_ragdoll --seed
// <file> it writes the document read, a seed of the glTF fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/ragdoll.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static char s_variant[16384];
static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_ragdollFile[16384];

// Nodes: 0 pelvis, 1 spine, 2 chest, 3 head (the skin's joints); 4 a
// floor and 5 a crate at the root; 6 the pelvis's body, 7 its collider,
// 12 its joint side; 8 the chest's body and collider, 9 its joint to the
// pelvis, 11 the head joint's chest side; 10 the head's body, 13 its
// joint.
static const char kDocument[] =
    "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0,4,5]}],"
    "\"nodes\":["
    "{\"name\":\"pelvis\",\"translation\":[0,1,0],\"children\":[1,6]},"
    "{\"name\":\"spine\",\"translation\":[0,0.2,0],\"children\":[2]},"
    "{\"name\":\"chest\",\"translation\":[0,0.3,0],\"children\":[3,8]},"
    "{\"name\":\"head\",\"translation\":[0,0.3,0],\"children\":[10]},"
    "{\"name\":\"floor\",\"extensions\":{\"OMI_physics_body\":{\"motion\":{\"type\":\"static\"},"
    "\"collider\":{\"shape\":3}}}},"
    "{\"name\":\"crate\",\"extensions\":{\"OMI_physics_body\":{\"motion\":{\"type\":\"dynamic\"},"
    "\"collider\":{\"shape\":3}}}},"
    "{\"name\":\"pelvisBody\",\"translation\":[0,0.05,0],\"children\":[7,12],"
    "\"extensions\":{\"OMI_physics_body\":{\"motion\":{\"type\":\"dynamic\",\"mass\":12,"
    "\"centerOfMass\":[0,0,0],\"gravityFactor\":1,\"inertiaDiagonal\":[1,2,3],"
    "\"linearVelocity\":[1,0,0]}}}},"
    "{\"name\":\"pelvisShape\",\"rotation\":[0,0.70710677,0,0.70710677],"
    "\"extensions\":{\"OMI_physics_body\":{\"collider\":{\"shape\":0}}}},"
    "{\"name\":\"chestBody\",\"translation\":[0,0.1,0],\"children\":[9,11],"
    "\"extensions\":{\"OMI_physics_body\":{\"motion\":{\"type\":\"kinematic\",\"mass\":15},"
    "\"collider\":{\"shape\":1},\"trigger\":{\"shape\":2}}}},"
    "{\"name\":\"chestJoint\",\"translation\":[0,-0.3,0],"
    "\"extensions\":{\"OMI_physics_joint\":{\"joint\":0,\"connectedNode\":12,"
    "\"enableCollision\":true}}},"
    "{\"name\":\"headBody\",\"translation\":[0,0.1,0],\"rotation\":[0,0,0.70710677,0.70710677],"
    "\"scale\":[2,2,2],\"children\":[13],"
    "\"extensions\":{\"OMI_physics_body\":{\"motion\":{\"type\":\"dynamic\",\"mass\":5},"
    "\"collider\":{\"shape\":2}}}},"
    "{\"name\":\"chestSide\",\"translation\":[0,0.2,0]},"
    "{\"name\":\"pelvisSide\",\"translation\":[0,0.25,0]},"
    "{\"name\":\"headJoint\",\"translation\":[-0.05,0,0],"
    "\"rotation\":[0,0,-0.70710677,0.70710677],"
    "\"extensions\":{\"OMI_physics_joint\":{\"joint\":1,\"connectedNode\":11}}}"
    "],"
    "\"skins\":[{\"joints\":[0,1,2,3]}],"
    "\"extensions\":{"
    "\"OMI_physics_shape\":{\"shapes\":["
    "{\"type\":\"box\",\"box\":{\"size\":[0.3,0.2,0.4]}},"
    "{\"type\":\"capsule\",\"capsule\":{\"height\":0.2,\"radiusBottom\":0.1,"
    "\"radiusTop\":0.1}},"
    "{\"type\":\"sphere\",\"sphere\":{\"radius\":0.1}},"
    "{\"type\":\"box\"}]},"
    "\"OMI_physics_joint\":{\"physicsJoints\":["
    "{\"limits\":[{\"linearAxes\":[0,1,2],\"min\":0,\"max\":0},"
    "{\"angularAxes\":[0],\"min\":-0.5,\"max\":0.7},"
    "{\"angularAxes\":[1,2],\"min\":-0.4,\"max\":0.4,\"stiffness\":-1}],"
    "\"drives\":[{\"type\":\"angular\",\"mode\":\"force\",\"axis\":0,\"stiffness\":50,"
    "\"damping\":5,\"maxForce\":100},"
    "{\"type\":\"angular\",\"mode\":\"force\",\"axis\":1,\"stiffness\":50,\"damping\":5,"
    "\"maxForce\":100,\"positionTarget\":0},"
    "{\"type\":\"angular\",\"mode\":\"force\",\"axis\":2,\"stiffness\":50,\"damping\":5,"
    "\"maxForce\":100,\"velocityTarget\":0}]},"
    "{\"limits\":[{\"linearAxes\":[0,1,2],\"min\":0,\"max\":0}]}]}}}";

static const maniVec3 kFacing = {0.0f, 0.0f, 1.0f};
static const maniVec3 kUp = {0.0f, 1.0f, 0.0f};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Opens text and builds skeleton 0 with the options.
static maniGltf* Open(maniBuilder* builder, const char* text, const maniGltfOptions* options,
                      maniSkeleton* skeleton)
{
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, text, strlen(text), &gltf);
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

static bool Near(float a, double b)
{
    return fabs((double)a - b) <= 1e-6;
}

static bool NearV(maniVec3 a, double x, double y, double z)
{
    return Near(a.x, x) && Near(a.y, y) && Near(a.z, z);
}

static bool NearQ(maniQuat q, double x, double y, double z, double w)
{
    return Near(q.x, x) && Near(q.y, y) && Near(q.z, z) && Near(q.w, w);
}

static void TestRead(void)
{
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniGltf* gltf = Open(builder, kDocument, &options, &skeleton);
    maniRagdollSource source;
    maniImportResult r = maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kUp, &source);
    CHECK(r.result == mani_success && source.skeleton == &skeleton && source.bodyCount == 3,
          "three bodies, the floor and the crate left out");
    if (r.result != mani_success || source.bodyCount != 3)
    {
        printf("reason %u offset %llu item %u\n", r.reason, (unsigned long long)r.offset, r.item);
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
        return;
    }
    const maniRagdollBodySource* pelvis = &source.bodies[0];
    const maniRagdollBodySource* chest = &source.bodies[1];
    const maniRagdollBodySource* head = &source.bodies[2];
    CHECK(pelvis->bone == maniSkeletonFindBone(&skeleton, "pelvis", 6) &&
              chest->bone == maniSkeletonFindBone(&skeleton, "chest", 5) &&
              head->bone == maniSkeletonFindBone(&skeleton, "head", 4),
          "each body on its bone");
    CHECK(pelvis->nameLength == 10 && memcmp(pelvis->name, "pelvisBody", 10) == 0 &&
              pelvis->mass == 12.0f && chest->mass == 15.0f && head->mass == 5.0f,
          "names and masses");
    CHECK(pelvis->shape == maniRagdollBox && NearV(pelvis->size, 0.15, 0.1, 0.2) &&
              NearV(pelvis->frameTranslation, 0.0, 0.05, 0.0) &&
              NearQ(pelvis->frameRotation, 0.0, 0.70710677, 0.0, 0.70710677),
          "the pelvis: a half-size box on its turned collider's frame");
    CHECK(chest->shape == maniRagdollCapsule && NearV(chest->size, 0.1, 0.2, 0.0) &&
              NearV(chest->frameTranslation, 0.0, 0.1, 0.0),
          "the chest: a capsule");
    CHECK(head->shape == maniRagdollSphere && NearV(head->size, 0.2, 0.0, 0.0) &&
              NearV(head->frameTranslation, 0.0, 0.1, 0.0) &&
              NearQ(head->frameRotation, 0.0, 0.0, 0.70710677, 0.70710677),
          "the head: a sphere twice its radius, its frame rigid and turned");
    const maniRagdollJointSource* j = &chest->joint;
    CHECK(NearV(j->translation, 0.0, -0.3, 0.0) && NearQ(j->rotation, 0.0, 0.0, 0.0, 1.0) &&
              j->twistMin == -0.5f && j->twistMax == 0.7f && j->swingY == 0.4f && j->swingZ == 0.4f,
          "the chest's joint: its frame, twist and swings");
    CHECK(j->driveStiffness == 50.0f && j->driveDamping == 5.0f && j->driveMaxTorque == 100.0f,
          "its drive");
    j = &head->joint;
    CHECK(NearV(j->translation, -0.1, 0.0, 0.0) &&
              NearQ(j->rotation, 0.0, 0.0, -0.70710677, 0.70710677) &&
              j->twistMin == -3.14159265f && j->twistMax == 3.14159265f &&
              j->swingY == 3.14159265f && j->swingZ == 3.14159265f && j->driveStiffness == 0.0f &&
              j->driveMaxTorque == 0.0f,
          "the head's pin: every angle free, no drive");
    CHECK(NearV(source.facing, -1.0, 0.0, 0.0) && NearV(source.up, 0.0, 1.0, 0.0),
          "facing and up in the pelvis body's turned frame");
    maniRagdoll ragdoll;
    size_t size = 0;
    CHECK(maniBuildRagdoll(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_ragdollFile, sizeof s_ragdollFile, &size) == mani_success &&
              maniOpenRagdoll(s_ragdollFile, size, &ragdoll).result == mani_success &&
              maniRagdollBodyCount(&ragdoll) == 3,
          "the source builds");
    CHECK(maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kFacing, &source).result ==
                  mani_errorInvalid &&
              maniGltfReadRagdoll(gltf, &skeleton, &options, (maniVec3){0.0f, 0.0f, 1.1f}, kUp,
                                  &source)
                      .result == mani_errorInvalid &&
              maniGltfReadRagdoll(NULL, &skeleton, &options, kFacing, kUp, &source).result ==
                  mani_errorInvalid &&
              maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kUp, NULL).result ==
                  mani_errorInvalid,
          "axes not perpendicular or not unit, NULL arguments");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// glTF's +Y up as the output's +Z: the axes and frames in that basis.
static void TestBasis(void)
{
    maniBuilder* builder = NewBuilder();
    maniGltfOptions zUp = {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 0};
    maniSkeleton skeleton;
    maniGltf* gltf = Open(builder, kDocument, &zUp, &skeleton);
    maniRagdollSource source;
    CHECK(maniGltfReadRagdoll(gltf, &skeleton, &zUp, kFacing, kUp, &source).result ==
                  mani_success &&
              NearV(source.bodies[1].frameTranslation, 0.0, 0.0, 0.1) &&
              NearV(source.bodies[1].joint.translation, 0.0, 0.0, -0.3) &&
              NearV(source.up, 0.0, 0.0, 1.0),
          "frames and axes in the output's basis");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// The document with the first `from` after `after` replaced by `to`.
static const char* Variant(const char* after, const char* from, const char* to)
{
    const char* start = strstr(kDocument, after);
    const char* at = start != NULL ? strstr(start, from) : NULL;
    CHECK(at != NULL, "the variant's text is in the document");
    if (at == NULL)
    {
        return kDocument;
    }
    size_t head = (size_t)(at - kDocument);
    snprintf(s_variant, sizeof s_variant, "%.*s%s%s", (int)head, kDocument, to, at + strlen(from));
    return s_variant;
}

typedef struct Refusal
{
    const char* after;
    const char* from;
    const char* to;
    maniImportReason reason;
    // The refused value: the variant's text at the first `marker` after
    // `after`.
    const char* marker;
    const char* what;
} Refusal;

static const Refusal kRefusals[] = {
    {"\"pelvisBody\"", "\"dynamic\"", "\"static\"", maniImportReasonUnsupported, "static\"",
     "a static body on a bone"},
    {"\"pelvisBody\"", "\"dynamic\"", "\"wobbly\"", maniImportReasonProperty, "wobbly\"",
     "a motion type OMI has not"},
    {"\"pelvisBody\"", "\"mass\":12", "\"mass\":0", maniImportReasonProperty, "0,\"center",
     "a mass of 0"},
    {"\"pelvisBody\"", "[0,0,0]", "[0,0.1,0]", maniImportReasonUnsupported, "[0,0.1,0]",
     "a center of mass off the origin"},
    {"\"pelvisBody\"", "\"gravityFactor\":1", "\"gravityFactor\":2", maniImportReasonUnsupported,
     "2,\"inertia", "a gravity factor"},
    {"\"pelvisShape\"", "\"shape\":0", "\"shape\":4", maniImportReasonIndex, "4}",
     "a shape past "
     "the shapes"},
    {"\"chestBody\"", "\"shape\":1",
     "\"shape\":0},\"trigger\":{\"shape\":2}}}},"
     "{\"name\":\"x\",\"extensions\":{\"OMI_physics_body\":{\"collider\":{\"shape\":1",
     maniImportReasonUnsupported, "{\"shape\":1", "a second collider"},
    {"\"shapes\"", "\"type\":\"box\",\"box\"", "\"type\":\"cylinder\",\"box\"",
     maniImportReasonUnsupported, "cylinder\"", "a cylinder"},
    {"\"shapes\"", "\"type\":\"box\",\"box\"", "\"type\":\"cone\",\"box\"",
     maniImportReasonProperty, "cone\"", "a shape OMI has not"},
    {"\"radiusTop\"", "\"radiusTop\":0.1", "\"radiusTop\":0.12", maniImportReasonUnsupported,
     "0.12", "a tapered capsule"},
    {"\"radiusBottom\"", ",\"radiusTop\":0.1", "", maniImportReasonUnsupported, "0.1}",
     "a tapered capsule by the top's default"},
    {"\"capsule\":{", "\"height\":0.2", "\"height\":-0.2", maniImportReasonProperty, "-0.2",
     "a negative height"},
    {"\"pelvisBody\"", "[0,0,0]", "[0,0]", maniImportReasonUnsupported, "[0,0]",
     "a center of mass of two numbers"},
    {"\"chestBody\"", "\"collider\":{\"shape\":1},", "", maniImportReasonUnsupported,
     "{\"motion\":{\"type\":\"kinematic\"", "a body without a collider"},
    {"\"chestSide\"", "\"translation\":[0,0.2,0]}",
     "\"translation\":[0,0.2,0],\"extensions\":{\"OMI_physics_joint\":{\"joint\":1,"
     "\"connectedNode\":13}}}",
     maniImportReasonUnsupported, "{\"joint\":1,\"connectedNode\":11", "a second joint on a body"},
    {"\"chestJoint\"", "\"translation\":[0,-0.3,0]",
     "\"translation\":[0,-0.3,0],\"rotation\":[0,0,0.002,0.999998]", maniImportReasonUnsupported,
     "{\"joint\":0", "a joint's sides turned apart at rest"},
    {"\"drives\"",
     ",{\"type\":\"angular\",\"mode\":\"force\",\"axis\":2,\"stiffness\":50,\"damping\":5,"
     "\"maxForce\":100,\"velocityTarget\":0}",
     "", maniImportReasonUnsupported, "[{\"type\":\"angular\"", "drives on two axes only"},
    {"\"drives\"",
     "\"stiffness\":50,\"damping\":5,\"maxForce\":100},{\"type\":\"angular\",\"mode\":\"force\","
     "\"axis\":1,\"stiffness\":50,\"damping\":5,\"maxForce\":100,\"positionTarget\":0},"
     "{\"type\":\"angular\",\"mode\":\"force\",\"axis\":2,\"stiffness\":50,\"damping\":5,"
     "\"maxForce\":100,\"velocityTarget\":0}",
     "\"stiffness\":0,\"damping\":0,\"maxForce\":0},{\"type\":\"angular\",\"mode\":\"force\","
     "\"axis\":1,\"stiffness\":0,\"damping\":0,\"maxForce\":0}",
     maniImportReasonUnsupported, "[{\"type\":\"angular\"", "zero drives on two axes only"},
    {"\"headBody\"", "\"scale\":[2,2,2]", "\"scale\":[2,1,2]", maniImportReasonUnsupported,
     "{\"shape\":2}", "a collider scaled unevenly"},
    {"\"chestJoint\"", "[0,-0.3,0]", "[0,-0.2,0]", maniImportReasonUnsupported, "{\"joint\":0",
     "a joint's sides apart at rest"},
    {"\"headJoint\"", "\"connectedNode\":11", "\"connectedNode\":12", maniImportReasonUnsupported,
     "{\"joint\":1", "a joint between bodies not parent and child"},
    {"\"headJoint\"", "\"connectedNode\":11", "\"connectedNode\":1", maniImportReasonProperty,
     "{\"joint\":1", "a joint to a node no body holds"},
    {"\"headJoint\"", "\"connectedNode\":11", "\"connectedNode\":99", maniImportReasonIndex, "99",
     "a connected node past the nodes"},
    {"\"headJoint\"", "\"joint\":1", "\"joint\":2", maniImportReasonIndex, "2,",
     "settings past "
     "the joints"},
    {"\"headBody\"", "{\"OMI_physics_joint\":{\"joint\":1", "{\"x\":{\"joint\":1",
     maniImportReasonUnsupported, "{\"motion\":{\"type\":\"dynamic\",\"mass\":5",
     "a body without its joint"},
    {"\"physicsJoints\"", "\"linearAxes\":[0,1,2]", "\"linearAxes\":[0,1]",
     maniImportReasonUnsupported, "{\"limits\"", "a linear axis free"},
    {"\"physicsJoints\"", "\"min\":0,\"max\":0", "\"min\":0,\"max\":0.1",
     maniImportReasonUnsupported, "{\"linearAxes\"", "a linear axis not fixed at 0"},
    {"\"physicsJoints\"", "\"min\":-0.4", "\"min\":-0.3", maniImportReasonUnsupported,
     "[{\"linearAxes\"", "an asymmetric swing"},
    {"\"physicsJoints\"", "\"stiffness\":-1", "\"stiffness\":10", maniImportReasonUnsupported,
     "10}", "a soft limit"},
    {"\"physicsJoints\"", "\"min\":-0.5,\"max\":0.7", "\"min\":0.8,\"max\":0.7",
     maniImportReasonProperty, "{\"angularAxes\":[0]", "a limit's min above its max"},
    {"\"physicsJoints\"", "{\"angularAxes\":[0]", "{\"angularAxes\":[1]", maniImportReasonProperty,
     "{\"angularAxes\":[1,2]", "an axis limited twice"},
    {"\"physicsJoints\"", "{\"angularAxes\":[0]", "{\"angularAxes\":[3]", maniImportReasonProperty,
     "3]", "an axis past z"},
    {"\"drives\"", "\"type\":\"angular\"", "\"type\":\"linear\"", maniImportReasonUnsupported,
     "linear\"", "a linear drive"},
    {"\"drives\"", "\"mode\":\"force\"", "\"mode\":\"acceleration\"", maniImportReasonUnsupported,
     "acceleration\"", "acceleration mode"},
    {"\"drives\"", "\"positionTarget\":0", "\"positionTarget\":0.2", maniImportReasonUnsupported,
     "0.2", "a drive with a position target"},
    {"\"drives\"", "\"velocityTarget\":0", "\"velocityTarget\":1", maniImportReasonUnsupported,
     "1}]", "a drive with a velocity target"},
    {"\"drives\"", "\"axis\":1,\"stiffness\":50", "\"axis\":1,\"stiffness\":40",
     maniImportReasonUnsupported, "[{\"type\":\"angular\"", "drives unequal across the axes"},
    {"\"drives\"", "\"axis\":2", "\"axis\":1", maniImportReasonProperty,
     "{\"type\":\"angular\",\"mode\":\"force\",\"axis\":1,\"stiffness\":50,\"damping\":5,"
     "\"maxForce\":100,\"velocityTarget\"",
     "an axis driven twice"},
    {"\"drives\"", "\"damping\":5,\"maxForce\":100}", "\"damping\":-5,\"maxForce\":100}",
     maniImportReasonProperty, "-5", "a negative damping"},
};

static void TestRefusals(void)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    for (size_t k = 0; k < sizeof kRefusals / sizeof kRefusals[0]; ++k)
    {
        const Refusal* c = &kRefusals[k];
        const char* text = Variant(c->after, c->from, c->to);
        maniBuilder* builder = NewBuilder();
        maniSkeleton skeleton;
        maniGltf* gltf = Open(builder, text, &options, &skeleton);
        maniRagdollSource source;
        maniImportResult r =
            gltf != NULL ? maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kUp, &source)
                         : (maniImportResult){mani_errorInvalid, 0, 0, 0, 0};
        const char* start = strstr(text, c->after);
        const char* marker = start != NULL ? strstr(start, c->marker) : NULL;
        uint64_t want = marker != NULL ? (uint64_t)(marker - text) : UINT64_MAX;
        bool ok = r.result == mani_errorFormat && r.reason == c->reason && r.offset == want;
        CHECK(ok, c->what);
        if (!ok)
        {
            printf("  %s: result %d reason %u offset %llu want %llu\n", c->what, (int)r.result,
                   r.reason, (unsigned long long)r.offset, (unsigned long long)want);
        }
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
    }
}

// text with every `from` replaced by `to`, into out.
static void ReplaceAll(const char* text, const char* from, const char* to, char* out,
                       size_t capacity)
{
    size_t n = 0;
    size_t length = strlen(from);
    for (const char* at = text; *at != '\0' && n + 1 < capacity;)
    {
        if (strncmp(at, from, length) == 0)
        {
            n += (size_t)snprintf(out + n, capacity - n, "%s", to);
            at += length;
        }
        else
        {
            out[n++] = *at++;
        }
    }
    out[n < capacity ? n : capacity - 1] = '\0';
}

// The chest's drive with no largest force (the largest finite float),
// and with stiffness and damping 0 (no drive at all).
static void TestDriveValues(void)
{
    static const struct
    {
        const char* from;
        const char* to;
        float stiffness;
        float maxTorque;
        const char* what;
    } kCases[2] = {{",\"maxForce\":100", "", 50.0f, FLT_MAX, "unbounded: the largest finite float"},
                   {"\"stiffness\":50,\"damping\":5", "\"stiffness\":0,\"damping\":0", 0.0f, 0.0f,
                    "stiffness and damping 0: no drive"}};
    maniGltfOptions options = maniDefaultGltfOptions();
    for (int k = 0; k < 2; ++k)
    {
        ReplaceAll(kDocument, kCases[k].from, kCases[k].to, s_variant, sizeof s_variant);
        maniBuilder* builder = NewBuilder();
        maniSkeleton skeleton;
        maniGltf* gltf = Open(builder, s_variant, &options, &skeleton);
        maniRagdollSource source;
        bool read = maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kUp, &source).result ==
                    mani_success;
        const maniRagdollJointSource* j = read ? &source.bodies[1].joint : NULL;
        CHECK(j != NULL && j->driveStiffness == kCases[k].stiffness &&
                  j->driveMaxTorque == kCases[k].maxTorque,
              kCases[k].what);
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
    }
}

// Without the extensions: an empty source; a second read replaces the
// first.
static void TestNone(void)
{
    static const char kPlain[] = "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"name\":\"a\"}],"
                                 "\"skins\":[{\"joints\":[0]}]}";
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniGltf* gltf = Open(builder, kPlain, &options, &skeleton);
    maniRagdollSource source;
    memset(&source, 0xFF, sizeof source);
    CHECK(maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kUp, &source).result ==
                  mani_success &&
              source.bodyCount == 0 && source.bodies == NULL && source.skeleton == &skeleton,
          "no bodies");
    CHECK(maniGltfReadRagdoll(gltf, &skeleton, &options, kFacing, kUp, &source).result ==
              mani_success,
          "read again");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static int Seed(const char* path)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(kDocument, 1, sizeof kDocument - 1, out);
    return fclose(out) == 0 && written == sizeof kDocument - 1 ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return Seed(argv[2]);
    }
    TestRead();
    TestBasis();
    TestRefusals();
    TestDriveValues();
    TestNone();
    return s_failures == 0 ? 0 : 1;
}
