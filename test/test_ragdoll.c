// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ragdoll profiles: a humanoid's ragdoll (pelvis, chest
// above a spine without a body, head, an arm of two bodies, the hand left
// to the animation) built from bodies given out of order, opened and read
// back: the bodies by bone, each parent the nearest ancestor with a body,
// each joint's frame in the parent body the same place in the rest pose
// as in the child, under turned and scaled bones; the builder's refusals
// with their reasons; opening's checks of every record against a file
// changed at its byte. Run as test_ragdoll --seed <file> it writes the
// fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "ragdoll_file.h"
#include "ragdoll_fixture.h"
#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/ragdoll.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static _Alignas(16) uint8_t s_crafted[8192];

typedef struct Rigid
{
    maniVec3 t;
    maniQuat r;
} Rigid;

static maniQuat Mul(maniQuat a, maniQuat b)
{
    return (maniQuat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static maniVec3 Rotate(maniQuat q, maniVec3 v)
{
    maniQuat p = Mul(Mul(q, (maniQuat){v.x, v.y, v.z, 0.0f}), (maniQuat){-q.x, -q.y, -q.z, q.w});
    return (maniVec3){p.x, p.y, p.z};
}

static Rigid Compose(Rigid a, Rigid b)
{
    maniVec3 t = Rotate(a.r, b.t);
    return (Rigid){{a.t.x + t.x, a.t.y + t.y, a.t.z + t.z}, Mul(a.r, b.r)};
}

// A bone's rest model frame, its scale left out, from the pose.
static Rigid BoneFrame(uint32_t bone)
{
    maniTransform m = maniPoseModelTransform(s_pose, bone);
    return (Rigid){m.translation, m.rotation};
}

// The joint's model frame as a body's frame of it places it.
static Rigid JointIn(const maniRagdollBody* b, maniVec3 at, maniQuat turn)
{
    Rigid body = Compose(BoneFrame(b->bone), (Rigid){b->frameTranslation, b->frameRotation});
    return Compose(body, (Rigid){at, turn});
}

static float Apart(Rigid a, Rigid b)
{
    float d = fabsf(a.t.x - b.t.x) + fabsf(a.t.y - b.t.y) + fabsf(a.t.z - b.t.z);
    float dot = fabsf(a.r.x * b.r.x + a.r.y * b.r.y + a.r.z * b.r.z + a.r.w * b.r.w);
    return d + (1.0f - dot);
}

static void TestBuild(void)
{
    maniRagdollSource source = Source();
    maniRagdoll ragdoll;
    CHECK(Build(&source).result == mani_success &&
              maniOpenRagdoll(s_file, s_size, &ragdoll).result == mani_success,
          "built and opened");
    CHECK(maniRagdollBodyCount(&ragdoll) == 5 && maniRagdollIdentity(&ragdoll) != 0 &&
              maniRagdollSkeletonIdentity(&ragdoll) == maniSkeletonIdentity(&s_skeleton) &&
              ragdoll.facing.z == 1.0f && ragdoll.up.y == 1.0f,
          "its count, identities and axes");
    // By bone: pelvis, chest (its parent the pelvis, past the spine),
    // head and upper arm (the chest's), lower arm (the upper arm's).
    const uint32_t bones[5] = {s_bone[kPelvis], s_bone[kChest], s_bone[kHead], s_bone[kUpperArm],
                               s_bone[kLowerArm]};
    const uint32_t parents[5] = {MANI_NO_BONE, 0, 1, 1, 3};
    const char* names[5] = {"pelvis", "chest", "head", "upperArm", "lowerArm"};
    maniRagdollBody b[5];
    bool read = true;
    for (uint32_t i = 0; i < 5; ++i)
    {
        read = read && maniRagdollReadBody(&ragdoll, i, &b[i]) == mani_success &&
               b[i].bone == bones[i] && b[i].parent == parents[i] &&
               b[i].nameLength == strlen(names[i]) &&
               memcmp(b[i].name, names[i], b[i].nameLength) == 0;
    }
    CHECK(read, "the bodies by bone, their parents and names");
    CHECK(b[0].shape == maniRagdollBox && b[0].size.x == 0.15f && b[0].mass == 12.0f &&
              b[3].shape == maniRagdollCapsule && b[3].frameTranslation.y == 0.14f &&
              b[2].shape == maniRagdollSphere && b[2].size.y == 0.0f &&
              b[1].frameRotation.y == 0.38268343f && b[4].twistMin == -0.5f &&
              b[4].twistMax == 0.75f && b[4].swingY == 0.6f && b[4].swingZ == 0.4f &&
              b[4].driveStiffness == 200.0f && b[4].driveDamping == 20.0f &&
              b[4].driveMaxTorque == 500.0f && b[4].jointTranslation.x == 0.02f,
          "shapes, masses, frames, limits and drives kept");
    CHECK(b[0].jointRotation.w == 1.0f && b[0].parentJointRotation.w == 1.0f &&
              b[0].jointTranslation.y == 0.0f && b[0].twistMax == 0.0f &&
              b[0].driveStiffness == 0.0f,
          "the root without a joint");
    float worst = 0.0f;
    for (uint32_t i = 1; i < 5; ++i)
    {
        Rigid child = JointIn(&b[i], b[i].jointTranslation, b[i].jointRotation);
        Rigid parent =
            JointIn(&b[b[i].parent], b[i].parentJointTranslation, b[i].parentJointRotation);
        worst = fmaxf(worst, Apart(child, parent));
    }
    CHECK(worst < 2e-6f, "each joint at one place in both bodies at rest");
    maniRagdollBody none;
    CHECK(maniRagdollReadBody(&ragdoll, 5, &none) == mani_errorInvalid &&
              maniRagdollReadBody(NULL, 0, &none) == mani_errorInvalid &&
              maniRagdollReadBody(&ragdoll, 0, NULL) == mani_errorInvalid &&
              maniRagdollBodyCount(NULL) == 0 && maniRagdollIdentity(NULL) == 0 &&
              maniRagdollSkeletonIdentity(NULL) == 0,
          "reads refused and NULL views");
}

// A single body: the root, its joint ignored even when out of range.
static void TestOneBody(void)
{
    maniRagdollSource source = Source();
    source.bodies = &s_bodies[1];
    source.bodyCount = 1;
    s_bodies[1].joint.twistMin = 9.0f;
    maniRagdoll ragdoll;
    maniRagdollBody b;
    CHECK(Build(&source).result == mani_success &&
              maniOpenRagdoll(s_file, s_size, &ragdoll).result == mani_success &&
              maniRagdollReadBody(&ragdoll, 0, &b) == mani_success && b.parent == MANI_NO_BONE &&
              b.twistMin == 0.0f && b.jointTranslation.x == 0.0f,
          "a lone body the root");
}

static bool Refused(const maniRagdollSource* source, maniBuildReason reason, uint32_t item,
                    uint32_t other)
{
    maniBuildResult r = maniBuildRagdoll(s_builder, source);
    if (r.result != mani_errorInvalid || r.reason != reason || r.item != item || r.other != other)
    {
        printf("  refused %d for %d at %u, %u; want %d at %u, %u\n", (int)r.result, (int)r.reason,
               r.item, r.other, (int)reason, item, other);
    }
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item &&
           r.other == other;
}

static void TestBodyRefusals(void)
{
    maniRagdollSource s = Source();
    s.bodyCount = 0;
    CHECK(Refused(&s, maniBuildReasonCount, 0, 0), "no bodies");
    s.bodyCount = kBones + 1;
    CHECK(Refused(&s, maniBuildReasonCount, 0, 0), "more bodies than bones");
    s = Source();
    s_bodies[3].bone = kBones;
    CHECK(Refused(&s, maniBuildReasonRagdollBody, 3, 0), "a bone out of range");
    s = Source();
    s_bodies[4].bone = s_bone[kLowerArm];
    CHECK(Refused(&s, maniBuildReasonRagdollBody, 4, 1), "a bone taken twice");
    s = Source();
    s_bodies[4].bone = s_bone[kProp];
    CHECK(Refused(&s, maniBuildReasonRagdollBody, 4, 2), "a second root");
    s = Source();
    s_bodies[2].shape = 3;
    CHECK(Refused(&s, maniBuildReasonRagdollBody, 2, 0), "an unknown shape");
    const struct
    {
        uint32_t body;
        int field;
        float value;
    } kSizes[] = {
        {0, 0, 0.0f},  {0, 0, INFINITY}, {0, 1, 0.1f},     {0, 2, 0.1f}, {1, 1, 0.0f},
        {1, 2, 0.01f}, {2, 2, -0.1f},    {2, 1, NAN},      {1, 3, 0.0f}, {1, 3, -1.0f},
        {1, 3, NAN},   {3, 4, NAN},      {3, 5, INFINITY}, {3, 6, 0.9f}, {3, 7, NAN},
    };
    for (size_t k = 0; k < sizeof kSizes / sizeof kSizes[0]; ++k)
    {
        s = Source();
        maniRagdollBodySource* b = &s_bodies[kSizes[k].body];
        float* fields[8] = {&b->size.x,
                            &b->size.y,
                            &b->size.z,
                            &b->mass,
                            &b->frameTranslation.x,
                            &b->frameTranslation.z,
                            &b->frameRotation.w,
                            &b->frameRotation.y};
        *fields[kSizes[k].field] = kSizes[k].value;
        CHECK(Refused(&s, maniBuildReasonRagdollBody, kSizes[k].body, 0),
              "a size, mass or frame refused");
    }
    s = Source();
    s_bodies[1].name = "\xFF";
    s_bodies[1].nameLength = 1;
    CHECK(Refused(&s, maniBuildReasonName, 1, 0), "a name not UTF-8");
    s_bodies[1].nameLength = (size_t)MANI_MAX_NAME_BYTES + 1;
    maniBuildResult longName = maniBuildRagdoll(s_builder, &s);
    CHECK(longName.result == mani_errorLimit && longName.reason == maniBuildReasonName &&
              longName.item == 1 && longName.other == 0,
          "a name past MANI_MAX_NAME_BYTES");
    s = Source();
    s_bodies[1].name = NULL;
    CHECK(Refused(&s, maniBuildReasonName, 1, 0), "a name missing");
    s = Source();
    s_bodies[1].name = NULL;
    s_bodies[1].nameLength = 0;
    CHECK(Build(&s).result == mani_success, "a body without a name");
}

static void TestJointRefusals(void)
{
    // Each field's bad values, the float just past pi at each limit's
    // bound and NaN in each limit among them.
    const struct
    {
        int field;
        float value;
    } kJoints[] = {
        {0, NAN},         {1, INFINITY}, {2, 0.9f},         {3, NAN},         {4, -3.2f},
        {4, 0.8f},        {5, 3.2f},     {6, -0.01f},       {6, 3.2f},        {7, -0.01f},
        {7, 3.2f},        {8, -1.0f},    {8, INFINITY},     {9, -1.0f},       {10, -1.0f},
        {10, INFINITY},   {9, NAN},      {4, -3.14159298f}, {5, 3.14159298f}, {6, 3.14159298f},
        {7, 3.14159298f}, {4, NAN},      {5, NAN},          {6, NAN},         {7, NAN},
    };
    for (size_t k = 0; k < sizeof kJoints / sizeof kJoints[0]; ++k)
    {
        maniRagdollSource s = Source();
        maniRagdollJointSource* j = &s_bodies[4].joint;
        float* fields[11] = {&j->translation.x, &j->translation.y, &j->rotation.w,
                             &j->rotation.x,    &j->twistMin,      &j->twistMax,
                             &j->swingY,        &j->swingZ,        &j->driveStiffness,
                             &j->driveDamping,  &j->driveMaxTorque};
        *fields[kJoints[k].field] = kJoints[k].value;
        CHECK(Refused(&s, maniBuildReasonRagdollJoint, 4, 0), "a joint refused");
    }
    maniRagdollSource s = Source();
    s_bodies[4].joint.twistMin = -3.14159265f;
    s_bodies[4].joint.twistMax = 3.14159265f;
    s_bodies[4].joint.swingY = 3.14159265f;
    s_bodies[4].joint.swingZ = 0.0f;
    s_bodies[4].joint.driveStiffness = 0.0f;
    s_bodies[4].joint.driveDamping = 0.0f;
    s_bodies[4].joint.driveMaxTorque = 0.0f;
    s_bodies[1].joint.twistMin = 0.3f;
    s_bodies[1].joint.twistMax = 0.3f;
    CHECK(Build(&s).result == mani_success, "limits at their ends, a fixed twist, no drive");
}

static void TestAxesAndMisuse(void)
{
    const maniVec3 kAxes[][2] = {
        {{0.0f, 0.0f, 0.9f}, {0.0f, 1.0f, 0.0f}},     {{0.0f, 0.0f, 1.0f}, {0.0f, 1.1f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}, {0.0f, 0.9998f, 0.02f}}, {{NAN, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}, {0.0f, INFINITY, 0.0f}},
    };
    for (size_t k = 0; k < sizeof kAxes / sizeof kAxes[0]; ++k)
    {
        maniRagdollSource s = Source();
        s.facing = kAxes[k][0];
        s.up = kAxes[k][1];
        CHECK(Refused(&s, maniBuildReasonRagdollAxes, 0, 0), "axes refused");
    }
    maniRagdollSource s = Source();
    maniBuildResult r = maniBuildRagdoll(NULL, &s);
    CHECK(r.result == mani_errorInvalid &&
              maniBuildRagdoll(s_builder, NULL).result == mani_errorInvalid,
          "no builder, no source");
    s.skeleton = NULL;
    CHECK(maniBuildRagdoll(s_builder, &s).result == mani_errorInvalid, "no skeleton");
    s = Source();
    s.bodies = NULL;
    CHECK(maniBuildRagdoll(s_builder, &s).result == mani_errorInvalid, "no bodies given");
}

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindRagdoll, MANI_RAGDOLL_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

// The file with the word at offset set to value, its identity and hash
// made right: refused at want.
static bool RefusedAt(uint64_t offset, uint32_t value, uint64_t want)
{
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + offset, value);
    maniContainer c;
    CHECK(maniOpenContainer(s_file, s_size, maniKindRagdoll, MANI_RAGDOLL_FORMAT_VERSION, &c)
                  .result == mani_success,
          "the container");
    static const uint8_t kEmpty = 0;
    const uint8_t* data[MANI_RAGDOLL_SECTIONS];
    uint64_t sizes[MANI_RAGDOLL_SECTIONS];
    for (uint32_t t = 1; t <= MANI_RAGDOLL_SECTIONS; ++t)
    {
        uint64_t at = 0;
        data[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        if (maniContainerSection(&c, t, &data[t - 1], &sizes[t - 1], &at))
        {
            data[t - 1] = s_crafted + at;
        }
    }
    maniStoreU64(s_crafted + 40, maniRagdollComputeIdentity(data, sizes));
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               (uint64_t)s_size - MANI_CONTAINER_HEADER_SIZE));
    maniRagdoll ragdoll;
    maniFileResult r = maniOpenRagdoll(s_crafted, s_size, &ragdoll);
    if (r.result != mani_errorFormat || r.offset != want)
    {
        printf("  at %llu: refused %d at %llu, want %llu\n", (unsigned long long)offset,
               (int)r.result, (unsigned long long)r.offset, (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want &&
           r.tag == TestTagAt(s_crafted, s_size, want);
}

static uint32_t Bits(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniRagdoll ragdoll;
    return maniOpenRagdoll(bytes, size, &ragdoll);
}

static void TestOpen(void)
{
    maniRagdollSource source = Source();
    CHECK(Build(&source).result == mani_success, "built");
    uint64_t h = SectionAt(maniRagdollTagHeader);
    uint64_t b = SectionAt(maniRagdollTagBodies);
    uint64_t n = SectionAt(maniRagdollTagNames);
    // Body 0 the pelvis (the root), 1 the chest, 4 the lower arm.
    uint64_t r = b;
    uint64_t c = b + MANI_RAGDOLL_BODY_SIZE;
    uint64_t a = b + 4 * MANI_RAGDOLL_BODY_SIZE;
    const uint64_t kWords[][3] = {
        {h + 8, 0, h + 8},
        {h + 8, MANI_MAX_BONES + 1, h + 8},
        {h + 8, 5, a},
        {h + 8, 4, h + 12},
        {h + 16, 32, h + 16},
        {h + 12, 0, h + 12},
        {h + 12, 9, h + 12},
        {h + 12, 6, h + 12},
        {h + 16, 3, h + 16},
        {h + 20, Bits(0.5f), h + 20},
        {h + 40, Bits(0.04f), h + 32},
        {h + 36, Bits(0.9f), h + 32},
        {r, kBones, r},
        {c, 0, c},
        {r + 4, 0, r + 4},
        {c + 4, 1, c + 4},
        {c + 4, MANI_NO_BONE, c + 4},
        {c + 8, 3, c + 8},
        {c + 12, 0, c + 12},
        {a + 16, 100, a + 16},
        {n, 0xFFu, n},
        {c + 28, Bits(NAN), c + 28},
        {c + 44, Bits(0.5f), c + 32},
        {b + 2 * MANI_RAGDOLL_BODY_SIZE + 52, Bits(0.1f), b + 2 * MANI_RAGDOLL_BODY_SIZE + 52},
        {r + 48, 0, r + 48},
        {a + 52, Bits(0.0f), a + 52},
        {a + 56, Bits(0.01f), a + 56},
        {a + 60, 0, a + 60},
        {r + 64, Bits(0.1f), r + 64},
        {r + 88, 0, r + 88},
        {r + 116, 0, r + 116},
        {r + 120, Bits(-0.1f), r + 120},
        {r + 144, 1, r + 144},
        {a + 88, Bits(0.5f), a + 76},
        {a + 116, Bits(0.5f), a + 104},
        {a + 120, Bits(-3.2f), a + 120},
        {a + 120, Bits(0.8f), a + 120},
        {a + 124, Bits(3.2f), a + 124},
        {a + 128, Bits(-0.1f), a + 128},
        {a + 132, Bits(3.2f), a + 132},
        {a + 120, Bits(-3.14159298f), a + 120},
        {a + 124, Bits(3.14159298f), a + 124},
        {a + 128, Bits(3.14159298f), a + 128},
        {a + 132, Bits(3.14159298f), a + 132},
        {a + 120, Bits(NAN), a + 120},
        {a + 124, Bits(NAN), a + 124},
        {a + 128, Bits(NAN), a + 128},
        {a + 132, Bits(NAN), a + 132},
        {a + 136, Bits(-1.0f), a + 136},
        {a + 144, Bits(-1.0f), a + 144},
    };
    for (size_t k = 0; k < sizeof kWords / sizeof kWords[0]; ++k)
    {
        CHECK(RefusedAt(kWords[k][0], (uint32_t)kWords[k][1], kWords[k][2]),
              "a word refused at its offset");
    }
    maniRagdoll ragdoll;
    CHECK(maniOpenRagdoll(s_file + 1, s_size - 1, &ragdoll).result == mani_errorInvalid &&
              maniOpenRagdoll(s_file, s_size, NULL).result == mani_errorInvalid &&
              maniOpenRagdoll(NULL, s_size, &ragdoll).result == mani_errorInvalid,
          "misaligned, no view, no bytes");
    CHECK(TestOtherVersionsRefused(s_file, s_size, OpenFile), "another format version refused");
    memcpy(s_crafted, s_file, s_size);
    maniStoreU64(s_crafted + 40, 1);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               (uint64_t)s_size - MANI_CONTAINER_HEADER_SIZE));
    CHECK(maniOpenRagdoll(s_crafted, s_size, &ragdoll).offset == 40, "another identity");
}

static int WriteSeed(const char* path)
{
    maniRagdollSource source = Source();
    FILE* file = Build(&source).result == mani_success ? fopen(path, "wb") : NULL;
    bool ok = file != NULL && fwrite(s_file, 1, s_size, file) == s_size;
    ok = file != NULL && fclose(file) == 0 && ok;
    return ok ? 0 : 1;
}

// The same file whatever bytes pad the source and its bodies.
static void TestPaddingIgnored(void)
{
    uint8_t* built[2];
    size_t sizes[2];
    for (int k = 0; k < 2; ++k)
    {
        maniRagdollSource source = Source();
        maniRagdollBodySource bodies[5];
        memcpy(bodies, s_bodies, sizeof bodies);
        TEST_PAD(bodies, 5, kFieldsRagdollBodySource, kTestPadPatterns[k]);
        source.bodies = bodies;
        TEST_PAD(&source, 1, kFieldsRagdollSource, kTestPadPatterns[k]);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        CHECK(maniBuildRagdoll(builder, &source).result == mani_success, "builds padded");
        built[k] = TestCopyBuilt(builder, &sizes[k]);
        maniDestroyBuilder(builder);
    }
    CHECK(TestSameBuilt(built, sizes), "the same bytes under either padding");
}

int main(int argc, char** argv)
{
    OpenSkeleton();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        int r = WriteSeed(argv[2]);
        maniDestroyBuilder(s_builder);
        return r;
    }
    TestBuild();
    TestOneBody();
    TestBodyRefusals();
    TestJointRefusals();
    TestAxesAndMisuse();
    TestOpen();
    TestPaddingIgnored();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
