// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a ragdoll profile: the container, the header's
// counts and axes, every body record (bones within the skeleton's count
// and increasing, a parent earlier than its child and none for the root
// alone, shapes and their sizes, masses, frames finite with unit
// rotations, the root's joint empty, limits and drives in range), the
// names, and the identity. That each parent is its child's nearest
// ancestor with a body needs the skeleton: its identity, checked where
// the profile is used with a pose, stands for the builder's check.

#include "maul-anim/ragdoll.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "ragdoll_file.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint64_t maniRagdollComputeIdentity(const uint8_t* const data[MANI_RAGDOLL_SECTIONS],
                                    const uint64_t sizes[MANI_RAGDOLL_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_RAGDOLL_SECTIONS; ++i)
    {
        hash = maniHashBytes(hash, data[i], sizes[i]);
    }
    return hash;
}

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static const maniFileResult kOk = {mani_success, 0, 0};

#define PI_F 3.14159265358979f

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static maniVec3 LoadVec(const uint8_t* p)
{
    return (maniVec3){LoadF32(p), LoadF32(p + 4), LoadF32(p + 8)};
}

static maniQuat LoadQuat(const uint8_t* p)
{
    return (maniQuat){LoadF32(p), LoadF32(p + 4), LoadF32(p + 8), LoadF32(p + 12)};
}

// Within a thousandth of length 1, as the builder takes it.
static bool Unit(float lengthSquared)
{
    return lengthSquared >= 0.998f && lengthSquared <= 1.002f;
}

static bool UnitQuat(const uint8_t* p)
{
    maniQuat q = LoadQuat(p);
    return Unit(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

// The sections, located and sized.
typedef struct Sections
{
    const uint8_t* data[MANI_RAGDOLL_SECTIONS];
    uint64_t sizes[MANI_RAGDOLL_SECTIONS];
    uint64_t at[MANI_RAGDOLL_SECTIONS];
} Sections;

static maniFileResult Header(const Sections* s, maniRagdoll* out)
{
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    if (s->sizes[0] != MANI_RAGDOLL_HEADER_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE);
    }
    out->skeletonIdentity = maniLoadU64(h);
    out->boneCount = maniLoadU32(h + 8);
    out->bodyCount = maniLoadU32(h + 12);
    out->nameBytes = maniLoadU32(h + 16);
    out->facing = LoadVec(h + 20);
    out->up = LoadVec(h + 32);
    if (out->boneCount < 1 || out->boneCount > MANI_MAX_BONES)
    {
        return Fail(at + 8);
    }
    if (out->bodyCount < 1 || out->bodyCount > out->boneCount)
    {
        return Fail(at + 12);
    }
    maniVec3 f = out->facing;
    maniVec3 u = out->up;
    if (!Unit(f.x * f.x + f.y * f.y + f.z * f.z))
    {
        return Fail(at + 20);
    }
    if (!Unit(u.x * u.x + u.y * u.y + u.z * u.z) ||
        !(fabsf(f.x * u.x + f.y * u.y + f.z * u.z) <= 1e-3f))
    {
        return Fail(at + 32);
    }
    return kOk;
}

// A body's shape and its sizes: a sphere's and a capsule's unused sizes
// 0, the used ones positive.
static maniFileResult Shape(const uint8_t* e, uint64_t at)
{
    uint32_t shape = maniLoadU32(e + maniRagdollBodyShape);
    if (shape > maniRagdollBox)
    {
        return Fail(at + maniRagdollBodyShape);
    }
    uint32_t used = shape == maniRagdollSphere ? 1u : shape == maniRagdollCapsule ? 2u : 3u;
    for (uint32_t k = 0; k < 3; ++k)
    {
        float v = LoadF32(e + maniRagdollBodySize + 4 * k);
        if (k < used ? !(v > 0.0f) : v != 0.0f)
        {
            return Fail(at + maniRagdollBodySize + 4 * k);
        }
    }
    return LoadF32(e + maniRagdollBodyMass) > 0.0f ? kOk : Fail(at + maniRagdollBodyMass);
}

// The root's joint: both frames the identity, limits and drive 0.
static maniFileResult EmptyJoint(const uint8_t* e, uint64_t at)
{
    static const float kIdentity[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    for (uint32_t k = 0; k < 14; ++k)
    {
        if (LoadF32(e + maniRagdollBodyJoint + 4 * k) != kIdentity[k % 7])
        {
            return Fail(at + maniRagdollBodyJoint + 4 * k);
        }
    }
    for (uint32_t k = 0; k < 7; ++k)
    {
        if (maniLoadU32(e + maniRagdollBodyLimits + 4 * k) != 0)
        {
            return Fail(at + maniRagdollBodyLimits + 4 * k);
        }
    }
    return kOk;
}

// A child's joint: unit rotations, the twist range within [-pi, pi] and
// not empty, the swing half angles in [0, pi], the drive not negative.
static maniFileResult Joint(const uint8_t* e, uint64_t at)
{
    if (!UnitQuat(e + maniRagdollBodyJoint + 12))
    {
        return Fail(at + maniRagdollBodyJoint + 12);
    }
    if (!UnitQuat(e + maniRagdollBodyParentJoint + 12))
    {
        return Fail(at + maniRagdollBodyParentJoint + 12);
    }
    const uint8_t* l = e + maniRagdollBodyLimits;
    float low = LoadF32(l);
    float high = LoadF32(l + 4);
    if (!(low >= -PI_F && low <= high))
    {
        return Fail(at + maniRagdollBodyLimits);
    }
    if (!(high <= PI_F))
    {
        return Fail(at + maniRagdollBodyLimits + 4);
    }
    for (uint32_t k = 2; k < 4; ++k)
    {
        float swing = LoadF32(l + 4 * k);
        if (!(swing >= 0.0f && swing <= PI_F))
        {
            return Fail(at + maniRagdollBodyLimits + 4 * k);
        }
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (!(LoadF32(e + maniRagdollBodyDrive + 4 * k) >= 0.0f))
        {
            return Fail(at + maniRagdollBodyDrive + 4 * k);
        }
    }
    return kOk;
}

// Body i: its bone after the one before, its parent, shape, name, its
// floats finite, frame, joint; name the running name offset.
static maniFileResult Body(const Sections* s, const maniRagdoll* r, uint32_t i, uint32_t* name)
{
    const uint8_t* e = s->data[1] + (size_t)i * MANI_RAGDOLL_BODY_SIZE;
    uint64_t at = s->at[1] + (uint64_t)i * MANI_RAGDOLL_BODY_SIZE;
    uint32_t bone = maniLoadU32(e + maniRagdollBodyBone);
    uint32_t before = i > 0 ? maniLoadU32(e - MANI_RAGDOLL_BODY_SIZE + maniRagdollBodyBone) : 0;
    if (bone >= r->boneCount || (i > 0 && bone <= before))
    {
        return Fail(at);
    }
    uint32_t parent = maniLoadU32(e + maniRagdollBodyParent);
    if (i == 0 ? parent != MANI_NO_BONE : parent >= i)
    {
        return Fail(at + maniRagdollBodyParent);
    }
    uint32_t length = maniLoadU32(e + maniRagdollBodyNameLength);
    if (maniLoadU32(e + maniRagdollBodyName) != *name || length > r->nameBytes - *name)
    {
        return Fail(maniLoadU32(e + maniRagdollBodyName) != *name ? at + maniRagdollBodyName
                                                                  : at + maniRagdollBodyNameLength);
    }
    if (!maniValidUtf8(s->data[2] + *name, length))
    {
        return Fail(s->at[2] + *name);
    }
    *name += length;
    for (uint32_t k = maniRagdollBodyFrame; k < MANI_RAGDOLL_BODY_SIZE; k += 4)
    {
        if (!(fabsf(LoadF32(e + k)) < HUGE_VALF))
        {
            return Fail(at + k);
        }
    }
    if (!UnitQuat(e + maniRagdollBodyFrame + 12))
    {
        return Fail(at + maniRagdollBodyFrame + 12);
    }
    maniFileResult f = Shape(e, at);
    f = f.result == mani_success ? (i == 0 ? EmptyJoint(e, at) : Joint(e, at)) : f;
    return f;
}

static maniFileResult Records(const Sections* s, const maniRagdoll* r)
{
    if (s->sizes[1] != (uint64_t)r->bodyCount * MANI_RAGDOLL_BODY_SIZE)
    {
        return Fail(s->at[0] + 12);
    }
    if (s->sizes[2] != ((uint64_t)r->nameBytes + 3) / 4 * 4)
    {
        return Fail(s->at[0] + 16);
    }
    uint32_t name = 0;
    maniFileResult f = kOk;
    for (uint32_t i = 0; f.result == mani_success && i < r->bodyCount; ++i)
    {
        f = Body(s, r, i, &name);
    }
    if (f.result == mani_success && name != r->nameBytes)
    {
        return Fail(s->at[0] + 16);
    }
    return f;
}

static MANI_NOINLINE maniFileResult OpenRagdollWork(const void* bytes, size_t size, void* out)
{
    maniRagdoll* ragdollOut = out;
    if (ragdollOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindRagdoll, MANI_RAGDOLL_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    static const uint8_t kEmpty = 0;
    Sections s;
    for (uint32_t t = 1; t <= MANI_RAGDOLL_SECTIONS; ++t)
    {
        s.data[t - 1] = &kEmpty;
        s.sizes[t - 1] = 0;
        s.at[t - 1] = 0;
        (void)maniContainerSection(&c, t, &s.data[t - 1], &s.sizes[t - 1], &s.at[t - 1]);
    }
    maniRagdoll ragdoll;
    memset(&ragdoll, 0, sizeof ragdoll);
    r = Header(&s, &ragdoll);
    r = r.result == mani_success ? Records(&s, &ragdoll) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniRagdollComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    ragdoll.bodies = s.data[1];
    ragdoll.names = s.data[2];
    ragdoll.identity = c.identity;
    *ragdollOut = ragdoll;
    return kOk;
}

maniFileResult maniOpenRagdoll(const void* bytes, size_t size, maniRagdoll* ragdollOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = maniContainerOpenTagged(bytes, size, ragdollOut, OpenRagdollWork);
    maniFloatEnvLeave(env);
    return result;
}

uint64_t maniRagdoll_GetIdentity(const maniRagdoll* ragdoll)
{
    return ragdoll != NULL ? ragdoll->identity : 0;
}

uint64_t maniRagdoll_GetSkeletonIdentity(const maniRagdoll* ragdoll)
{
    return ragdoll != NULL ? ragdoll->skeletonIdentity : 0;
}

uint32_t maniRagdoll_GetBodyCount(const maniRagdoll* ragdoll)
{
    return ragdoll != NULL ? ragdoll->bodyCount : 0;
}

maniResult maniRagdoll_GetBody(const maniRagdoll* ragdoll, uint32_t body, maniRagdollBody* bodyOut)
{
    if (ragdoll == NULL || bodyOut == NULL || body >= ragdoll->bodyCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e = ragdoll->bodies + (size_t)body * MANI_RAGDOLL_BODY_SIZE;
    const uint8_t* l = e + maniRagdollBodyLimits;
    const uint8_t* d = e + maniRagdollBodyDrive;
    *bodyOut = (maniRagdollBody){
        .name = (const char*)(ragdoll->names + maniLoadU32(e + maniRagdollBodyName)),
        .nameLength = maniLoadU32(e + maniRagdollBodyNameLength),
        .bone = maniLoadU32(e + maniRagdollBodyBone),
        .parent = maniLoadU32(e + maniRagdollBodyParent),
        .shape = maniLoadU32(e + maniRagdollBodyShape),
        .size = LoadVec(e + maniRagdollBodySize),
        .mass = LoadF32(e + maniRagdollBodyMass),
        .frameTranslation = LoadVec(e + maniRagdollBodyFrame),
        .frameRotation = LoadQuat(e + maniRagdollBodyFrame + 12),
        .jointTranslation = LoadVec(e + maniRagdollBodyJoint),
        .jointRotation = LoadQuat(e + maniRagdollBodyJoint + 12),
        .parentJointTranslation = LoadVec(e + maniRagdollBodyParentJoint),
        .parentJointRotation = LoadQuat(e + maniRagdollBodyParentJoint + 12),
        .twistMin = LoadF32(l),
        .twistMax = LoadF32(l + 4),
        .swingY = LoadF32(l + 8),
        .swingZ = LoadF32(l + 12),
        .driveStiffness = LoadF32(d),
        .driveDamping = LoadF32(d + 4),
        .driveMaxTorque = LoadF32(d + 8)};
    return mani_success;
}
