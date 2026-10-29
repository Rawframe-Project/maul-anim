// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a ragdoll profile: the source checked (each body on
// its own bone, one root, shapes, sizes, masses, frames, names, joints'
// frames, limits and drives, the root's axes), the bodies ordered by
// bone, each body's parent the body of its bone's nearest ancestor with
// one. A joint's frame in the parent body is where the child's frame of
// it lies in the rest pose: with each bone's rigid model frame (the rest
// pose's model matrix, its scale left out), the body's frame B = M F, the
// joint's J = B C in the model, and in the parent P^-1 J.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "decompose.h"
#include "fpenv.h"
#include "ragdoll_file.h"
#include "text.h"
#include "vecmath.h"

#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorInvalid, reason, item, other);
}

#define PI_F 3.14159265358979f

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

static bool FiniteVec(maniVec3 v)
{
    return Finite(v.x) && Finite(v.y) && Finite(v.z);
}

// Within a thousandth of length 1, as exporters' rounding leaves it.
static bool Unit(float lengthSquared)
{
    return lengthSquared >= 0.998f && lengthSquared <= 1.002f;
}

static bool UnitQuat(maniQuat q)
{
    return Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w) && Unit(maniQuatDot(q, q));
}

// A rigid frame: a translation and a unit rotation.
typedef struct Rigid
{
    maniVec3 t;
    maniQuat r;
} Rigid;

static Rigid Compose(Rigid a, Rigid b)
{
    return (Rigid){maniVec3Add(a.t, maniQuatRotate(a.r, b.t)),
                   maniQuatNormalize(maniQuatMul(a.r, b.r))};
}

static Rigid Inverse(Rigid a)
{
    maniQuat r = maniQuatConjugate(a.r);
    return (Rigid){maniVec3Scale(maniQuatRotate(r, a.t), -1.0f), r};
}

// Per bone: its rest model matrix and rigid frame, its body's index in
// the source plus 1 (0 for none) and its index in the file.
typedef struct Bone
{
    maniAffine affine;
    Rigid model;
    uint32_t bodyOf;
    uint32_t indexOf;
} Bone;

typedef struct Work
{
    Bone* bones;
    uint64_t nameBytes;
} Work;

// The body's shape, its sizes (a sphere's and a capsule's unused ones 0)
// and mass.
static bool ShapeValid(const maniRagdollBodySource* b)
{
    if (b->shape > maniRagdollBox)
    {
        return false;
    }
    uint32_t used = b->shape == maniRagdollSphere ? 1u : b->shape == maniRagdollCapsule ? 2u : 3u;
    const float size[3] = {b->size.x, b->size.y, b->size.z};
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (k < used ? !(size[k] > 0.0f && Finite(size[k])) : size[k] != 0.0f)
        {
            return false;
        }
    }
    return b->mass > 0.0f && Finite(b->mass) && FiniteVec(b->frameTranslation) &&
           UnitQuat(b->frameRotation);
}

static bool JointValid(const maniRagdollJointSource* j)
{
    bool drive = j->driveStiffness >= 0.0f && Finite(j->driveStiffness) &&
                 j->driveDamping >= 0.0f && Finite(j->driveDamping) && j->driveMaxTorque >= 0.0f &&
                 Finite(j->driveMaxTorque);
    return FiniteVec(j->translation) && UnitQuat(j->rotation) && j->twistMin >= -PI_F &&
           j->twistMin <= j->twistMax && j->twistMax <= PI_F && j->swingY >= 0.0f &&
           j->swingY <= PI_F && j->swingZ >= 0.0f && j->swingZ <= PI_F && drive;
}

static maniBuildResult CheckBody(const maniRagdollSource* s, uint32_t i, Work* w)
{
    const maniRagdollBodySource* b = &s->bodies[i];
    uint32_t bones = maniSkeleton_GetBoneCount(s->skeleton);
    if (b->bone >= bones)
    {
        return Refuse(maniBuildReasonRagdollBody, i, 0);
    }
    if (w->bones[b->bone].bodyOf != 0)
    {
        return Refuse(maniBuildReasonRagdollBody, i, w->bones[b->bone].bodyOf - 1);
    }
    w->bones[b->bone].bodyOf = i + 1;
    if (!ShapeValid(b))
    {
        return Refuse(maniBuildReasonRagdollBody, i, 0);
    }
    if (maniNamePastLimit(b->nameLength))
    {
        return Result(mani_errorLimit, maniBuildReasonName, i, 0);
    }
    if (b->nameLength > 0 &&
        (b->name == NULL || !maniValidUtf8((const uint8_t*)b->name, b->nameLength)))
    {
        return Refuse(maniBuildReasonName, i, 0);
    }
    w->nameBytes += b->nameLength;
    return kOk;
}

// The nearest ancestor of bone with a body, or MANI_NO_BONE.
static uint32_t AncestorBody(const maniSkeleton* skeleton, const Work* w, uint32_t bone)
{
    for (uint32_t a = maniSkeleton_GetParent(skeleton, bone); a != MANI_NO_BONE;
         a = maniSkeleton_GetParent(skeleton, a))
    {
        if (w->bones[a].bodyOf != 0)
        {
            return a;
        }
    }
    return MANI_NO_BONE;
}

// One root; every other body's joint valid.
static maniBuildResult CheckTree(const maniRagdollSource* s, const Work* w)
{
    uint32_t root = MANI_NO_BONE;
    for (uint32_t i = 0; i < s->bodyCount; ++i)
    {
        bool isRoot = AncestorBody(s->skeleton, w, s->bodies[i].bone) == MANI_NO_BONE;
        if (isRoot && root != MANI_NO_BONE)
        {
            return Refuse(maniBuildReasonRagdollBody, i, root);
        }
        root = isRoot ? i : root;
        if (!isRoot && !JointValid(&s->bodies[i].joint))
        {
            return Refuse(maniBuildReasonRagdollJoint, i, 0);
        }
    }
    maniVec3 f = s->facing;
    maniVec3 u = s->up;
    bool axes = FiniteVec(f) && FiniteVec(u) && Unit(maniVec3Dot(f, f)) &&
                Unit(maniVec3Dot(u, u)) && fabsf(maniVec3Dot(f, u)) <= 1e-3f;
    return axes ? kOk : Refuse(maniBuildReasonRagdollAxes, 0, 0);
}

static maniBuildResult Check(const maniRagdollSource* s, Work* w)
{
    maniBuildResult r = kOk;
    for (uint32_t i = 0; r.result == mani_success && i < s->bodyCount; ++i)
    {
        r = CheckBody(s, i, w);
    }
    r = r.result == mani_success ? CheckTree(s, w) : r;
    if (r.result == mani_success && w->nameBytes > UINT32_MAX - 3)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    return r;
}

// Each bone's rigid rest model frame: the rest pose's model matrices,
// parents first, split with their scales dropped.
static void Models(const maniSkeleton* skeleton, Bone* bones)
{
    uint32_t count = maniSkeleton_GetBoneCount(skeleton);
    for (uint32_t b = 0; b < count; ++b)
    {
        maniTransform rest = maniSkeleton_GetRest(skeleton, b);
        maniAffine local = maniAffineFromTrs(rest.translation, rest.rotation, rest.scale);
        uint32_t parent = maniSkeleton_GetParent(skeleton, b);
        bones[b].affine =
            parent != MANI_NO_BONE ? maniAffineMul(&bones[parent].affine, &local) : local;
        maniTransform split = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
        (void)maniDecompose(&bones[b].affine, &split);
        bones[b].model = (Rigid){split.translation, maniQuatNormalize(split.rotation)};
    }
}

static void StoreF32(uint8_t* p, float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    maniStoreU32(p, bits);
}

static void StoreVec(uint8_t* p, maniVec3 v)
{
    StoreF32(p, v.x);
    StoreF32(p + 4, v.y);
    StoreF32(p + 8, v.z);
}

static void StoreRigid(uint8_t* p, Rigid a)
{
    StoreVec(p, a.t);
    StoreF32(p + 12, a.r.x);
    StoreF32(p + 16, a.r.y);
    StoreF32(p + 20, a.r.z);
    StoreF32(p + 24, a.r.w);
}

static Rigid BodyFrame(const maniRagdollBodySource* b, const Work* w)
{
    return Compose(w->bones[b->bone].model,
                   (Rigid){b->frameTranslation, maniQuatNormalize(b->frameRotation)});
}

// A child's joint: its frame in the body and in the parent body, limits
// and drive.
static void LayJoint(const maniRagdollBodySource* b, const maniRagdollBodySource* parent,
                     const Work* w, uint8_t* e)
{
    const maniRagdollJointSource* j = &b->joint;
    Rigid joint = {j->translation, maniQuatNormalize(j->rotation)};
    Rigid model = Compose(BodyFrame(b, w), joint);
    StoreRigid(e + maniRagdollBodyJoint, joint);
    StoreRigid(e + maniRagdollBodyParentJoint, Compose(Inverse(BodyFrame(parent, w)), model));
    const float values[7] = {j->twistMin,       j->twistMax,     j->swingY,        j->swingZ,
                             j->driveStiffness, j->driveDamping, j->driveMaxTorque};
    for (uint32_t k = 0; k < 7; ++k)
    {
        StoreF32(e + maniRagdollBodyLimits + 4 * k, values[k]);
    }
}

// The bodies by bone, each one's index in the file kept in indexOf.
static void LayBodies(const maniRagdollSource* s, Work* w, uint8_t* bodies, uint8_t* names)
{
    static const Rigid kIdentity = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    uint32_t bones = maniSkeleton_GetBoneCount(s->skeleton);
    uint32_t index = 0;
    uint32_t name = 0;
    for (uint32_t bone = 0; bone < bones; ++bone)
    {
        if (w->bones[bone].bodyOf == 0)
        {
            continue;
        }
        const maniRagdollBodySource* b = &s->bodies[w->bones[bone].bodyOf - 1];
        uint8_t* e = bodies + (size_t)index * MANI_RAGDOLL_BODY_SIZE;
        uint32_t ancestor = AncestorBody(s->skeleton, w, bone);
        // Ancestors come first: their indexOf is set.
        uint32_t parent = ancestor != MANI_NO_BONE ? w->bones[ancestor].indexOf : MANI_NO_BONE;
        maniStoreU32(e + maniRagdollBodyBone, bone);
        maniStoreU32(e + maniRagdollBodyParent, parent);
        maniStoreU32(e + maniRagdollBodyShape, b->shape);
        maniStoreU32(e + maniRagdollBodyName, name);
        // The check took only names within MANI_MAX_NAME_BYTES.
        maniStoreU32(e + maniRagdollBodyNameLength, (uint32_t)b->nameLength);
        StoreRigid(e + maniRagdollBodyFrame,
                   (Rigid){b->frameTranslation, maniQuatNormalize(b->frameRotation)});
        StoreVec(e + maniRagdollBodySize, b->size);
        StoreF32(e + maniRagdollBodyMass, b->mass);
        if (ancestor != MANI_NO_BONE)
        {
            LayJoint(b, &s->bodies[w->bones[ancestor].bodyOf - 1], w, e);
        }
        else
        {
            StoreRigid(e + maniRagdollBodyJoint, kIdentity);
            StoreRigid(e + maniRagdollBodyParentJoint, kIdentity);
        }
        if (b->nameLength > 0)
        {
            memcpy(names + name, b->name, b->nameLength);
        }
        name += b->nameLength;
        w->bones[bone].indexOf = index;
        ++index;
    }
}

static void LayHeader(const maniRagdollSource* s, const Work* w, uint8_t* out)
{
    maniStoreU64(out, maniSkeleton_GetIdentity(s->skeleton));
    maniStoreU32(out + 8, maniSkeleton_GetBoneCount(s->skeleton));
    maniStoreU32(out + 12, s->bodyCount);
    maniStoreU32(out + 16, (uint32_t)w->nameBytes);
    StoreVec(out + 20, s->facing);
    StoreVec(out + 32, s->up);
}

static maniBuildResult Write(maniBuilder* builder, const maniRagdollSource* s, Work* w)
{
    const uint64_t sizes[MANI_RAGDOLL_SECTIONS] = {MANI_RAGDOLL_HEADER_SIZE,
                                                   (uint64_t)s->bodyCount * MANI_RAGDOLL_BODY_SIZE,
                                                   (w->nameBytes + 3) / 4 * 4};
    size_t own = (size_t)(sizes[0] + sizes[1] + sizes[2]);
    uint8_t* staging = maniAllocate(&builder->allocator, own, 1, 8);
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(staging, 0, own);
    uint8_t* data[MANI_RAGDOLL_SECTIONS] = {staging, staging + sizes[0],
                                            staging + sizes[0] + sizes[1]};
    const uint8_t* views[MANI_RAGDOLL_SECTIONS] = {data[0], data[1], data[2]};
    maniSectionSource sections[MANI_RAGDOLL_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_RAGDOLL_SECTIONS; ++i)
    {
        sections[count] = (maniSectionSource){i + 1, data[i], sizes[i]};
        count += sizes[i] > 0 ? 1u : 0u;
    }
    LayHeader(s, w, data[0]);
    LayBodies(s, w, data[1], data[2]);
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindRagdoll, MANI_RAGDOLL_FORMAT_VERSION,
                           maniRagdollComputeIdentity(views, sizes), sections, count);
        maniBuilderKeep(builder, maniKindRagdoll, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, own, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

static MANI_NOINLINE maniBuildResult BuildRagdollWork(maniBuilder* builder,
                                                      const maniRagdollSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (source == NULL || source->skeleton == NULL || source->bodies == NULL)
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    uint32_t bones = maniSkeleton_GetBoneCount(source->skeleton);
    if (source->bodyCount < 1 || source->bodyCount > bones)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    Bone* work = maniAllocate(&builder->allocator, bones, sizeof(Bone), 8);
    if (work == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(work, 0, (size_t)bones * sizeof(Bone));
    Work w = {work, 0};
    maniBuildResult r = Check(source, &w);
    if (r.result == mani_success)
    {
        Models(source->skeleton, work);
        r = Write(builder, source, &w);
    }
    maniRelease(&builder->allocator, work, bones, sizeof(Bone), 8);
    return r;
}

maniBuildResult maniBuildRagdoll(maniBuilder* builder, const maniRagdollSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildRagdollWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
