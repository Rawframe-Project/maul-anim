// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a spring set: the source checked as
// VRMC_springBone 1.0 and its extended collider and limit extensions
// require (bones, shapes, chains each joint above the next, no joint in
// two chains, centers on or above their chain's first joint and off
// every other chain, every value in its range), each joint's drag,
// stiffness and gravity converted from a frame of the reference rate to
// a step of the step rate: with h the step's seconds and
// f0 the reference rate, the drag dh = 1 - (1 - d)^(h f0), stiffness and
// gravity times h dh / d (h² f0 when d = 0). Each spring lists the
// colliders of its groups in order without repeats.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "scalar.h"
#include "spring_file.h"
#include "spring_frame.h"
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

#define PI_F 3.14159265358979f

// What the build works out: per bone and per collider the spring that
// took it (index + 1, 0 for none), and the counts the file holds.
typedef struct Work
{
    uint32_t* boneOwner;
    uint32_t* colliderMark;
    uint64_t joints;
    uint64_t links;
    uint64_t nameBytes;
} Work;

static maniBuildResult CheckCollider(const maniSpringSetSource* s, uint32_t i)
{
    const maniColliderSource* c = &s->colliders[i];
    bool plane = c->shape == maniColliderPlane;
    bool ok = c->bone < s->skeleton->boneCount && (uint32_t)c->shape <= maniColliderPlane &&
              c->inside <= 1 && !(plane && c->inside != 0) && Finite(c->radius) &&
              c->radius >= 0.0f && FiniteVec(c->offset) && FiniteVec(c->tail) &&
              (!plane || Unit(maniVec3Dot(c->tail, c->tail)));
    return ok ? kOk : Refuse(maniBuildReasonCollider, i, 0);
}

static maniBuildResult CheckGroups(const maniSpringSetSource* s)
{
    for (uint32_t g = 0; g < s->colliderGroupCount; ++g)
    {
        const maniColliderGroupSource* group = &s->colliderGroups[g];
        for (uint32_t k = 0; k < group->colliderCount; ++k)
        {
            if (group->colliders == NULL || group->colliders[k] >= s->colliderCount)
            {
                return Refuse(maniBuildReasonColliderGroup, g, k);
            }
        }
    }
    return kOk;
}

// Whether ancestor is a strict ancestor of bone.
static bool Above(const maniSkeleton* skeleton, uint32_t ancestor, uint32_t bone)
{
    uint32_t b = maniSkeleton_GetParent(skeleton, bone);
    for (uint32_t depth = 0; b != MANI_NO_BONE && depth < skeleton->boneCount; ++depth)
    {
        if (b == ancestor)
        {
            return true;
        }
        b = maniSkeleton_GetParent(skeleton, b);
    }
    return false;
}

// A joint's values in their ranges; its limit only before the last.
static bool JointValid(const maniSpringJointSource* j, bool last)
{
    bool values = Finite(j->hitRadius) && j->hitRadius >= 0.0f && Finite(j->stiffness) &&
                  j->stiffness >= 0.0f && Finite(j->gravityPower) && j->gravityPower >= 0.0f &&
                  FiniteVec(j->gravityDirection) && j->drag >= 0.0f && j->drag <= 1.0f;
    if (!values || last || j->limit == maniSpringLimitNone)
    {
        return values;
    }
    maniQuat q = j->limitRotation;
    bool yaw = j->limit != maniSpringLimitSpherical || (Finite(j->limitYaw) && j->limitYaw >= 0.0f);
    return (uint32_t)j->limit <= maniSpringLimitSpherical && Finite(j->limitAngle) &&
           j->limitAngle >= 0.0f && yaw && Finite(q.x) && Finite(q.y) && Finite(q.z) &&
           Finite(q.w) && Unit(maniQuatDot(q, q));
}

// Spring i's joints: bones in range, each above the next, none taken by
// an earlier spring, their values.
static maniBuildResult CheckJoints(const maniSpringSetSource* s, uint32_t i, Work* w)
{
    const maniSpringSource* spring = &s->springs[i];
    const maniSkeleton* skeleton = s->skeleton;
    for (uint32_t k = 0; k < spring->jointCount; ++k)
    {
        uint32_t bone = spring->joints[k].bone;
        if (bone >= skeleton->boneCount ||
            (k > 0 && !Above(skeleton, spring->joints[k - 1].bone, bone)))
        {
            return Refuse(maniBuildReasonSpring, i, k);
        }
        if (w->boneOwner[bone] != 0)
        {
            return Refuse(maniBuildReasonRepeatedJoint, i, k);
        }
        w->boneOwner[bone] = i + 1;
        if (!JointValid(&spring->joints[k], k + 1 == spring->jointCount))
        {
            return Refuse(maniBuildReasonSpringJoint, i, k);
        }
    }
    return kOk;
}

static maniBuildResult CheckSpring(const maniSpringSetSource* s, uint32_t i, Work* w)
{
    const maniSpringSource* spring = &s->springs[i];
    if (spring->jointCount < 2 || spring->joints == NULL)
    {
        return Refuse(maniBuildReasonSpring, i, 0);
    }
    if (maniNamePastLimit(spring->nameLength))
    {
        return Result(mani_errorLimit, maniBuildReasonSpringList | maniBuildReasonName, i, 0);
    }
    if (spring->nameLength > 0 &&
        (spring->name == NULL || !maniValidUtf8((const uint8_t*)spring->name, spring->nameLength)))
    {
        return Refuse(maniBuildReasonSpringList | maniBuildReasonName, i, 0);
    }
    for (uint32_t g = 0; g < spring->colliderGroupCount; ++g)
    {
        if (spring->colliderGroups == NULL || spring->colliderGroups[g] >= s->colliderGroupCount)
        {
            return Refuse(maniBuildReasonSpring, i, spring->jointCount + g);
        }
    }
    maniBuildResult r = CheckJoints(s, i, w);
    w->joints += spring->jointCount - 1;
    w->nameBytes += spring->nameLength;
    return r;
}

// Spring i's center: none, its first joint or above it, and neither on
// nor below another spring's joint.
static maniBuildResult CheckCenter(const maniSpringSetSource* s, uint32_t i, const Work* w)
{
    const maniSpringSource* spring = &s->springs[i];
    uint32_t c = spring->center;
    if (c == MANI_NO_BONE)
    {
        return kOk;
    }
    uint32_t first = spring->joints[0].bone;
    bool placed = c < s->skeleton->boneCount && (c == first || Above(s->skeleton, c, first));
    for (uint32_t b = c, depth = 0; placed && b != MANI_NO_BONE && depth <= s->skeleton->boneCount;
         ++depth)
    {
        placed = w->boneOwner[b] == 0 || w->boneOwner[b] == i + 1;
        b = maniSkeleton_GetParent(s->skeleton, b);
    }
    return placed ? kOk : Refuse(maniBuildReasonSpring, i, spring->jointCount);
}

// Spring i's colliders, its groups' in order without repeats: counted,
// and written to links when given.
static uint64_t Links(const maniSpringSetSource* s, uint32_t i, Work* w, uint8_t* links)
{
    const maniSpringSource* spring = &s->springs[i];
    uint64_t count = 0;
    for (uint32_t g = 0; g < spring->colliderGroupCount; ++g)
    {
        const maniColliderGroupSource* group = &s->colliderGroups[spring->colliderGroups[g]];
        for (uint32_t k = 0; k < group->colliderCount; ++k)
        {
            uint32_t c = group->colliders[k];
            if (w->colliderMark[c] == i + 1)
            {
                continue;
            }
            w->colliderMark[c] = i + 1;
            if (links != NULL)
            {
                maniStoreU32(links + 4 * count, c);
            }
            ++count;
        }
    }
    return count;
}

static maniBuildResult Check(const maniSpringSetSource* s, Work* w)
{
    float reference = s->referenceRate == 0.0f ? 60.0f : s->referenceRate;
    float step = s->stepRate == 0.0f ? reference : s->stepRate;
    // Item 0 the reference rate, 1 the step rate.
    if (reference > MANI_MAX_SPRING_RATE)
    {
        return Result(mani_errorLimit, maniBuildReasonOption, 0, 0);
    }
    if (!(reference > 0.0f))
    {
        return Refuse(maniBuildReasonOption, 0, 0);
    }
    if (step > MANI_MAX_SPRING_RATE)
    {
        return Result(mani_errorLimit, maniBuildReasonOption, 1, 0);
    }
    if (!(step > 0.0f))
    {
        return Refuse(maniBuildReasonOption, 1, 0);
    }
    maniBuildResult r = kOk;
    for (uint32_t i = 0; r.result == mani_success && i < s->colliderCount; ++i)
    {
        r = CheckCollider(s, i);
    }
    r = r.result == mani_success ? CheckGroups(s) : r;
    for (uint32_t i = 0; r.result == mani_success && i < s->springCount; ++i)
    {
        r = CheckSpring(s, i, w);
    }
    for (uint32_t i = 0; r.result == mani_success && i < s->springCount; ++i)
    {
        r = CheckCenter(s, i, w);
        w->links += Links(s, i, w, NULL);
    }
    bool fits = w->joints <= UINT32_MAX && w->links <= UINT32_MAX && w->nameBytes <= UINT32_MAX;
    return r.result != mani_success || fits ? r : Refuse(maniBuildReasonCount, 0, 0);
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

static void LayColliders(const maniSpringSetSource* s, uint8_t* out)
{
    for (uint32_t i = 0; i < s->colliderCount; ++i)
    {
        const maniColliderSource* c = &s->colliders[i];
        uint8_t* e = out + (size_t)i * MANI_SPRING_COLLIDER_SIZE;
        maniStoreU32(e + maniSpringColliderBone, c->bone);
        maniStoreU32(e + maniSpringColliderShape, c->shape);
        maniStoreU32(e + maniSpringColliderInside, c->inside);
        StoreVec(e + maniSpringColliderOffset, c->offset);
        maniVec3 tail = c->shape == maniColliderPlane
                            ? maniVec3Normalize(c->tail, maniVec3Make(0.0f, 0.0f, 1.0f))
                            : c->tail;
        StoreVec(e + maniSpringColliderTail, tail);
        StoreF32(e + maniSpringColliderRadius, c->radius);
    }
}

// The limit's frame in the head's space: the turn from +y to the rest
// direction, then the limit's rotation.
static maniQuat LimitFrame(maniVec3 axis, maniQuat rotation)
{
    return maniQuatMul(maniSpringTurnFromY(axis), maniQuatNormalize(rotation));
}

// The step's share of the reference's motion: the drag and the factor on
// stiffness and gravity.
static void Convert(double drag, double h, double f0, double* dragOut, double* forcing)
{
    double x = h * f0;
    double dh = drag >= 1.0 ? 1.0 : 1.0 - maniPowDouble(1.0 - drag, x);
    *dragOut = dh;
    *forcing = h * (drag > 0.0 ? dh / drag : x);
}

static void LayJoint(const maniSpringSetSource* s, const maniSpringJointSource* j, uint32_t tail,
                     uint8_t* e)
{
    double h =
        1.0 / (double)(s->stepRate == 0.0f ? (s->referenceRate == 0.0f ? 60.0f : s->referenceRate)
                                           : s->stepRate);
    double f0 = (double)(s->referenceRate == 0.0f ? 60.0f : s->referenceRate);
    double drag = 0.0;
    double forcing = 0.0;
    Convert((double)j->drag, h, f0, &drag, &forcing);
    maniStoreU32(e + maniSpringJointHead, j->bone);
    maniStoreU32(e + maniSpringJointTail, tail);
    maniStoreU32(e + maniSpringJointLimit, j->limit);
    StoreF32(e + maniSpringJointForcing, (float)forcing);
    StoreF32(e + maniSpringJointHitRadius, j->hitRadius);
    StoreF32(e + maniSpringJointDrag, (float)drag);
    StoreF32(e + maniSpringJointStiffness, (float)((double)j->stiffness * forcing));
    float g = (float)((double)j->gravityPower * forcing);
    StoreVec(e + maniSpringJointGravity, maniVec3Scale(j->gravityDirection, g));
    float pi = PI_F;
    float angle = j->limitAngle < pi ? j->limitAngle : pi;
    float yaw = j->limitYaw < pi / 2.0f ? j->limitYaw : pi / 2.0f;
    bool limited = j->limit != maniSpringLimitNone;
    StoreF32(e + maniSpringJointAngle, limited ? angle : 0.0f);
    StoreF32(e + maniSpringJointYaw, j->limit == maniSpringLimitSpherical ? yaw : 0.0f);
    maniQuat frame =
        limited ? LimitFrame(maniSpringRestDirection(s->skeleton, j->bone, tail), j->limitRotation)
                : maniQuatIdentity();
    StoreF32(e + maniSpringJointFrame, frame.x);
    StoreF32(e + maniSpringJointFrame + 4, frame.y);
    StoreF32(e + maniSpringJointFrame + 8, frame.z);
    StoreF32(e + maniSpringJointFrame + 12, frame.w);
}

static void LaySprings(const maniSpringSetSource* s, Work* w, uint8_t* const data[])
{
    uint32_t joint = 0;
    uint64_t link = 0;
    uint32_t name = 0;
    memset(w->colliderMark, 0, sizeof(uint32_t) * s->colliderCount);
    for (uint32_t i = 0; i < s->springCount; ++i)
    {
        const maniSpringSource* spring = &s->springs[i];
        uint8_t* e = data[2] + (size_t)i * MANI_SPRING_SPRING_SIZE;
        uint64_t links = Links(s, i, w, data[4] + 4 * link);
        // The check took only names within MANI_MAX_NAME_BYTES.
        const uint32_t words[7] = {
            joint, spring->jointCount - 1,      (uint32_t)link, (uint32_t)links, spring->center,
            name,  (uint32_t)spring->nameLength};
        for (uint32_t k = 0; k < 7; ++k)
        {
            maniStoreU32(e + 4 * k, words[k]);
        }
        for (uint32_t k = 0; k + 1 < spring->jointCount; ++k, ++joint)
        {
            LayJoint(s, &spring->joints[k], spring->joints[k + 1].bone,
                     data[3] + (size_t)joint * MANI_SPRING_JOINT_SIZE);
        }
        if (spring->nameLength > 0)
        {
            memcpy(data[5] + name, spring->name, spring->nameLength);
        }
        link += links;
        name += spring->nameLength;
    }
}

static void LayHeader(const maniSpringSetSource* s, const Work* w, uint8_t* out)
{
    float reference = s->referenceRate == 0.0f ? 60.0f : s->referenceRate;
    maniStoreU64(out, maniSkeleton_GetIdentity(s->skeleton));
    const uint32_t counts[6] = {s->skeleton->boneCount, s->colliderCount,   s->springCount,
                                (uint32_t)w->joints,    (uint32_t)w->links, (uint32_t)w->nameBytes};
    for (uint32_t k = 0; k < 6; ++k)
    {
        maniStoreU32(out + 8 + 4 * k, counts[k]);
    }
    StoreF32(out + 32, reference);
    StoreF32(out + 36, s->stepRate == 0.0f ? reference : s->stepRate);
}

static maniBuildResult Write(maniBuilder* builder, const maniSpringSetSource* s, Work* w)
{
    const uint64_t sizes[MANI_SPRING_SECTIONS] = {
        MANI_SPRING_HEADER_SIZE,
        (uint64_t)s->colliderCount * MANI_SPRING_COLLIDER_SIZE,
        (uint64_t)s->springCount * MANI_SPRING_SPRING_SIZE,
        w->joints * MANI_SPRING_JOINT_SIZE,
        4 * w->links,
        (w->nameBytes + 3) / 4 * 4};
    uint64_t own = 0;
    for (uint32_t i = 0; i < MANI_SPRING_SECTIONS; ++i)
    {
        own += sizes[i];
    }
    uint8_t* staging =
        own < SIZE_MAX ? maniAllocate(&builder->allocator, (size_t)own + 1, 1, 8) : NULL;
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(staging, 0, (size_t)own + 1);
    uint8_t* data[MANI_SPRING_SECTIONS];
    const uint8_t* views[MANI_SPRING_SECTIONS];
    maniSectionSource sections[MANI_SPRING_SECTIONS];
    uint32_t count = 0;
    size_t at = 0;
    for (uint32_t i = 0; i < MANI_SPRING_SECTIONS; at += (size_t)sizes[i], ++i)
    {
        data[i] = staging + at;
        views[i] = data[i];
        sections[count] = (maniSectionSource){i + 1, data[i], sizes[i]};
        count += sizes[i] > 0 ? 1u : 0u;
    }
    LayHeader(s, w, data[0]);
    LayColliders(s, data[1]);
    LaySprings(s, w, data);
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION,
                           maniSpringComputeIdentity(views, sizes), sections, count);
        maniBuilderKeep(builder, maniKindSpringSet, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, (size_t)own + 1, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

// The lists the counts name are given.
static bool Given(const maniSpringSetSource* s)
{
    return s->skeleton != NULL && (s->colliders != NULL || s->colliderCount == 0) &&
           (s->colliderGroups != NULL || s->colliderGroupCount == 0) &&
           (s->springs != NULL || s->springCount == 0);
}

static MANI_NOINLINE maniBuildResult BuildSpringSetWork(maniBuilder* builder,
                                                        const maniSpringSetSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (source == NULL || !Given(source))
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (source->springCount > MANI_MAX_BONES)
    {
        return Result(mani_errorLimit, maniBuildReasonCount, 0, 0);
    }
    size_t marks = (size_t)source->skeleton->boneCount + source->colliderCount + 1;
    uint32_t* stamps = maniAllocate(&builder->allocator, marks, sizeof(uint32_t), 4);
    if (stamps == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(stamps, 0, marks * sizeof(uint32_t));
    Work w = {stamps, stamps + source->skeleton->boneCount, 0, 0, 0};
    maniBuildResult r = Check(source, &w);
    r = r.result == mani_success ? Write(builder, source, &w) : r;
    maniRelease(&builder->allocator, stamps, marks, sizeof(uint32_t), 4);
    return r;
}

maniBuildResult maniBuildSpringSet(maniBuilder* builder, const maniSpringSetSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildSpringSetWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
