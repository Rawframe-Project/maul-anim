// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a spring set (mani-0026): the container, the header's counts
// and rates, every collider, spring, joint and link record (bones within
// the skeleton's count, shapes, floats finite and in range, records
// contiguous, each joint's head its chain's previous tail), the names,
// and the identity. A joint above the next and the centers' places need
// the skeleton: its identity, checked where a state is made, stands for
// the builder's checks.

#include "maul-anim/spring.h"

#include "bytes.h"
#include "container.h"
#include "spring_file.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint64_t maniSpringComputeIdentity(const uint8_t* const data[MANI_SPRING_SECTIONS],
                                   const uint64_t sizes[MANI_SPRING_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_SPRING_SECTIONS; ++i)
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

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

// count floats from p finite; the offset of the first that is not, or 0.
static uint32_t FirstNotFinite(const uint8_t* p, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!Finite(LoadF32(p + 4 * k)))
        {
            return 4 * k + 1;
        }
    }
    return 0;
}

static bool Unit(float lengthSquared)
{
    return lengthSquared >= 0.998f && lengthSquared <= 1.002f;
}

// The sections, located and sized.
typedef struct Sections
{
    const uint8_t* data[MANI_SPRING_SECTIONS];
    uint64_t sizes[MANI_SPRING_SECTIONS];
    uint64_t at[MANI_SPRING_SECTIONS];
} Sections;

static maniFileResult Header(const Sections* s, maniSpringSet* out)
{
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    if (s->sizes[0] != MANI_SPRING_HEADER_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE);
    }
    out->skeletonIdentity = maniLoadU64(h);
    out->boneCount = maniLoadU32(h + 8);
    out->colliderCount = maniLoadU32(h + 12);
    out->springCount = maniLoadU32(h + 16);
    out->jointCount = maniLoadU32(h + 20);
    out->linkCount = maniLoadU32(h + 24);
    out->nameBytes = maniLoadU32(h + 28);
    out->referenceRate = LoadF32(h + 32);
    out->stepRate = LoadF32(h + 36);
    if (out->boneCount < 1 || out->boneCount > MANI_MAX_BONES)
    {
        return Fail(at + 8);
    }
    if (out->springCount > MANI_MAX_BONES)
    {
        return Fail(at + 16);
    }
    for (uint32_t k = 0; k < 2; ++k)
    {
        float rate = LoadF32(h + 32 + 4 * k);
        if (!(rate > 0.0f && rate <= MANI_SPRING_MAX_RATE))
        {
            return Fail(at + 32 + 4 * k);
        }
    }
    return maniLoadU64(h + 40) == 0 ? kOk : Fail(at + 40);
}

// Each section's size as the header's counts say.
static maniFileResult Sizes(const Sections* s, const maniSpringSet* set)
{
    const uint64_t want[MANI_SPRING_SECTIONS] = {
        MANI_SPRING_HEADER_SIZE,
        (uint64_t)set->colliderCount * MANI_SPRING_COLLIDER_SIZE,
        (uint64_t)set->springCount * MANI_SPRING_SPRING_SIZE,
        (uint64_t)set->jointCount * MANI_SPRING_JOINT_SIZE,
        4 * (uint64_t)set->linkCount,
        ((uint64_t)set->nameBytes + 3) / 4 * 4};
    const uint32_t counts[MANI_SPRING_SECTIONS] = {0, 12, 16, 20, 24, 28};
    for (uint32_t i = 1; i < MANI_SPRING_SECTIONS; ++i)
    {
        if (s->sizes[i] != want[i])
        {
            return Fail(s->at[0] + counts[i]);
        }
    }
    for (uint64_t k = set->nameBytes; k < s->sizes[5]; ++k)
    {
        if (s->data[5][k] != 0)
        {
            return Fail(s->at[5] + k);
        }
    }
    return kOk;
}

static maniFileResult Collider(const Sections* s, const maniSpringSet* set, uint32_t i)
{
    const uint8_t* e = s->data[1] + (size_t)i * MANI_SPRING_COLLIDER_SIZE;
    uint64_t at = s->at[1] + (uint64_t)i * MANI_SPRING_COLLIDER_SIZE;
    uint32_t shape = maniLoadU32(e + 4);
    uint32_t inside = maniLoadU32(e + 8);
    if (maniLoadU32(e) >= set->boneCount)
    {
        return Fail(at);
    }
    if (shape > maniColliderPlane)
    {
        return Fail(at + 4);
    }
    if (inside > 1 || (shape == maniColliderPlane && inside != 0))
    {
        return Fail(at + 8);
    }
    uint32_t wrong = FirstNotFinite(e + 16, 7);
    if (maniLoadU32(e + 12) != 0 || wrong != 0)
    {
        return Fail(wrong != 0 ? at + 16 + wrong - 1 : at + 12);
    }
    float nx = LoadF32(e + 28);
    float ny = LoadF32(e + 32);
    float nz = LoadF32(e + 36);
    if (shape == maniColliderPlane && !Unit(nx * nx + ny * ny + nz * nz))
    {
        return Fail(at + 28);
    }
    if (!(LoadF32(e + 40) >= 0.0f))
    {
        return Fail(at + 40);
    }
    return maniLoadU32(e + 44) == 0 ? kOk : Fail(at + 44);
}

// A joint's limit: its angles in range, none's 0 with the identity frame.
static maniFileResult Limit(const uint8_t* e, uint64_t at)
{
    uint32_t limit = maniLoadU32(e + maniSpringJointLimit);
    float angle = LoadF32(e + maniSpringJointAngle);
    float yaw = LoadF32(e + maniSpringJointYaw);
    float pi = 3.14159265358979f;
    if (!(angle >= 0.0f && angle <= pi) || (limit == maniSpringLimitNone && angle != 0.0f))
    {
        return Fail(at + maniSpringJointAngle);
    }
    if (!(yaw >= 0.0f && yaw <= pi / 2.0f) || (limit != maniSpringLimitSpherical && yaw != 0.0f))
    {
        return Fail(at + maniSpringJointYaw);
    }
    const uint8_t* f = e + maniSpringJointFrame;
    float x = LoadF32(f);
    float y = LoadF32(f + 4);
    float z = LoadF32(f + 8);
    float w = LoadF32(f + 12);
    bool identity = x == 0.0f && y == 0.0f && z == 0.0f && w == 1.0f;
    bool frame = limit == maniSpringLimitNone ? identity : Unit(x * x + y * y + z * z + w * w);
    return frame ? kOk : Fail(at + maniSpringJointFrame);
}

// Joint j of a spring: bones in range and distinct, the head the tail
// before it (previous, MANI_NO_BONE for a chain's first), its values.
static maniFileResult Joint(const Sections* s, const maniSpringSet* set, uint32_t j,
                            uint32_t previous)
{
    const uint8_t* e = s->data[3] + (size_t)j * MANI_SPRING_JOINT_SIZE;
    uint64_t at = s->at[3] + (uint64_t)j * MANI_SPRING_JOINT_SIZE;
    uint32_t head = maniLoadU32(e + maniSpringJointHead);
    uint32_t tail = maniLoadU32(e + maniSpringJointTail);
    if (head >= set->boneCount || (previous != MANI_NO_BONE && head != previous))
    {
        return Fail(at);
    }
    if (tail >= set->boneCount || tail == head)
    {
        return Fail(at + maniSpringJointTail);
    }
    if (maniLoadU32(e + maniSpringJointLimit) > maniSpringLimitSpherical)
    {
        return Fail(at + maniSpringJointLimit);
    }
    uint32_t wrong = FirstNotFinite(e + 16, 12);
    if (maniLoadU32(e + 12) != 0 || wrong != 0)
    {
        return Fail(wrong != 0 ? at + 16 + wrong - 1 : at + 12);
    }
    float drag = LoadF32(e + maniSpringJointDrag);
    if (!(LoadF32(e + maniSpringJointHitRadius) >= 0.0f))
    {
        return Fail(at + maniSpringJointHitRadius);
    }
    if (!(drag >= 0.0f && drag <= 1.0f))
    {
        return Fail(at + maniSpringJointDrag);
    }
    if (!(LoadF32(e + maniSpringJointStiffness) >= 0.0f))
    {
        return Fail(at + maniSpringJointStiffness);
    }
    return Limit(e, at);
}

// The running firsts of the springs' joints, links and names.
typedef struct Runs
{
    uint32_t joint;
    uint32_t link;
    uint32_t name;
} Runs;

static maniFileResult Spring(const Sections* s, const maniSpringSet* set, uint32_t i, Runs* r)
{
    const uint8_t* e = s->data[2] + (size_t)i * MANI_SPRING_SPRING_SIZE;
    uint64_t at = s->at[2] + (uint64_t)i * MANI_SPRING_SPRING_SIZE;
    uint32_t joints = maniLoadU32(e + 4);
    uint32_t links = maniLoadU32(e + 12);
    uint32_t center = maniLoadU32(e + 16);
    uint32_t length = maniLoadU32(e + 24);
    if (maniLoadU32(e) != r->joint || joints < 1 || joints > set->jointCount - r->joint)
    {
        return Fail(maniLoadU32(e) != r->joint ? at : at + 4);
    }
    if (maniLoadU32(e + 8) != r->link || links > set->linkCount - r->link)
    {
        return Fail(maniLoadU32(e + 8) != r->link ? at + 8 : at + 12);
    }
    if (center >= set->boneCount && center != MANI_NO_BONE)
    {
        return Fail(at + 16);
    }
    if (maniLoadU32(e + 20) != r->name || length > set->nameBytes - r->name)
    {
        return Fail(maniLoadU32(e + 20) != r->name ? at + 20 : at + 24);
    }
    if (!maniValidUtf8(s->data[5] + r->name, length))
    {
        return Fail(s->at[5] + r->name);
    }
    if (maniLoadU32(e + 28) != 0)
    {
        return Fail(at + 28);
    }
    maniFileResult f = kOk;
    for (uint32_t k = 0; f.result == mani_success && k < joints; ++k)
    {
        uint32_t previous =
            k == 0 ? MANI_NO_BONE
                   : maniLoadU32(s->data[3] + (size_t)(r->joint + k - 1) * MANI_SPRING_JOINT_SIZE +
                                 maniSpringJointTail);
        f = Joint(s, set, r->joint + k, previous);
    }
    r->joint += joints;
    r->link += links;
    r->name += length;
    return f;
}

static maniFileResult Records(const Sections* s, const maniSpringSet* set)
{
    maniFileResult r = kOk;
    for (uint32_t i = 0; r.result == mani_success && i < set->colliderCount; ++i)
    {
        r = Collider(s, set, i);
    }
    Runs runs = {0, 0, 0};
    for (uint32_t i = 0; r.result == mani_success && i < set->springCount; ++i)
    {
        r = Spring(s, set, i, &runs);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (runs.joint != set->jointCount || runs.link != set->linkCount || runs.name != set->nameBytes)
    {
        return Fail(s->at[0] + (runs.joint != set->jointCount ? 20
                                : runs.link != set->linkCount ? 24
                                                              : 28));
    }
    for (uint32_t k = 0; k < set->linkCount; ++k)
    {
        if (maniLoadU32(s->data[4] + 4 * (size_t)k) >= set->colliderCount)
        {
            return Fail(s->at[4] + 4 * (uint64_t)k);
        }
    }
    return kOk;
}

maniFileResult maniOpenSpringSet(const void* bytes, size_t size, maniSpringSet* setOut)
{
    if (setOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    static const uint8_t kEmpty = 0;
    Sections s;
    for (uint32_t t = 1; t <= MANI_SPRING_SECTIONS; ++t)
    {
        s.data[t - 1] = &kEmpty;
        s.sizes[t - 1] = 0;
        s.at[t - 1] = 0;
        (void)maniContainerSection(&c, t, &s.data[t - 1], &s.sizes[t - 1], &s.at[t - 1]);
    }
    maniSpringSet set;
    memset(&set, 0, sizeof set);
    r = Header(&s, &set);
    r = r.result == mani_success ? Sizes(&s, &set) : r;
    r = r.result == mani_success ? Records(&s, &set) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniSpringComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    set.colliders = s.data[1];
    set.springs = s.data[2];
    set.joints = s.data[3];
    set.links = s.data[4];
    set.names = s.data[5];
    set.identity = c.identity;
    *setOut = set;
    return kOk;
}

uint64_t maniSpringSetIdentity(const maniSpringSet* set)
{
    return set != NULL ? set->identity : 0;
}

uint64_t maniSpringSetSkeletonIdentity(const maniSpringSet* set)
{
    return set != NULL ? set->skeletonIdentity : 0;
}

uint32_t maniSpringSetSpringCount(const maniSpringSet* set)
{
    return set != NULL ? set->springCount : 0;
}

const char* maniSpringSetSpringName(const maniSpringSet* set, uint32_t spring, uint32_t* lengthOut)
{
    bool valid = set != NULL && spring < set->springCount;
    const uint8_t* e = valid ? set->springs + (size_t)spring * MANI_SPRING_SPRING_SIZE : NULL;
    if (lengthOut != NULL)
    {
        *lengthOut = valid ? maniLoadU32(e + 24) : 0;
    }
    return valid ? (const char*)(set->names + maniLoadU32(e + 20)) : NULL;
}

float maniSpringSetStepRate(const maniSpringSet* set, float* referenceRateOut)
{
    if (referenceRateOut != NULL)
    {
        *referenceRateOut = set != NULL ? set->referenceRate : 0.0f;
    }
    return set != NULL ? set->stepRate : 0.0f;
}
