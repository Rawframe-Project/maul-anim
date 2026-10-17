// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring states (mani-0026): made in caller memory, stepped, applied,
// reset, saved and restored. A saved state is the block behind a header
// and is checked as external bytes before it is copied in.

#include "spring_state.h"

#include "bytes.h"
#include "spring_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const uint64_t kValid = UINT64_MAX;

uint64_t maniSpringBlockWords(const maniSpringSet* set)
{
    return maniSpringCenterAt(set, set->springCount);
}

uint64_t maniSpringWorkWords(const maniSpringSet* set)
{
    return maniSpringShapeAt(set, set->colliderCount);
}

size_t maniSpringStateSize(const maniSpringSet* set)
{
    if (set == NULL)
    {
        return 0;
    }
    size_t size = sizeof(maniSpringState) +
                  4 * (size_t)(maniSpringBlockWords(set) + maniSpringWorkWords(set));
    return (size + MANI_SPRING_STATE_ALIGNMENT - 1) & ~(size_t)(MANI_SPRING_STATE_ALIGNMENT - 1);
}

maniResult maniInitSpringState(const maniSpringSet* set, const maniSkeleton* skeleton, void* memory,
                               size_t size, maniSpringState** stateOut)
{
    if (set == NULL || skeleton == NULL || memory == NULL || stateOut == NULL ||
        (uintptr_t)memory % MANI_SPRING_STATE_ALIGNMENT != 0 ||
        maniSkeletonIdentity(skeleton) != set->skeletonIdentity ||
        maniSkeletonBoneCount(skeleton) != set->boneCount)
    {
        return mani_errorInvalid;
    }
    size_t need = maniSpringStateSize(set);
    if (size < need)
    {
        return mani_errorCapacity;
    }
    memset(memory, 0, need);
    maniSpringState* state = memory;
    state->set = set;
    state->skeleton = skeleton;
    state->words = maniSpringBlockWords(set);
    *stateOut = state;
    return mani_success;
}

static bool PlacementValid(const maniTransform* t)
{
    const maniVec3 p = t->translation;
    const maniQuat q = t->rotation;
    float length = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(length) && length > 0.0f;
}

// The pose's model space, composed when stale; false for a pose that
// does not fit the state.
static bool ModelReady(const maniSpringState* state, maniPose* pose)
{
    if (maniPoseBoneCount(pose) != state->set->boneCount || maniPoseIsDifference(pose))
    {
        return false;
    }
    return maniPoseModelCurrent(pose) || maniComputeModel(pose, state->skeleton) == mani_success;
}

maniResult maniStepSprings(maniSpringState* state, maniPose* pose, const maniTransform* placement,
                           const maniVec3* force, float delta)
{
    maniTransform place = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    maniVec3 push = {0.0f, 0.0f, 0.0f};
    if (placement != NULL)
    {
        place = *placement;
    }
    if (force != NULL)
    {
        push = *force;
    }
    if (state == NULL || pose == NULL || !isfinite(delta) || !PlacementValid(&place) ||
        !isfinite(push.x) || !isfinite(push.y) || !isfinite(push.z) || !ModelReady(state, pose))
    {
        return mani_errorInvalid;
    }
    place.rotation = maniQuatNormalize(place.rotation);
    uint32_t* block = maniSpringBlock(state);
    maniSpringStepBlock(state->set, block, block + state->words, pose, &place, push,
                        fabs((double)delta));
    return mani_success;
}

maniResult maniApplySprings(const maniSpringState* state, maniPose* pose)
{
    if (state == NULL || pose == NULL || !ModelReady(state, pose))
    {
        return mani_errorInvalid;
    }
    maniSpringApplyBlock(state->set, state->skeleton, maniSpringConstBlock(state), pose);
    return mani_success;
}

void maniResetSprings(maniSpringState* state)
{
    if (state != NULL)
    {
        memset(maniSpringBlock(state), 0, 4 * (size_t)state->words);
    }
}

size_t maniSpringSavedSize(const maniSpringSet* set)
{
    return set == NULL ? 0 : MANI_SPRING_STATE_HEADER_SIZE + 4 * (size_t)maniSpringBlockWords(set);
}

maniResult maniSaveSpringState(const maniSpringState* state, void* out, size_t capacity)
{
    if (state == NULL || out == NULL)
    {
        return mani_errorInvalid;
    }
    size_t size = maniSpringSavedSize(state->set);
    if (capacity < size)
    {
        return mani_errorCapacity;
    }
    uint8_t* p = out;
    maniStoreU32(p, MANI_SPRING_STATE_MAGIC);
    maniStoreU32(p + 4, 0);
    maniStoreU64(p + 8, state->set->identity);
    maniStoreU64(p + 16, size);
    const uint32_t* block = maniSpringConstBlock(state);
    for (uint64_t w = 0; w < state->words; ++w)
    {
        maniStoreU32(p + MANI_SPRING_STATE_HEADER_SIZE + 4 * w, block[w]);
    }
    return mani_success;
}

// The first of count floats at a word that is not finite, or kValid.
static uint64_t FiniteRun(const uint8_t* bytes, uint64_t at, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t bits = maniLoadU32(bytes + 4 * (at + i));
        float v = 0.0f;
        memcpy(&v, &bits, sizeof v);
        if (!isfinite(v))
        {
            return at + i;
        }
    }
    return kValid;
}

static float FloatAt(const uint8_t* bytes, uint64_t at)
{
    uint32_t bits = maniLoadU32(bytes + 4 * at);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

// A rotation's four words finite and of length 1 within a thousandth.
static uint64_t Rotation(const uint8_t* bytes, uint64_t at)
{
    uint64_t wrong = FiniteRun(bytes, at, 4);
    if (wrong != kValid)
    {
        return wrong;
    }
    float x = FloatAt(bytes, at);
    float y = FloatAt(bytes, at + 1);
    float z = FloatAt(bytes, at + 2);
    float w = FloatAt(bytes, at + 3);
    float length = x * x + y * y + z * z + w * w;
    return length > 0.998f && length < 1.002f ? kValid : at;
}

// A placement (position, rotation) or a bone's transform (translation,
// rotation, scale).
static uint64_t Placed(const uint8_t* bytes, uint64_t at, bool scaled)
{
    uint64_t wrong = FiniteRun(bytes, at, 3);
    wrong = wrong != kValid ? wrong : Rotation(bytes, at + 3);
    return wrong != kValid || !scaled ? wrong : FiniteRun(bytes, at + 7, 3);
}

static uint64_t Joint(const uint8_t* bytes, uint64_t at)
{
    uint64_t wrong = FiniteRun(bytes, at, maniSpringHeadRotation);
    wrong = wrong != kValid ? wrong : Rotation(bytes, at + maniSpringHeadRotation);
    if (wrong == kValid && !(FloatAt(bytes, at + maniSpringHeadScale) >= 0.0f &&
                             isfinite(FloatAt(bytes, at + maniSpringHeadScale))))
    {
        wrong = at + maniSpringHeadScale;
    }
    return wrong != kValid ? wrong : FiniteRun(bytes, at + maniSpringTailPosition, 3);
}

// A spring's center words: a bone's transform, or zeros for none.
static uint64_t Center(const maniSpringSet* set, const uint8_t* bytes, uint32_t spring)
{
    uint64_t at = maniSpringCenterAt(set, spring);
    if (maniLoadU32(set->springs + (size_t)spring * MANI_SPRING_SPRING_SIZE + 16) != MANI_NO_BONE)
    {
        return Placed(bytes, at, true);
    }
    for (uint32_t i = 0; i < MANI_SPRING_BONE_WORDS; ++i)
    {
        if (maniLoadU32(bytes + 4 * (at + i)) != 0)
        {
            return at + i;
        }
    }
    return kValid;
}

// A block in motion: the clock, placements, joints, colliders, centers.
static uint64_t Moving(const maniSpringSet* set, const uint8_t* bytes)
{
    uint64_t bits = maniLoadU64(bytes + 4 * maniSpringClock);
    double clock = 0.0;
    memcpy(&clock, &bits, sizeof clock);
    // As the steps keep it (NaN and the infinities fail one or the other).
    if (!(clock >= 0.0 && clock < 1.0 / (double)set->stepRate))
    {
        return maniSpringClock;
    }
    uint64_t wrong = Placed(bytes, maniSpringPlacement, false);
    wrong = wrong != kValid ? wrong : Placed(bytes, maniSpringStepPlacement, false);
    wrong = wrong != kValid ? wrong : FiniteRun(bytes, maniSpringForce, 3);
    for (uint32_t j = 0; wrong == kValid && j < set->jointCount; ++j)
    {
        wrong = Joint(bytes, maniSpringJointAt(j));
    }
    for (uint32_t c = 0; wrong == kValid && c < set->colliderCount; ++c)
    {
        wrong = Placed(bytes, maniSpringColliderAt(set, c), true);
    }
    for (uint32_t i = 0; wrong == kValid && i < set->springCount; ++i)
    {
        wrong = Center(set, bytes, i);
    }
    return wrong;
}

uint64_t maniSpringBlockValid(const maniSpringSet* set, const uint8_t* bytes)
{
    uint32_t has = maniLoadU32(bytes);
    if (has > 1)
    {
        return 0;
    }
    if (maniLoadU32(bytes + 4) != 0)
    {
        return 4;
    }
    if (has == 1)
    {
        uint64_t wrong = Moving(set, bytes);
        return wrong == kValid ? kValid : 4 * wrong;
    }
    uint64_t words = maniSpringBlockWords(set);
    for (uint64_t w = 2; w < words; ++w)
    {
        if (maniLoadU32(bytes + 4 * w) != 0)
        {
            return 4 * w;
        }
    }
    return kValid;
}

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

maniFileResult maniRestoreSpringState(maniSpringState* state, const void* bytes, size_t size)
{
    if (state == NULL || bytes == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    const uint8_t* b = bytes;
    size_t expected = maniSpringSavedSize(state->set);
    if (size < MANI_SPRING_STATE_HEADER_SIZE || maniLoadU32(b) != MANI_SPRING_STATE_MAGIC)
    {
        return Fail(0);
    }
    if (maniLoadU32(b + 4) != 0)
    {
        return Fail(4);
    }
    if (maniLoadU64(b + 8) != state->set->identity)
    {
        return Fail(8);
    }
    if (maniLoadU64(b + 16) != expected || size < expected)
    {
        return Fail(16);
    }
    uint64_t wrong = maniSpringBlockValid(state->set, b + MANI_SPRING_STATE_HEADER_SIZE);
    if (wrong != kValid)
    {
        return Fail(MANI_SPRING_STATE_HEADER_SIZE + wrong);
    }
    uint32_t* block = maniSpringBlock(state);
    for (uint64_t w = 0; w < state->words; ++w)
    {
        block[w] = maniLoadU32(b + MANI_SPRING_STATE_HEADER_SIZE + 4 * w);
    }
    return (maniFileResult){mani_success, 0, 0};
}
