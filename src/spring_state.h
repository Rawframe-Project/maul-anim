// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A spring state's layout, shared by the standalone state
// and the graph's spring node: a block of 32-bit words with no pointers,
// so a graph instance holds it as it is and a saved state copies it.
// Positions are in the simulation's space: a spring's center bone's
// space when it has one, else model space as the last step placed it.
//
//    0  1 when the joints have motion, else 0 (every other word 0)
//    1  0
//    2  the time since the last step, binary64 (two words), in [0, step)
//    4  the last call's placement: position (3), rotation (4)
//   11  the last step's placement: position (3), rotation (4)
//   18  the last call's external force, in the world (3)
//   21  per joint MANI_SPRING_JOINT_WORDS: the tail now (3) and a step
//       before (3); the last call's inputs: the head's position (3),
//       rotation (4) and largest absolute scale (1), the tail's position
//       (3), in model space
//       per collider MANI_SPRING_BONE_WORDS: the last call's bone
//       transform (translation 3, rotation 4, scale 3) in model space
//       per spring MANI_SPRING_BONE_WORDS: its center's, likewise (0s
//       for none)
//
// A call works in an area of its own after the block (no part of it or
// of a saved state; a graph instance's room holds a node's): each bone
// it reads decomposed once, and the colliders' shapes at the step.
//
//    0  per joint MANI_SPRING_HEAD_WORDS: the head's position (3),
//       rotation (4), largest absolute scale (1), the tail's position
//       (3), the head's scale (3), in model space
//       per collider MANI_SPRING_BONE_WORDS: its bone's transform
//       per spring MANI_SPRING_BONE_WORDS: its center's (0s for none)
//       per collider MANI_SPRING_SHAPE_WORDS: its shape at the step
//       (offset 3, tail 3, radius 1, shape 1, inside 1)

#ifndef MAUL_ANIM_SRC_SPRING_STATE_H
#define MAUL_ANIM_SRC_SPRING_STATE_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum
{
    maniSpringHas = 0,
    maniSpringClock = 2,
    maniSpringPlacement = 4,
    maniSpringStepPlacement = 11,
    maniSpringForce = 18,
    maniSpringJoints = 21,
};

// Joint word offsets.
enum
{
    maniSpringTailNow = 0,
    maniSpringTailBefore = 3,
    maniSpringHeadPosition = 6,
    maniSpringHeadRotation = 9,
    maniSpringHeadScale = 13,
    maniSpringTailPosition = 14,
};

#define MANI_SPRING_JOINT_WORDS 17u
#define MANI_SPRING_BONE_WORDS  10u

// A joint's words in the work area.
enum
{
    maniSpringNowPosition = 0,
    maniSpringNowRotation = 3,
    maniSpringNowLargest = 7,
    maniSpringNowTail = 8,
    maniSpringNowScale = 11,
};

#define MANI_SPRING_HEAD_WORDS  14u
#define MANI_SPRING_SHAPE_WORDS 9u

// A saved state: the state prefix ("MSST", the writer's ABI version, the
// format version, the header size), the set's identity (u64), the size
// (u64); the block.
#define MANI_SPRING_STATE_MAGIC       0x5453534Du
#define MANI_SPRING_STATE_VERSION     1u
#define MANI_SPRING_STATE_HEADER_SIZE 32u

struct maniSpringState
{
    const maniSpringSet* set;
    const maniSkeleton* skeleton;
    uint64_t words;
};

// The words a set's block holds.
uint64_t maniSpringBlockWords(const maniSpringSet* set);

// A state's block, just after the state.
static inline uint32_t* maniSpringBlock(maniSpringState* state)
{
    return (uint32_t*)(void*)(state + 1);
}

static inline const uint32_t* maniSpringConstBlock(const maniSpringState* state)
{
    return (const uint32_t*)(const void*)(state + 1);
}

static inline uint64_t maniSpringJointAt(uint32_t joint)
{
    return maniSpringJoints + (uint64_t)joint * MANI_SPRING_JOINT_WORDS;
}

static inline uint64_t maniSpringColliderAt(const maniSpringSet* set, uint32_t collider)
{
    return maniSpringJointAt(set->jointCount) + (uint64_t)collider * MANI_SPRING_BONE_WORDS;
}

static inline uint64_t maniSpringCenterAt(const maniSpringSet* set, uint32_t spring)
{
    return maniSpringColliderAt(set, set->colliderCount) +
           (uint64_t)spring * MANI_SPRING_BONE_WORDS;
}

// The words of a set's work area.
uint64_t maniSpringWorkWords(const maniSpringSet* set);

static inline uint64_t maniSpringNowJointAt(uint32_t joint)
{
    return (uint64_t)joint * MANI_SPRING_HEAD_WORDS;
}

static inline uint64_t maniSpringNowColliderAt(const maniSpringSet* set, uint32_t collider)
{
    return maniSpringNowJointAt(set->jointCount) + (uint64_t)collider * MANI_SPRING_BONE_WORDS;
}

static inline uint64_t maniSpringNowCenterAt(const maniSpringSet* set, uint32_t spring)
{
    return maniSpringNowColliderAt(set, set->colliderCount) +
           (uint64_t)spring * MANI_SPRING_BONE_WORDS;
}

static inline uint64_t maniSpringShapeAt(const maniSpringSet* set, uint32_t collider)
{
    return maniSpringNowCenterAt(set, set->springCount) +
           (uint64_t)collider * MANI_SPRING_SHAPE_WORDS;
}

static inline float maniSpringWord(const uint32_t* block, uint64_t at)
{
    float v = 0.0f;
    memcpy(&v, &block[at], sizeof v);
    return v;
}

static inline void maniSpringSetWord(uint32_t* block, uint64_t at, float v)
{
    memcpy(&block[at], &v, sizeof v);
}

static inline maniVec3 maniSpringVec(const uint32_t* block, uint64_t at)
{
    maniVec3 v = {maniSpringWord(block, at), maniSpringWord(block, at + 1),
                  maniSpringWord(block, at + 2)};
    return v;
}

static inline void maniSpringSetVec(uint32_t* block, uint64_t at, maniVec3 v)
{
    maniSpringSetWord(block, at, v.x);
    maniSpringSetWord(block, at + 1, v.y);
    maniSpringSetWord(block, at + 2, v.z);
}

static inline maniQuat maniSpringQuat(const uint32_t* block, uint64_t at)
{
    maniQuat q = {maniSpringWord(block, at), maniSpringWord(block, at + 1),
                  maniSpringWord(block, at + 2), maniSpringWord(block, at + 3)};
    return q;
}

static inline void maniSpringSetQuat(uint32_t* block, uint64_t at, maniQuat q)
{
    maniSpringSetWord(block, at, q.x);
    maniSpringSetWord(block, at + 1, q.y);
    maniSpringSetWord(block, at + 2, q.z);
    maniSpringSetWord(block, at + 3, q.w);
}

// A bone's transform as the block keeps it at word at.
static inline maniTransform maniSpringBoneWords(const uint32_t* block, uint64_t at)
{
    return (maniTransform){maniSpringVec(block, at), maniSpringQuat(block, at + 3),
                           maniSpringVec(block, at + 7)};
}

// A rotation seen through a model scale's mirror: D q D for D the
// scale's signs, so a turn above a mirrored bone is the mirror of the
// turn below it (the identity without a mirror).
static inline maniQuat maniSpringMirror(maniQuat q, maniVec3 scale)
{
    float x = scale.x < 0.0f ? -1.0f : 1.0f;
    float y = scale.y < 0.0f ? -1.0f : 1.0f;
    float z = scale.z < 0.0f ? -1.0f : 1.0f;
    float det = x * y * z;
    maniQuat m = {q.x * x * det, q.y * y * det, q.z * z * det, q.w};
    return m;
}

// The time since the last step.
static inline double maniSpringClockOf(const uint32_t* block)
{
    uint64_t bits = (uint64_t)block[maniSpringClock] | (uint64_t)block[maniSpringClock + 1] << 32;
    double v = 0.0;
    memcpy(&v, &bits, sizeof v);
    return v;
}

// The byte offset of the first word of a block (as little-endian bytes)
// found wrong, or UINT64_MAX: has 0 or 1 (0 with every word 0), the
// clock finite and in [0, 1 / the step rate), every float finite,
// rotations of length 1 within a thousandth, head scales not negative.
uint64_t maniSpringBlockValid(const maniSpringSet* set, const uint8_t* bytes);

// Moves a block on by step seconds (at most MANI_SPRING_MAX_STEPS
// steps'), toward a pose whose model space is current, at a placement
// whose rotation is unit, with a finite external force in the world,
// working in work (maniSpringWorkWords words).
void maniSpringStepBlock(const maniSpringSet* set, uint32_t* block, uint32_t* work,
                         const maniPose* pose, const maniTransform* placement, maniVec3 force,
                         double step);

// Turns a pose's spring joints toward the block's tails; a pose of the
// skeleton whose model space is current, left stale.
void maniSpringApplyBlock(const maniSpringSet* set, const maniSkeleton* skeleton,
                          const uint32_t* block, maniPose* pose);

#endif // MAUL_ANIM_SRC_SPRING_STATE_H
