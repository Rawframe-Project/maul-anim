// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The retargeting map file's layout (mani-0032), shared by its view and
// its builder: kind 4 of the container. Records are little-endian. A
// bone's model rotation is the product of its root path's local
// rotations (scale left out); positions are the reference pose's model
// translations.

#ifndef MAUL_ANIM_SRC_RETARGET_FILE_H
#define MAUL_ANIM_SRC_RETARGET_FILE_H

#include <stdint.h>

#define MANI_RETARGET_FORMAT_VERSION 1u
#define MANI_RETARGET_SECTIONS       4u

enum
{
    // The header, laid out below.
    maniRetargetTagHeader = 1,
    // MANI_RETARGET_SOURCE_SIZE a source bone: its reference model
    // rotation (4 binary32), local translation (3) and local scale (3).
    maniRetargetTagSource = 2,
    // MANI_RETARGET_TARGET_SIZE a target bone: its reference model
    // rotation (4), local rotation (4), local translation (3) and local
    // scale (3).
    maniRetargetTagTarget = 3,
    // MANI_RETARGET_PLAN_SIZE a target bone, laid out below.
    maniRetargetTagPlan = 4,
};

#define MANI_RETARGET_HEADER_SIZE 112u
#define MANI_RETARGET_SOURCE_SIZE 40u
#define MANI_RETARGET_TARGET_SIZE 56u
#define MANI_RETARGET_PLAN_SIZE   32u

// Header offsets: u64 the source's and the target's identities; u32 their
// bone counts; u32 the pelvis's source and target bones (MANI_NO_BONE
// both without one); binary32: the up axis (3), the horizontal and
// vertical scales (the height ratio times the factors), the absolute
// blends (3), the source blends (3), the translation weight, the source
// and target reference pelvis positions (3 each), the height ratio; a 0
// word. Without a pelvis the axis is +y, the scales and the ratio 1 and
// every other number 0.
enum
{
    maniRetargetHeaderSourceIdentity = 0,
    maniRetargetHeaderTargetIdentity = 8,
    maniRetargetHeaderSourceBones = 16,
    maniRetargetHeaderTargetBones = 20,
    maniRetargetHeaderPelvisSource = 24,
    maniRetargetHeaderPelvisTarget = 28,
    maniRetargetHeaderUp = 32,
    maniRetargetHeaderHorizontal = 44,
    maniRetargetHeaderVertical = 48,
    maniRetargetHeaderAbsolute = 52,
    maniRetargetHeaderToSource = 64,
    maniRetargetHeaderTranslationWeight = 76,
    maniRetargetHeaderSourcePelvis = 80,
    maniRetargetHeaderTargetPelvis = 92,
    maniRetargetHeaderRatio = 104,
    maniRetargetHeaderEnd = 108,
};

// A target bone's plan: u32 its kind; u32 source bones a and b; binary32
// its place t between them and its rotation weight; u32 its translation
// (maniRetargetTranslation, or maniRetargetPlanPelvis for the pelvis's
// target) and flags; a 0 word. A reference bone's a and b are
// MANI_NO_BONE and every other word 0; a one-source bone's b is
// MANI_NO_BONE and its t 0; a two-source bone's t lies in (0, 1).
enum
{
    maniRetargetPlanKind = 0,
    maniRetargetPlanA = 4,
    maniRetargetPlanB = 8,
    maniRetargetPlanT = 12,
    maniRetargetPlanWeight = 16,
    maniRetargetPlanTranslation = 20,
    maniRetargetPlanFlags = 24,
    maniRetargetPlanEnd = 28,
};

// Plan kinds.
enum
{
    maniRetargetPlanReference = 0,
    maniRetargetPlanOne = 1,
    maniRetargetPlanTwo = 2,
};

// The pelvis's translation, past maniRetargetTranslation's values.
#define maniRetargetPlanPelvis 3u

// Plan flags: the scale carries.
#define MANI_RETARGET_PLAN_SCALE 1u

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniRetargetComputeIdentity(const uint8_t* const data[MANI_RETARGET_SECTIONS],
                                     const uint64_t sizes[MANI_RETARGET_SECTIONS]);

#endif // MAUL_ANIM_SRC_RETARGET_FILE_H
