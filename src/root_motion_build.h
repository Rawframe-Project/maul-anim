// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Taking root motion out of a bone at build (mani-0022), over the bone's
// local transforms at the grid's samples, which the clip builder gives.

#ifndef MAUL_ANIM_SRC_ROOT_MOTION_BUILD_H
#define MAUL_ANIM_SRC_ROOT_MOTION_BUILD_H

#include "allocator.h"

#include "maul-anim/base.h"
#include "maul-anim/rootmotion.h"

#include <stdint.h>

// The motion of a clip being built, per sample in binary64: translation
// x, y, z and rotation x, y, z, w.
typedef struct maniRootMotionWork
{
    maniRootMotionSource source;
    double* motion;
    uint32_t sampleCount;
} maniRootMotionWork;

// Takes the motion at each of n samples: locals holds the bone's local
// translation and rotation at each (seven binary64), rest its rest's,
// times the samples' times (the last the duration); the loop option
// applied. Capacity when the allocator fails.
maniResult maniRootMotionTake(const maniRootMotionSource* source, const double* locals,
                              const double* rest, const double* times, uint32_t n,
                              const maniAllocator* allocator, maniRootMotionWork* out);

// Sample k's local translation and rotation with the motion taken out:
// t' = m_r^-1 (t - m_t), r' = m_r^-1 r.
void maniRootMotionBake(const maniRootMotionWork* work, uint32_t k, double t[3], double r[4]);

// The section's bytes (maniRootMotionSize of the sample count).
void maniRootMotionWrite(const maniRootMotionWork* work, uint8_t* out);

void maniRootMotionRelease(maniRootMotionWork* work, const maniAllocator* allocator);

#endif // MAUL_ANIM_SRC_ROOT_MOTION_BUILD_H
