// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX animation curves (mani-0016): keys decoded as ufbx decodes them
// into Bézier handles (weights, auto tangents with bias and clamps, TCB,
// constant previous and next, linear), evaluated as ufbx evaluates them,
// extrapolated in whole ticks; all in binary64 (research 24).

#ifndef MAUL_ANIM_SRC_FBX_CURVE_H
#define MAUL_ANIM_SRC_FBX_CURVE_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

enum
{
    maniFbxConstantPrevious = 0,
    maniFbxConstantNext = 1,
    maniFbxLinear = 2,
    maniFbxCubic = 3,
};

enum
{
    maniFbxExtrapolateConstant = 0,
    maniFbxExtrapolateRepeat = 1,
    maniFbxExtrapolateRelative = 2,
    maniFbxExtrapolateMirror = 3,
    maniFbxExtrapolateSlope = 4,
};

// A key: its time in seconds, its value, its left and right handles
// (time and value offsets), and how it interpolates to the next.
typedef struct maniFbxKey
{
    double time;
    double value;
    double leftDx;
    double leftDy;
    double rightDx;
    double rightDy;
    uint32_t interpolation;
    uint32_t reserved;
} maniFbxKey;

// How a curve continues before its first key or after its last: a mode
// and a repeat count (negative: forever).
typedef struct maniFbxExtrapolation
{
    uint32_t mode;
    int32_t repeat;
} maniFbxExtrapolation;

typedef struct maniFbxCurve
{
    maniFbxKey* keys;
    uint32_t count;
    uint32_t reserved;
    maniFbxExtrapolation pre;
    maniFbxExtrapolation post;
    // Ticks a second, for extrapolating in whole ticks.
    double ticksPerSecond;
} maniFbxCurve;

// Reads an AnimationCurve object; refused (Property, item: the object)
// for arrays missing, of the wrong kinds or sizes, run lengths that do
// not cover the keys or times that decrease, Data for values not
// finite; Capacity when the allocator fails.
maniImportResult maniFbxReadCurve(const maniFbx* fbx, uint32_t object, maniFbxCurve* out);

void maniFbxReleaseCurve(const maniFbx* fbx, maniFbxCurve* curve);

// The curve's value at a time in seconds; fallback for a curve with no
// keys.
double maniFbxEvaluateCurve(const maniFbxCurve* curve, double time, double fallback);

#endif // MAUL_ANIM_SRC_FBX_CURVE_H
