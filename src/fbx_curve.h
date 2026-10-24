// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX animation curves: keys decoded as ufbx decodes them
// into Bézier handles (weights, auto tangents with bias and clamps, TCB,
// constant previous and next, linear), evaluated as ufbx evaluates them,
// extrapolated in whole ticks; all in binary64.

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
    // Ticks a second, and the end keys' ticks, for extrapolating in
    // whole ticks.
    double ticksPerSecond;
    int64_t firstTick;
    int64_t lastTick;
} maniFbxCurve;

// Auto tangent flags, as KeyAttrFlags holds them.
enum
{
    maniFbxKeyClamp = 0x1000u,
    maniFbxKeyTimeIndependent = 0x2000u,
    maniFbxKeyClampProgressive = 0x4000u,
};

// FBX's unweighted handle length, as binary32 the way ufbx keeps it.
#define MANI_FBX_THIRD 0.333333f

// A double rounded to binary32 as ufbx keeps tangents, infinite past
// its range (where C leaves the conversion undefined).
float maniFbxNarrow(double x);

// An auto tangent's slope as ufbx (and Maya) solve it, under the flags
// above and an auto bias; the clamp threshold is ufbx's default, 0.
float maniFbxAutoSlope(double prevTime, double time, double nextTime, double prevValue,
                       double value, double nextValue, float weightLeft, float weightRight,
                       float bias, uint32_t flags);

// The extrapolation a node's child of a name gives (its Type a character,
// its Repetition a count), constant forever without one.
maniFbxExtrapolation maniFbxReadExtrapolation(const maniFbx* fbx, uint32_t node, const char* name);

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
