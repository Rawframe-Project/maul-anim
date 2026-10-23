// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX's local transform in binary64: Euler rotations in each
// order, the pivot and offset chain as one TRS, and the 3x4 matrices
// worlds are composed with.

#ifndef MAUL_ANIM_SRC_FBX_TRANSFORM_H
#define MAUL_ANIM_SRC_FBX_TRANSFORM_H

#include "fbx_document.h"
#include "fbx_object.h"

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// A transform: translation, rotation (x, y, z, w) and scale.
typedef struct maniFbxTrs
{
    double translation[3];
    double rotation[4];
    double scale[3];
} maniFbxTrs;

// A 3x4 matrix as maniAffine lays one out: three columns, then the
// translation.
typedef struct maniFbxMatrix
{
    double m[12];
} maniFbxMatrix;

// FBX's inherit types.
enum
{
    maniFbxInheritRrSs = 0,
    maniFbxInheritRSrs = 1,
    maniFbxInheritRrs = 2,
};

// A model's local transform and how it inherits its parent's scale.
typedef struct maniFbxLocal
{
    maniFbxTrs trs;
    uint32_t inherit;
} maniFbxLocal;

// The sine and cosine of an angle in degrees, exact at multiples of 90,
// within 1e-13 elsewhere (the library's quarter series, ample for
// binary32 results).
void maniFbxSinCosDegrees(double degrees, double* sinOut, double* cosOut);

// Euler angles in degrees as a quaternion: order 0 to 5 is XYZ, XZY,
// YZX, YXZ, ZXY, ZYX, the first axis applied first.
void maniFbxEulerQuat(const double degrees[3], uint32_t order, double q[4]);

// A unit quaternion as Euler angles in degrees in an order (0 to 5), by
// ufbx's formulas: the middle angle by its arc sine, within 1e-9 of a
// quarter turn the first angle 0.
void maniFbxQuatEuler(const double q[4], uint32_t order, double degrees[3]);

// The product a b of two quaternions.
void maniFbxQuatMul(const double a[4], const double b[4], double out[4]);

// A vector rotated by a unit quaternion.
void maniFbxQuatRotate(const double q[4], const double v[3], double out[3]);

maniFbxMatrix maniFbxTrsMatrix(const maniFbxTrs* t);

maniFbxMatrix maniFbxMatrixMul(const maniFbxMatrix* a, const maniFbxMatrix* b);

// The inverse; false when the matrix is singular or not finite.
bool maniFbxMatrixInverse(const maniFbxMatrix* a, maniFbxMatrix* out);

// The matrix rounded to binary32.
maniAffine maniFbxMatrixAffine(const maniFbxMatrix* a);

// The transform rounded to binary32.
maniTransform maniFbxTrsTransform(const maniFbxTrs* t);

// The inputs of a model's local transform, in the chain's order of
// application: vectors (translation, rotation offset and pivot, pre-,
// own and post-rotation in degrees, scaling offset and pivot, scaling)
// and integers (rotation order 0 to 6, rotation active, rotation space
// for limits only, inherit type 0 to 2), so named by FBX.
enum
{
    maniFbxLocalTranslation,
    maniFbxLocalRotationOffset,
    maniFbxLocalRotationPivot,
    maniFbxLocalPreRotation,
    maniFbxLocalRotation,
    maniFbxLocalPostRotation,
    maniFbxLocalScalingOffset,
    maniFbxLocalScalingPivot,
    maniFbxLocalScaling,
    maniFbxLocalVectors
};

enum
{
    maniFbxLocalOrder,
    maniFbxLocalActive,
    maniFbxLocalLimitOnly,
    maniFbxLocalInherit,
    maniFbxLocalIntegers
};

extern const char* const maniFbxLocalVectorNames[maniFbxLocalVectors];
extern const char* const maniFbxLocalIntegerNames[maniFbxLocalIntegers];
// Each integer's largest value (the least is 0).
extern const int64_t maniFbxLocalIntegerLimits[maniFbxLocalIntegers];

typedef struct maniFbxLocalInputs
{
    double vectors[maniFbxLocalVectors][3];
    int64_t integers[maniFbxLocalIntegers];
} maniFbxLocalInputs;

// A model's inputs from its properties (and its template's); refused
// as maniFbxModelLocal. A cache (NULL for none) kept for these reads
// finds a template shared by the models read once.
maniImportResult maniFbxReadLocalInputs(const maniFbx* fbx, uint32_t object,
                                        maniFbxTemplateCache* cache, maniFbxLocalInputs* out);

// Composes inputs whose integers are within their limits.
void maniFbxComposeLocal(const maniFbxLocalInputs* in, maniFbxLocal* out);

// A model's local transform from its properties (and its template's):
// Lcl translation, rotation and scaling with pivots, offsets, pre- and
// post-rotation and the rotation order; refused (Property, item: the
// object) for a value not finite or missing, or an order or inherit
// type out of range; a cache as maniFbxReadLocalInputs takes.
maniImportResult maniFbxModelLocal(const maniFbx* fbx, uint32_t object, maniFbxTemplateCache* cache,
                                   maniFbxLocal* out);

// A model's geometric transform: GeometricTranslation, XYZ
// GeometricRotation and GeometricScaling as T R S, moving only the
// geometry it carries; refused as maniFbxModelLocal.
maniImportResult maniFbxGeometricTransform(const maniFbx* fbx, uint32_t object, maniFbxTrs* out);

#endif // MAUL_ANIM_SRC_FBX_TRANSFORM_H
