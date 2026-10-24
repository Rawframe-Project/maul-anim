// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ufbx differential's shared state and helpers: the file
// being compared, and what fbx_differential.c and
// fbx_differential_mesh.c both use.

#ifndef MAUL_ANIM_TOOLS_FBX_DIFFERENTIAL_H
#define MAUL_ANIM_TOOLS_FBX_DIFFERENTIAL_H

#include "ufbx.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

enum
{
    kMaxDeviations = 256,
    kTimes = 5,
    kMaxRoots = 256,
};

typedef struct Deviation
{
    char file[128];
    char aspect[32];
} Deviation;

// A built file, kept aligned for opening.
typedef struct Blob
{
    void* raw;
    void* data;
    size_t size;
} Blob;

typedef struct Context
{
    const char* path;
    uint32_t file;
    bool expect;
    // The expected values so far, while expecting.
    FILE* out;
    const Deviation* deviations;
    uint32_t deviationCount;
    uint32_t mismatches;
    uint32_t deviated;
    // The skins (influences and skinned points at rest) and morph target
    // sets compared whole.
    uint32_t skinsCompared;
    uint32_t morphsCompared;
    // Skins compared at a clip's sample time, and those dual quaternion
    // or blended.
    uint32_t posedCompared;
    uint32_t posedDualQuaternion;
    ufbx_scene* scene;
    maniBuilder* builder;
    maniFbx* fbx;
    maniSkeleton skeleton;
    Blob skeletonBlob;
    maniAffine placement;
    bool hasSkeleton;
    // With no skeleton, the subtrees of the top models (by ufbx's
    // hierarchy) stand in for skeletons.
    bool byModel;
    uint32_t sourceCount;
    uint32_t roots[kMaxRoots];
} Context;

void Release(Blob* blob);

// Copies the builder's file into an aligned blob.
bool Copy(maniBuilder* builder, Blob* blob);

// Reports a mismatch of an aspect, or a deviation where the deviations
// list the file and aspect.
void Mismatch(Context* c, const char* aspect, const char* format, ...);

// a within 1e-4 of b's size (at least 1).
bool Near(double a, double b);

void FromAffine(const maniAffine* a, double out[12]);

// a times b, both 3x4 column-major affine matrices.
void Compose(const double a[12], const double b[12], double out[12]);

// A pose of the skeleton at rest, its memory into raw (freed by the
// caller); NULL when it fails.
maniPose* NewPose(const maniSkeleton* skeleton, void** raw);

// Source s's placement (skeleton, or model subtree).
maniImportResult Placement(const Context* c, uint32_t s, const maniFbxOptions* options,
                           maniAffine* out);

// The ufbx skin deformer Maul Anim's skin k stands for, and its mesh node.
ufbx_skin_deformer* TheirSkin(const Context* c, uint32_t k, const ufbx_node** meshOut);

// Whether a channel's shape k has a track name.
bool ShapeNamed(const ufbx_blend_channel* channel, size_t k, const char* name, size_t length);

// Skin k's influences, blend weights and skinned control points at rest
// against ufbx's (fbx_differential_mesh.c).
void CompareInfluences(Context* c, uint32_t k, uint32_t s, const maniSkin* skin,
                       const maniSkeleton* skeleton);

// The clip skeleton's skins skinned at a sampled pose against the
// evaluated scene's (dual quaternion and blended skins under their own
// aspect, skinning-dq).
void CompareSkinsAt(Context* c, uint32_t clip, const maniPose* pose, const ufbx_scene* evaluated,
                    double time);

// Every set of morph targets against ufbx's blend deformers.
void CompareMorphs(Context* c);

#endif // MAUL_ANIM_TOOLS_FBX_DIFFERENTIAL_H
