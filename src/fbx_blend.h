// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX blend shape channels (mani-0017): a
// channel's mesh model, its shapes ordered by target weight, its static
// weight, and each shape's effective weight at a channel weight, as
// ufbx computes them.

#ifndef MAUL_ANIM_SRC_FBX_BLEND_H
#define MAUL_ANIM_SRC_FBX_BLEND_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct maniFbxBlendShape
{
    uint32_t object;
    uint32_t reserved;
    // The channel weight at which the shape is whole.
    double target;
} maniFbxBlendShape;

// Whether an object is a blend shape channel.
bool maniFbxIsBlendChannel(const maniFbx* fbx, uint32_t object);

// A channel's mesh model: its BlendShape deformer's, or MANI_FBX_NONE.
uint32_t maniFbxBlendChannelModel(const maniFbx* fbx, uint32_t channel);

// A channel's shapes (Shape geometries in connection order), counted
// with shapes NULL, else read with their targets (FullWeights / 100, 1
// past the array or in files of Blender's binary exporter) and stably
// sorted by them; Data for a FullWeights value not finite, Capacity when
// the allocator fails.
maniImportResult maniFbxReadBlendShapes(const maniFbx* fbx, uint32_t channel,
                                        maniFbxBlendShape* shapes, uint32_t* countOut);

// A channel's DeformPercent at rest: its property, or Blender's child
// node of that name where the channel has no properties (100 when
// unreadable), else 0.
double maniFbxBlendRestPercent(const maniFbx* fbx, uint32_t channel);

// Each shape's effective weight at a channel weight (DeformPercent /
// 100), into effective (one per shape).
void maniFbxBlendWeights(const maniFbxBlendShape* shapes, uint32_t count, double weight,
                         double* effective);

// Before 7.2 (mani-0017), whether an object holds blend shapes as
// Shape children: a 6.x Model of subclass Mesh (or of none, holding
// Vertices and PolygonVertexIndex), or a 7.0 or 7.1 Geometry of
// subclass Mesh. Each Shape is a channel of its name with one shape at
// target 1, weighted by the owner's property of that name.
bool maniFbxIsShapeOwner(const maniFbx* fbx, uint32_t object);

// A shape owner's mesh model, as an index into fbx->models: the owner
// (6.x) or its geometry's model; MANI_FBX_NONE for none.
uint32_t maniFbxShapeOwnerModel(const maniFbx* fbx, uint32_t owner);

#endif // MAUL_ANIM_SRC_FBX_BLEND_H
