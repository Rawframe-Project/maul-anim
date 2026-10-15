// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX animation stacks (mani-0016): a stack's layers in connection
// order, their curve nodes bound to the properties they drive, and a
// property's value at a time with every layer applied as ufbx applies
// them (research 24).

#ifndef MAUL_ANIM_SRC_FBX_ANIM_H
#define MAUL_ANIM_SRC_FBX_ANIM_H

#include "fbx_curve.h"
#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum
{
    maniFbxBlendAdditive = 0,
    maniFbxBlendOverride = 1,
    maniFbxBlendPassthrough = 2,
};

typedef struct maniFbxLayer
{
    uint32_t object;
    uint32_t blend;
    // Weight in [0, 1]: Weight / 100, clamped, 1 above 0.99999.
    double weight;
    // RotationAccumulationMode and ScaleAccumulationMode 0: rotations
    // composed as rotations, scales multiplied.
    bool composeRotation;
    bool composeScale;
} maniFbxLayer;

// A curve node's binding to one property it drives: the object, the
// string property naming the property (and its bytes' hash), the layer,
// the node, and per component a curve (none when its count is 0 and keys
// NULL) over a static value.
typedef struct maniFbxChannel
{
    uint64_t nameHash;
    uint32_t target;
    uint32_t name;
    uint32_t layer;
    uint32_t node;
    maniFbxCurve curves[3];
    double values[3];
} maniFbxChannel;

typedef struct maniFbxAnimation
{
    maniFbxLayer* layers;
    maniFbxChannel* channels;
    uint32_t layerCount;
    uint32_t channelCount;
    // The stack's range in ticks (mani-0016's order of sources).
    int64_t start;
    int64_t stop;
} maniFbxAnimation;

// Lists the AnimationStack objects in object order, with their names;
// Capacity when the allocator fails.
maniImportResult maniFbxReadStacks(maniFbx* fbx);
void maniFbxReleaseStacks(maniFbx* fbx);

// The stacks: their count, and the k-th (MANI_FBX_NONE past the count).
uint32_t maniFbxStackCount(const maniFbx* fbx);
uint32_t maniFbxStackObject(const maniFbx* fbx, uint32_t k);

// Reads a stack; refused (Property, item: the curve node) for a property
// driven twice in one layer, or as its curves are; Capacity when the
// allocator fails. Channels come sorted by target, then layer, then
// name hash.
maniImportResult maniFbxReadAnimation(const maniFbx* fbx, uint32_t stack, maniFbxAnimation* out);

void maniFbxReleaseAnimation(const maniFbx* fbx, maniFbxAnimation* animation);

// The run of channels driving an object: its first index and count.
uint32_t maniFbxAnimatedRun(const maniFbxAnimation* animation, uint32_t object, uint32_t* first);

// A property of an object at a time in seconds: value holds its static
// value and receives the animated one, each layer applied in order (the
// first sets, later ones combine by their weight and blend mode; a
// composed Lcl Rotation goes through quaternions in the order given and
// back). Whether any channel drives it.
bool maniFbxAnimateProperty(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t object,
                            const char* name, size_t length, double time, uint32_t order,
                            double value[3]);

#endif // MAUL_ANIM_SRC_FBX_ANIM_H
