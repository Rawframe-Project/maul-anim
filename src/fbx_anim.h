// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX animation stacks (mani-0016): a stack's layers in connection
// order, their curve nodes bound to the properties they drive, and a
// property's value at a time with every layer applied as ufbx applies
// them (mani-0016).

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

// A binding to one property a stack drives: the object, the property's
// name (and its bytes' hash), the layer, and per component a curve (none
// when its count is 0 and keys NULL) over a static value. A 7.x curve
// node's name is a string property of the file; a 6.x take's Transform
// channels name the Lcl properties by a literal. A 6.x property that is
// not the model's own belongs to its attribute (mani-0017). Refusals
// point at the file node source, with item the curve node (7.x) or the
// target (6.x).
typedef struct maniFbxChannel
{
    uint64_t nameHash;
    uint32_t target;
    uint32_t name;
    const char* literal;
    uint32_t literalLength;
    uint32_t layer;
    uint32_t item;
    uint32_t source;
    bool attribute;
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
    // Channel names decoded with their " (Shape)" suffix cut (before 6.0,
    // mani-0017), which those channels' literals point into; empty in
    // later files.
    uint8_t* names;
    size_t namesSize;
} maniFbxAnimation;

// Lists the stacks with their names: the AnimationStack objects in
// object order (7.x), or the top-level Takes' Take nodes in file order
// (6.x); Capacity when the allocator fails.
maniImportResult maniFbxReadStacks(maniFbx* fbx);
void maniFbxReleaseStacks(maniFbx* fbx);

// The stacks: their count, and the k-th's object (7.x) or Take node
// (6.x), MANI_FBX_NONE past the count.
uint32_t maniFbxStackCount(const maniFbx* fbx);
uint32_t maniFbxStackSource(const maniFbx* fbx, uint32_t k);

// Reads an AnimationStack object; refused (Property, item: the curve node) for a property
// driven twice in one layer, or as its curves are; Capacity when the
// allocator fails. Channels come sorted by target, then layer, then
// name hash.
maniImportResult maniFbxReadAnimation(const maniFbx* fbx, uint32_t stack, maniFbxAnimation* out);

void maniFbxReleaseAnimation(const maniFbx* fbx, maniFbxAnimation* animation);

// Sorts read channels by target, layer and name hash; refused (Property,
// at the source, the item) for a property driven twice in one layer;
// Capacity when the allocator fails.
maniImportResult maniFbxSortChannels(const maniFbx* fbx, maniFbxAnimation* animation);

// A channel's name: whether it is the bytes given; whether two channels
// share it; it decoded into out (the bytes it may need with out NULL).
bool maniFbxChannelNameIs(const maniFbx* fbx, const maniFbxChannel* channel, const char* name,
                          size_t length);
bool maniFbxSameChannelName(const maniFbx* fbx, const maniFbxChannel* a, const maniFbxChannel* b);
size_t maniFbxChannelName(const maniFbx* fbx, const maniFbxChannel* channel, uint8_t* out,
                          size_t capacity);

// A Take node's two times under a child name (LocalTime,
// ReferenceTime), both whole numbers.
bool maniFbxTakeTimes(const maniFbx* fbx, uint32_t take, const char* name, int64_t* a, int64_t* b);

// The first and last key's ticks over every curve, 0 and 0 for none.
void maniFbxKeySpan(const maniFbxAnimation* animation, int64_t* start, int64_t* stop);

// The run of channels driving an object: its first index and count.
uint32_t maniFbxAnimatedRun(const maniFbxAnimation* animation, uint32_t object, uint32_t* first);

// A property of an object at a time in seconds: value holds its static
// value and receives the animated one, each layer applied in order (the
// first sets, later ones combine by their weight and blend mode; a
// composed Lcl Rotation goes through quaternions in the order given, 0
// to 5, and back). Whether any channel drives it. A 6.x model's attribute's
// properties are animated through maniFbxAnimateAttribute.
bool maniFbxAnimateProperty(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t object,
                            const char* name, size_t length, double time, uint32_t order,
                            double value[3]);
// maniFbxAnimateProperty through the object's run already found
// (maniFbxAnimatedRun), its channels' names already matched: those
// whose mark (one a channel of the animation) is the one given, the
// property scale (Lcl Scaling) or rotation (Lcl Rotation) as said.
bool maniFbxAnimateMarked(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t first,
                          uint32_t count, const uint8_t* marks, uint8_t mark, bool scale,
                          bool rotation, double time, uint32_t order, double value[3]);
bool maniFbxAnimateAttribute(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t model,
                             const char* name, size_t length, double time, double value[3]);

#endif // MAUL_ANIM_SRC_FBX_ANIM_H
