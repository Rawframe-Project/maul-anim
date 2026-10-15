// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's animations (mani-0010): every channel checked at
// open, read the same way when a clip is built.

#ifndef MAUL_ANIM_SRC_GLTF_ANIMATION_H
#define MAUL_ANIM_SRC_GLTF_ANIMATION_H

#include "gltf_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

typedef enum maniGltfTarget
{
    // No node and no pointer: the specification says to ignore it.
    maniGltfTargetIgnored = 0,
    maniGltfTargetTranslation = 1,
    maniGltfTargetRotation = 2,
    maniGltfTargetScale = 3,
    maniGltfTargetWeights = 4,
    // A `KHR_animation_pointer` pointer to anything but a node's TRS or
    // weights.
    maniGltfTargetPointer = 5,
} maniGltfTarget;

typedef struct maniGltfChannel
{
    maniGltfTarget target;
    uint32_t node;
    uint32_t input;
    uint32_t output;
    // maniInterpolation.
    uint32_t interpolation;
    uint32_t keys;
    // Floats a key's value has: 3, 4, the morph target count, or the
    // pointer's components.
    uint32_t width;
    // The pointer string's JSON value, for pointer targets.
    uint32_t pointer;
    // The channel's JSON value, for offsets.
    uint32_t value;
} maniGltfChannel;

// Reads and checks `animations` at open: samplers, channels, their
// accessors' types and counts, targets used twice, names.
maniImportResult maniGltfReadAnimations(maniGltf* gltf);

void maniGltfReleaseAnimations(maniGltf* gltf);

// An animation's channels array value and count.
uint32_t maniGltfAnimationChannels(const maniGltf* gltf, uint32_t animation, uint32_t* countOut);

// A channel (checked at open) of an animation.
maniImportResult maniGltfReadChannel(const maniGltf* gltf, uint32_t animation, uint32_t channel,
                                     uint32_t item, maniGltfChannel* out);

// A mesh's morph target count: that of its first primitive with targets
// (the specification's rule); 0 for none.
uint32_t maniGltfMeshTargets(const maniGltf* gltf, uint32_t mesh);

// A node's morph target count: its mesh's; 0 for none.
uint32_t maniGltfMorphTargets(const maniGltf* gltf, uint32_t node);

// The mesh's extras.targetNames, when it holds a non-empty string for each
// of its targets; MANI_JSON_NONE otherwise. Morph channels and weight
// tracks are named by them, or by the target's index.
uint32_t maniGltfTargetNames(const maniGltf* gltf, uint32_t mesh, uint32_t targets);

#endif // MAUL_ANIM_SRC_GLTF_ANIMATION_H
