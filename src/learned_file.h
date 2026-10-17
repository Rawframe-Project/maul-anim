// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The learned model file's layout (mani-0025), shared by its view, its
// evaluation and its builder: kind 9 of the container. Records are
// little-endian. A network's values and weights follow from its layers'
// shapes, so the file keeps the shapes and the values end to end:
// networks in order (the Decompressor, the Stepper, the Projectors),
// each network's input offsets and scales, output offsets and scales,
// then each layer's biases (and, for 16-bit weights, its inputs'
// scales); the weights likewise, layer by layer.

#ifndef MAUL_ANIM_SRC_LEARNED_FILE_H
#define MAUL_ANIM_SRC_LEARNED_FILE_H

#include <stdint.h>

#define MANI_LEARNED_FORMAT_VERSION 1u
#define MANI_LEARNED_SECTIONS       11u

enum
{
    // The skeleton's and the database's identities (u64 each), then u32
    // each: the bone, feature, channel, time, latent, scaled bone, float
    // track, event and Projector counts, the layer count over every
    // network, the weight format, the step rate (binary32), the up and
    // forward axes.
    maniLearnedTagHeader = 1,
    // MANI_LEARNED_CHANNEL_SIZE each: kind, bone, axis, time count, first
    // time, first feature, feature count (u32 each), weight (binary32).
    maniLearnedTagChannels = 2,
    // A binary32 each: the channels' times, channel by channel.
    maniLearnedTagTimes = 3,
    // A binary32 per feature: the database's offsets, then its scales.
    maniLearnedTagNormalization = 4,
    // The scaled bones (u32 each, increasing), then three binary32 scales
    // a bone.
    maniLearnedTagBones = 5,
    // MANI_LEARNED_NETWORK_SIZE each: layer count, required tags, excluded
    // tags, 0 (u32 each); the Decompressor's and Stepper's tags 0.
    maniLearnedTagNetworks = 6,
    // MANI_LEARNED_LAYER_SIZE each: input count, output count (u32 each).
    maniLearnedTagLayers = 7,
    // binary32 values, as the layout above.
    maniLearnedTagValues = 8,
    // The weights: binary32 or 16-bit (padded to 4 bytes).
    maniLearnedTagWeights = 9,
    // MANI_LEARNED_NAME_SIZE each: offset, length (u32 each): the float
    // tracks', then the events'.
    maniLearnedTagNames = 10,
    // The names, UTF-8, end to end.
    maniLearnedTagNameBytes = 11,
};

#define MANI_LEARNED_HEADER_SIZE  72u
#define MANI_LEARNED_CHANNEL_SIZE 32u
#define MANI_LEARNED_NETWORK_SIZE 16u
#define MANI_LEARNED_LAYER_SIZE   8u
#define MANI_LEARNED_NAME_SIZE    8u

// The networks before the Projectors: the Decompressor, the Stepper.
#define MANI_LEARNED_DECOMPRESSOR 0u
#define MANI_LEARNED_STEPPER      1u
#define MANI_LEARNED_PROJECTORS   2u

// The Decompressor's outputs: nine a bone, three a scaled bone, six for
// the root, one a float track and one an event.
static inline uint64_t maniLearnedOutputCount(uint32_t bones, uint32_t scaled, uint32_t tracks,
                                              uint32_t events)
{
    return 9 * (uint64_t)bones + 3 * (uint64_t)scaled + 6 + tracks + events;
}

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniLearnedComputeIdentity(const uint8_t* const data[MANI_LEARNED_SECTIONS],
                                    const uint64_t sizes[MANI_LEARNED_SECTIONS]);

#endif // MAUL_ANIM_SRC_LEARNED_FILE_H
