// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX models at a time of a stack (mani-0016): each needed model's local
// inputs, as its properties give them, overridden by what the stack
// animates, composed and evaluated parents first as at rest (mani-0014).

#ifndef MAUL_ANIM_SRC_FBX_SAMPLE_H
#define MAUL_ANIM_SRC_FBX_SAMPLE_H

#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_transform.h"
#include "fbx_world.h"

#include "maul-anim/import.h"

#include <stdint.h>

typedef struct maniFbxSampler
{
    const maniFbx* fbx;
    const maniFbxAnimation* animation;
    // The needed models, parents first, and their inputs at rest.
    uint32_t* order;
    maniFbxLocalInputs* rest;
    uint32_t count;
    uint32_t reserved;
} maniFbxSampler;

// Prepares to sample the models marked in needed (a byte per model; a
// needed model's parent is needed too); refused as maniFbxModelLocal,
// Capacity when the allocator fails.
maniImportResult maniFbxStartSampler(const maniFbx* fbx, const maniFbxAnimation* animation,
                                     const uint8_t* needed, maniFbxSampler* out);

void maniFbxReleaseSampler(maniFbxSampler* sampler);

// The needed models' worlds at a time in seconds, into worlds (one per
// model); refused (Property, item: the model) for an animated integer
// input not whole or out of its range.
maniImportResult maniFbxSampleWorlds(const maniFbxSampler* sampler, double time,
                                     maniFbxWorld* worlds);

#endif // MAUL_ANIM_SRC_FBX_SAMPLE_H
