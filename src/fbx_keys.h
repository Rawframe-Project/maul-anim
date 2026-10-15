// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX 6.x keyed channels (research 26): a take channel's `Key` list read
// into a curve as ufbx reads it, evaluated as every other curve is.

#ifndef MAUL_ANIM_SRC_FBX_KEYS_H
#define MAUL_ANIM_SRC_FBX_KEYS_H

#include "fbx_curve.h"
#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

// Reads a channel node's keys into a curve, none without a Key child;
// refused (Property, at the node, item given) for KeyCount missing or
// past what the values hold, a key, slope or weight mode ufbx does not
// know, values left over or missing, times not whole ticks or
// decreasing; Data for values not finite; Capacity when the allocator
// fails.
maniImportResult maniFbxReadKeys(const maniFbx* fbx, uint32_t node, uint32_t item,
                                 maniFbxCurve* out);

#endif // MAUL_ANIM_SRC_FBX_KEYS_H
