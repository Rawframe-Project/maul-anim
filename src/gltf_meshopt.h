// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF's meshopt compression (EXT_meshopt_compression and
// KHR_meshopt_compression, mani-0008): a buffer view's three stream
// modes decoded and its filters applied, as the KHR specification's
// Appendix A and B give them.

#ifndef MAUL_ANIM_SRC_GLTF_MESHOPT_H
#define MAUL_ANIM_SRC_GLTF_MESHOPT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// A stream's mode, as the extension names them.
typedef enum maniMeshoptMode
{
    maniMeshoptAttributes = 0,
    maniMeshoptTriangles = 1,
    maniMeshoptIndices = 2,
} maniMeshoptMode;

// A filter applied after decoding attributes.
typedef enum maniMeshoptFilter
{
    maniMeshoptNone = 0,
    maniMeshoptOctahedral = 1,
    maniMeshoptQuaternion = 2,
    maniMeshoptExponential = 3,
    maniMeshoptColor = 4,
} maniMeshoptFilter;

// Decodes a stream of size bytes into count elements of stride bytes at
// out (little-endian; indices 2 or 4 bytes): attributes of version 0,
// and of version 1 when v1 is true (KHR only); triangles; indices. The
// stride and count are the extension's, already checked against its
// rules. False for a stream the specification calls invalid (a wrong
// header, too short or too long, a FIFO entry never written, a channel
// mode above 2, a malformed code table or LEB128 value); out is then
// partly written.
bool maniMeshoptDecode(uint8_t* out, uint32_t count, uint32_t stride, maniMeshoptMode mode, bool v1,
                       const uint8_t* in, size_t size);

// Applies a filter in place to count elements of stride bytes, the
// specification's formulas in binary32 in their order (rounding halfway
// away from zero), so its bits are the same everywhere.
void maniMeshoptApplyFilter(uint8_t* data, uint32_t count, uint32_t stride,
                            maniMeshoptFilter filter);

#endif // MAUL_ANIM_SRC_GLTF_MESHOPT_H
