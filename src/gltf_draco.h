// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A Draco mesh (KHR_draco_mesh_compression, mani-0021) decoded from its
// bitstream 2.2 bytes: faces of point ids, and each attribute's values
// with the point each one belongs to, as Google's decoder gives them.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_H
#define MAUL_ANIM_SRC_GLTF_DRACO_H

#include "arena.h"
#include "gltf_draco_read.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Draco's attribute types and data types, as the bitstream numbers them.
typedef enum maniDracoType
{
    maniDracoPosition = 0,
    maniDracoNormal = 1,
    maniDracoColor = 2,
    maniDracoTexCoord = 3,
    maniDracoGeneric = 4,
} maniDracoType;

typedef enum maniDracoDataType
{
    maniDracoInt8 = 1,
    maniDracoUInt8 = 2,
    maniDracoInt16 = 3,
    maniDracoUInt16 = 4,
    maniDracoInt32 = 5,
    maniDracoUInt32 = 6,
    maniDracoInt64 = 7,
    maniDracoUInt64 = 8,
    maniDracoFloat32 = 9,
    maniDracoFloat64 = 10,
    maniDracoBool = 11,
} maniDracoDataType;

typedef struct maniDracoAttribute
{
    uint32_t uniqueId;
    uint8_t type;
    uint8_t dataType;
    uint8_t components;
    bool normalized;
    // valueCount values of components elements of dataType, little-endian.
    uint32_t valueCount;
    uint8_t* values;
    // Each point's value; NULL when point i holds value i.
    const uint32_t* pointToValue;
} maniDracoAttribute;

typedef struct maniDracoMesh
{
    uint32_t faceCount;
    uint32_t pointCount;
    // 3 point ids a face.
    uint32_t* faces;
    uint32_t attributeCount;
    maniDracoAttribute* attributes;
} maniDracoMesh;

// The most components an attribute may have: glTF's MAT4.
#define MANI_DRACO_MAX_COMPONENTS 16u

// A data type's size in bytes, 0 for none.
uint32_t maniDracoTypeSize(uint8_t dataType);

// Decodes a Draco mesh of at most maxPoints points from size bytes,
// everything it holds taken from the arena (released by the caller).
maniDracoStatus maniDracoDecode(maniArena* arena, const uint8_t* in, size_t size,
                                uint32_t maxPoints, maniDracoMesh* out);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_H
