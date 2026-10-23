// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The glTF document's parts: its JSON, its buffers, buffer
// views and accessors, shared by the modules that open and read it.

#ifndef MAUL_ANIM_SRC_GLTF_DOCUMENT_H
#define MAUL_ANIM_SRC_GLTF_DOCUMENT_H

#include "arena.h"
#include "gltf_property.h"
#include "hierarchy.h"
#include "json.h"
#include "names.h"

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// The most elements an accessor without a buffer view may have.
#define MANI_GLTF_MAX_IMPLICIT 16777216u

typedef struct maniGltfBuffer
{
    // The bytes, NULL while an external buffer waits for the host.
    const uint8_t* data;
    uint64_t byteLength;
    // Decoded `data:` bytes, owned by the document.
    uint8_t* owned;
    size_t ownedSize;
    // An external buffer's URI, decoded, owned by the document.
    char* uri;
    uint32_t uriLength;
    bool external;
    // Marked a meshopt fallback: 0, or 1 for EXT and 2 for KHR; never read.
    uint8_t fallback;
} maniGltfBuffer;

typedef struct maniGltfView
{
    uint32_t buffer;
    uint32_t stride;
    uint64_t offset;
    uint64_t length;
    // A meshopt stream (gltf_compression.h): its buffer (MANI_JSON_NONE
    // for none), range, element count and stride, mode, filter, and
    // whether KHR's; the decoded bytes, owned, length of them, NULL until
    // the stream's buffer arrives.
    uint32_t source;
    uint32_t count;
    uint64_t sourceOffset;
    uint64_t sourceLength;
    uint8_t* decoded;
    uint16_t elementStride;
    uint8_t mode;
    uint8_t filter;
    bool khr;
} maniGltfView;

// glTF's component types, in the order of their codes 5120 to 5126.
typedef enum maniGltfComponent
{
    maniGltfByte = 0,
    maniGltfUnsignedByte = 1,
    maniGltfShort = 2,
    maniGltfUnsignedShort = 3,
    maniGltfUnsignedInt = 5,
    maniGltfFloat = 6,
} maniGltfComponent;

typedef struct maniGltfAccessor
{
    // MANI_JSON_NONE for zeros.
    uint32_t view;
    uint32_t count;
    uint64_t offset;
    uint8_t component;
    uint8_t componentSize;
    // Rows and columns: 1 x 1 to 4 x 1 for SCALAR to VEC4, n x n for
    // MATn; columns of 1- and 2-byte components start at multiples of 4.
    uint8_t rows;
    uint8_t columns;
    uint32_t columnStride;
    uint32_t elementSize;
    bool normalized;
    // Sparse substitution, when count is above 0.
    uint32_t sparseCount;
    uint32_t sparseIndexView;
    uint32_t sparseValueView;
    uint8_t sparseIndexSize;
    uint64_t sparseIndexOffset;
    uint64_t sparseValueOffset;
    // The accessor's JSON value, for offsets.
    uint32_t value;
} maniGltfAccessor;

struct maniGltf
{
    maniAllocator allocator;
    maniJson json;
    maniGltfJson g;
    // The GLB's BIN chunk, if any.
    const uint8_t* bin;
    uint64_t binSize;
    bool isGlb;
    maniGltfBuffer* buffers;
    maniGltfView* views;
    maniGltfAccessor* accessors;
    uint32_t bufferCount;
    uint32_t viewCount;
    uint32_t accessorCount;
    // Nodes: their JSON values, parents (MANI_JSON_NONE for roots), and
    // the forest they make (depths, common ancestors).
    uint32_t* nodeValues;
    uint32_t* nodeParents;
    maniHierarchy hierarchy;
    uint32_t nodeCount;
    // Skins: their JSON values, common roots and skeletons.
    uint32_t* skinValues;
    uint32_t* skinRoots;
    uint32_t* skinSkeletons;
    uint32_t skinCount;
    // Skeletons: each one's root node.
    uint32_t* skeletonRoots;
    uint32_t skeletonCount;
    // Every node's bone name: node v's is text[starts[v]] to
    // text[starts[v + 1]].
    maniBoneNames names;
    // Animations: their JSON values and names (empty when unnamed); all
    // their samplers' values end to end, animation a's from
    // samplerStarts[a]; meshes' values.
    uint32_t* animationValues;
    uint32_t animationCount;
    maniBoneNames animationNames;
    uint32_t* samplerValues;
    uint32_t* samplerStarts;
    uint32_t samplerTotal;
    // Every channel as the open read it, animation a's from
    // channelStarts[a].
    struct maniGltfChannel* channels;
    uint32_t* channelStarts;
    uint32_t channelTotal;
    uint32_t* meshValues;
    uint32_t meshCount;
    // What maniGltfReadSprings last gave.
    maniArena springs;
    // What maniGltfReadRagdoll last gave.
    maniArena ragdoll;
};

// Whether an accessor of non-vertex data reads as packed: from no view,
// a view without a stride, or one whose stride is its element size,
// which reads the same bytes.
static inline bool maniGltfPacked(const maniGltf* gltf, const maniGltfAccessor* a)
{
    return a->view == MANI_JSON_NONE || gltf->views[a->view].stride == 0 ||
           gltf->views[a->view].stride == a->elementSize;
}

#endif // MAUL_ANIM_SRC_GLTF_DOCUMENT_H
