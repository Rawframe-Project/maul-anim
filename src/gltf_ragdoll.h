// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The OMI physics extensions as a ragdoll source (mani-0027): the reader
// the bodies' and the joints' halves share, and the joints' half.

#ifndef MAUL_ANIM_SRC_GLTF_RAGDOLL_H
#define MAUL_ANIM_SRC_GLTF_RAGDOLL_H

#include "gltf_document.h"
#include "gltf_property.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

// A node's body when that body is not the skeleton's (no bone holds it).
#define MANI_GLTF_NOT_OURS (UINT32_MAX - 1u)

// A rigid frame: a translation and a unit rotation.
typedef struct maniGltfRigid
{
    maniVec3 t;
    maniQuat r;
} maniGltfRigid;

typedef struct maniGltfRagdollReader
{
    maniGltf* gltf;
    const maniSkeleton* skeleton;
    const maniGltfOptions* options;
    // The bodies read so far, in node order; body i's motion node, its
    // bone's node and its collider node's frame in the world.
    maniRagdollBodySource* bodies;
    uint32_t* bodyNodes;
    uint32_t* boneNodes;
    maniGltfRigid* bodyWorld;
    uint32_t bodyCount;
    // Each node's body: the index of the body whose motion node is the
    // node or its nearest ancestor with a motion, MANI_GLTF_NOT_OURS for
    // one no bone holds, or UINT32_MAX for none.
    uint32_t* nodeBody;
    // A node's world matrix's chain, nodeCount entries of room.
    uint32_t* chain;
    // The document-level `physicsJoints` array, or MANI_JSON_NONE.
    uint32_t settings;
    uint32_t settingsCount;
} maniGltfRagdollReader;

maniImportResult maniGltfRagdollFail(const maniGltfRagdollReader* r, maniImportReason reason,
                                     uint32_t value, uint32_t item);

// A node's frame in the world (the output's basis), its scale in
// *scaleOut when not NULL.
maniImportResult maniGltfRagdollWorld(const maniGltfRagdollReader* r, uint32_t node,
                                      maniGltfRigid* frameOut, maniVec3* scaleOut);

// b's frame in a's.
maniGltfRigid maniGltfRigidIn(maniGltfRigid a, maniGltfRigid b);

// A node's `OMI_physics_*` extension object, or MANI_JSON_NONE.
maniImportResult maniGltfRagdollExtension(const maniGltfRagdollReader* r, uint32_t object,
                                          const char* name, uint32_t item, uint32_t* out);

// A number member, its default when absent; not finite refused.
maniImportResult maniGltfRagdollNumber(const maniGltfRagdollReader* r, uint32_t object,
                                       const char* name, uint32_t item, float* out);

// Every joint node's joint onto its child body; each body but the root
// with exactly one.
maniImportResult maniGltfReadRagdollJoints(maniGltfRagdollReader* r);

#endif // MAUL_ANIM_SRC_GLTF_RAGDOLL_H
