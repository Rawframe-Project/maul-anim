// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The influences file's layout, shared by its view and its
// builder: kind 6 of the container.

#ifndef MAUL_ANIM_SRC_INFLUENCE_FILE_H
#define MAUL_ANIM_SRC_INFLUENCE_FILE_H

#include <stdint.h>

#define MANI_INFLUENCE_FORMAT_VERSION 2u
#define MANI_INFLUENCE_SECTIONS       5u
// The skin's identity (u64), its joint count, the vertex count, the group
// count (u32 each).
#define MANI_INFLUENCE_BINDING_SIZE 20u
// Influences come in groups of four a vertex; at most this many groups.
#define MANI_INFLUENCE_GROUP      4u
#define MANI_INFLUENCE_MAX_GROUPS 16u

enum
{
    maniInfluenceTagBinding = 1,
    // u16 per influence, a joint of the skin: vertex by vertex, each
    // vertex's groups of four in turn.
    maniInfluenceTagJoints = 2,
    // binary32 per influence, in the joints' order: decreasing, summing
    // to 1 or all 0.
    maniInfluenceTagWeights = 3,
    // binary32 per vertex, 0 (linear) to 1 (dual quaternion); absent for
    // none.
    maniInfluenceTagDualQuaternionWeights = 4,
    // Three binary32 per vertex, the center of rotation; absent for none.
    maniInfluenceTagCenters = 5,
};

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniInfluenceComputeIdentity(const uint8_t* const data[MANI_INFLUENCE_SECTIONS],
                                      const uint64_t sizes[MANI_INFLUENCE_SECTIONS]);

#endif // MAUL_ANIM_SRC_INFLUENCE_FILE_H
