// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's binding and instances as they lie in caller memory
// (mani-0023), shared by binding, instances, update and evaluation.

#ifndef MAUL_ANIM_SRC_GRAPH_INSTANCE_H
#define MAUL_ANIM_SRC_GRAPH_INSTANCE_H

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

// Stamps a binding and an instance carry while they are made.
#define MANI_GRAPH_BINDING_STAMP  0x4D474244u
#define MANI_GRAPH_INSTANCE_STAMP 0x4D474953u
// A saved state's header: "MGST", 0, the graph's identity (u64), the
// state's size (u64), then 16 bytes per parameter and per node.
#define MANI_GRAPH_STATE_MAGIC       0x5453474Du
#define MANI_GRAPH_STATE_HEADER_SIZE 24u
#define MANI_GRAPH_VALUE_SIZE        16u
#define MANI_GRAPH_NODE_STATE_SIZE   16u

struct maniGraphBinding
{
    uint32_t stamp;
    uint32_t reserved;
    maniGraph graph;
    const maniSkeleton* skeleton;
    // The bytes each clip node's cursor takes (a multiple of 8).
    size_t cursorSize;
    // The bone the clips' root motion names (MANI_NO_BONE for none).
    uint32_t rootMotionBone;
    uint32_t reserved2;
    // The clip each slot binds, in slot order.
    const maniClip* clips[];
};

// A node's flags during an update: its time factor set from a parent,
// and a seek above it firing.
#define MANI_GRAPH_REACHED 1u
#define MANI_GRAPH_SEEKING 2u

// A node's state: its effective weight, a word its kind keeps (a
// select's input) and its playhead in seconds (a clip's), all saved; and
// for the update in progress its root motion weight, time factor, flags
// and the time a seek above it sets.
typedef struct maniGraphNodeState
{
    float weight;
    uint32_t word;
    double time;
    float rootWeight;
    float factor;
    uint32_t flags;
    float seekTo;
} maniGraphNodeState;

// A cell of an instance: a parameter's value or a node's state.
typedef union maniGraphCell
{
    maniGraphValue value;
    maniGraphNodeState node;
} maniGraphCell;

// The instance's cells follow it, a value per parameter then a state per
// node; then, at cursorsAt, a cursor per clip node in node order.
struct maniGraphInstance
{
    uint32_t stamp;
    uint32_t clipNodeCount;
    const maniGraphBinding* binding;
    size_t size;
    size_t cursorsAt;
    maniGraphCell cells[];
};

#endif // MAUL_ANIM_SRC_GRAPH_INSTANCE_H
