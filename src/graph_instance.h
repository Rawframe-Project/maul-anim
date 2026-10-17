// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's binding and instances as they lie in caller memory
// (mani-0023), shared by binding, instances, update and evaluation.

#ifndef MAUL_ANIM_SRC_GRAPH_INSTANCE_H
#define MAUL_ANIM_SRC_GRAPH_INSTANCE_H

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
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
// After them, every state machine's block, a little-endian word each.
#define MANI_GRAPH_VALUE_SIZE      16u
#define MANI_GRAPH_NODE_STATE_SIZE 16u

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
    // The most inputs a node takes: the layers evaluation lists at once.
    uint32_t maxInputs;
    // The clip each slot binds, in slot order; then the database each
    // database slot binds, read by maniGraphBoundDatabase; then the model
    // each model slot binds, read by maniGraphBoundModel.
    const maniClip* clips[];
};

// The databases a binding's database slots bind, in slot order: after its
// clips.
static inline const maniMotionDatabase** maniGraphBoundDatabases(maniGraphBinding* binding)
{
    return (const maniMotionDatabase**)(uint8_t*)(binding->clips + binding->graph.clipCount);
}

static inline const maniMotionDatabase* maniGraphBoundDatabase(const maniGraphBinding* binding,
                                                               uint32_t slot)
{
    const uint8_t* at = (const uint8_t*)(binding->clips + binding->graph.clipCount);
    return ((const maniMotionDatabase* const*)at)[slot];
}

// The models a binding's model slots bind, in slot order: after its
// databases.
static inline const maniLearnedModel** maniGraphBoundModels(maniGraphBinding* binding)
{
    return (const maniLearnedModel**)(uint8_t*)(maniGraphBoundDatabases(binding) +
                                                binding->graph.databaseCount);
}

static inline const maniLearnedModel* maniGraphBoundModel(const maniGraphBinding* binding,
                                                          uint32_t slot)
{
    const uint8_t* at = (const uint8_t*)(binding->clips + binding->graph.clipCount);
    at += (size_t)binding->graph.databaseCount * sizeof(const maniMotionDatabase*);
    return ((const maniLearnedModel* const*)at)[slot];
}

// A node's flags during an update: its time factor set from a parent, a
// seek above it firing (to a normalized time with phase), and a clip's
// events listed.
#define MANI_GRAPH_REACHED 1u
#define MANI_GRAPH_SEEKING 2u
#define MANI_GRAPH_PHASE   4u
#define MANI_GRAPH_LISTED  8u

// A node's state: its effective weight, a word its kind keeps (a
// select's input) and its playhead in seconds (a clip's), all saved; and
// for the update in progress its root motion weight, time factor, flags
// and the time a seek above it sets; a clip's playhead before the update
// and how far it moved (0 for a seek), kept until the next; the nearest
// inertialize node above it on its first parent's path (UINT32_MAX for
// none).
typedef struct maniGraphNodeState
{
    float weight;
    uint32_t word;
    double time;
    float rootWeight;
    float factor;
    uint32_t flags;
    float seekTo;
    double from;
    double step;
    uint32_t inertia;
    uint32_t reserved;
} maniGraphNodeState;

// A sync group's flags during an update: its leader declared, its
// leader seeking, the group synced by markers.
#define MANI_GRAPH_DECLARED       1u
#define MANI_GRAPH_LEADER_SEEKING 2u
#define MANI_GRAPH_MARKERS        4u

// A sync group during an update (not saved): its leader, and the leader's
// phase (its playhead over its duration) before the update and how far
// the update moves it; by markers, its playhead and step in seconds.
typedef struct maniGraphGroupState
{
    uint32_t leader;
    uint32_t flags;
    double phase;
    double delta;
} maniGraphGroupState;

// A cell of an instance: a parameter's value, a node's state or a sync
// group's.
typedef union maniGraphCell
{
    maniGraphValue value;
    maniGraphNodeState node;
    maniGraphGroupState group;
} maniGraphCell;

// The instance's cells follow it, a value per parameter, a state per
// node, a state per sync group; then, at cursorsAt, a cursor per clip
// node in node order; then, at machinesAt, every state machine's block
// (the graph's machineWords words).
struct maniGraphInstance
{
    uint32_t stamp;
    uint32_t clipNodeCount;
    const maniGraphBinding* binding;
    size_t size;
    size_t cursorsAt;
    size_t machinesAt;
    maniGraphCell cells[];
};

#endif // MAUL_ANIM_SRC_GRAPH_INSTANCE_H
