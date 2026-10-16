// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs (mani-0023): a graph file built offline against one skeleton,
// opened in place, bound to its clips, and played by instances in caller
// memory. Its nodes form a DAG stored in topological order, each after
// its inputs, the output last.

#ifndef MAUL_ANIM_GRAPH_H
#define MAUL_ANIM_GRAPH_H

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// What a lookup by name finds for no parameter, and a scalar's parameter
// when it is a literal.
#define MANI_NO_PARAMETER 0xFFFFFFFFu
// A clip node in no sync group.
#define MANI_NO_SYNC_GROUP 0xFFFFFFFFu
// Named limits: nodes, inputs of one node, parameters, clips and sync
// groups of one graph.
#define MANI_GRAPH_MAX_NODES       65536u
#define MANI_GRAPH_MAX_INPUTS      4096u
#define MANI_GRAPH_MAX_PARAMETERS  65536u
#define MANI_GRAPH_MAX_CLIPS       65536u
#define MANI_GRAPH_MAX_SYNC_GROUPS 65536u

    // A parameter's type. A trigger is set by the host and consumed by the
    // first node that reads it within an update.
    typedef uint32_t maniGraphParameterType;
    enum
    {
        maniGraphParameterBool = 0,
        maniGraphParameterInt = 1,
        maniGraphParameterFloat = 2,
        maniGraphParameterVec2 = 3,
        maniGraphParameterVec3 = 4,
        // Normalized when set.
        maniGraphParameterQuat = 5,
        maniGraphParameterTrigger = 6,
    };

    // A parameter's value: f for float (f[0]) and the vectors, i for int,
    // b (0 or 1) for bool and trigger.
    typedef union maniGraphValue
    {
        float f[4];
        int32_t i;
        uint32_t b;
    } maniGraphValue;

    // A node's kind.
    typedef uint32_t maniGraphNodeKind;
    enum
    {
        // Plays a clip; no inputs.
        maniGraphNodeClip = 1,
        // Blends its inputs by weights, normalized or direct.
        maniGraphNodeBlend = 2,
        // Its first input on the bones a mask weighs, its second on the
        // rest, blended per bone.
        maniGraphNodeMask = 3,
        // Its first input with each other, a layer of differences, added.
        maniGraphNodeAdditive = 4,
        // One of its inputs, chosen by an int or bool parameter.
        maniGraphNodeSelect = 5,
        // Its input with time scaled.
        maniGraphNodeTimeScale = 6,
        // Its input, its playheads set to a time when a trigger fires.
        maniGraphNodeSeek = 7,
    };

    // A graph over a file's bytes, which the caller keeps alive and
    // unchanged while the view is used. Its fields are the library's; read
    // a graph through the functions below. A view is immutable and
    // copyable.
    typedef struct maniGraph
    {
        const uint8_t* parameters;
        const uint8_t* nodes;
        const uint8_t* inputs;
        const uint8_t* data;
        const uint8_t* clips;
        const uint8_t* syncGroups;
        const uint8_t* maskWeights;
        const uint8_t* names;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint64_t namesSize;
        uint32_t skeletonBoneCount;
        uint32_t parameterCount;
        uint32_t nodeCount;
        uint32_t inputCount;
        uint32_t dataWords;
        uint32_t clipCount;
        uint32_t syncGroupCount;
        uint32_t maskCount;
        uint32_t slotCount;
        uint32_t reserved;
    } maniGraph;

    /// Opens a graph file in place: checks the container, every record,
    /// index, count, float and name, and that every node's inputs come
    /// before it.
    ///
    /// @param bytes     The file, 16-byte aligned.
    /// @param size      Its size in bytes; bytes past the file are ignored.
    /// @param graphOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenGraph(const void* bytes, size_t size,
                                                         maniGraph* graphOut);

    /// Returns the graph's identity, the hash of its contents.
    ///
    /// @param graph  An opened graph.
    /// @return The identity; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphIdentity(const maniGraph* graph);

    /// Returns the identity of the skeleton the graph was built against.
    ///
    /// @param graph  An opened graph.
    /// @return The identity; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphSkeletonIdentity(const maniGraph* graph);

    /// Returns the number of nodes.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphNodeCount(const maniGraph* graph);

    /// Returns the number of parameters.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphParameterCount(const maniGraph* graph);

    /// Finds a parameter by its name.
    ///
    /// @param graph   An opened graph.
    /// @param name    The name's bytes, UTF-8.
    /// @param length  Its length in bytes.
    /// @return The parameter, or MANI_NO_PARAMETER.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphFindParameter(const maniGraph* graph, const char* name,
                                             size_t length);

    /// Returns a parameter's type.
    ///
    /// @param graph      An opened graph.
    /// @param parameter  The parameter.
    /// @param typeOut    Receives the type.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// parameter past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniGraphParameterTypeOf(const maniGraph* graph,
                                                                uint32_t parameter,
                                                                maniGraphParameterType* typeOut);

    /// Returns a parameter's name, its bytes inside the file.
    ///
    /// @param graph      An opened graph.
    /// @param parameter  The parameter.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphParameterName(const maniGraph* graph, uint32_t parameter,
                                                uint32_t* lengthOut);

    /// Returns the number of clip slots the graph plays.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphClipCount(const maniGraph* graph);

    /// Returns the identity of the clip a slot names.
    ///
    /// @param graph  An opened graph.
    /// @param clip   The slot.
    /// @return The clip's identity; 0 past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphClipIdentity(const maniGraph* graph, uint32_t clip);

    /// Returns the name a slot gives its clip, its bytes inside the file.
    ///
    /// @param graph      An opened graph.
    /// @param clip       The slot.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphClipName(const maniGraph* graph, uint32_t clip,
                                           uint32_t* lengthOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_GRAPH_H
