// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a graph file (mani-0023), offline: a graph's parameters, clip
// slots, sync groups and nodes, checked against one skeleton and its
// clips, ordered so each node follows its inputs, its poses given slots,
// and written.

#ifndef MAUL_ANIM_GRAPHBUILD_H
#define MAUL_ANIM_GRAPHBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A number a node takes: the value of a float parameter, or a literal
    // value when parameter is MANI_NO_PARAMETER.
    typedef struct maniGraphScalar
    {
        uint32_t parameter;
        float value;
    } maniGraphScalar;

    // A parameter: its name (UTF-8, not empty, unique in the graph), type,
    // default and, for int and float when bounded is 1, the bounds that
    // clamp every value set (minimum at most maximum, the default within).
    typedef struct maniGraphParameterSource
    {
        const char* name;
        uint32_t nameLength;
        maniGraphParameterType type;
        maniGraphValue defaultValue;
        maniGraphValue minimum;
        maniGraphValue maximum;
        uint32_t bounded;
        uint32_t reserved;
    } maniGraphParameterSource;

    // A clip slot: an opened clip of the graph's skeleton, each clip once,
    // and a name for it (UTF-8, not empty, unique in the graph). The graph
    // keeps the clip's identity; binding finds the clip by it.
    typedef struct maniGraphClipSource
    {
        const maniClip* clip;
        const char* name;
        uint32_t nameLength;
        uint32_t reserved;
    } maniGraphClipSource;

    // A sync group's flags.
    enum
    {
        // Every member's events fire, not only the leader's.
        maniGraphSyncAllEvents = 1,
    };

    // A sync group: its name (UTF-8, not empty, unique in the graph) and
    // flags.
    typedef struct maniGraphSyncGroupSource
    {
        const char* name;
        uint32_t nameLength;
        uint32_t flags;
    } maniGraphSyncGroupSource;

    // How a clip node loops: as its clip says, or overridden.
    typedef uint32_t maniGraphLoop;
    enum
    {
        maniGraphLoopClip = 0,
        maniGraphLoopClamp = 1,
        maniGraphLoopRepeat = 2,
    };

    // A clip node's flags.
    enum
    {
        // Leads its sync group whatever its weight.
        maniGraphClipLeader = 1,
    };

    // A clip node: its clip slot, sync group (or MANI_NO_SYNC_GROUP),
    // speed (negative plays backward), loop and flags.
    typedef struct maniGraphClipNodeSource
    {
        uint32_t clip;
        uint32_t syncGroup;
        maniGraphScalar speed;
        maniGraphLoop loop;
        uint32_t flags;
    } maniGraphClipNodeSource;

    // A blend node: a weight per input (not negative), normalized unless
    // direct is 1 (then each at most 1, for weights set by the host).
    typedef struct maniGraphBlendNodeSource
    {
        const maniGraphScalar* weights;
        uint32_t direct;
        uint32_t reserved;
    } maniGraphBlendNodeSource;

    // A mask node: chains making per-bone weights (each 0 to 1) as
    // maniBuildMask makes them; its first input takes each bone's weight,
    // its second the remainder.
    typedef struct maniGraphMaskNodeSource
    {
        const maniMaskChain* chains;
        uint32_t chainCount;
        float defaultWeight;
    } maniGraphMaskNodeSource;

    // An additive node: a weight per layer (each input after the first),
    // 0 to 1.
    typedef struct maniGraphAdditiveNodeSource
    {
        const maniGraphScalar* weights;
    } maniGraphAdditiveNodeSource;

    // A select node: the int or bool parameter choosing an input (an int
    // clamped to the inputs).
    typedef struct maniGraphSelectNodeSource
    {
        uint32_t parameter;
    } maniGraphSelectNodeSource;

    // A time scale node: the factor on time below it (negative plays
    // backward).
    typedef struct maniGraphTimeScaleNodeSource
    {
        maniGraphScalar scale;
    } maniGraphTimeScaleNodeSource;

    // A seek node: the trigger that sets every playhead below it to time.
    typedef struct maniGraphSeekNodeSource
    {
        uint32_t trigger;
        maniGraphScalar time;
    } maniGraphSeekNodeSource;

    // A source-only node kind: another graph inlined in its place at
    // build (its nodes appended, its output standing where this node is).
    enum
    {
        maniGraphNodeSubgraph = 64,
    };

    typedef struct maniGraphSource maniGraphSource;

    // A subgraph node: the graph (of the same skeleton; its clips joined
    // to this graph's by identity, its sync groups by name), and for each
    // of its parameters the parameter of this graph it reads, of the same
    // type. It takes no inputs.
    typedef struct maniGraphSubgraphNodeSource
    {
        const maniGraphSource* graph;
        const uint32_t* parameters;
    } maniGraphSubgraphNodeSource;

    // A node: its kind, its inputs by index in the source's nodes, and the
    // part of the union its kind reads.
    typedef struct maniGraphNodeSource
    {
        maniGraphNodeKind kind;
        uint32_t inputCount;
        const uint32_t* inputs;
        union
        {
            maniGraphClipNodeSource clip;
            maniGraphBlendNodeSource blend;
            maniGraphMaskNodeSource mask;
            maniGraphAdditiveNodeSource additive;
            maniGraphSelectNodeSource select;
            maniGraphTimeScaleNodeSource timeScale;
            maniGraphSeekNodeSource seek;
            maniGraphSubgraphNodeSource subgraph;
        };
    } maniGraphNodeSource;

    // A graph: everything it holds, and the node whose pose it outputs.
    // Every node must reach the output.
    struct maniGraphSource
    {
        const maniSkeleton* skeleton;
        const maniGraphParameterSource* parameters;
        const maniGraphClipSource* clips;
        const maniGraphSyncGroupSource* syncGroups;
        const maniGraphNodeSource* nodes;
        uint32_t parameterCount;
        uint32_t clipCount;
        uint32_t syncGroupCount;
        uint32_t nodeCount;
        uint32_t output;
        uint32_t reserved;
    };

    /// Builds a graph file: inlines subgraphs (their nodes after the
    /// source's, in order, nested ones after them; refusals name nodes by
    /// their place there), checks every parameter, clip slot, sync group
    /// and node, orders the nodes so each follows its inputs (the output
    /// last, the source's order kept where it may be), makes each mask's
    /// bone weights, marks each node as giving poses or differences, gives
    /// each node's pose a slot reused once its last reader has run, and
    /// keeps the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The graph.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument (a
    /// misuse) or with the reason, the item and another the reason names
    /// for a refused source: Count (more items than the named limits, or
    /// none of nodes), Name, DuplicateName and NameHash with the list
    /// bits of parameters, clips or sync groups, Parameter, Clip, Node,
    /// Input, Cycle, Unreachable, Scalar, Weight and Difference; and
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildGraph(maniBuilder* builder,
                                                           const maniGraphSource* source);

    /// Writes a graph source as its JSON text form (UTF-8, two-space
    /// indents, a line per list item, ending in a new line), building it
    /// first: a source maniBuildGraph refuses is refused the same way and
    /// not written. Parameters, clip slots, sync groups and the bones of
    /// mask chains are written by name, nodes and the output by their
    /// place in the source, subgraphs as nested graphs with their
    /// parameters mapped by name, and numbers as the shortest text that
    /// reads back to the same binary32 (ECMAScript's layout). The text is
    /// kept in place of any file or text kept before; maniCopyBuilt
    /// copies it.
    ///
    /// @param builder  The builder.
    /// @param source   The graph.
    /// @return As maniBuildGraph.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniWriteGraphJson(maniBuilder* builder,
                                                               const maniGraphSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_GRAPHBUILD_H
