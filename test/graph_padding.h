// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph source copied with one byte pattern in the padding of every
// struct it reaches that has padding: its parameters, clips, databases,
// models, spring sets and nodes (each by the union member its kind
// reads), their transitions and mask chains, and its subgraphs. A graph
// suite builds a source and its copies under two patterns and requires
// the same bytes (source_padding.h).

#ifndef MAUL_ANIM_TEST_GRAPH_PADDING_H
#define MAUL_ANIM_TEST_GRAPH_PADDING_H

#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define GRAPH_PAD_BLOCKS 1024

// The copies made, freed together.
typedef struct GraphPad
{
    void* blocks[GRAPH_PAD_BLOCKS];
    uint32_t count;
    uint8_t pattern;
} GraphPad;

// n structs of size bytes from p, in a block of their own (NULL for none).
static inline void* GraphPadCopy(GraphPad* pad, const void* p, size_t n, size_t size)
{
    if (p == NULL || n == 0 || pad->count == GRAPH_PAD_BLOCKS)
    {
        CHECK(p == NULL || n == 0, "room for the padded copies");
        return NULL;
    }
    void* q = malloc(n * size);
    CHECK(q != NULL, "a padded copy");
    if (q != NULL)
    {
        memcpy(q, p, n * size);
        pad->blocks[pad->count++] = q;
    }
    return q;
}

#define GRAPH_PAD(pad, type, p, n, table)                                                          \
    do                                                                                             \
    {                                                                                              \
        type* graph_pad_copy = (type*)GraphPadCopy(pad, p, n, sizeof(type));                       \
        if (graph_pad_copy != NULL)                                                                \
        {                                                                                          \
            TEST_PAD(graph_pad_copy, n, table, (pad)->pattern);                                    \
        }                                                                                          \
        (p) = graph_pad_copy;                                                                      \
    } while (0)

static inline const maniGraphSource* GraphPadSource(GraphPad* pad, const maniGraphSource* g);

// A mask node source's chains.
static inline const maniGraphMaskNodeSource* GraphPadMask(GraphPad* pad,
                                                          const maniGraphMaskNodeSource* mask)
{
    if (mask == NULL)
    {
        return NULL;
    }
    maniGraphMaskNodeSource* m = (maniGraphMaskNodeSource*)GraphPadCopy(pad, mask, 1, sizeof *mask);
    if (m != NULL)
    {
        GRAPH_PAD(pad, maniMaskChain, m->chains, m->chainCount, kFieldsMaskChain);
    }
    return m;
}

// A node's own fields, without its union.
static const TestField kGraphPadNodeOwn[] = {
    TEST_FIELD(maniGraphNodeSource, kind),
    TEST_FIELD(maniGraphNodeSource, inputCount),
    TEST_FIELD(maniGraphNodeSource, inputs),
};

// One node: its padding by its kind's member, then what it points to.
static inline void GraphPadNode(GraphPad* pad, maniGraphNodeSource* n)
{
    switch (n->kind)
    {
    case maniGraphNodeClip:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_clip, pad->pattern);
        break;
    case maniGraphNodeBlend:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_blend, pad->pattern);
        break;
    case maniGraphNodeMask:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_mask, pad->pattern);
        GRAPH_PAD(pad, maniMaskChain, n->mask.chains, n->mask.chainCount, kFieldsMaskChain);
        break;
    case maniGraphNodeAdditive:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_additive, pad->pattern);
        break;
    case maniGraphNodeSelect:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_select, pad->pattern);
        break;
    case maniGraphNodeTimeScale:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_timeScale, pad->pattern);
        break;
    case maniGraphNodeSeek:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_seek, pad->pattern);
        break;
    case maniGraphNodeBlendSpace1D:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_blendSpace1D, pad->pattern);
        break;
    case maniGraphNodeBlendSpaceTriangles:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_triangulated, pad->pattern);
        break;
    case maniGraphNodeBlendSpaceBands:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_bands, pad->pattern);
        break;
    case maniGraphNodeMachine:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_machine, pad->pattern);
        GRAPH_PAD(pad, maniGraphTransitionSource, n->machine.transitions,
                  n->machine.transitionCount, kFieldsGraphTransitionSource);
        break;
    case maniGraphNodeOneShot:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_oneShot, pad->pattern);
        n->oneShot.mask = GraphPadMask(pad, n->oneShot.mask);
        break;
    case maniGraphNodeInertialize:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_inertialize, pad->pattern);
        break;
    case maniGraphNodeMatch:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_match, pad->pattern);
        break;
    case maniGraphNodeLearned:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_learned, pad->pattern);
        break;
    case maniGraphNodeSpring:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_spring, pad->pattern);
        break;
    case maniGraphNodePose:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_pose, pad->pattern);
        break;
    case maniGraphNodeTwoBone:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_twoBone, pad->pattern);
        break;
    case maniGraphNodeAim:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_aim, pad->pattern);
        break;
    case maniGraphNodeCcd:
    case maniGraphNodeFabrik:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_chain, pad->pattern);
        break;
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_body, pad->pattern);
        break;
    case maniGraphNodeTurnLimit:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_turnLimit, pad->pattern);
        n->turnLimit.mask = GraphPadMask(pad, n->turnLimit.mask);
        break;
    case maniGraphNodeSubgraph:
        TEST_PAD(n, 1, kFieldsGraphNodeSource_subgraph, pad->pattern);
        n->subgraph.graph = GraphPadSource(pad, n->subgraph.graph);
        break;
    default:
        // A node reading no member (mirror): the whole union padding.
        TEST_PAD(n, 1, kGraphPadNodeOwn, pad->pattern);
        break;
    }
}

// The source and everything it reaches, copied with padded padding.
static inline const maniGraphSource* GraphPadSource(GraphPad* pad, const maniGraphSource* g)
{
    if (g == NULL)
    {
        return NULL;
    }
    maniGraphSource* s = (maniGraphSource*)GraphPadCopy(pad, g, 1, sizeof *g);
    if (s == NULL)
    {
        return NULL;
    }
    GRAPH_PAD(pad, maniGraphParameterSource, s->parameters, s->parameterCount,
              kFieldsGraphParameterSource_f);
    GRAPH_PAD(pad, maniGraphClipSource, s->clips, s->clipCount, kFieldsGraphClipSource);
    GRAPH_PAD(pad, maniGraphDatabaseSource, s->databases, s->databaseCount,
              kFieldsGraphDatabaseSource);
    GRAPH_PAD(pad, maniGraphModelSource, s->models, s->modelCount, kFieldsGraphModelSource);
    GRAPH_PAD(pad, maniGraphSpringSetSource, s->springSets, s->springSetCount,
              kFieldsGraphSpringSetSource);
    maniGraphNodeSource* nodes =
        (maniGraphNodeSource*)GraphPadCopy(pad, s->nodes, s->nodeCount, sizeof *nodes);
    for (uint32_t i = 0; nodes != NULL && i < s->nodeCount; ++i)
    {
        GraphPadNode(pad, &nodes[i]);
    }
    s->nodes = nodes;
    TEST_PAD(s, 1, kFieldsGraphSource, pad->pattern);
    return s;
}

static inline void GraphPadFree(GraphPad* pad)
{
    for (uint32_t i = 0; i < pad->count; ++i)
    {
        free(pad->blocks[i]);
    }
    pad->count = 0;
}

// Whether the source builds to the same bytes under both patterns.
static inline bool GraphPaddingIgnored(const maniGraphSource* source)
{
    uint8_t* built[2] = {NULL, NULL};
    size_t sizes[2] = {0, 0};
    for (int k = 0; k < 2; ++k)
    {
        GraphPad* pad = (GraphPad*)calloc(1, sizeof(GraphPad));
        if (pad == NULL)
        {
            continue;
        }
        pad->pattern = kTestPadPatterns[k];
        const maniGraphSource* padded = GraphPadSource(pad, source);
        maniBuilderDef def = maniDefaultBuilderDef();
        maniBuilder* builder = NULL;
        if (padded != NULL && maniCreateBuilder(&def, &builder) == mani_success)
        {
            if (maniBuildGraph(builder, padded).result == mani_success)
            {
                built[k] = TestCopyBuilt(builder, &sizes[k]);
            }
            maniDestroyBuilder(builder);
        }
        GraphPadFree(pad);
        free(pad);
    }
    return TestSameBuilt(built, sizes);
}

#endif // MAUL_ANIM_TEST_GRAPH_PADDING_H
