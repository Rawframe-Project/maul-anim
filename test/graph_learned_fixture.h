// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The learned node suites' helpers (mani-0025): the learned fixture's
// model (its events' outputs about 0.5, so they rise and fall as the
// state moves) in slot "steps"; a graph of an inertialize node over a
// learned node whose query is the parameter ahead, its trigger search,
// its tags those of the first Projector, its duration d; the node's
// block read. Included by one suite each.

#ifndef MAUL_ANIM_TEST_GRAPH_LEARNED_FIXTURE_H
#define MAUL_ANIM_TEST_GRAPH_LEARNED_FIXTURE_H

#include "graph_learned.h"
#include "learned_fixture.h"

static maniLearnedModel s_learned;
static const maniGraphModelSource kModelSlot[1] = {{&s_learned, "steps", 5, 0}};
// The model MakeLearned binds slot 0 to.
static const maniLearnedModel* s_bound = &s_learned;
static _Alignas(8) uint8_t s_learnedBinding[1024];

// The events are the Decompressor's last two outputs.
static inline void OpenModel(void)
{
    MakeNetworks();
    float* offsets = (float*)s_decompressor.outputOffsets;
    float* scales = (float*)s_decompressor.outputScales;
    for (uint32_t e = OUTPUTS - 2; e < OUTPUTS; ++e)
    {
        offsets[e] = 0.5f;
        scales[e] = 4.0f;
    }
    maniLearnedModelSource source = Source();
    CHECK(Opened(&source, &s_learned), "the model");
}

// A node over model slot 0, projecting by the trigger and every period
// seconds, past the transition cost; the first Projector's tags.
static inline maniGraphNodeSource LearnedNode(float period, float cost)
{
    maniGraphLearnedNodeSource l = {.model = 0,
                                    .trigger = 1,
                                    .query = kQuery,
                                    .queryCount = 1,
                                    .period = period,
                                    .transitionCost = cost,
                                    .requiredTags = {MANI_NO_PARAMETER, 1},
                                    .excludedTags = {MANI_NO_PARAMETER, 0},
                                    .duration = {3, 0.0f}};
    return (maniGraphNodeSource){maniGraphNodeLearned, 0, NULL, .learned = l};
}

// The source (the fixture's clip slots added, and its model slot unless
// it lists at most four) built, opened, bound to its slots' models and
// instanced.
static inline maniGraphInstance* MakeLearned(maniGraphSource source, maniGraph* g)
{
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    source.models = source.models != NULL ? source.models : kModelSlot;
    source.modelCount = source.modelCount > 0 ? source.modelCount : 1;
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    // Each slot's model, s_bound for the fixture's.
    const maniLearnedModel* models[4];
    for (uint32_t k = 0; k < source.modelCount && k < 4; ++k)
    {
        const maniLearnedModel* m = source.models[k].model;
        models[k] = m == &s_learned ? s_bound : m;
    }
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, g).result == mani_success &&
              maniBindGraph(g,
                            &(maniGraphAssets){&s_skeleton, clips, NULL, models, CLIPS, 0,
                                               source.modelCount, 0},
                            s_learnedBinding, sizeof s_learnedBinding, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a learned graph");
    maniDestroyBuilder(builder);
    return instance;
}

// An inertialize node over the node: the node 0, the inertialize node 1.
static inline maniGraphInstance* Learning(maniGraphNodeSource node, maniGraph* g)
{
    MatchParameters();
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    return MakeLearned((maniGraphSource){.parameters = s_matchParameters,
                                         .nodes = nodes,
                                         .parameterCount = 4,
                                         .nodeCount = 2,
                                         .output = 1},
                       g);
}

static inline uint32_t* LearnedBlock(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    return NodeBlock(instance, g, p, maniGraphLearnedBlock);
}

#endif // MAUL_ANIM_TEST_GRAPH_LEARNED_FIXTURE_H
