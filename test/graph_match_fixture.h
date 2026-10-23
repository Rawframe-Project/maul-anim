// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motion matching node suites' helpers: a database of the
// graph fixture's walk (looping, tags 1) and run (clamped, tags 2) at 10
// frames a second, its one channel the root's travel half a second ahead
// (the walk's 0.5 along x, the run's none); a graph of an inertialize
// node over a motion matching node whose query is the parameter ahead;
// the node's and the inertialize node's blocks read. Included by one
// suite each.

#ifndef MAUL_ANIM_TEST_GRAPH_MATCH_FIXTURE_H
#define MAUL_ANIM_TEST_GRAPH_MATCH_FIXTURE_H

#include "graph_inertia.h"
#include "graph_match.h"
#include "graph_space_fixture.h"

#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/pose.h"

// Frames: the walk's 0 to 9 (0 to 0.9 s), the run's 10 to 20 (0 to 1 s).
#define WALK_FRAMES 10u
#define RUN_FIRST   10u

static _Alignas(16) uint8_t s_databaseFile[1 << 15];
static maniMotionDatabase s_database;
static const float kAhead[1] = {0.5f};

// A database of the walk and run whose channels are given.
static inline void BuildDatabase(const maniMatchChannelSource* channels, uint32_t count,
                                 uint8_t* file, size_t capacity, maniMotionDatabase* out)
{
    static const maniClip* clips[2];
    static maniMatchEntrySource entries[2];
    clips[0] = &s_clips[0];
    clips[1] = &s_clips[1];
    entries[0] = (maniMatchEntrySource){0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u};
    entries[1] = (maniMatchEntrySource){1, 0, 0.0f, 1.0f, 0.0f, 2u};
    maniMotionDatabaseSource source = {&s_skeleton, clips, channels, entries,        2,
                                       count,       2,     10.0f,    maniMatchAxisY, maniMatchAxisZ,
                                       NULL};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, capacity, &size) == mani_success &&
              maniOpenMotionDatabase(file, size, out).result == mani_success,
          "a database");
    maniDestroyBuilder(builder);
}

static inline void OpenDatabase(void)
{
    static const maniMatchChannelSource kChannel = {
        maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f};
    BuildDatabase(&kChannel, 1, s_databaseFile, sizeof s_databaseFile, &s_database);
}

// Parameters: ahead (vec2, the walk's), search (a trigger), tags (int),
// d (float, 0.25).
static maniGraphParameterSource s_matchParameters[4];
static const uint32_t kQuery[1] = {0};

static inline void MatchParameters(void)
{
    maniGraphValue walk = {{0.5f, 0.0f, 0.0f, 0.0f}};
    s_matchParameters[0] = Parameter("ahead", maniGraphParameterVec2, walk);
    s_matchParameters[1] = Parameter("search", maniGraphParameterTrigger, Int(0));
    s_matchParameters[2] = Parameter("tags", maniGraphParameterInt, Int(0));
    s_matchParameters[3] = Parameter("d", maniGraphParameterFloat, Float(0.25f));
}

// A node over database slot 0, searched by the trigger and every period
// seconds, its end exclusion 0.2 s, no tags, no history, its duration d.
static inline maniGraphNodeSource MatchNode(float period)
{
    maniGraphMatchNodeSource m = {.database = 0,
                                  .trigger = 1,
                                  .query = kQuery,
                                  .queryCount = 1,
                                  .period = period,
                                  .endExclusion = 0.2f,
                                  .requiredTags = {MANI_NO_PARAMETER, 0},
                                  .excludedTags = {MANI_NO_PARAMETER, 0},
                                  .duration = {3, 0.0f}};
    return (maniGraphNodeSource){maniGraphNodeMatch, 0, NULL, .match = m};
}

static inline maniGraphNodeSource Above(uint32_t input)
{
    static uint32_t inputs[1];
    inputs[0] = input;
    return (maniGraphNodeSource){
        maniGraphNodeInertialize, 1, inputs,
        .inertialize = {MANI_NO_PARAMETER, maniGraphInertialization, LITERAL(0.0f), 0, 0, 0}};
}

static const maniGraphDatabaseSource kDatabaseSlot[1] = {{&s_database, "moves", 5, 0}};

static _Alignas(8) uint8_t s_matchBinding[1024];

// The source (the fixture's clip slots and the database slot added)
// built, opened, bound and instanced.
static inline maniGraphInstance* MakeMatch(maniGraphSource source, maniGraph* g)
{
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    source.databases = source.databases != NULL ? source.databases : kDatabaseSlot;
    source.databaseCount = source.databaseCount > 0 ? source.databaseCount : 1;
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    const maniMotionDatabase* databases[1] = {source.databases[0].database};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, g).result == mani_success &&
              maniBindGraph(g,
                            &(maniGraphAssets){&s_skeleton, clips, databases, NULL, NULL, CLIPS, 1,
                                               0, 0, NULL, NULL, 0, 0, source.mirrorMap},
                            s_matchBinding, sizeof s_matchBinding, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a matching graph");
    maniDestroyBuilder(builder);
    return instance;
}

// An inertialize node over a node searching every period seconds: the
// node 0, the inertialize node 1.
static inline maniGraphInstance* Matching(maniGraphNodeSource node, maniGraph* g)
{
    MatchParameters();
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    return MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                       .nodes = nodes,
                                       .parameterCount = 4,
                                       .nodeCount = 2,
                                       .output = 1},
                     g);
}

static inline uint32_t* NodeBlock(maniGraphInstance* instance, const maniGraph* g, uint32_t p,
                                  uint32_t word)
{
    uint32_t data = maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    return (uint32_t*)((uint8_t*)instance + instance->machinesAt) +
           maniLoadU32(g->data + 4 * (size_t)(data + word));
}

// Node p's entry, and its playhead.
static inline uint32_t EntryOf(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    return NodeBlock(instance, g, p, maniGraphMatchBlock)[maniGraphMatchEntry];
}

static inline bool Playing(maniGraphInstance* instance, const maniGraph* g, uint32_t p,
                           uint32_t entry, double time)
{
    bool ok = EntryOf(instance, g, p) == entry && fabs(State(instance, g, p)->time - time) <= 1e-6;
    if (!ok)
    {
        printf("  playing entry %u at %.9g, want %u at %.9g\n", EntryOf(instance, g, p),
               State(instance, g, p)->time, entry, time);
    }
    return ok;
}

// The inertialize node p's pending request (+inf for none).
static inline float Pending(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    return maniGraphInertiaGet(NodeBlock(instance, g, p, maniGraphInertiaBlock),
                               maniGraphInertiaPending);
}

static inline void Ahead(maniGraphInstance* instance, float x, float z)
{
    maniGraphValue v = {{x, z, 0.0f, 0.0f}};
    Set(instance, 0, maniGraphParameterVec2, v);
}

#endif // MAUL_ANIM_TEST_GRAPH_MATCH_FIXTURE_H
