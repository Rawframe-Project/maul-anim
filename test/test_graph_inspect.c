// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's structure and an instance's nodes read for a debugger:
// kinds, inputs, the output and each node's place in its
// source though the builder reorders them; then, update by update, each
// node's share of the output, a clip's playhead, a select's input, a
// machine's state and transition with its fade, a one-shot's phase and
// weight, an inertialization's easing and a motion matching node's entry;
// shares under a mask and an additive layer; every refusal.

#include "graph_json_assets.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/pose.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The output first, so the builder reorders every node.
static const char kText[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": \"pick\", "
    "\"type\": \"int\"}, {\"name\": \"jump\", \"type\": \"trigger\"}, {\"name\": \"fire\", "
    "\"type\": \"trigger\"}, {\"name\": \"blink\", \"type\": \"trigger\"}, {\"name\": \"go\", "
    "\"type\": \"trigger\"}, {\"name\": \"back\", \"type\": \"trigger\"}], \"clips\": [\"walk\", "
    "\"run\"], \"databases\": [\"moves\"], "
    "\"nodes\": [{\"kind\": \"inertialize\", \"inputs\": [1], \"trigger\": \"blink\", "
    "\"duration\": 0.4}, {\"kind\": \"blend\", \"inputs\": [2, 5], \"weights\": [0.5, 0.5]}, "
    "{\"kind\": \"oneShot\", \"inputs\": [3, 4], \"fire\": \"fire\", \"fadeIn\": 0.2, "
    "\"fadeOut\": 0.2}, {\"kind\": \"machine\", \"inputs\": [6, 7], \"transitions\": "
    "[{\"from\": 1, \"to\": 0, \"conditions\": [{\"trigger\": \"back\"}]}, "
    "{\"from\": 0, \"to\": 1, \"duration\": 0.5, \"conditions\": [{\"trigger\": \"jump\"}]}]}, "
    "{\"kind\": \"clip\", \"clip\": \"run\"}, {\"kind\": \"select\", \"inputs\": [8, 9], "
    "\"parameter\": \"pick\"}, {\"kind\": \"clip\", \"clip\": \"walk\"}, {\"kind\": \"clip\", "
    "\"clip\": \"run\"}, {\"kind\": \"match\", \"database\": \"moves\", \"trigger\": \"go\", "
    "\"period\": 10}, "
    "{\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}";

enum
{
    kPlaces = 10
};

static _Alignas(16) uint8_t s_binding[1u << 16];
static _Alignas(16) uint8_t s_instanceMemory[1u << 18];
static _Alignas(64) uint8_t s_scratch[1u << 18];
static _Alignas(64) uint8_t s_poseMemory[4096];
static maniPose* s_pose;
static maniGraph s_graph;
static maniGraphInstance* s_instance;
// Each source place's node in the file.
static uint32_t s_at[kPlaces];

// A mask over the spine and the arm at 0.75 (the hips 0) under an additive
// layer of 0.6; then a mask with one clip on both sides.
static const char kMasked[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"clips\": [\"walk\", \"run\", "
    "\"wave\"], \"nodes\": [{\"kind\": \"additive\", \"inputs\": [1, 4], \"weights\": [0.6]}, "
    "{\"kind\": \"mask\", \"inputs\": [2, 3], \"chains\": [{\"bone\": \"spine\", \"weight\": "
    "0.75}]}, {\"kind\": \"clip\", \"clip\": \"walk\"}, {\"kind\": \"clip\", \"clip\": \"run\"}, "
    "{\"kind\": \"clip\", \"clip\": \"wave\"}], \"output\": 0}";
static const char kTwice[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"clips\": [\"walk\"], \"nodes\": "
    "[{\"kind\": \"mask\", \"inputs\": [1, 1], \"chains\": [{\"bone\": \"spine\", \"weight\": "
    "0.75}]}, {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}";

// Builds, opens and binds a graph naming the first clips of the assets,
// with an instance and a pose; s_at maps each source place to its node.
static void Load(const char* text, size_t length, uint32_t clipCount, uint32_t places)
{
    size_t size = 0;
    CHECK(maniBuildGraphJson(s_builder, text, length,
                             &(maniGraphNamedAssets){&s_skeleton, kHost, kHostDatabase, NULL, NULL,
                                                     clipCount, 1, 0, 0, NULL})
                      .result == mani_success &&
              maniCopyBuilt(s_builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, &s_graph).result == mani_success,
          "the graph");
    const maniClip* clips[3] = {&s_clips[0], &s_clips[1], &s_clips[2]};
    const maniMotionDatabase* databases[1] = {&s_database};
    maniGraphBinding* binding = NULL;
    CHECK(maniBindGraph(&s_graph,
                        &(maniGraphAssets){&s_skeleton, clips, databases, NULL, NULL, clipCount, 1,
                                           0, 0, NULL, NULL, 0, 0, NULL},
                        s_binding, sizeof s_binding, &binding, NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &s_instance) == mani_success,
          "an instance");
    CHECK(maniGraphScratchSize(binding) <= sizeof s_scratch &&
              maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success,
          "a pose");
    for (uint32_t n = 0; n < places; ++n)
    {
        uint32_t place = maniGraphNodeSourcePlace(&s_graph, n);
        s_at[place < kPlaces ? place : 0] = n;
    }
}

// Each node's kind, inputs and place in the source.
static void TestStructure(void)
{
    CHECK(maniGraphNodeCount(&s_graph) == kPlaces, "ten nodes");
    uint32_t seen = 0;
    for (uint32_t n = 0; n < kPlaces; ++n)
    {
        uint32_t place = maniGraphNodeSourcePlace(&s_graph, n);
        if (place < kPlaces)
        {
            s_at[place] = n;
            seen |= 1u << place;
        }
    }
    CHECK(seen == (1u << kPlaces) - 1u, "every place once");
    CHECK(maniGraphOutputNode(&s_graph) == s_at[0] && s_at[0] == kPlaces - 1,
          "the output, last, from place 0");
    CHECK(s_at[1] != 1, "the builder reordered");
    CHECK(maniGraphNodeKindOf(&s_graph, s_at[0]) == maniGraphNodeInertialize &&
              maniGraphNodeKindOf(&s_graph, s_at[3]) == maniGraphNodeMachine &&
              maniGraphNodeKindOf(&s_graph, s_at[8]) == maniGraphNodeMatch &&
              maniGraphNodeKindOf(&s_graph, s_at[9]) == maniGraphNodeClip,
          "kinds");
    CHECK(maniGraphNodeInputCount(&s_graph, s_at[3]) == 2 &&
              maniGraphNodeInput(&s_graph, s_at[3], 0) == s_at[6] &&
              maniGraphNodeInput(&s_graph, s_at[3], 1) == s_at[7] &&
              maniGraphNodeInput(&s_graph, s_at[1], 1) == s_at[5] &&
              maniGraphNodeInputCount(&s_graph, s_at[9]) == 0,
          "inputs");
    CHECK(maniGraphNodeKindOf(NULL, 0) == 0 && maniGraphNodeKindOf(&s_graph, kPlaces) == 0 &&
              maniGraphNodeInputCount(NULL, 0) == 0 &&
              maniGraphNodeInputCount(&s_graph, kPlaces) == 0 &&
              maniGraphNodeInput(&s_graph, s_at[3], 2) == MANI_GRAPH_NONE &&
              maniGraphNodeInput(NULL, 0, 0) == MANI_GRAPH_NONE &&
              maniGraphOutputNode(NULL) == MANI_GRAPH_NONE &&
              maniGraphNodeSourcePlace(NULL, 0) == MANI_GRAPH_NONE &&
              maniGraphNodeSourcePlace(&s_graph, kPlaces) == MANI_GRAPH_NONE,
          "past the counts");
    // A count one short: its last node is there to be read wrongly.
    maniGraph shorter = s_graph;
    shorter.nodeCount = kPlaces - 1;
    CHECK(maniGraphNodeKindOf(&shorter, kPlaces - 1) == 0, "the count bounds the kind");
}

static maniGraphNodeView View(uint32_t place)
{
    maniGraphNodeView v;
    memset(&v, 0xA5, sizeof v);
    CHECK(maniInspectGraphNode(s_instance, s_at[place], &v) == mani_success, "inspected");
    return v;
}

static void Set(uint32_t parameter, maniGraphParameterType type, int32_t value)
{
    maniGraphValue v = {.i = value};
    CHECK(maniSetGraphParameter(s_instance, parameter, type, v) == mani_success, "set");
}

static void Update(float delta)
{
    uint32_t count = 0;
    maniRootMove move;
    CHECK(maniUpdateGraph(s_instance, delta, NULL, 0, &count, &move) == mani_success &&
              maniEvaluateGraph(s_instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "updated and evaluated");
}

static bool Near(double a, double b)
{
    return fabs(a - b) < 1e-5;
}

// A place's view: its share, current word, transition and progress.
static bool Is(uint32_t place, float weight, uint32_t current, uint32_t transition, float progress)
{
    maniGraphNodeView v = View(place);
    return Near((double)v.weight, (double)weight) && v.current == current &&
           v.transition == transition && Near((double)v.progress, (double)progress) &&
           v.failed == 0;
}

#define NONE MANI_GRAPH_NONE

static void TestViews(void)
{
    maniGraphNodeView v = View(3);
    CHECK(v.kind == maniGraphNodeMachine && v.weight == 0.0f && v.current == 0 &&
              v.transition == NONE && v.progress == 0.0f && v.failed == 0,
          "a machine before the first update");
    CHECK(View(0).weight == 0.0f, "no weights before the first update");
    Update(0.25f);
    CHECK(Is(0, 1.0f, NONE, NONE, 0.0f) && Is(1, 1.0f, NONE, NONE, 0.0f) &&
              Is(2, 0.5f, 0, NONE, 0.0f) && Is(3, 0.5f, 0, NONE, 0.0f) &&
              Is(4, 0.0f, NONE, NONE, 0.0f) && Is(5, 0.5f, 0, NONE, 0.0f) &&
              Is(6, 0.5f, NONE, NONE, 0.0f) && Is(7, 0.0f, NONE, NONE, 0.0f) &&
              Is(8, 0.5f, 0, NONE, 0.0f) && Is(9, 0.0f, NONE, NONE, 0.0f),
          "shares: a blend's halves, the machine's entry state, the select's first input, the "
          "match node's entry");
    CHECK(View(6).time == 0.25 && View(7).time == 0.0 && Near(View(8).time, 0.7) &&
              View(5).time == 0.0,
          "playheads of clips and the match node only");
    Set(1, maniGraphParameterTrigger, 1);
    Update(0.1f);
    CHECK(Is(3, 0.5f, 1, 1, 0.0f) && Is(6, 0.5f, NONE, NONE, 0.0f), "a transition fired");
    Update(0.1f);
    CHECK(Is(3, 0.5f, 1, 1, 0.2f) && Is(6, 0.4f, NONE, NONE, 0.0f) && Is(7, 0.1f, NONE, NONE, 0.0f),
          "its fade a fifth along, the states' shares with it");
    Set(2, maniGraphParameterTrigger, 1);
    Update(0.1f);
    CHECK(Is(2, 0.5f, 1, NONE, 0.0f) && Is(3, 0.5f, 1, 1, 0.4f), "a one-shot fired");
    Set(3, maniGraphParameterTrigger, 1);
    Update(0.1f);
    CHECK(Is(2, 0.5f, 1, NONE, 0.5f) && Is(4, 0.25f, NONE, NONE, 0.0f) &&
              Is(3, 0.25f, 1, 1, 0.6f) && Is(0, 1.0f, NONE, NONE, 0.25f),
          "fading in, its shot's share; an inertialization begun");
    Update(0.1f);
    CHECK(Is(2, 0.5f, 2, NONE, 1.0f) && Is(3, 0.0f, 1, 1, 0.8f) && Is(0, 1.0f, NONE, NONE, 0.5f),
          "playing, its base out of the output");
    Set(0, maniGraphParameterInt, 1);
    Update(0.1f);
    CHECK(Is(5, 0.5f, 1, NONE, 0.0f) && Is(9, 0.5f, NONE, NONE, 0.0f) &&
              Is(8, 0.0f, 0, NONE, 0.0f) && Is(3, 0.0f, 1, NONE, 0.0f) &&
              Is(0, 1.0f, NONE, NONE, 0.75f),
          "the select's other input; the transition done");
}

static void TestRefusals(void)
{
    maniGraphNodeView v;
    static _Alignas(16) uint8_t kNever[256];
    CHECK(maniInspectGraphNode(NULL, 0, &v) == mani_errorInvalid &&
              maniInspectGraphNode((const maniGraphInstance*)kNever, 0, &v) == mani_errorInvalid &&
              maniInspectGraphNode(s_instance, kPlaces, &v) == mani_errorInvalid &&
              maniInspectGraphNode(s_instance, 0, NULL) == mani_errorInvalid,
          "refused");
}

// A mask's inputs take the largest of its weights and of one minus them,
// an additive layer its weight times the node's; a node on two paths sums.
static void TestShares(void)
{
    Load(kMasked, sizeof kMasked - 1, 3, 5);
    Update(0.1f);
    CHECK(Is(0, 1.0f, NONE, NONE, 0.0f) && Is(1, 1.0f, NONE, NONE, 0.0f) &&
              Is(2, 0.75f, NONE, NONE, 0.0f) && Is(3, 1.0f, NONE, NONE, 0.0f) &&
              Is(4, 0.6f, NONE, NONE, 0.0f),
          "a mask's two sides and an additive layer");
    Load(kTwice, sizeof kTwice - 1, 1, 2);
    Update(0.1f);
    CHECK(Is(1, 1.75f, NONE, NONE, 0.0f), "both sides of a mask summed");
}

int main(void)
{
    SetupGraphAssets();
    Load(kText, sizeof kText - 1, 2, kPlaces);
    TestStructure();
    TestViews();
    TestRefusals();
    TestShares();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
