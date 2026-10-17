// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned motion matching nodes' files (mani-0025): the builder's
// refusals of each field (no inertialize node above, literal tags no
// Projector learned, a query not fitting the model's database);
// opening's checks of every node word; binding's refusal of a model the
// node's counts or query do not fit; a subgraph's model joined at
// inlining, every reference remapped; the JSON form, round trip, its
// defaults and refusals. Run as test_graph_learned_file --seed <graph
// file> <json file> it writes the round trip's file and text, seeds of
// the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_learned_fixture.h"

#include <stdio.h>

static maniLearnedModel s_other;

static maniBuildResult BuildLearned(const maniGraphNodeSource* nodes, uint32_t count)
{
    MatchParameters();
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = s_matchParameters,
                              .nodes = nodes,
                              .parameterCount = 4,
                              .nodeCount = count,
                              .output = count - 1,
                              .modelCount = 1,
                              .models = kModelSlot};
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, &source);
    maniDestroyBuilder(builder);
    return r;
}

static bool RefusedAs(maniBuildResult r, maniBuildReason reason, uint32_t item, uint32_t other)
{
    bool ok =
        r.result == mani_errorInvalid && r.reason == reason && r.item == item && r.other == other;
    if (!ok)
    {
        printf("  refused %d: reason %u item %u other %u\n", (int)r.result, r.reason, r.item,
               r.other);
    }
    return ok;
}

// The node changed, under an inertialize node.
static maniBuildResult NodeBuilt(maniGraphNodeSource node)
{
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    return BuildLearned(nodes, 2);
}

static void TestRefusals(void)
{
    static const uint32_t kFloat[1] = {3};
    static const uint32_t kTwo[2] = {0, 0};
    maniGraphNodeSource n[12];
    for (int k = 0; k < 12; ++k)
    {
        n[k] = LearnedNode(0.1f, 0.0f);
    }
    n[0].learned.model = 1;
    n[1].learned.trigger = 0;
    n[2].learned.period = 0.0f;
    n[3].learned.period = INFINITY;
    n[4].learned.transitionCost = -1.0f;
    n[5].learned.transitionCost = NAN;
    n[6].learned.requiredTags = (maniGraphTags){3, 0};
    n[7].learned.excludedTags = (maniGraphTags){2, 1};
    n[8].learned.query = kFloat;
    n[9].learned.query = kTwo;
    n[9].learned.queryCount = 2;
    n[10].learned.queryCount = 0;
    n[11].learned.query = NULL;
    for (int k = 0; k < 12; ++k)
    {
        CHECK(RefusedAs(NodeBuilt(n[k]), maniBuildReasonNode, 0, 0), "a field refused");
    }
    maniGraphNodeSource d = LearnedNode(0.1f, 0.0f);
    d.learned.duration = LITERAL(-1.0f);
    CHECK(RefusedAs(NodeBuilt(d), maniBuildReasonScalar, 0, 0), "a negative duration");
    d.learned.duration = (maniGraphScalar){0, 0.0f};
    CHECK(RefusedAs(NodeBuilt(d), maniBuildReasonScalar, 0, 0), "a duration not a float");
    maniGraphNodeSource t = LearnedNode(0.1f, 0.0f);
    t.learned.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, 3};
    CHECK(RefusedAs(NodeBuilt(t), maniBuildReasonTags, 0, 0), "tags no Projector learned");
    t.learned.excludedTags = (maniGraphTags){MANI_NO_PARAMETER, 1};
    t.learned.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, 2};
    CHECK(NodeBuilt(t).result == mani_success, "the second Projector's tags");
    t.learned.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, 1};
    CHECK(RefusedAs(NodeBuilt(t), maniBuildReasonTags, 0, 0),
          "the first Projector's required tags, the second's excluded");
    t.learned.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, 3};
    t.learned.excludedTags = (maniGraphTags){2, 0};
    CHECK(NodeBuilt(t).result == mani_success, "excluded tags from a parameter: any at update");
    t.learned.requiredTags = (maniGraphTags){2, 0};
    t.learned.excludedTags = (maniGraphTags){MANI_NO_PARAMETER, 7};
    CHECK(NodeBuilt(t).result == mani_success, "tags from a parameter: any at update");
    maniGraphNodeSource alone[1] = {LearnedNode(0.1f, 0.0f)};
    CHECK(RefusedAs(BuildLearned(alone, 1), maniBuildReasonNode, 0, 0),
          "no inertialize node above");
}

// The file of the graph last made, a word set: it opens, and binding it
// refuses its model slot (after the clips).
static bool Unbindable(const maniGraph* g, uint32_t k, uint32_t value)
{
    uint64_t at = 0;
    maniGraph crafted;
    if (Crafted(g, 0, k, value, &at).result != mani_success ||
        maniOpenGraph(s_crafted, s_size, &crafted).result != mani_success)
    {
        return false;
    }
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    const maniLearnedModel* models[1] = {&s_learned};
    maniGraphBinding* binding = NULL;
    uint32_t unbound = 0;
    return maniBindGraph(&crafted,
                         &(maniGraphAssets){&s_skeleton, clips, NULL, models, NULL, CLIPS, 0, 1, 0},
                         s_learnedBinding, sizeof s_learnedBinding, &binding,
                         &unbound) == mani_errorInvalid &&
           unbound == CLIPS;
}

static void TestOpen(void)
{
    maniGraph g;
    (void)Learning(LearnedNode(0.1f, 0.0f), &g);
    const uint32_t kWords[17][2] = {{maniGraphLearnedModel, 1},
                                    {maniGraphLearnedTrigger, 0},
                                    {maniGraphLearnedPeriod, 0},
                                    {maniGraphLearnedPeriod, 0x7F800000u},
                                    {maniGraphLearnedTransitionCost, 0xBF800000u},
                                    {maniGraphLearnedTransitionCost, 0x7FC00000u},
                                    {maniGraphLearnedTransitionCost, 0x7F800000u},
                                    {maniGraphLearnedStates, 1},
                                    {maniGraphLearnedEvents, MANI_LEARNED_MAX_EVENTS + 1},
                                    {maniGraphLearnedRequiredTags, 3},
                                    {maniGraphLearnedExcludedTags, 0},
                                    {maniGraphLearnedDurationScalar, 0},
                                    {maniGraphLearnedBlock, 5},
                                    {maniGraphLearnedFeatures, 0},
                                    {maniGraphLearnedFeatures, STATE},
                                    {maniGraphLearnedQuery, 1},
                                    {maniGraphLearnedQuery, 4}};
    for (int k = 0; k < 17; ++k)
    {
        if (!RefusedAtWord(&g, 0, kWords[k][0], kWords[k][1]))
        {
            printf("  word %u\n", kWords[k][0]);
            CHECK(false, "a node word refused");
        }
    }
    CHECK(RefusedAtWordOf(&g, 0, maniGraphLearnedDurationScalar + 1, 0xBF800000u,
                          maniGraphLearnedDurationScalar),
          "a duration parameter's literal not 0");
    uint64_t at = 0;
    CHECK(Crafted(&g, 0, maniGraphLearnedQueryCount, 2, &at).result == mani_errorFormat,
          "a query count past the words");
    CHECK(Crafted(&g, 0, maniGraphLearnedQueryCount, MANI_MATCH_MAX_FEATURES + 1, &at).result ==
              mani_errorFormat,
          "a query count past the features");
    // Literal tags and duration, the parameter tags' literal not 0.
    maniGraphNodeSource literal = LearnedNode(0.1f, 0.0f);
    literal.learned.duration = LITERAL(0.25f);
    literal.learned.requiredTags = (maniGraphTags){2, 0};
    (void)Learning(literal, &g);
    CHECK(RefusedAtWordOf(&g, 0, maniGraphLearnedDurationScalar + 1, 0xFF800000u,
                          maniGraphLearnedDurationScalar),
          "a literal duration not finite");
    CHECK(RefusedAtWordOf(&g, 0, maniGraphLearnedRequiredTags + 1, 1, maniGraphLearnedRequiredTags),
          "a tags parameter's literal not 0");
    CHECK(Crafted(&g, 0, maniGraphLearnedDurationScalar + 1, 0, &at).result == mani_success,
          "a literal duration of 0");
}

// A query count past MANI_MATCH_MAX_FEATURES with the words for it (a
// blend's weights after the node's): refused at the node's record, before
// any query word is read.
static void TestLongQuery(void)
{
    MatchParameters();
    static uint32_t inputs[600];
    static maniGraphScalar weights[600];
    for (uint32_t k = 0; k < 600; ++k)
    {
        inputs[k] = k == 0 ? 1u : 2u;
        weights[k] = LITERAL(1.0f);
    }
    maniGraphNodeSource nodes[4] = {LearnedNode(0.1f, 0.0f),
                                    Above(0),
                                    ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    {maniGraphNodeBlend, 600, inputs, .blend = {weights, 0, 0}}};
    maniGraph g;
    (void)MakeLearned((maniGraphSource){.parameters = s_matchParameters,
                                        .nodes = nodes,
                                        .parameterCount = 4,
                                        .nodeCount = 4,
                                        .output = 3},
                      &g);
    uint64_t at = 0;
    maniFileResult r = Crafted(&g, 0, maniGraphLearnedQueryCount, MANI_MATCH_MAX_FEATURES + 1, &at);
    CHECK(r.result == mani_errorFormat && r.offset < at, "refused before the node's words");
    // The record's word count made to agree: still refused at it.
    maniContainer c;
    const uint8_t* bytes = NULL;
    uint64_t size = 0;
    uint64_t records = 0;
    CHECK(
        maniOpenContainer(s_crafted, s_size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
                mani_success &&
            maniContainerSection(&c, maniGraphTagNodes, &bytes, &size, &records),
        "the nodes");
    maniStoreU32(s_crafted + records + 16, MANI_GRAPH_LEARNED_WORDS + MANI_MATCH_MAX_FEATURES + 1);
    Rehash(s_crafted);
    maniGraph crafted;
    r = maniOpenGraph(s_crafted, s_size, &crafted);
    CHECK(r.result == mani_errorFormat && r.offset == records + 16,
          "a query count past the features, its words counted");
}

// A model whose Decompressor is wider (48, past its 39 outputs), in its
// own bytes; built
// before the fixture's model, whose bytes Build reuses.
static _Alignas(16) uint8_t s_wideFile[65536];
static maniLearnedModel s_wide;

static void OpenWide(void)
{
    MakeNetworks();
    static maniLearnedLayerSource layers[2];
    static const uint32_t kWide[3] = {STATE, 48, OUTPUTS};
    maniLearnedNetworkSource wide = Network(layers, kWide, 2);
    maniLearnedModelSource source = Source();
    source.decompressor = &wide;
    CHECK(Build(&source).result == mani_success && s_modelSize <= sizeof s_wideFile, "built");
    memcpy(s_wideFile, s_model, s_modelSize);
    CHECK(maniOpenLearnedModel(s_wideFile, s_modelSize, &s_wide).result == mani_success,
          "a wider model");
}

// The room a model's node needs: its scratch, a state and its motion
// outputs (the root's, the float tracks, the events).
static size_t Need(const maniLearnedModel* m)
{
    size_t motion = 6 + (size_t)m->floatTrackCount + m->eventCount;
    return maniLearnedScratchSize(m) + 4 * ((size_t)m->featureCount + m->latentCount + motion);
}

// Two nodes over two models, the wider first or last: the room the
// wider needs, and both play.
static void TestRoom(bool wideFirst)
{
    MatchParameters();
    const maniGraphModelSource slots[2] = {{wideFirst ? &s_wide : &s_learned, "first", 5, 0},
                                           {wideFirst ? &s_learned : &s_wide, "second", 6, 0}};
    static const uint32_t kInputs[2] = {2, 3};
    static const maniGraphScalar kWeights[2] = {{MANI_NO_PARAMETER, 1.0f},
                                                {MANI_NO_PARAMETER, 1.0f}};
    maniGraphNodeSource nodes[5] = {LearnedNode(0.1f, 0.0f),
                                    LearnedNode(0.1f, 0.0f),
                                    Above(0),
                                    Above(1),
                                    {maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0, 0}}};
    // Above keeps one input array: each its own here.
    static const uint32_t kFirst[1] = {0};
    static const uint32_t kSecond[1] = {1};
    nodes[1].learned.model = 1;
    nodes[2].inputs = kFirst;
    nodes[3].inputs = kSecond;
    maniGraphSource source = {.parameters = s_matchParameters,
                              .nodes = nodes,
                              .parameterCount = 4,
                              .nodeCount = 5,
                              .output = 4,
                              .modelCount = 2,
                              .models = slots};
    maniGraph g;
    maniGraphInstance* instance = MakeLearned(source, &g);
    const maniGraphBinding* binding = instance->binding;
    size_t room = (Need(&s_wide) + 7u) & ~(size_t)7u;
    CHECK(room > Need(&s_learned), "the wider model needs more");
    CHECK(maniGraphLearnedRoom(binding) == room &&
              maniGraphInstanceSize(binding) == instance->learnedAt + room,
          "the room the wider model needs");
    Update(instance, 0.0f);
    Update(instance, 0.05f);
    uint32_t playing = 0;
    for (uint32_t p = 0; p < g.nodeCount; ++p)
    {
        bool learned =
            maniLoadU16(g.nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) == maniGraphNodeLearned;
        playing += learned && LearnedBlock(instance, &g, p)[maniGraphLearnedHas] == 1 ? 1u : 0u;
    }
    CHECK(playing == 2, "both playing");
}

// A graph whose first slot's model no node plays: bound whatever its
// counts.
static void TestIdleSlot(void)
{
    maniLearnedModel odd = s_learned;
    odd.identity ^= 2;
    odd.eventCount = 1;
    const maniGraphModelSource slots[2] = {{&odd, "odd", 3, 0}, kModelSlot[0]};
    maniGraphNodeSource node = LearnedNode(0.1f, 0.0f);
    node.learned.model = 1;
    MatchParameters();
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    maniGraphSource source = {.parameters = s_matchParameters,
                              .nodes = nodes,
                              .parameterCount = 4,
                              .nodeCount = 2,
                              .output = 1,
                              .modelCount = 2,
                              .models = slots};
    maniGraph g;
    maniGraphInstance* instance = MakeLearned(source, &g);
    Update(instance, 0.0f);
    CHECK(maniGraphBoundModel(instance->binding, 0) == &odd &&
              LearnedBlock(instance, &g, 0)[maniGraphLearnedHas] == 1,
          "the idle slot bound, the node playing its own");
}

// Files that open but name a model the node does not fit.
static void TestUnfit(void)
{
    maniGraph g;
    (void)Learning(LearnedNode(0.1f, 0.0f), &g);
    CHECK(Unbindable(&g, maniGraphLearnedQuery, 3), "a float parameter where it wants a vec2");
    CHECK(Unbindable(&g, maniGraphLearnedFeatures, FEATURES + 1), "a feature count not its");
    CHECK(!Unbindable(&g, maniGraphLearnedQuery, 0), "the query as built binds");
}

// A subgraph holding the node and its model; the root's own model
// first, its parameters the subgraph's reversed.
static void TestSubgraph(void)
{
    MatchParameters();
    maniGraphParameterSource reversed[4] = {s_matchParameters[3], s_matchParameters[2],
                                            s_matchParameters[1], s_matchParameters[0]};
    static const uint32_t kMap[4] = {3, 2, 1, 0};
    maniGraphNodeSource inner[1] = {LearnedNode(0.1f, 0.0f)};
    inner[0].learned.requiredTags = (maniGraphTags){2, 0};
    maniGraphSource sub = {.skeleton = &s_skeleton,
                           .parameters = s_matchParameters,
                           .nodes = inner,
                           .parameterCount = 4,
                           .nodeCount = 1,
                           .modelCount = 1,
                           .models = kModelSlot};
    const maniGraphModelSource both[2] = {{&s_other, "moves", 5, 0}, kModelSlot[0]};
    maniGraphNodeSource outer[2] = {{maniGraphNodeSubgraph, 0, NULL, .subgraph = {&sub, kMap}},
                                    Above(0)};
    maniGraphSource root = {.skeleton = &s_skeleton,
                            .parameters = reversed,
                            .nodes = outer,
                            .parameterCount = 4,
                            .nodeCount = 2,
                            .output = 1,
                            .modelCount = 2,
                            .models = both};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniGraph g;
    CHECK(maniBuildGraph(builder, &root).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, &g).result == mani_success &&
              maniGraphModelCount(&g) == 2,
          "the subgraph's model joined to the root's");
    CHECK(DataWord(&g, 0, maniGraphLearnedModel) == 1 &&
              DataWord(&g, 0, maniGraphLearnedTrigger) == 2 &&
              DataWord(&g, 0, maniGraphLearnedRequiredTags) == 1 &&
              DataWord(&g, 0, maniGraphLearnedDurationScalar) == 0 &&
              DataWord(&g, 0, maniGraphLearnedQuery) == 3 &&
              DataWord(&g, 0, maniGraphLearnedStates) == STATE &&
              DataWord(&g, 0, maniGraphLearnedFeatures) == FEATURES &&
              DataWord(&g, 0, maniGraphLearnedEvents) == 2,
          "the model, trigger, tags, duration and query as the root names them");
    // The root's model the subgraph's (as MakeLearned gives it): one
    // slot, played.
    root.models = NULL;
    root.modelCount = 0;
    root.parameters = s_matchParameters;
    static const uint32_t kSame[4] = {0, 1, 2, 3};
    outer[0].subgraph.parameters = kSame;
    maniGraphInstance* instance = MakeLearned(root, &g);
    Update(instance, 0.0f);
    CHECK(LearnedBlock(instance, &g, 0)[maniGraphLearnedHas] == 0,
          "the tags 0: no Projector's, nothing projected");
    maniGraphValue v = {{0}};
    v.i = 1;
    Set(instance, 2, maniGraphParameterInt, v);
    Update(instance, 0.0f);
    CHECK(maniGraphModelCount(&g) == 1 && LearnedBlock(instance, &g, 0)[maniGraphLearnedHas] == 1,
          "one slot for both, played");
    maniDestroyBuilder(builder);
}

static uint8_t s_built[16384];
static uint8_t s_read[16384];
static char s_text[16384];
static size_t s_builtSize;
static size_t s_textSize;

static const maniGraphNamedAssets kNamed = {&s_skeleton, kSlots, NULL, kModelSlot, NULL,
                                            CLIPS,       0,      1,    0};

static bool RoundTrip(const maniGraphSource* source)
{
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
              maniWriteGraphJson(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &s_textSize) == mani_success &&
              maniBuildGraphJson(builder, s_text, s_textSize, &kNamed).result == mani_success &&
              maniCopyBuilt(builder, s_read, sizeof s_read, &readSize) == mani_success;
    maniDestroyBuilder(builder);
    s_text[ok ? s_textSize : 0] = 0;
    return ok && readSize == s_builtSize && memcmp(s_read, s_built, s_builtSize) == 0;
}

static maniBuildResult Read(const char* text)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraphJson(builder, text, strlen(text), &kNamed);
    maniDestroyBuilder(builder);
    return r;
}

static void TestJson(void)
{
    MatchParameters();
    maniGraphNodeSource nodes[2] = {LearnedNode(0.1f, 0.5f), Above(0)};
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = s_matchParameters,
                              .nodes = nodes,
                              .parameterCount = 4,
                              .nodeCount = 2,
                              .output = 1,
                              .modelCount = 1,
                              .models = kModelSlot};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "{\"kind\": \"learned\", \"model\": \"steps\", \"trigger\": "
                         "\"search\", \"query\": [\"ahead\"], \"period\": 0.1, "
                         "\"transitionCost\": 0.5, \"requiredTags\": 1, \"excludedTags\": 0, "
                         "\"duration\": {\"parameter\": \"d\"}}") != NULL,
          "the form");
    nodes[0].learned.trigger = MANI_NO_PARAMETER;
    nodes[0].learned.requiredTags = (maniGraphTags){2, 0};
    nodes[0].learned.excludedTags = (maniGraphTags){MANI_NO_PARAMETER, 4294967295u};
    nodes[0].learned.duration = LITERAL(0.125f);
    CHECK(RoundTrip(&source) &&
              strstr(s_text, "\"model\": \"steps\", \"query\": [\"ahead\"]") != NULL &&
              strstr(s_text, "\"requiredTags\": {\"parameter\": \"tags\"}, \"excludedTags\": "
                             "4294967295, \"duration\": 0.125}") != NULL,
          "no trigger, tags from a parameter and literal");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"ahead\", \"type\": \"vec2\"}], \"models\": [\"steps\"], \"nodes\": [{\"kind\": "            \
    "\"learned\", \"model\": \"steps\", \"query\": [\"ahead\"], \"requiredTags\": 1"
#define TAIL "}, {\"kind\": \"inertialize\", \"inputs\": [0]}], \"output\": 1}"

// The text with members left out builds what the source of their
// defaults builds.
static bool Defaults(void)
{
    static const uint32_t kOne[1] = {0};
    maniGraphParameterSource ahead = Parameter("ahead", maniGraphParameterVec2, Int(0));
    maniGraphNodeSource nodes[2] = {{maniGraphNodeLearned, 0, NULL,
                                     .learned = {.trigger = MANI_NO_PARAMETER,
                                                 .query = kOne,
                                                 .queryCount = 1,
                                                 .period = 0.1f,
                                                 .requiredTags = {MANI_NO_PARAMETER, 1},
                                                 .excludedTags = {MANI_NO_PARAMETER, 0},
                                                 .duration = {MANI_NO_PARAMETER, 0.0f}}},
                                    Above(0)};
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = &ahead,
                              .nodes = nodes,
                              .parameterCount = 1,
                              .nodeCount = 2,
                              .output = 1,
                              .modelCount = 1,
                              .models = kModelSlot};
    static const char kText[] = HEAD TAIL;
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    bool ok =
        maniBuildGraph(builder, &source).result == mani_success &&
        maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
        maniBuildGraphJson(builder, kText, sizeof kText - 1, &kNamed).result == mani_success &&
        maniCopyBuilt(builder, s_read, sizeof s_read, &readSize) == mani_success;
    maniDestroyBuilder(builder);
    return ok && readSize == s_builtSize && memcmp(s_read, s_built, s_builtSize) == 0;
}

static void TestJsonForms(void)
{
    CHECK(Read(HEAD TAIL).result == mani_success, "members left out");
    CHECK(Defaults(), "left out, the defaults: a period of 0.1, the rest none or 0");
    static const struct
    {
        const char* text;
        uint32_t form;
        const char* marker;
    } kRefusals[7] = {
        {HEAD ", \"inputs\": []" TAIL, maniGraphFormMember, "inputs"},
        {HEAD ", \"excludedTags\": 4294967296" TAIL, maniGraphFormValue, "4294967296"},
        {HEAD ", \"excludedTags\": {\"parameter\": \"tags\"}" TAIL, maniGraphFormName, "tags\"}"},
        {HEAD ", \"period\": \"often\"" TAIL, maniGraphFormValue, "often\""},
        {HEAD ", \"trigger\": \"search\"" TAIL, maniGraphFormName, "search\""},
        {"{\"format\": \"maul-anim graph\", \"version\": 1, \"models\": [\"others\"], "
         "\"nodes\": [], \"output\": 0}",
         maniGraphFormUnbound, "others"},
        {"{\"format\": \"maul-anim graph\", \"version\": 1, \"nodes\": [{\"kind\": "
         "\"learned\", \"model\": \"steps\"}], \"output\": 0}",
         maniGraphFormName, "steps\"}"}};
    for (int k = 0; k < 7; ++k)
    {
        maniBuildResult r = Read(kRefusals[k].text);
        const char* at = strstr(kRefusals[k].text, kRefusals[k].marker);
        bool ok = r.result == mani_errorInvalid && r.reason == maniBuildReasonForm &&
                  r.other == kRefusals[k].form && at != NULL &&
                  r.item == (uint32_t)(at - kRefusals[k].text);
        if (!ok)
        {
            printf("  form %d: reason %u other %u at %u, want %u at %d\n", k, r.reason, r.other,
                   r.item, kRefusals[k].form, at != NULL ? (int)(at - kRefusals[k].text) : -1);
        }
        CHECK(ok, "a form refused at its bytes");
    }
}

static bool WriteFile(const char* path, const void* data, size_t size)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return false;
    }
    size_t written = fwrite(data, 1, size, out);
    return fclose(out) == 0 && written == size;
}

int main(int argc, char** argv)
{
    Open();
    OpenDatabase();
    OpenWide();
    OpenModel();
    s_other = s_learned;
    s_other.identity ^= 1;
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestRefusals();
    TestOpen();
    TestUnfit();
    TestLongQuery();
    TestRoom(true);
    TestRoom(false);
    TestIdleSlot();
    TestSubgraph();
    TestJson();
    TestJsonForms();
    return s_failures == 0 ? 0 : 1;
}
