// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion matching nodes' files and states (mani-0024): the builder's
// refusals of database slots (none, a repeat, a clip no slot has, names)
// and of nodes (no inertialize node above, each field out of range, a
// query not fitting the database); binding's refusals (a database
// missing or not the slot's); opening's checks of the database records,
// their clip slots and every node word; a state saved and restored
// playing alike, and each word of a saved block checked; a subgraph's
// database and its clips joined at inlining; the JSON form, round trip
// and refusals; a float parameter querying a float track. Run as
// test_graph_match_file --seed <graph file> <json file> it writes the
// round trip's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_match_fixture.h"

#include <stdio.h>

static maniBuildResult BuildMatch(const maniGraphNodeSource* nodes, uint32_t count,
                                  const maniGraphDatabaseSource* databases, uint32_t databaseCount,
                                  const maniGraphClipSource* clips, uint32_t clipCount)
{
    MatchParameters();
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = s_matchParameters,
                              .clips = clips,
                              .nodes = nodes,
                              .parameterCount = 4,
                              .clipCount = clipCount,
                              .nodeCount = count,
                              .output = count - 1,
                              .databaseCount = databaseCount,
                              .databases = databases};
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

// The node changed, under an inertialize node: refused for its field.
static maniBuildResult NodeBuilt(maniGraphNodeSource node)
{
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    return BuildMatch(nodes, 2, kDatabaseSlot, 1, kSlots, CLIPS);
}

static void TestDatabaseRefusals(void)
{
    maniGraphNodeSource nodes[2] = {MatchNode(1.0f), Above(0)};
    const maniGraphDatabaseSource none[1] = {{NULL, "moves", 5, 0}};
    const maniGraphDatabaseSource twice[2] = {{&s_database, "a", 1, 0}, {&s_database, "b", 1, 0}};
    const maniGraphDatabaseSource unnamed[1] = {{&s_database, "", 0, 0}};
    const maniGraphDatabaseSource same[2] = {{&s_database, "a", 1, 0}, {&s_database, "a", 1, 0}};
    CHECK(BuildMatch(nodes, 2, kDatabaseSlot, 1, kSlots, CLIPS).result == mani_success,
          "the graph builds");
    CHECK(RefusedAs(BuildMatch(nodes, 2, none, 1, kSlots, CLIPS), maniBuildReasonDatabase, 0, 0),
          "a slot without a database");
    CHECK(RefusedAs(BuildMatch(nodes, 2, twice, 2, kSlots, CLIPS), maniBuildReasonDatabase, 1, 0),
          "a database twice");
    CHECK(RefusedAs(BuildMatch(nodes, 2, kDatabaseSlot, 1, kSlots + 1, CLIPS - 1),
                    maniBuildReasonDatabase, 0, 0),
          "the walk in no clip slot");
    CHECK(RefusedAs(BuildMatch(nodes, 2, unnamed, 1, kSlots, CLIPS),
                    maniBuildReasonDatabaseList | maniBuildReasonName, 0, 0),
          "a slot without a name");
    CHECK(RefusedAs(BuildMatch(nodes, 2, same, 2, kSlots, CLIPS),
                    maniBuildReasonDatabaseList | maniBuildReasonDuplicateName, 1, 0),
          "two slots of one name");
    CHECK(RefusedAs(BuildMatch(nodes, 2, NULL, 1, kSlots, CLIPS), maniBuildReasonCount, 0, 0),
          "a count without slots");
    static maniGraphDatabaseSource many[MANI_GRAPH_MAX_DATABASES + 1];
    for (uint32_t k = 0; k <= MANI_GRAPH_MAX_DATABASES; ++k)
    {
        many[k] = kDatabaseSlot[0];
    }
    CHECK(RefusedAs(BuildMatch(nodes, 2, many, MANI_GRAPH_MAX_DATABASES + 1, kSlots, CLIPS),
                    maniBuildReasonCount, 0, 0),
          "more slots than the limit");
    CHECK(
        RefusedAs(BuildMatch(nodes, 1, kDatabaseSlot, 1, kSlots, CLIPS), maniBuildReasonNode, 0, 0),
        "no inertialize node above");
}

static void TestNodeRefusals(void)
{
    static const uint32_t kFloat[1] = {3};
    static const uint32_t kTwo[2] = {0, 0};
    maniGraphNodeSource n[12];
    for (int k = 0; k < 12; ++k)
    {
        n[k] = MatchNode(1.0f);
    }
    n[0].match.database = 1;
    n[1].match.trigger = 0;
    n[2].match.period = 0.0f;
    n[3].match.jumpThreshold = -1.0f;
    n[4].match.continuingBias = INFINITY;
    n[5].match.historyCount = MANI_MATCH_MAX_HISTORY + 1;
    n[6].match.requiredTags = (maniGraphTags){3, 0};
    n[7].match.excludedTags = (maniGraphTags){2, 1};
    n[8].match.query = kFloat;
    n[9].match.query = kTwo;
    n[9].match.queryCount = 2;
    n[10].match.queryCount = 0;
    n[11].match.historyTime = NAN;
    maniGraphNodeSource unlisted = MatchNode(1.0f);
    unlisted.match.query = NULL;
    CHECK(RefusedAs(NodeBuilt(unlisted), maniBuildReasonNode, 0, 0), "a query count, no query");
    for (int k = 0; k < 12; ++k)
    {
        CHECK(RefusedAs(NodeBuilt(n[k]), maniBuildReasonNode, 0, 0), "a field refused");
    }
    maniGraphNodeSource d = MatchNode(1.0f);
    d.match.duration = LITERAL(-1.0f);
    CHECK(RefusedAs(NodeBuilt(d), maniBuildReasonScalar, 0, 0), "a negative duration");
    d.match.duration = (maniGraphScalar){0, 0.0f};
    CHECK(RefusedAs(NodeBuilt(d), maniBuildReasonScalar, 0, 0), "a duration not a float");
}

static void TestBind(void)
{
    maniGraph g;
    (void)Matching(MatchNode(1.0f), &g);
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    maniGraphBinding* binding = NULL;
    uint32_t unbound = 0;
    CHECK(maniBindGraph(&g,
                        &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, CLIPS, 0, 0, 0,
                                           NULL, NULL, 0, 0, NULL},
                        s_matchBinding, sizeof s_matchBinding, &binding,
                        &unbound) == mani_errorInvalid &&
              unbound == CLIPS,
          "no database: the database slot unbound");
    CHECK(maniBindGraph(&g,
                        &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, CLIPS, 1, 0, 0,
                                           NULL, NULL, 0, 0, NULL},
                        s_matchBinding, sizeof s_matchBinding, &binding, NULL) == mani_errorInvalid,
          "a count without databases");
    CHECK(maniGraphDatabaseCount(&g) == 1 &&
              maniGraphDatabaseIdentity(&g, 0) == maniMotionDatabaseIdentity(&s_database) &&
              maniGraphDatabaseIdentity(&g, 1) == 0 && maniGraphDatabaseCount(NULL) == 0,
          "the slot's identity");
    uint32_t length = 0;
    const char* name = maniGraphDatabaseName(&g, 0, &length);
    CHECK(name != NULL && length == 5 && memcmp(name, "moves", 5) == 0 &&
              maniGraphDatabaseName(&g, 1, &length) == NULL && length == 0,
          "the slot's name");
    CHECK(maniGraphBindingSize(&g) >=
              offsetof(maniGraphBinding, clips) + (CLIPS + 1) * sizeof(void*),
          "room for the database");
}

// The graph last made with a byte of a section set, opened; the offset
// refused (0 when opened).
static uint64_t CraftedAt(uint32_t tag, uint64_t offset, uint32_t value, uint64_t* sectionAt)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    s_size = (size_t)maniLoadU64(s_file + 24);
    CHECK(maniOpenContainer(s_file, s_size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
                  mani_success &&
              maniContainerSection(&c, tag, &data, &size, sectionAt),
          "the section");
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + *sectionAt + offset, value);
    Rehash(s_crafted);
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, s_size, &crafted);
    return r.result == mani_errorFormat ? r.offset : 0;
}

static bool RefusedAtByte(uint32_t tag, uint64_t offset, uint32_t value, uint64_t want)
{
    uint64_t at = 0;
    uint64_t got = CraftedAt(tag, offset, value, &at);
    if (got != at + want)
    {
        printf("  refused at %llu, want %llu\n", (unsigned long long)got,
               (unsigned long long)(at + want));
    }
    return got == at + want;
}

static void TestOpen(void)
{
    maniGraph g;
    (void)Matching(MatchNode(1.0f), &g);
    CHECK(RefusedAtByte(maniGraphTagDatabases, 16, 100000, 16), "a name past the names");
    CHECK(RefusedAtByte(maniGraphTagDatabases, 24, 1, 24), "clips not from the first");
    CHECK(RefusedAtByte(maniGraphTagDatabases, 28, 0, 28), "no clips");
    CHECK(RefusedAtByte(maniGraphTagDatabases, 28, 3, 28), "clips past the list");
    CHECK(RefusedAtByte(maniGraphTagDatabaseClips, 4, CLIPS, 4), "a clip slot past the slots");
    CHECK(RefusedAtByte(maniGraphTagDatabaseClips, 0, 2, 0), "a clip slot of differences");
    const uint32_t kWords[11][2] = {{maniGraphMatchDatabase, 1},
                                    {maniGraphMatchTrigger, 0},
                                    {maniGraphMatchPeriod, 0},
                                    {maniGraphMatchJumpThreshold, 0xBF800000u},
                                    {maniGraphMatchContinuingBias, 0x7F800000u},
                                    {maniGraphMatchHistoryCount, MANI_MATCH_MAX_HISTORY + 1},
                                    {maniGraphMatchRequiredTags, 3},
                                    {maniGraphMatchExcludedTags, 0},
                                    {maniGraphMatchDurationScalar, 0},
                                    {maniGraphMatchBlock, 5},
                                    {maniGraphMatchQuery, 1}};
    for (int k = 0; k < 11; ++k)
    {
        CHECK(RefusedAtWord(&g, 0, kWords[k][0], kWords[k][1]), "a node word refused");
    }
    CHECK(RefusedAtWordOf(&g, 0, maniGraphMatchDurationScalar + 1, 0xBF800000u,
                          maniGraphMatchDurationScalar),
          "a negative duration");
    uint64_t at = 0;
    maniFileResult r = Crafted(&g, 0, maniGraphMatchQueryCount, 2, &at);
    CHECK(r.result == mani_errorFormat, "a query count past the words");
    uint64_t databases = 0;
    uint64_t clipsAt = 0;
    uint64_t refused = CraftedAt(maniGraphTagDatabases, 28, 1, &databases);
    (void)CraftedAt(maniGraphTagDatabaseClips, 0, 0, &clipsAt);
    CHECK(refused == clipsAt, "fewer clips than the list");
    CHECK(RefusedAtByte(maniGraphTagNodes, 2, 1, 2), "a node of differences");
    // A literal duration and tags from the int parameter tags.
    maniGraphNodeSource literal = MatchNode(1.0f);
    literal.match.duration = (maniGraphScalar){MANI_NO_PARAMETER, 0.25f};
    literal.match.requiredTags = (maniGraphTags){2, 0};
    (void)Matching(literal, &g);
    CHECK(RefusedAtWordOf(&g, 0, maniGraphMatchDurationScalar + 1, 0xBF800000u,
                          maniGraphMatchDurationScalar),
          "a negative literal duration");
    CHECK(RefusedAtWordOf(&g, 0, maniGraphMatchRequiredTags + 1, 1, maniGraphMatchRequiredTags),
          "a tags parameter's literal not 0");
}

// The graph last made with a word of a section set: it opens, and
// binding it refuses its database slot.
static bool Unbindable(uint32_t tag, uint64_t offset, uint32_t value)
{
    uint64_t at = 0;
    maniGraph crafted;
    CHECK(CraftedAt(tag, offset, value, &at) == 0 &&
              maniOpenGraph(s_crafted, s_size, &crafted).result == mani_success,
          "a crafted graph opens");
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    const maniMotionDatabase* databases[1] = {&s_database};
    maniGraphBinding* binding = NULL;
    uint32_t unbound = 0;
    return maniBindGraph(&crafted,
                         &(maniGraphAssets){&s_skeleton, clips, databases, NULL, NULL, CLIPS, 1, 0,
                                            0, NULL, NULL, 0, 0, NULL},
                         s_matchBinding, sizeof s_matchBinding, &binding,
                         &unbound) == mani_errorInvalid &&
           unbound == CLIPS;
}

// Files that open but name a database its slot does not fit: its clips
// in other slots, a query parameter of the wrong type.
static void TestUnfit(void)
{
    maniGraph g;
    (void)Matching(MatchNode(1.0f), &g);
    CHECK(Unbindable(maniGraphTagDatabaseClips, 0, 1), "the walk's place given the run");
    uint32_t data = maniLoadU32(g.nodes + 12);
    CHECK(Unbindable(maniGraphTagData, 4 * (uint64_t)(data + maniGraphMatchQuery), 3),
          "a float parameter where the trajectory wants a vec2");
}

static _Alignas(8) uint8_t s_other[8192];
static uint8_t s_saved[2][4096];

static void TestSaved(void)
{
    maniGraphNodeSource node = MatchNode(0.1f);
    node.match.historyCount = 2;
    node.match.historyTime = 0.3f;
    maniGraph g;
    maniGraphInstance* instance = Matching(node, &g);
    Update(instance, 0.0f);
    Update(instance, 0.35f);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.1f);
    size_t size = maniGraphStateSize(instance->binding);
    maniGraphInstance* other = NULL;
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniRestoreGraphInstance(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    Ahead(instance, 0.5f, 0.0f);
    Ahead(other, 0.5f, 0.0f);
    for (int k = 0; k < 6; ++k)
    {
        Update(instance, 0.15f);
        Update(other, 0.15f);
    }
    CHECK(EntryOf(instance, &g, 0) == EntryOf(other, &g, 0) &&
              State(instance, &g, 0)->time == State(other, &g, 0)->time &&
              memcmp(NodeBlock(instance, &g, 0, maniGraphMatchBlock),
                     NodeBlock(other, &g, 0, maniGraphMatchBlock),
                     4 * MANI_GRAPH_MATCH_BLOCK_WORDS) == 0,
          "playing alike");
    // The node's block is the first; its playhead the first node's time.
    uint32_t block = MANI_GRAPH_STATE_HEADER_SIZE + 6 * MANI_GRAPH_VALUE_SIZE;
    uint32_t time = MANI_GRAPH_STATE_HEADER_SIZE + 4 * MANI_GRAPH_VALUE_SIZE + 8;
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              s_saved[0][block + 4 * maniGraphMatchPicks] == 2,
          "saved with two picks");
    const uint32_t kBad[7][3] = {{block + 4 * maniGraphMatchEntry, 2, 0},
                                 {time + 4, 0x40140000u, 0},
                                 {block + 4 * maniGraphMatchClock, 0xBF800000u, 1},
                                 {block + 4 * maniGraphMatchPicks, 3, 2},
                                 {block + 4 * maniGraphMatchNext, 2, 3},
                                 {block + 4 * maniGraphMatchHistory, 21, 4},
                                 {block + 4 * (maniGraphMatchHistory + 2), 1, 6}};
    for (int k = 0; k < 7; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + kBad[k][0], kBad[k][1]);
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
        if (r.result != mani_errorFormat || r.offset != block + 4 * kBad[k][2])
        {
            printf("  saved %d refused %d at %llu, want %u\n", k, (int)r.result,
                   (unsigned long long)r.offset, block + 4 * kBad[k][2]);
        }
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kBad[k][2],
              "a saved block refused at its word");
    }
    // One pick kept of two: the next place must follow it.
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + block + 4 * maniGraphMatchPicks, 1);
    maniStoreU32(s_saved[1] + block + 4 * maniGraphMatchNext, 0);
    maniStoreU32(s_saved[1] + block + 4 * (maniGraphMatchHistory + 1), 0);
    maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
    CHECK(r.result == mani_errorFormat && r.offset == block + 4 * maniGraphMatchNext,
          "the next place not after the picks");
}

// A subgraph holding the node and the database, its clips the walk and
// the run alone; the root none of its own.
static void TestSubgraph(void)
{
    MatchParameters();
    static const uint32_t kMap[4] = {0, 1, 2, 3};
    maniGraphNodeSource inner[1] = {MatchNode(1.0f)};
    maniGraphSource sub = {.skeleton = &s_skeleton,
                           .parameters = s_matchParameters,
                           .clips = kSlots,
                           .nodes = inner,
                           .parameterCount = 4,
                           .clipCount = 2,
                           .nodeCount = 1,
                           .databaseCount = 1,
                           .databases = kDatabaseSlot};
    maniGraphNodeSource outer[2] = {{maniGraphNodeSubgraph, 0, NULL, .subgraph = {&sub, kMap}},
                                    Above(0)};
    maniGraphSource root = {.skeleton = &s_skeleton,
                            .parameters = s_matchParameters,
                            .nodes = outer,
                            .parameterCount = 4,
                            .nodeCount = 2,
                            .output = 1};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniGraph g;
    const maniClip* clips[2] = {&s_clips[1], &s_clips[0]};
    const maniMotionDatabase* databases[1] = {&s_database};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(maniBuildGraph(builder, &root).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, &g).result == mani_success &&
              maniGraphClipCount(&g) == 2 && maniGraphDatabaseCount(&g) == 1 &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, clips, databases, NULL, NULL, 2, 1, 0,
                                               0, NULL, NULL, 0, 0, NULL},
                            s_matchBinding, sizeof s_matchBinding, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "the subgraph's database and clips joined");
    maniDestroyBuilder(builder);
    Update(instance, 0.0f);
    CHECK(Playing(instance, &g, 0, 0, 0.0), "the walk");
    // A slot without its database under two match nodes: one slot,
    // refused.
    static const maniGraphDatabaseSource kNone[1] = {{NULL, "none", 4, 0}};
    static const uint32_t kTwo[2] = {1, 2};
    static const maniGraphScalar kHalves[2] = {LITERAL(0.5f), LITERAL(0.5f)};
    maniGraphNodeSource two[3] = {
        {maniGraphNodeBlend, 2, kTwo, .blend = {kHalves, 0, 0}}, MatchNode(1.0f), MatchNode(1.0f)};
    sub.nodes = two;
    sub.nodeCount = 3;
    sub.databases = kNone;
    builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, &root);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonDatabase && r.item == 0,
          "a subgraph's slot without its database");
    maniDestroyBuilder(builder);
}

// The subgraph's parameters mapped to the root's in reverse, its
// database joined to the root's second: every reference remapped.
static _Alignas(16) uint8_t s_boneFile[1 << 15];

static void TestSubgraphMapped(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channel = {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f};
    maniMotionDatabase bones;
    BuildDatabase(&channel, 1, s_boneFile, sizeof s_boneFile, &bones);
    MatchParameters();
    maniGraphParameterSource reversed[4] = {s_matchParameters[3], s_matchParameters[2],
                                            s_matchParameters[1], s_matchParameters[0]};
    static const uint32_t kMap[4] = {3, 2, 1, 0};
    maniGraphNodeSource inner[1] = {MatchNode(1.0f)};
    inner[0].match.requiredTags = (maniGraphTags){2, 0};
    maniGraphSource sub = {.skeleton = &s_skeleton,
                           .parameters = s_matchParameters,
                           .clips = kSlots,
                           .nodes = inner,
                           .parameterCount = 4,
                           .clipCount = 2,
                           .nodeCount = 1,
                           .databaseCount = 1,
                           .databases = kDatabaseSlot};
    const maniGraphDatabaseSource both[2] = {{&bones, "bones", 5, 0}, kDatabaseSlot[0]};
    maniGraphNodeSource outer[2] = {{maniGraphNodeSubgraph, 0, NULL, .subgraph = {&sub, kMap}},
                                    Above(0)};
    maniGraphSource root = {.skeleton = &s_skeleton,
                            .parameters = reversed,
                            .clips = kSlots,
                            .nodes = outer,
                            .parameterCount = 4,
                            .clipCount = 2,
                            .nodeCount = 2,
                            .output = 1,
                            .databaseCount = 2,
                            .databases = both};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniGraph g;
    CHECK(maniBuildGraph(builder, &root).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, &g).result == mani_success &&
              maniGraphDatabaseCount(&g) == 2,
          "the subgraph's database joined to the root's");
    maniDestroyBuilder(builder);
    CHECK(DataWord(&g, 0, maniGraphMatchDatabase) == 1 &&
              DataWord(&g, 0, maniGraphMatchTrigger) == 2 &&
              DataWord(&g, 0, maniGraphMatchRequiredTags) == 1 &&
              DataWord(&g, 0, maniGraphMatchDurationScalar) == 0 &&
              DataWord(&g, 0, maniGraphMatchQuery) == 3,
          "the database, trigger, tags, duration and query as the root names them");
}

static uint8_t s_built[16384];
static uint8_t s_read[16384];
static char s_text[16384];
static size_t s_builtSize;
static size_t s_textSize;

static bool RoundTrip(const maniGraphSource* source)
{
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
              maniWriteGraphJson(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &s_textSize) == mani_success &&
              maniBuildGraphJson(builder, s_text, s_textSize,
                                 &(maniGraphNamedAssets){&s_skeleton, kSlots, kDatabaseSlot, NULL,
                                                         NULL, CLIPS, 1, 0, 0, NULL})
                      .result == mani_success &&
              maniCopyBuilt(builder, s_read, sizeof s_read, &readSize) == mani_success;
    maniDestroyBuilder(builder);
    s_text[ok ? s_textSize : 0] = 0;
    return ok && readSize == s_builtSize && memcmp(s_read, s_built, s_builtSize) == 0;
}

static maniBuildResult Read(const char* text)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r =
        maniBuildGraphJson(builder, text, strlen(text),
                           &(maniGraphNamedAssets){&s_skeleton, kSlots, kDatabaseSlot, NULL, NULL,
                                                   CLIPS, 1, 0, 0, NULL});
    maniDestroyBuilder(builder);
    return r;
}

static void TestJson(void)
{
    MatchParameters();
    maniGraphNodeSource nodes[2] = {MatchNode(1.0f), Above(0)};
    // The walk and the run alone, as the JSON fuzz target's host has them.
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = s_matchParameters,
                              .clips = kSlots,
                              .nodes = nodes,
                              .parameterCount = 4,
                              .clipCount = 2,
                              .nodeCount = 2,
                              .output = 1,
                              .databaseCount = 1,
                              .databases = kDatabaseSlot};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "\"databases\": [\n    \"moves\"\n  ],") != NULL &&
              strstr(s_text, "{\"kind\": \"match\", \"database\": \"moves\", \"trigger\": "
                             "\"search\", \"query\": [\"ahead\"], \"period\": 1, "
                             "\"jumpThreshold\": 0, \"endExclusion\": 0.2, \"historyTime\": 0, "
                             "\"continuingBias\": 0, \"transitionCost\": 0, \"historyCount\": "
                             "0, \"requiredTags\": 0, \"excludedTags\": 0, \"duration\": "
                             "{\"parameter\": \"d\"}}") != NULL,
          "the form");
    nodes[0].match.trigger = MANI_NO_PARAMETER;
    nodes[0].match.requiredTags = (maniGraphTags){2, 0};
    nodes[0].match.excludedTags = (maniGraphTags){MANI_NO_PARAMETER, 4294967295u};
    nodes[0].match.historyCount = 3;
    nodes[0].match.continuingBias = -0.5f;
    CHECK(RoundTrip(&source) &&
              strstr(s_text, "\"database\": \"moves\", \"query\": [\"ahead\"]") != NULL &&
              strstr(s_text, "\"historyCount\": 3, \"requiredTags\": {\"parameter\": "
                             "\"tags\"}, \"excludedTags\": 4294967295") != NULL,
          "no trigger, tags from a parameter and literal");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"ahead\", \"type\": \"vec2\"}], \"clips\": [\"walk\", \"run\"], \"databases\": "             \
    "[\"moves\"], \"nodes\": [{\"kind\": \"match\", \"database\": \"moves\", \"query\": "          \
    "[\"ahead\"]"
#define TAIL "}, {\"kind\": \"inertialize\", \"inputs\": [0]}], \"output\": 1}"

// The text with members left out builds what the source of their
// defaults builds.
static bool Defaults(void)
{
    static const uint32_t kOne[1] = {0};
    maniGraphParameterSource ahead = Parameter("ahead", maniGraphParameterVec2, Int(0));
    maniGraphNodeSource nodes[2] = {{maniGraphNodeMatch, 0, NULL,
                                     .match = {.trigger = MANI_NO_PARAMETER,
                                               .query = kOne,
                                               .queryCount = 1,
                                               .period = 0.1f,
                                               .requiredTags = {MANI_NO_PARAMETER, 0},
                                               .excludedTags = {MANI_NO_PARAMETER, 0},
                                               .duration = {MANI_NO_PARAMETER, 0.0f}}},
                                    Above(0)};
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .parameters = &ahead,
                              .clips = kSlots,
                              .nodes = nodes,
                              .parameterCount = 1,
                              .clipCount = 2,
                              .nodeCount = 2,
                              .output = 1,
                              .databaseCount = 1,
                              .databases = kDatabaseSlot};
    static const char kText[] = HEAD TAIL;
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
              maniBuildGraphJson(builder, kText, sizeof kText - 1,
                                 &(maniGraphNamedAssets){&s_skeleton, kSlots, kDatabaseSlot, NULL,
                                                         NULL, CLIPS, 1, 0, 0, NULL})
                      .result == mani_success &&
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
    } kRefusals[6] = {
        {HEAD ", \"inputs\": []" TAIL, maniGraphFormMember, "inputs"},
        {HEAD ", \"requiredTags\": 4294967296" TAIL, maniGraphFormValue, "4294967296"},
        {HEAD ", \"excludedTags\": {\"parameter\": \"tags\"}" TAIL, maniGraphFormName, "tags\"}"},
        {HEAD ", \"historyCount\": -1" TAIL, maniGraphFormValue, "-1"},
        {"{\"format\": \"maul-anim graph\", \"version\": 1, \"databases\": [\"others\"], "
         "\"nodes\": [], \"output\": 0}",
         maniGraphFormUnbound, "others"},
        {"{\"format\": \"maul-anim graph\", \"version\": 1, \"clips\": [\"walk\"], "
         "\"nodes\": [{\"kind\": \"match\", \"database\": \"moves\"}], \"output\": 0}",
         maniGraphFormName, "moves\"}"}};
    for (int k = 0; k < 6; ++k)
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

// A database of a float track: clip 7 made again with a float track
// speed, the database's one channel its value now, the query the float
// parameter d. Last, for clip 7 changes.
static _Alignas(16) uint8_t s_speedFile[1 << 14];

static void TestFloatQuery(void)
{
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kWalk[6] = {0, 1, 0, 1, 1, 0};
    static const float kSpeed[2] = {0.0f, 1.0f};
    static const float kNow[1] = {0.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const uint32_t kSpeedQuery[1] = {3};
    const maniBoneTrackSource walk = {
        0, 0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone};
    const maniFloatTrackSource speed = {
        "speed", 5, 0, {kTimes, kSpeed, 2, maniInterpolationLinear}};
    maniBuilder* builder = NewBuilder();
    OpenClip(builder, 7,
             (maniClipSource){.boneTracks = &walk, .floatTracks = &speed, .floatTrackCount = 1});
    const maniClip* clips[1] = {&s_clips[7]};
    const maniMatchChannelSource channel = {maniMatchFloatTrack, 0, 0, 1, kNow, "speed", 5, 1.0f};
    const maniMatchEntrySource entry = {0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 0u};
    maniMotionDatabaseSource source = {&s_skeleton, clips, &channel,       &entry,        1, 1,
                                       1,           10.0f, maniMatchAxisY, maniMatchAxisZ};
    maniMotionDatabase d;
    size_t size = 0;
    CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_speedFile, sizeof s_speedFile, &size) == mani_success &&
              maniOpenMotionDatabase(s_speedFile, size, &d).result == mani_success,
          "a database of speed");
    maniDestroyBuilder(builder);
    const maniGraphDatabaseSource slot[1] = {{&d, "speeds", 6, 0}};
    maniGraphNodeSource node = MatchNode(1.0f);
    node.match.query = kSpeedQuery;
    MatchParameters();
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    maniGraph g;
    CHECK(MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                      .nodes = nodes,
                                      .parameterCount = 4,
                                      .nodeCount = 2,
                                      .output = 1,
                                      .databaseCount = 1,
                                      .databases = slot},
                    &g) != NULL,
          "a float parameter for a float track");
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
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestDatabaseRefusals();
    TestNodeRefusals();
    TestBind();
    TestOpen();
    TestUnfit();
    TestSaved();
    TestSubgraph();
    TestSubgraphMapped();
    TestJson();
    TestJsonForms();
    TestFloatQuery();
    return s_failures == 0 ? 0 : 1;
}
