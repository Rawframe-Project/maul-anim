// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring set slots in graphs: a graph listing spring sets by
// name, built, opened and read back; the builder's refusals of a slot
// without a set, of another skeleton or listed twice, and of its names;
// opening's checks of the slots' count, section and names; binding by
// identity through the assets, a slot left unbound reported after the
// clip, database and model slots; the JSON form's "springSets" list
// written and read back, and a name the host's sets lack refused; a
// subgraph's slots carried into the graph it is inlined in.

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_padding.h"
#include "learned_fixture.h"

#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/spring.h"

#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_setFiles[2][2048];
static maniSpringSet s_set;
static maniSpringSet s_other;
static _Alignas(16) uint8_t s_graphFile[16384];
static size_t s_graphSize;
static _Alignas(16) uint8_t s_crafted[16384];
static _Alignas(8) uint8_t s_binding[1024];
static maniGraphNodeSource s_nodes[1];
static maniLearnedModel s_learned;

// Two sets of a spring from the spine to the arm, of two stiffnesses.
static void OpenSets(void)
{
    maniBuilder* builder = NewBuilder();
    maniSpringSet* sets[2] = {&s_set, &s_other};
    for (int k = 0; k < 2; ++k)
    {
        maniSpringJointSource joints[2];
        for (int j = 0; j < 2; ++j)
        {
            joints[j] = (maniSpringJointSource){.bone = (uint32_t)j + 1,
                                                .stiffness = 1.0f + (float)k,
                                                .gravityDirection = {0.0f, -1.0f, 0.0f},
                                                .drag = 0.4f,
                                                .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
        }
        maniSpringSource spring = {"arm", 3, 2, joints, NULL, 0, MANI_NO_BONE};
        maniSpringSetSource source = {&s_skeleton, NULL, NULL, &spring, 0, 0, 1, 0.0f, 0.0f};
        size_t size = 0;
        CHECK(maniBuildSpringSet(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_setFiles[k], sizeof s_setFiles[k], &size) ==
                      mani_success &&
                  maniOpenSpringSet(s_setFiles[k], size, sets[k]).result == mani_success,
              "a spring set");
    }
    maniDestroyBuilder(builder);
}

// A clip node over the walk and the spring sets given.
static maniGraphSource Graph(const maniGraphSpringSetSource* sets, uint32_t count)
{
    s_nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    return (maniGraphSource){.skeleton = &s_skeleton,
                             .clips = kSlots,
                             .nodes = s_nodes,
                             .clipCount = CLIPS,
                             .nodeCount = 1,
                             .springSetCount = count,
                             .springSets = sets};
}

static maniBuildResult BuildGraph(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result == mani_success)
    {
        CHECK(GraphPaddingIgnored(source), "the same graph under either padding");
        CHECK(maniCopyBuilt(builder, s_graphFile, sizeof s_graphFile, &s_graphSize) == mani_success,
              "copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static bool RefusedFor(const maniGraphSource* source, maniBuildReason reason, uint32_t item,
                       uint32_t other)
{
    maniBuildResult r = BuildGraph(source);
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item &&
           r.other == other;
}

static const maniGraphSpringSetSource kTwo[2] = {{&s_set, "hair", 4}, {&s_other, "tail", 4}};

static void TestBuild(void)
{
    maniGraphSource source = Graph(kTwo, 2);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of two spring sets");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    uint32_t length = 0;
    const char* name = maniGraphSpringSetName(&g, 1, &length);
    CHECK(maniGraphSpringSetCount(&g) == 2 &&
              maniGraphSpringSetIdentity(&g, 0) == maniSpringSetIdentity(&s_set) &&
              maniGraphSpringSetIdentity(&g, 1) == maniSpringSetIdentity(&s_other) && length == 4 &&
              memcmp(name, "tail", 4) == 0,
          "its spring set slots read back");
    CHECK(maniGraphSpringSetCount(NULL) == 0 && maniGraphSpringSetIdentity(&g, 2) == 0 &&
              maniGraphSpringSetIdentity(NULL, 0) == 0 &&
              maniGraphSpringSetName(&g, 2, &length) == NULL && length == 0 &&
              maniGraphSpringSetName(NULL, 0, NULL) == NULL,
          "past the slots");
    maniGraphSpringSetSource bad[2] = {kTwo[0], kTwo[1]};
    bad[1].set = NULL;
    source = Graph(bad, 2);
    CHECK(RefusedFor(&source, maniBuildReasonSpringSetSlot, 1, 0), "a slot without a set");
    bad[0].set = NULL;
    bad[1].set = kTwo[1].set;
    CHECK(RefusedFor(&source, maniBuildReasonSpringSetSlot, 0, 0), "a first slot without a set");
    bad[0] = kTwo[0];
    bad[1].set = &s_set;
    CHECK(RefusedFor(&source, maniBuildReasonSpringSetSlot, 1, 0), "a set listed twice");
    maniSpringSet stranger = s_other;
    stranger.skeletonIdentity ^= 1;
    bad[1].set = &stranger;
    CHECK(RefusedFor(&source, maniBuildReasonSpringSetSlot, 1, 0), "a set of another skeleton");
    bad[1] = (maniGraphSpringSetSource){&s_other, "hair", 4};
    CHECK(RefusedFor(&source, maniBuildReasonSpringSetList | maniBuildReasonDuplicateName, 1, 0),
          "a name twice");
    bad[1] = (maniGraphSpringSetSource){&s_other, "", 0};
    CHECK(RefusedFor(&source, maniBuildReasonSpringSetList | maniBuildReasonName, 1, 0), "no name");
    source = Graph(NULL, 1);
    CHECK(RefusedFor(&source, maniBuildReasonCount, 0, 0), "a count without its list");
    source = Graph(kTwo, MANI_GRAPH_MAX_SPRING_SETS + 1);
    CHECK(RefusedFor(&source, maniBuildReasonCount, 0, 0), "too many sets");
}

static uint64_t SectionAt(uint32_t tag, uint64_t* sizeOut)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_graphFile, s_graphSize, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, sizeOut, &at),
          "the section");
    return at;
}

static void WriteHash(uint8_t* file)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + MANI_CONTAINER_HEADER_SIZE,
                                          (uint64_t)s_graphSize - MANI_CONTAINER_HEADER_SIZE));
}

static bool RefusedAt(uint64_t offset, uint32_t value, uint64_t want)
{
    memcpy(s_crafted, s_graphFile, s_graphSize);
    maniStoreU32(s_crafted + offset, value);
    WriteHash(s_crafted);
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_crafted, s_graphSize, &g);
    if (r.offset != want)
    {
        printf("  at %llu, expected %llu\n", (unsigned long long)r.offset,
               (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want;
}

static void TestOpen(void)
{
    maniGraphSource source = Graph(kTwo, 2);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of two spring sets");
    uint64_t size = 0;
    uint64_t header = SectionAt(maniGraphTagHeader, &size);
    CHECK(size == MANI_GRAPH_HEADER_SIZE, "the header");
    uint64_t sets = SectionAt(maniGraphTagSpringSets, &size);
    CHECK(size == 2 * MANI_GRAPH_SPRING_SET_SIZE, "two records");
    CHECK(RefusedAt(header + 56, MANI_GRAPH_MAX_SPRING_SETS + 1, header + 56), "too many sets");
    CHECK(RefusedAt(header + 60, MANI_GRAPH_MAX_MAPS + 1, header + 60), "too many maps");
    // The sets' entry is the table's last.
    uint64_t entry =
        MANI_CONTAINER_HEADER_SIZE + 24 * (uint64_t)(maniLoadU32(s_graphFile + 12) - 1) + 16;
    CHECK(RefusedAt(header + 56, 1, entry), "a count the section does not hold");
    CHECK(RefusedAt(sets + MANI_GRAPH_SPRING_SET_SIZE + 16, 1u << 20,
                    sets + MANI_GRAPH_SPRING_SET_SIZE + 16),
          "a name past the names");
    CHECK(RefusedAt(sets + 20, 0, sets + 16), "an empty name");
    CHECK(RefusedAt(sets + 8, 0, sets + 16), "a name of another hash");
}

static void TestBind(void)
{
    const maniGraphSpringSetSource slots[1] = {{&s_set, "hair", 4}};
    maniGraphSource source = Graph(slots, 1);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of a set");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    maniSpringSet stranger = s_set;
    stranger.skeletonIdentity ^= 1;
    const maniSpringSet* sets[3] = {&stranger, &s_other, &s_set};
    maniGraphAssets assets = {.skeleton = &s_skeleton,
                              .clips = clips,
                              .springSets = sets,
                              .clipCount = CLIPS,
                              .springSetCount = 3};
    maniGraphBinding* binding = NULL;
    uint32_t unbound = 0;
    CHECK(maniGraphBindingSize(&g) <= sizeof s_binding &&
              maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_success &&
              unbound == MANI_NO_TRACK && maniGraphBoundSpringSet(binding, 0) == &s_set,
          "bound to the set of its identity");
    assets.springSetCount = 2;
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_errorInvalid &&
              unbound == CLIPS,
          "no set of its identity and skeleton: the slot after the other slots");
    assets.springSets = NULL;
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_errorInvalid &&
              unbound == MANI_NO_TRACK,
          "sets NULL with a count");
    assets.springSetCount = 0;
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, NULL) ==
              mani_errorInvalid,
          "no sets");
    // With a database and a model slot bound, the set's slot after them.
    MakeNetworks();
    maniLearnedModelSource model = Source();
    CHECK(Opened(&model, &s_learned), "a model");
    const maniGraphModelSource modelSlot[1] = {{&s_learned, "steps", 5}};
    source.databases = kDatabaseSlot;
    source.databaseCount = 1;
    source.models = modelSlot;
    source.modelCount = 1;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success,
          "a graph of a database, a model and a set");
    const maniMotionDatabase* databases[1] = {&s_database};
    const maniLearnedModel* models[1] = {&s_learned};
    assets = (maniGraphAssets){.skeleton = &s_skeleton,
                               .clips = clips,
                               .databases = databases,
                               .models = models,
                               .springSets = sets,
                               .clipCount = CLIPS,
                               .databaseCount = 1,
                               .modelCount = 1,
                               .springSetCount = 2};
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_errorInvalid &&
              unbound == CLIPS + 2,
          "after the database and model slots");
    source = Graph(slots, 1);
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success,
          "the graph of a set again");
    size_t size = maniGraphBindingSize(&g);
    source = Graph(NULL, 0);
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              size == maniGraphBindingSize(&g) + sizeof(const maniSpringSet*),
          "a set's pointer counted in the binding's size");
}

static char s_text[16384];

static void TestJson(void)
{
    const maniGraphSpringSetSource slots[1] = {{&s_set, "hair", 4}};
    maniGraphSource source = Graph(slots, 1);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of a set");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    uint64_t identity = g.identity;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text, "\"springSets\": [\n    \"hair\"\n  ],") != NULL,
          "a springSets list by name");
    maniGraphNamedAssets assets = {.skeleton = &s_skeleton,
                                   .clips = kSlots,
                                   .springSets = slots,
                                   .clipCount = CLIPS,
                                   .springSetCount = 1};
    maniGraph read;
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success &&
              maniCopyBuilt(builder, s_graphFile, sizeof s_graphFile, &s_graphSize) ==
                  mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &read).result == mani_success &&
              read.identity == identity,
          "read back to the same graph");
    const maniGraphSpringSetSource unnamed[2] = {{&s_other, NULL, 4}, {&s_set, "hair", 4}};
    assets.springSets = unnamed;
    assets.springSetCount = 2;
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success,
          "a host set without a name passed over");
    assets.springSetCount = 1;
    const maniGraphSpringSetSource other[1] = {{&s_set, "mane", 4}};
    assets.springSets = other;
    maniBuildResult r = maniBuildGraphJson(builder, s_text, size, &assets);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonForm &&
              r.other == maniGraphFormUnbound,
          "a set the host does not name");
    assets.springSets = NULL;
    uint32_t misuses = maniBuilderMisuses(builder);
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == misuses + 1,
          "sets NULL with a count");
    maniDestroyBuilder(builder);
}

static void TestSubgraph(void)
{
    // The root lists one set; a subgraph node over a graph listing it.
    const maniGraphSpringSetSource slots[1] = {{&s_set, "hair", 4}};
    maniGraphSource inner = Graph(slots, 1);
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&inner, NULL}};
    maniGraphSource outer = {.skeleton = &s_skeleton,
                             .nodes = &node,
                             .nodeCount = 1,
                             .springSetCount = 1,
                             .springSets = slots};
    maniGraph g;
    CHECK(BuildGraph(&outer).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              maniGraphSpringSetCount(&g) == 1 &&
              maniGraphSpringSetIdentity(&g, 0) == maniSpringSetIdentity(&s_set),
          "the root's slots kept through inlining");
    // The root's and the subgraph's lists past the limit together.
    static maniGraphSpringSetSource s_many[MANI_GRAPH_MAX_SPRING_SETS];
    inner.springSets = s_many;
    inner.springSetCount = MANI_GRAPH_MAX_SPRING_SETS;
    CHECK(RefusedFor(&outer, maniBuildReasonCount, 0, 0), "too many sets with a subgraph's");
}

int main(void)
{
    Open();
    OpenDatabase();
    OpenSets();
    TestBuild();
    TestOpen();
    TestBind();
    TestJson();
    TestSubgraph();
    return s_failures == 0 ? 0 : 1;
}
