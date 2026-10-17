// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned model slots in graphs (mani-0025): a graph listing a model by
// name, built, opened and read back; the builder's refusals of a slot
// without a model, of another skeleton or listed twice, and of its names;
// opening's checks of the slots' count, section and names; binding by
// identity through the assets, a slot left unbound reported after the
// clip and database slots; the JSON form's "models" list written and
// read back, and a name the host's models lack refused.

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "learned_fixture.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdio.h>
#include <string.h>

static maniLearnedModel s_learned;
static maniLearnedModel s_other;
static _Alignas(16) uint8_t s_graphFile[16384];
static size_t s_graphSize;
static _Alignas(16) uint8_t s_crafted[16384];
static _Alignas(8) uint8_t s_binding[1024];
static maniGraphNodeSource s_nodes[1];

// A clip node over the walk, the fixture's database and the models given.
static maniGraphSource Graph(const maniGraphModelSource* models, uint32_t count)
{
    s_nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    maniGraphSource source = {&s_skeleton, NULL, kSlots, NULL,          s_nodes, 0, CLIPS, 0,
                              1,           0,    1,      kDatabaseSlot, count,   0, models};
    return source;
}

static maniBuildResult BuildGraph(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result == mani_success)
    {
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

static void OpenModels(void)
{
    MakeNetworks();
    CHECK(Opened(&(maniLearnedModelSource){0}, &s_learned) == false, "no model of nothing");
    maniLearnedModelSource source = Source();
    CHECK(Opened(&source, &s_learned), "a model");
    // A copy under another identity: a second model to the slots' checks.
    s_other = s_learned;
    s_other.identity ^= 1;
}

static void TestBuild(void)
{
    const maniGraphModelSource slots[2] = {{&s_learned, "steps", 5, 0}, {&s_other, "moves", 5, 0}};
    maniGraphSource source = Graph(slots, 2);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of two models");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    uint32_t length = 0;
    const char* name = maniGraphModelName(&g, 1, &length);
    CHECK(maniGraphModelCount(&g) == 2 && maniGraphModelIdentity(&g, 0) == s_learned.identity &&
              maniGraphModelIdentity(&g, 1) == s_other.identity && length == 5 &&
              memcmp(name, "moves", 5) == 0,
          "its model slots read back");
    CHECK(maniGraphModelCount(NULL) == 0 && maniGraphModelIdentity(&g, 2) == 0 &&
              maniGraphModelIdentity(NULL, 0) == 0 && maniGraphModelName(&g, 2, &length) == NULL &&
              length == 0 && maniGraphModelName(NULL, 0, NULL) == NULL,
          "past the slots");
    maniGraphModelSource bad[2] = {slots[0], slots[1]};
    bad[1].model = NULL;
    source = Graph(bad, 2);
    CHECK(RefusedFor(&source, maniBuildReasonModel, 1, 0), "a slot without a model");
    bad[1].model = &s_learned;
    CHECK(RefusedFor(&source, maniBuildReasonModel, 1, 0), "a model listed twice");
    maniLearnedModel stranger = s_other;
    stranger.skeletonIdentity ^= 1;
    bad[1].model = &stranger;
    CHECK(RefusedFor(&source, maniBuildReasonModel, 1, 0), "a model of another skeleton");
    bad[1] = (maniGraphModelSource){&s_other, "steps", 5, 0};
    CHECK(RefusedFor(&source, maniBuildReasonModelList | maniBuildReasonDuplicateName, 1, 0),
          "a name twice");
    bad[1] = (maniGraphModelSource){&s_other, "", 0, 0};
    CHECK(RefusedFor(&source, maniBuildReasonModelList | maniBuildReasonName, 1, 0), "no name");
    source = Graph(NULL, 1);
    CHECK(RefusedFor(&source, maniBuildReasonCount, 0, 0), "a count without its list");
    source = Graph(slots, MANI_GRAPH_MAX_MODELS + 1);
    CHECK(RefusedFor(&source, maniBuildReasonCount, 0, 0), "too many models");
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
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, (uint64_t)s_graphSize - 64));
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
    const maniGraphModelSource slots[2] = {{&s_learned, "steps", 5, 0}, {&s_other, "moves", 5, 0}};
    maniGraphSource source = Graph(slots, 2);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of two models");
    uint64_t size = 0;
    uint64_t header = SectionAt(maniGraphTagHeader, &size);
    uint64_t models = SectionAt(maniGraphTagModels, &size);
    CHECK(size == 2 * MANI_GRAPH_MODEL_SIZE, "two records");
    CHECK(RefusedAt(header + 52, MANI_GRAPH_MAX_MODELS + 1, header + 52), "too many models");
    // The models' entry is the table's last.
    uint64_t entry = 64 + 24 * (uint64_t)(maniLoadU32(s_graphFile + 12) - 1) + 16;
    CHECK(RefusedAt(header + 52, 1, entry), "a count the section does not hold");
    CHECK(RefusedAt(models + MANI_GRAPH_MODEL_SIZE + 16, 1u << 20,
                    models + MANI_GRAPH_MODEL_SIZE + 16),
          "a name past the names");
    CHECK(RefusedAt(models + 20, 0, models + 16), "an empty name");
    CHECK(RefusedAt(models + 8, 0, models + 16), "a name of another hash");
}

static void TestBind(void)
{
    const maniGraphModelSource slots[1] = {{&s_learned, "steps", 5, 0}};
    maniGraphSource source = Graph(slots, 1);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of a model");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    const maniMotionDatabase* databases[1] = {&s_database};
    maniLearnedModel stranger = s_learned;
    stranger.skeletonIdentity ^= 1;
    const maniLearnedModel* models[3] = {&stranger, &s_other, &s_learned};
    maniGraphAssets assets = {&s_skeleton, clips, databases, models, CLIPS, 1, 3, 0};
    maniGraphBinding* binding = NULL;
    uint32_t unbound = 0;
    CHECK(maniGraphBindingSize(&g) <= sizeof s_binding &&
              maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_success &&
              unbound == MANI_NO_TRACK && maniGraphBoundModel(binding, 0) == &s_learned,
          "bound to the model of its identity");
    assets.modelCount = 2;
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_errorInvalid &&
              unbound == CLIPS + 1,
          "no model of its identity and skeleton: the slot after the clips and databases");
    assets.models = NULL;
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, &unbound) ==
                  mani_errorInvalid &&
              unbound == MANI_NO_TRACK,
          "models NULL with a count");
    assets.modelCount = 0;
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, NULL) ==
              mani_errorInvalid,
          "no models");
    assets = (maniGraphAssets){NULL, clips, databases, models, CLIPS, 1, 3, 0};
    CHECK(maniBindGraph(&g, &assets, s_binding, sizeof s_binding, &binding, NULL) ==
                  mani_errorInvalid &&
              maniBindGraph(&g, NULL, s_binding, sizeof s_binding, &binding, NULL) ==
                  mani_errorInvalid,
          "no skeleton, no assets");
    assets.skeleton = &s_skeleton;
    CHECK(maniBindGraph(&g, &assets, s_binding, maniGraphBindingSize(&g) - 1, &binding, NULL) ==
              mani_errorCapacity,
          "less than the binding's size");
    size_t size = maniGraphBindingSize(&g);
    source = Graph(NULL, 0);
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              size == maniGraphBindingSize(&g) + sizeof(const maniLearnedModel*),
          "a model's pointer counted in the binding's size");
}

static char s_text[16384];

static void TestJson(void)
{
    const maniGraphModelSource slots[1] = {{&s_learned, "steps", 5, 0}};
    maniGraphSource source = Graph(slots, 1);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of a model");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    uint64_t identity = g.identity;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text, "\"models\": [\n    \"steps\"\n  ],") != NULL, "a models list by name");
    maniGraphNamedAssets assets = {&s_skeleton, kSlots, kDatabaseSlot, slots, CLIPS, 1, 1, 0};
    maniGraph read;
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success &&
              maniCopyBuilt(builder, s_graphFile, sizeof s_graphFile, &s_graphSize) ==
                  mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &read).result == mani_success &&
              read.identity == identity,
          "read back to the same graph");
    const maniGraphModelSource unnamed[2] = {{&s_other, NULL, 5, 0}, {&s_learned, "steps", 5, 0}};
    assets.models = unnamed;
    assets.modelCount = 2;
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success,
          "a host model without a name passed over");
    assets.modelCount = 1;
    const maniGraphModelSource other[1] = {{&s_learned, "jumps", 5, 0}};
    assets.models = other;
    maniBuildResult r = maniBuildGraphJson(builder, s_text, size, &assets);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonForm &&
              r.other == maniGraphFormUnbound,
          "a model the host does not name");
    assets.models = NULL;
    uint32_t misuses = maniBuilderMisuses(builder);
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_errorInvalid &&
              maniBuildGraphJson(builder, s_text, size, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == misuses + 2,
          "models NULL with a count; no assets");
    maniDestroyBuilder(builder);
}

int main(void)
{
    Open();
    OpenDatabase();
    OpenModels();
    TestBuild();
    TestOpen();
    TestBind();
    TestJson();
    return s_failures == 0 ? 0 : 1;
}
