// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Subgraphs inside subgraphs: a graph whose subgraph holds
// another, the three carrying every slot kind (clips, a motion database,
// a learned model, a spring set) and every parameter type through their
// maps, built from its JSON form against the graph fuzz targets' assets:
// each slot and parameter once, the inner graph's clip the middle one's
// slot, every node inlined, the file opened; built again with every
// allocation failing in turn. Run under the sanitizers in CI, as every
// test. Run as test_graph_nested --seed <graph> <json> it writes seeds
// for the graph targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_json_assets.h"
#include "test_budget.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdio.h>
#include <string.h>

// Outer: the middle graph under an additive wave and an inertialize
// node. Middle: walk, a match node and the inner graph blended. Inner: a
// spring node over run, walk, a learned node and a pose input.
static const char kText[] =
    "{\"format\": \"maul-anim graph\", \"version\": 1,\n"
    " \"parameters\": [{\"name\": \"mix\", \"type\": \"float\", \"default\": 0.5}, {\"name\": "
    "\"gait\", \"type\": \"int\"},\n"
    "  {\"name\": \"on\", \"type\": \"bool\"}, {\"name\": \"jump\", \"type\": \"trigger\"},\n"
    "  {\"name\": \"ahead\", \"type\": \"vec2\"}, {\"name\": \"place\", \"type\": \"vec3\"},\n"
    "  {\"name\": \"turn\", \"type\": \"quat\"}, {\"name\": \"ragdoll\", \"type\": \"pose\"}],\n"
    " \"clips\": [\"wave\"],\n"
    " \"nodes\": [\n"
    "  {\"kind\": \"inertialize\", \"inputs\": [1], \"trigger\": \"jump\"},\n"
    "  {\"kind\": \"additive\", \"inputs\": [3, 2], \"weights\": [{\"parameter\": \"mix\"}]},\n"
    "  {\"kind\": \"clip\", \"clip\": \"wave\"},\n"
    "  {\"kind\": \"subgraph\", \"parameters\": {\"w\": \"mix\", \"go\": \"jump\", \"dir\": "
    "\"ahead\", \"p\": \"place\", \"r\": \"turn\", \"rd\": \"ragdoll\"}, \"graph\": {\n"
    "   \"parameters\": [{\"name\": \"w\", \"type\": \"float\"}, {\"name\": \"go\", \"type\": "
    "\"trigger\"},\n"
    "    {\"name\": \"dir\", \"type\": \"vec2\"}, {\"name\": \"p\", \"type\": \"vec3\"},\n"
    "    {\"name\": \"r\", \"type\": \"quat\"}, {\"name\": \"rd\", \"type\": \"pose\"}],\n"
    "   \"clips\": [\"walk\"], \"databases\": [\"moves\"],\n"
    "   \"nodes\": [\n"
    "    {\"kind\": \"blend\", \"inputs\": [1, 2, 3], \"weights\": [{\"parameter\": \"w\"}, 0.5, "
    "0.25]},\n"
    "    {\"kind\": \"clip\", \"clip\": \"walk\"},\n"
    "    {\"kind\": \"match\", \"database\": \"moves\", \"trigger\": \"go\"},\n"
    "    {\"kind\": \"subgraph\", \"parameters\": {\"dir\": \"dir\", \"p\": \"p\", \"r\": \"r\", "
    "\"rd\": \"rd\", \"drop\": \"go\"}, \"graph\": {\n"
    "     \"parameters\": [{\"name\": \"dir\", \"type\": \"vec2\"}, {\"name\": \"p\", \"type\": "
    "\"vec3\"},\n"
    "      {\"name\": \"r\", \"type\": \"quat\"}, {\"name\": \"rd\", \"type\": \"pose\"},\n"
    "      {\"name\": \"drop\", \"type\": \"trigger\"}],\n"
    "     \"clips\": [\"run\", \"walk\"], \"models\": [\"steps\"], \"springSets\": [\"hair\"],\n"
    "     \"nodes\": [\n"
    "      {\"kind\": \"spring\", \"inputs\": [1], \"springSet\": \"hair\", \"position\": \"p\", "
    "\"rotation\": \"r\", \"reset\": \"drop\"},\n"
    "      {\"kind\": \"blend\", \"inputs\": [2, 3, 4, 5], \"weights\": [0.25, 0.25, 0.25, "
    "0.25]},\n"
    "      {\"kind\": \"clip\", \"clip\": \"run\"},\n"
    "      {\"kind\": \"clip\", \"clip\": \"walk\"},\n"
    "      {\"kind\": \"learned\", \"model\": \"steps\"},\n"
    "      {\"kind\": \"pose\", \"parameter\": \"rd\"}],\n"
    "     \"output\": 0}}],\n"
    "   \"output\": 0}}],\n"
    " \"output\": 0}\n";

static maniBuildResult BuildText(maniBuilder* builder)
{
    return maniBuildGraphJson(builder, kText, sizeof kText - 1,
                              &(maniGraphNamedAssets){&s_skeleton, kHost, kHostDatabase, kHostModel,
                                                      kHostSet, 3, 1, 1, 1, NULL});
}

static size_t s_size;

static void TestInlined(void)
{
    maniGraph graph;
    CHECK(BuildText(s_builder).result == mani_success &&
              maniCopyBuilt(s_builder, s_file, sizeof s_file, &s_size) == mani_success &&
              maniOpenGraph(s_file, s_size, &graph).result == mani_success,
          "built and opened");
    CHECK(graph.clipCount == 3 && graph.databaseCount == 1 && graph.modelCount == 1 &&
              graph.springSetCount == 1,
          "each slot once, the inner walk the middle's");
    CHECK(graph.parameterCount == 8 && graph.nodeCount == 12,
          "the parameters mapped through, every node inlined");
}

static TestOutcome Build(maniBuilder* builder, void* context)
{
    (void)context;
    return TestOutcomeOf(BuildText(builder).result);
}

static void TestAllocations(void)
{
    CHECK(TestSweep(Build, NULL, 400), "every allocation failing in turn");
}

static bool WriteFile(const char* path, const void* bytes, size_t size)
{
    FILE* f = fopen(path, "wb");
    bool ok = f != NULL && fwrite(bytes, 1, size, f) == size;
    return f != NULL && fclose(f) == 0 && ok;
}

int main(int argc, char** argv)
{
    SetupGraphAssets();
    TestInlined();
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteFile(argv[2], s_file, s_size) && WriteFile(argv[3], kText, sizeof kText - 1)
                   ? 0
                   : 1;
    }
    TestAllocations();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
