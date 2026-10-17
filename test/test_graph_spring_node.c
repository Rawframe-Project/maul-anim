// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring bone nodes (mani-0026): over the walk, under a seek, a node of
// a spring from the spine to the arm gives, frame by frame, the same bits
// as the walk alone stepped by a standalone spring state at the same
// placement; its reset trigger and the seek above it drop the motion as
// maniResetSprings does; a saved state restores to the same motion and
// its spring block is checked; the builder's refusals (slot, parameter
// types, an input of differences, a set's state past the limit, the
// input count), opening's checks of its words and flags, binding's of
// the set's state against the node's words; the JSON form written and
// read back; a subgraph's spring node and set slot carried into its
// graph. Run as test_graph_spring_node --seed <graph file> <json file>
// it writes the graph's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_fixture.h"
#include "graph_instance.h"
#include "graph_spring.h"

#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_setFile[2048];
static maniSpringSet s_set;
static const maniGraphSpringSetSource kSetSlot[1] = {{&s_set, "hair", 4, 0}};

// Two graphs at once: each its file, binding and instance.
typedef struct Made
{
    _Alignas(16) uint8_t file[16384];
    _Alignas(8) uint8_t binding[1024];
    _Alignas(8) uint8_t instance[16384];
    maniGraph graph;
    maniGraphBinding* bound;
    maniGraphInstance* live;
} Made;

static Made s_made[3];
static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poses[3][4096];
static _Alignas(8) uint8_t s_stateMemory[1024];
static uint8_t s_saved[2][16384];

static void OpenSet(void)
{
    maniBuilder* builder = NewBuilder();
    maniSpringJointSource joints[2];
    for (int j = 0; j < 2; ++j)
    {
        joints[j] = (maniSpringJointSource){.bone = (uint32_t)j + 1,
                                            .stiffness = 0.5f,
                                            .gravityPower = 1.0f,
                                            .gravityDirection = {0.0f, -1.0f, 0.0f},
                                            .drag = 0.3f,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    maniSpringSource spring = {"arm", 3, 2, joints, NULL, 0, MANI_NO_BONE};
    maniSpringSetSource source = {&s_skeleton, NULL, NULL, &spring, 0, 0, 1, 0.0f, 0.0f, 0};
    size_t size = 0;
    CHECK(maniBuildSpringSet(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_setFile, sizeof s_setFile, &size) == mani_success &&
              maniOpenSpringSet(s_setFile, size, &s_set).result == mani_success,
          "a spring set");
    maniDestroyBuilder(builder);
}

// Parameters: a placement's position and rotation, a reset trigger and a
// seek's.
static maniGraphParameterSource s_parameters[4];
static maniGraphNodeSource s_nodes[3];
static const uint32_t kFirst[1] = {0};
static const uint32_t kSecond[1] = {1};

static void Parameters(void)
{
    maniGraphValue identity = {{0.0f, 0.0f, 0.0f, 1.0f}};
    s_parameters[0] = Parameter("place", maniGraphParameterVec3, Float(0.0f));
    s_parameters[1] = Parameter("turn", maniGraphParameterQuat, identity);
    s_parameters[2] = Parameter("drop", maniGraphParameterTrigger, Int(0));
    s_parameters[3] = Parameter("jump", maniGraphParameterTrigger, Int(0));
}

// The walk, a spring node over it, a seek over the spring node.
static maniGraphSource SpringGraph(maniGraphSpringNodeSource spring)
{
    Parameters();
    s_nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    s_nodes[1] = (maniGraphNodeSource){maniGraphNodeSpring, 1, kFirst, .spring = spring};
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeSeek, 1, kSecond, .seek = {3, LITERAL(0.25f)}};
    return (maniGraphSource){.skeleton = &s_skeleton,
                             .parameters = s_parameters,
                             .clips = kSlots,
                             .nodes = s_nodes,
                             .parameterCount = 4,
                             .clipCount = 1,
                             .nodeCount = 3,
                             .output = 2,
                             .springSetCount = 1,
                             .springSets = kSetSlot};
}

static const maniGraphSpringNodeSource kSpring = {0, 0, 1, 2};

static maniBuildResult Build(const maniGraphSource* source, Made* m, size_t* sizeOut)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, m->file, sizeof m->file, sizeOut) == mani_success, "copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static bool Instanced(const maniGraphSource* source, Made* m)
{
    size_t size = 0;
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    const maniSpringSet* sets[1] = {&s_set};
    maniGraphAssets assets = {.skeleton = &s_skeleton,
                              .clips = clips,
                              .springSets = sets,
                              .clipCount = CLIPS,
                              .springSetCount = 1};
    return Build(source, m, &size).result == mani_success &&
           maniOpenGraph(m->file, size, &m->graph).result == mani_success &&
           maniBindGraph(&m->graph, &assets, m->binding, sizeof m->binding, &m->bound, NULL) ==
               mani_success &&
           maniInitGraphInstance(m->bound, m->instance, sizeof m->instance, &m->live) ==
               mani_success &&
           maniGraphScratchSize(m->bound) <= sizeof s_scratch;
}

static maniPose* PoseAt(int k)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poses[k], sizeof s_poses[k], &pose) == mani_success,
          "a pose");
    return pose;
}

static bool Evaluated(Made* m, float delta, maniPose* pose)
{
    return maniUpdateGraph(m->live, delta, NULL, 0, NULL, NULL) == mani_success &&
           maniEvaluateGraph(m->live, s_scratch, sizeof s_scratch, pose) == mani_success;
}

static bool SameLocals(const maniPose* a, const maniPose* b)
{
    maniTransform x[3];
    maniTransform y[3];
    return maniPoseReadLocals(a, 0, 3, x) == mani_success &&
           maniPoseReadLocals(b, 0, 3, y) == mani_success && memcmp(x, y, sizeof x) == 0;
}

// The spring node's place in an opened graph.
static uint32_t SpringAt(const maniGraph* g)
{
    uint32_t p = 0;
    while (maniLoadU16(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) != maniGraphNodeSpring)
    {
        ++p;
    }
    return p;
}

// The placement of frame i, set on the graph's parameters.
static maniTransform Place(Made* m, uint32_t i)
{
    float a = 0.05f * (float)i;
    maniTransform t = {{0.1f * (float)i, 0.0f, 0.0f}, {0.0f, sinf(a), 0.0f, cosf(a)}, {1, 1, 1}};
    maniGraphValue position = {{t.translation.x, t.translation.y, t.translation.z, 0.0f}};
    maniGraphValue rotation = {{t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w}};
    CHECK(maniSetGraphParameter(m->live, 0, maniGraphParameterVec3, position) == mani_success &&
              maniSetGraphParameter(m->live, 1, maniGraphParameterQuat, rotation) == mani_success &&
              maniGetGraphParameter(m->live, 1, &rotation) == mani_success,
          "placed");
    t.rotation = (maniQuat){rotation.f[0], rotation.f[1], rotation.f[2], rotation.f[3]};
    return t;
}

static void Fire(Made* m, uint32_t trigger)
{
    maniGraphValue on = {{0}};
    on.b = 1;
    CHECK(maniSetGraphParameter(m->live, trigger, maniGraphParameterTrigger, on) == mani_success,
          "fired");
}

static void TestSame(void)
{
    maniGraphSource springy = SpringGraph(kSpring);
    // The walk under the same seek, without the spring node.
    static maniGraphNodeSource s_plainNodes[2];
    s_plainNodes[0] = s_nodes[0];
    s_plainNodes[1] = s_nodes[2];
    s_plainNodes[1].inputs = kFirst;
    maniGraphSource plain = springy;
    plain.nodes = s_plainNodes;
    plain.nodeCount = 2;
    plain.output = 1;
    CHECK(Instanced(&springy, &s_made[0]) && Instanced(&plain, &s_made[1]), "two graphs");
    maniPose* graphPose = PoseAt(0);
    maniPose* walk = PoseAt(1);
    maniSpringState* state = NULL;
    CHECK(maniInitSpringState(&s_set, &s_skeleton, s_stateMemory, sizeof s_stateMemory, &state) ==
              mani_success,
          "a standalone state");
    bool same = true;
    bool moved = false;
    for (uint32_t i = 1; i <= 90; ++i)
    {
        float delta = i % 3 == 0 ? 1.0f / 30.0f : 1.0f / 90.0f;
        maniTransform place = Place(&s_made[0], i);
        if (i == 40)
        {
            Fire(&s_made[0], 2);
            maniResetSprings(state);
        }
        if (i == 70)
        {
            Fire(&s_made[0], 3);
            Fire(&s_made[1], 3);
            maniResetSprings(state);
        }
        CHECK(Evaluated(&s_made[0], delta, graphPose) && Evaluated(&s_made[1], delta, walk),
              "a frame");
        maniQuat raw = maniPoseLocal(walk, 1).rotation;
        CHECK(maniStepSprings(state, walk, &place, delta) == mani_success &&
                  maniApplySprings(state, walk) == mani_success,
              "stepped");
        maniQuat turned = maniPoseLocal(walk, 1).rotation;
        same = same && SameLocals(graphPose, walk);
        moved = moved || memcmp(&raw, &turned, sizeof raw) != 0;
    }
    CHECK(same && moved, "the node's pose is the standalone state's, reset and seek alike");
}

static void TestSaved(void)
{
    maniGraphSource springy = SpringGraph(kSpring);
    CHECK(Instanced(&springy, &s_made[0]) && Instanced(&springy, &s_made[2]), "two instances");
    maniPose* a = PoseAt(0);
    maniPose* b = PoseAt(1);
    for (uint32_t i = 1; i <= 20; ++i)
    {
        (void)Place(&s_made[0], i);
        CHECK(Evaluated(&s_made[0], 1.0f / 50.0f, a), "a frame");
    }
    size_t size = maniGraphStateSize(s_made[0].bound);
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(s_made[0].live, s_saved[0], size) == mani_success &&
              maniRestoreGraphInstance(s_made[2].live, s_saved[0], size).result == mani_success,
          "saved and restored");
    bool same = true;
    for (uint32_t i = 21; i <= 40; ++i)
    {
        (void)Place(&s_made[0], i);
        (void)Place(&s_made[2], i);
        CHECK(Evaluated(&s_made[0], 1.0f / 50.0f, a) && Evaluated(&s_made[2], 1.0f / 50.0f, b),
              "a frame");
        same = same && SameLocals(a, b);
    }
    CHECK(same, "a restored instance moves as the saved one");
    // The spring node's block: the time kept, then the spring state.
    const maniGraph* g = &s_made[0].graph;
    uint32_t data = maniLoadU32(g->nodes + (size_t)SpringAt(g) * MANI_GRAPH_NODE_SIZE + 12);
    uint32_t first = maniLoadU32(g->data + 4 * (size_t)(data + maniGraphSpringBlock));
    uint64_t block = MANI_GRAPH_STATE_HEADER_SIZE +
                     (uint64_t)(g->parameterCount + g->nodeCount) * MANI_GRAPH_VALUE_SIZE +
                     4 * (uint64_t)first;
    const uint32_t words[3] = {0xFFF80000u, 0xBFF00000u, 2};
    const uint64_t at[3] = {block + 4, block + 4, block + 8};
    for (int k = 0; k < 3; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + at[k], words[k]);
        maniFileResult r = maniRestoreGraphInstance(s_made[2].live, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == (k < 2 ? block : block + 8),
              "a spring block's time and state checked");
    }
}

static bool RefusedFor(const maniGraphSource* source, maniBuildReason reason, uint32_t item)
{
    size_t size = 0;
    maniBuildResult r = Build(source, &s_made[0], &size);
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item;
}

static void TestRefusals(void)
{
    maniGraphSpringNodeSource s = kSpring;
    s.springSet = 1;
    maniGraphSource source = SpringGraph(s);
    CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "a slot past the sets");
    const maniGraphSpringNodeSource wrong[3] = {{0, 2, 1, 2}, {0, 0, 0, 2}, {0, 0, 1, 1}};
    for (int k = 0; k < 3; ++k)
    {
        source = SpringGraph(wrong[k]);
        CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "a parameter of another type");
    }
    source = SpringGraph(
        (maniGraphSpringNodeSource){0, MANI_NO_PARAMETER, MANI_NO_PARAMETER, MANI_NO_PARAMETER});
    size_t size = 0;
    CHECK(Build(&source, &s_made[0], &size).result == mani_success, "no parameters");
    source = SpringGraph(kSpring);
    source.clipCount = CLIPS;
    s_nodes[0] = ClipNode(2, 1.0f, maniGraphLoopRepeat);
    CHECK(RefusedFor(&source, maniBuildReasonDifference, 1), "an input of differences");
    source = SpringGraph(kSpring);
    s_nodes[1].inputCount = 0;
    CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "no input");
    // A set whose state passes the limit (its counts are all the size
    // reads).
    maniSpringSet huge = s_set;
    huge.jointCount = MANI_GRAPH_SPRING_MAX_WORDS / 17 + 1;
    const maniGraphSpringSetSource slot[1] = {{&huge, "hair", 4, 0}};
    source = SpringGraph(kSpring);
    source.springSets = slot;
    CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "a state past the limit");
}

static void WriteHash(uint8_t* file, size_t size)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, (uint64_t)size - 64));
}

static bool RefusedAt(size_t size, uint64_t offset, uint32_t value, uint64_t want)
{
    static _Alignas(16) uint8_t s_crafted[16384];
    memcpy(s_crafted, s_made[0].file, size);
    maniStoreU32(s_crafted + offset, value);
    WriteHash(s_crafted, size);
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_crafted, size, &g);
    if (r.offset != want)
    {
        printf("  at %llu, expected %llu\n", (unsigned long long)r.offset,
               (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want;
}

static void TestOpen(void)
{
    maniGraphSource source = SpringGraph(kSpring);
    size_t size = 0;
    CHECK(Build(&source, &s_made[0], &size).result == mani_success, "built");
    maniContainer c;
    const uint8_t* bytes = NULL;
    uint64_t length = 0;
    uint64_t nodes = 0;
    uint64_t data = 0;
    CHECK(maniOpenContainer(s_made[0].file, size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, maniGraphTagNodes, &bytes, &length, &nodes) &&
              maniContainerSection(&c, maniGraphTagData, &bytes, &length, &data),
          "the sections");
    maniGraph g;
    CHECK(maniOpenGraph(s_made[0].file, size, &g).result == mani_success, "opened");
    uint64_t record = nodes + (uint64_t)SpringAt(&g) * MANI_GRAPH_NODE_SIZE;
    uint64_t words = data + 4 * (uint64_t)maniLoadU32(s_made[0].file + record + 12);
    uint32_t state = maniLoadU32(s_made[0].file + words + 4 * maniGraphSpringStateWords);
    CHECK(state == 18 + 17 + 10, "the set's state words: a joint and a spring");
    CHECK(RefusedAt(size, words, 1, words) && RefusedAt(size, words + 4, 1, words + 4) &&
              RefusedAt(size, words + 8, 0, words + 8) &&
              RefusedAt(size, words + 12, 0, words + 12) &&
              RefusedAt(size, words + 16, 1, words + 16) &&
              RefusedAt(size, words + 20, 17, words + 20) &&
              RefusedAt(size, words + 20, MANI_GRAPH_SPRING_MAX_WORDS + 1, words + 20),
          "the words checked");
    CHECK(RefusedAt(size, record, 1u << 16 | maniGraphNodeSpring, record + 2), "differences");
}

static void TestBind(void)
{
    maniGraphSource source = SpringGraph(kSpring);
    size_t size = 0;
    CHECK(Build(&source, &s_made[0], &size).result == mani_success &&
              maniOpenGraph(s_made[0].file, size, &s_made[0].graph).result == mani_success,
          "opened");
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    // A view of the set's identity but other counts: its state differs.
    maniSpringSet other = s_set;
    other.colliderCount = 1;
    const maniSpringSet* sets[1] = {&other};
    maniGraphAssets assets = {.skeleton = &s_skeleton,
                              .clips = clips,
                              .springSets = sets,
                              .clipCount = CLIPS,
                              .springSetCount = 1};
    uint32_t unbound = 0;
    CHECK(maniBindGraph(&s_made[0].graph, &assets, s_made[0].binding, sizeof s_made[0].binding,
                        &s_made[0].bound, &unbound) == mani_errorInvalid &&
              unbound == 1,
          "a set whose state is not the node's");
}

static char s_text[16384];

static void TestJson(void)
{
    maniGraphSource source = SpringGraph(kSpring);
    size_t size = 0;
    CHECK(Build(&source, &s_made[0], &size).result == mani_success &&
              maniOpenGraph(s_made[0].file, size, &s_made[0].graph).result == mani_success,
          "opened");
    uint64_t identity = s_made[0].graph.identity;
    maniBuilder* builder = NewBuilder();
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text,
                 "{\"kind\": \"spring\", \"inputs\": [0], \"springSet\": \"hair\", "
                 "\"position\": \"place\", \"rotation\": \"turn\", \"reset\": \"drop\"}") != NULL,
          "a spring node by its set's and parameters' names");
    maniGraphNamedAssets assets = {.skeleton = &s_skeleton,
                                   .clips = kSlots,
                                   .springSets = kSetSlot,
                                   .clipCount = CLIPS,
                                   .springSetCount = 1};
    maniGraph read;
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success &&
              maniCopyBuilt(builder, s_made[1].file, sizeof s_made[1].file, &size) ==
                  mani_success &&
              maniOpenGraph(s_made[1].file, size, &read).result == mani_success &&
              read.identity == identity,
          "read back to the same graph");
    source = SpringGraph(
        (maniGraphSpringNodeSource){0, MANI_NO_PARAMETER, MANI_NO_PARAMETER, MANI_NO_PARAMETER});
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text, "\"springSet\": \"hair\"}") != NULL &&
              maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success,
          "no parameters left out");
    maniDestroyBuilder(builder);
}

static void TestSubgraph(void)
{
    // The spring graph as a subgraph of a graph without set slots,
    // mapping its parameters to the outer's.
    maniGraphSource inner = SpringGraph(kSpring);
    static const uint32_t kMap[4] = {0, 1, 2, 3};
    maniGraphParameterSource outerParameters[4];
    memcpy(outerParameters, s_parameters, sizeof outerParameters);
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&inner, kMap}};
    maniGraphSource outer = {.skeleton = &s_skeleton,
                             .parameters = outerParameters,
                             .nodes = &node,
                             .parameterCount = 4,
                             .nodeCount = 1};
    size_t size = 0;
    maniGraph g;
    CHECK(Build(&outer, &s_made[0], &size).result == mani_success &&
              maniOpenGraph(s_made[0].file, size, &g).result == mani_success &&
              maniGraphSpringSetCount(&g) == 1 &&
              maniGraphSpringSetIdentity(&g, 0) == maniSpringSetIdentity(&s_set),
          "the subgraph's set slot carried");
    // A slot without its set under two spring nodes: one slot, refused.
    static const maniGraphSpringSetSource kNone[1] = {{NULL, "none", 4, 0}};
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeSpring, 1, kSecond, .spring = kSpring};
    inner.springSets = kNone;
    CHECK(RefusedFor(&outer, maniBuildReasonSpringSetSlot, 0), "a subgraph's slot without its set");
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

// The spring graph's file and JSON text.
static bool Seed(const char* graphPath, const char* textPath)
{
    maniGraphSource source = SpringGraph(kSpring);
    size_t size = 0;
    size_t length = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = Build(&source, &s_made[0], &size).result == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success &&
              WriteFile(graphPath, s_made[0].file, size) && WriteFile(textPath, s_text, length);
    maniDestroyBuilder(builder);
    return ok;
}

int main(int argc, char** argv)
{
    Open();
    OpenSet();
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        return Seed(argv[2], argv[3]) && s_failures == 0 ? 0 : 1;
    }
    TestSame();
    TestSaved();
    TestRefusals();
    TestOpen();
    TestBind();
    TestJson();
    TestSubgraph();
    return s_failures == 0 ? 0 : 1;
}
