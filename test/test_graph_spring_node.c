// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring bone nodes: over the walk, under a seek, a node of a
// spring from the spine to the arm gives, frame by frame, the same bits
// as the walk alone stepped by a standalone spring state at the same
// placement and wind; its reset trigger and the seek above it drop the
// motion as maniResetSprings does; over a blend and under a time scale,
// with two updates to an evaluation and backward ones, it steps by the
// scaled times summed and made positive; a rotation restored off unit
// length is normalized; a saved state restores to the same motion and its
// spring block is checked; the builder's refusals (slot, parameter types,
// an input of differences, a set's state past the limit, the input
// count), opening's checks of its words and flags, its input's, and of
// blocks past 2^32 words; binding's of the set's state against the node's
// words, a slot no node uses free of them; the JSON form written and read
// back; a subgraph's spring nodes, parameters and set slot carried into
// its graph. Run as test_graph_spring_node --seed <graph file> <json
// file> it writes the graph's file and text, seeds of the fuzz targets.

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

static _Alignas(16) uint8_t s_setFiles[2][2048];
static maniSpringSet s_set;
// The same spring with a collider: another identity, a larger state.
static maniSpringSet s_other;
static const maniGraphSpringSetSource kSetSlot[1] = {{&s_set, "hair", 4}};

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

static void MakeSet(uint8_t* file, size_t capacity, uint32_t colliderCount, maniSpringSet* set)
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
    const maniColliderSource far = {
        0, maniColliderSphere, {0.0f, 50.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.1f, 0};
    maniSpringSetSource source = {&s_skeleton, &far, NULL, &spring, colliderCount,
                                  0,           1,    0.0f, 0.0f};
    size_t size = 0;
    CHECK(maniBuildSpringSet(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, capacity, &size) == mani_success &&
              maniOpenSpringSet(file, size, set).result == mani_success,
          "a spring set");
    maniDestroyBuilder(builder);
}

static void OpenSet(void)
{
    MakeSet(s_setFiles[0], sizeof s_setFiles[0], 0, &s_set);
    MakeSet(s_setFiles[1], sizeof s_setFiles[1], 1, &s_other);
}

// Parameters: a placement's position and rotation, a reset trigger, a
// seek's, a wind.
static maniGraphParameterSource s_parameters[5];
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
    s_parameters[4] = Parameter("wind", maniGraphParameterVec3, Float(0.0f));
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
                             .parameterCount = 5,
                             .clipCount = 1,
                             .nodeCount = 3,
                             .output = 2,
                             .springSetCount = 1,
                             .springSets = kSetSlot};
}

static const maniGraphSpringNodeSource kSpring = {0, 0, 1, 2, 4};

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

// The wind of frame i, set on the graph's parameter.
static maniVec3 Wind(Made* m, uint32_t i)
{
    maniVec3 w = {0.0f, 0.5f * cosf(0.2f * (float)i), 2.0f * sinf(0.3f * (float)i)};
    maniGraphValue v = {{w.x, w.y, w.z, 0.0f}};
    CHECK(maniSetGraphParameter(m->live, 4, maniGraphParameterVec3, v) == mani_success, "wind");
    return w;
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
        maniVec3 wind = Wind(&s_made[0], i);
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
        CHECK(maniStepSprings(state, walk, &place, &wind, delta) == mani_success &&
                  maniApplySprings(state, walk) == mani_success,
              "stepped");
        maniQuat turned = maniPoseLocal(walk, 1).rotation;
        same = same && SameLocals(graphPose, walk);
        moved = moved || memcmp(&raw, &turned, sizeof raw) != 0;
    }
    CHECK(same && moved, "the node's pose is the standalone state's, reset and seek alike");
}

// Over a blend of the walk and the run, under a time scale of a half:
// some frames two updates before an evaluation, some a backward update;
// as a standalone state stepped by the scaled times over the same blend.
static maniGraphNodeSource s_timed[5];
static maniGraphNodeSource s_untimed[4];
static const uint32_t kPair[2] = {1, 2};
static const uint32_t kFourth[1] = {3};
static const maniGraphScalar kHalves[2] = {LITERAL(0.5f), LITERAL(0.5f)};

static void TimedGraphs(maniGraphSource* timed, maniGraphSource* plain)
{
    *timed = SpringGraph(kSpring);
    s_timed[0] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kPair, .blend = {kHalves, 0}};
    s_timed[1] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    s_timed[2] = ClipNode(1, 1.0f, maniGraphLoopRepeat);
    s_timed[3] = (maniGraphNodeSource){maniGraphNodeSpring, 1, kFirst, .spring = kSpring};
    s_timed[4] =
        (maniGraphNodeSource){maniGraphNodeTimeScale, 1, kFourth, .timeScale = {LITERAL(0.5f)}};
    memcpy(s_untimed, s_timed, 3 * sizeof *s_timed);
    s_untimed[3] = s_timed[4];
    s_untimed[3].inputs = kFirst;
    timed->nodes = s_timed;
    timed->nodeCount = 5;
    timed->output = 4;
    timed->clipCount = CLIPS;
    *plain = *timed;
    plain->nodes = s_untimed;
    plain->nodeCount = 4;
    plain->output = 3;
}

static void TestTimes(void)
{
    maniGraphSource timed;
    maniGraphSource plain;
    TimedGraphs(&timed, &plain);
    CHECK(Instanced(&timed, &s_made[0]) && Instanced(&plain, &s_made[1]), "two graphs");
    maniPose* graphPose = PoseAt(0);
    maniPose* blended = PoseAt(1);
    maniSpringState* state = NULL;
    CHECK(maniInitSpringState(&s_set, &s_skeleton, s_stateMemory, sizeof s_stateMemory, &state) ==
              mani_success,
          "a standalone state");
    bool same = true;
    for (uint32_t i = 1; i <= 70; ++i)
    {
        float delta = i % 7 == 0 ? -1.0f / 60.0f : 1.0f / 64.0f;
        uint32_t updates = i % 5 == 0 ? 2u : 1u;
        maniTransform place = Place(&s_made[0], i);
        if (updates == 2)
        {
            CHECK(maniUpdateGraph(s_made[0].live, delta, NULL, 0, NULL, NULL) == mani_success &&
                      maniUpdateGraph(s_made[1].live, delta, NULL, 0, NULL, NULL) == mani_success,
                  "an update");
        }
        CHECK(Evaluated(&s_made[0], delta, graphPose) && Evaluated(&s_made[1], delta, blended),
              "a frame");
        float seconds = (float)updates * 0.5f * fabsf(delta);
        CHECK(maniStepSprings(state, blended, &place, NULL, seconds) == mani_success &&
                  maniApplySprings(state, blended) == mani_success,
              "stepped");
        same = same && SameLocals(graphPose, blended);
    }
    CHECK(same, "the node's time scaled, summed and forward, over a blend");
}

// A rotation restored a little off unit length (as a saved state may
// keep one) used as maniStepSprings uses it: normalized.
static void TestRestoredTurn(void)
{
    maniGraphSource springy = SpringGraph(kSpring);
    static maniGraphNodeSource s_plainNodes[2];
    s_plainNodes[0] = s_nodes[0];
    s_plainNodes[1] = s_nodes[2];
    s_plainNodes[1].inputs = kFirst;
    maniGraphSource plain = springy;
    plain.nodes = s_plainNodes;
    plain.nodeCount = 2;
    plain.output = 1;
    CHECK(Instanced(&springy, &s_made[0]) && Instanced(&plain, &s_made[1]), "two graphs");
    size_t size = maniGraphStateSize(s_made[0].bound);
    CHECK(maniSaveGraphInstance(s_made[0].live, s_saved[0], size) == mani_success, "saved");
    const float scale = 1.000004f;
    maniTransform place = {{0.0f, 0.0f, 0.0f},
                           {0.0f, sinf(0.3f) * scale, 0.0f, cosf(0.3f) * scale},
                           {1.0f, 1.0f, 1.0f}};
    const float turn[4] = {place.rotation.x, place.rotation.y, place.rotation.z, place.rotation.w};
    for (uint32_t k = 0; k < 4; ++k)
    {
        uint32_t bits = 0;
        memcpy(&bits, &turn[k], sizeof bits);
        maniStoreU32(s_saved[0] + MANI_GRAPH_STATE_HEADER_SIZE + MANI_GRAPH_VALUE_SIZE + 4 * k,
                     bits);
    }
    CHECK(maniRestoreGraphInstance(s_made[0].live, s_saved[0], size).result == mani_success,
          "restored a little off unit length");
    maniPose* graphPose = PoseAt(0);
    maniPose* walk = PoseAt(1);
    maniSpringState* state = NULL;
    CHECK(maniInitSpringState(&s_set, &s_skeleton, s_stateMemory, sizeof s_stateMemory, &state) ==
              mani_success,
          "a standalone state");
    bool same = true;
    for (uint32_t i = 0; i < 20; ++i)
    {
        CHECK(Evaluated(&s_made[0], 1.0f / 60.0f, graphPose) &&
                  Evaluated(&s_made[1], 1.0f / 60.0f, walk) &&
                  maniStepSprings(state, walk, &place, NULL, 1.0f / 60.0f) == mani_success &&
                  maniApplySprings(state, walk) == mani_success,
              "a frame");
        same = same && SameLocals(graphPose, walk);
    }
    CHECK(same, "the restored rotation normalized");
}

static void TestSaved(void)
{
    maniGraphSource springy = SpringGraph(kSpring);
    CHECK(Instanced(&springy, &s_made[0]) && Instanced(&springy, &s_made[2]), "two instances");
    maniPose* a = PoseAt(0);
    maniPose* b = PoseAt(1);
    // The room the spring node works in holds what another node left
    // there; nothing of it reaches the block.
    const maniGraphInstance* live = s_made[0].live;
    CHECK(maniGraphRoom(s_made[0].bound) >= 4 * (size_t)maniGraphSpringWorkWords(&s_set) &&
              maniGraphInstanceSize(s_made[0].bound) ==
                  live->roomAt + maniGraphRoom(s_made[0].bound),
          "the room holds the set's work area");
    memset((uint8_t*)s_made[0].live + live->roomAt, 0xFF, maniGraphRoom(s_made[0].bound));
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
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + block, 0);
    maniStoreU32(s_saved[1] + block + 4, 0x7FF00000u);
    maniFileResult r = maniRestoreGraphInstance(s_made[2].live, s_saved[1], size);
    CHECK(r.result == mani_errorFormat && r.offset == block, "a time of infinity");
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
    const maniGraphSpringNodeSource wrong[4] = {{0, 2, 1, 2, MANI_NO_PARAMETER},
                                                {0, 0, 0, 2, MANI_NO_PARAMETER},
                                                {0, 0, 1, 1, MANI_NO_PARAMETER},
                                                {0, 0, 1, 2, 1}};
    for (int k = 0; k < 4; ++k)
    {
        source = SpringGraph(wrong[k]);
        CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "a parameter of another type");
    }
    source = SpringGraph((maniGraphSpringNodeSource){0, MANI_NO_PARAMETER, MANI_NO_PARAMETER,
                                                     MANI_NO_PARAMETER, MANI_NO_PARAMETER});
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
    const maniGraphSpringSetSource slot[1] = {{&huge, "hair", 4}};
    source = SpringGraph(kSpring);
    source.springSets = slot;
    CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "a state past the limit");
}

static void WriteHash(uint8_t* file, size_t size)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, (uint64_t)size - 64));
}

// Opening refused at want with the words at offsets set to values.
static bool RefusedAtAll(size_t size, const uint64_t* offsets, const uint32_t* values,
                         uint32_t count, uint64_t want)
{
    static _Alignas(16) uint8_t s_crafted[16384];
    memcpy(s_crafted, s_made[0].file, size);
    for (uint32_t k = 0; k < count; ++k)
    {
        maniStoreU32(s_crafted + offsets[k], values[k]);
    }
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

static bool RefusedAt(size_t size, uint64_t offset, uint32_t value, uint64_t want)
{
    return RefusedAtAll(size, &offset, &value, 1, want);
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
    CHECK(state == 21 + 17 + 10, "the set's state words: a joint and a spring");
    CHECK(RefusedAt(size, words, 1, words) && RefusedAt(size, words + 4, 1, words + 4) &&
              RefusedAt(size, words + 8, 0, words + 8) &&
              RefusedAt(size, words + 12, 0, words + 12) &&
              RefusedAt(size, words + 16, 1, words + 16) &&
              RefusedAt(size, words + 20, 17, words + 20) &&
              RefusedAt(size, words + 20, MANI_GRAPH_SPRING_MAX_WORDS + 1, words + 20) &&
              RefusedAt(size, words + 24, 1, words + 24) &&
              RefusedAt(size, words + 24, 5, words + 24),
          "the words checked");
    CHECK(RefusedAt(size, record, 1u << 16 | maniGraphNodeSpring, record + 2), "differences");
    // The walk's clip node and slot made differences: the spring node's
    // input is.
    uint64_t clips = 0;
    CHECK(maniContainerSection(&c, maniGraphTagClips, &bytes, &length, &clips), "the clips");
    uint32_t walk = 0;
    while (maniLoadU16(g.nodes + (size_t)walk * MANI_GRAPH_NODE_SIZE) != maniGraphNodeClip)
    {
        ++walk;
    }
    const uint64_t offsets[2] = {nodes + (uint64_t)walk * MANI_GRAPH_NODE_SIZE, clips + 24};
    const uint32_t values[2] = {1u << 16 | maniGraphNodeClip, 1};
    CHECK(RefusedAtAll(size, offsets, values, 2, record + 2), "an input of differences");
}

// 257 spring nodes in a chain, each made to take the most state words:
// their blocks pass 2^32 words, refused at the node whose block would.
static void TestBlocksWrap(void)
{
    enum
    {
        kSprings = 257
    };
    static maniGraphNodeSource s_chain[kSprings + 1];
    static uint32_t s_inputs[kSprings];
    static _Alignas(16) uint8_t s_big[65536];
    maniGraphSource source = SpringGraph(kSpring);
    s_chain[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    for (uint32_t k = 1; k <= kSprings; ++k)
    {
        s_inputs[k - 1] = k - 1;
        s_chain[k] =
            (maniGraphNodeSource){maniGraphNodeSpring, 1, &s_inputs[k - 1], .spring = kSpring};
    }
    source.nodes = s_chain;
    source.nodeCount = kSprings + 1;
    source.output = kSprings;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniContainer c;
    const uint8_t* bytes = NULL;
    uint64_t length = 0;
    uint64_t nodes = 0;
    uint64_t data = 0;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_big, sizeof s_big, &size) == mani_success &&
              maniOpenContainer(s_big, size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
                  mani_success &&
              maniContainerSection(&c, maniGraphTagNodes, &bytes, &length, &nodes) &&
              maniContainerSection(&c, maniGraphTagData, &bytes, &length, &data),
          "a chain of spring nodes");
    maniDestroyBuilder(builder);
    uint64_t running = 0;
    uint64_t want = 0;
    for (uint32_t p = 0; p <= kSprings; ++p)
    {
        uint64_t record = nodes + (uint64_t)p * MANI_GRAPH_NODE_SIZE;
        if (maniLoadU16(s_big + record) != maniGraphNodeSpring)
        {
            continue;
        }
        uint64_t words = data + 4 * (uint64_t)maniLoadU32(s_big + record + 12);
        uint64_t block = maniGraphSpringState + (uint64_t)MANI_GRAPH_SPRING_MAX_WORDS;
        want = want == 0 && running + block > UINT32_MAX ? words : want;
        maniStoreU32(s_big + words + 4 * maniGraphSpringBlock, (uint32_t)running);
        maniStoreU32(s_big + words + 4 * maniGraphSpringStateWords, MANI_GRAPH_SPRING_MAX_WORDS);
        running += block;
    }
    WriteHash(s_big, size);
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_big, size, &g);
    CHECK(want != 0 && r.result == mani_errorFormat && r.offset == want, "blocks past 2^32 words");
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
    // A second slot no node uses: any state its set has.
    static const maniGraphSpringSetSource kTwo[2] = {{&s_set, "hair", 4}, {&s_other, "tail", 4}};
    source = SpringGraph(kSpring);
    source.springSets = kTwo;
    source.springSetCount = 2;
    const maniSpringSet* both[2] = {&s_set, &s_other};
    assets.springSets = both;
    assets.springSetCount = 2;
    CHECK(maniGraphSpringSetWords(&s_other) != maniGraphSpringSetWords(&s_set) &&
              Build(&source, &s_made[0], &size).result == mani_success &&
              maniOpenGraph(s_made[0].file, size, &s_made[0].graph).result == mani_success &&
              maniBindGraph(&s_made[0].graph, &assets, s_made[0].binding, sizeof s_made[0].binding,
                            &s_made[0].bound, NULL) == mani_success,
          "a slot no spring node uses bound to a set of another state");
}

static char s_text[16384];

// The writer refuses a source with the build's result: its status,
// reason and place.
static bool RefusedAlike(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult built = maniBuildGraph(builder, source);
    maniBuildResult written = maniWriteGraphJson(builder, source);
    maniDestroyBuilder(builder);
    return built.result == mani_errorInvalid && written.result == built.result &&
           written.reason == built.reason && written.item == built.item &&
           written.other == built.other;
}

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
    CHECK(strstr(s_text, "{\"kind\": \"spring\", \"inputs\": [0], \"springSet\": \"hair\", "
                         "\"position\": \"place\", \"rotation\": \"turn\", \"reset\": \"drop\", "
                         "\"force\": \"wind\"}") != NULL,
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
    source = SpringGraph((maniGraphSpringNodeSource){0, MANI_NO_PARAMETER, MANI_NO_PARAMETER,
                                                     MANI_NO_PARAMETER, MANI_NO_PARAMETER});
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text, "\"springSet\": \"hair\"}") != NULL &&
              maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success,
          "no parameters left out");
    maniDestroyBuilder(builder);
    source = SpringGraph((maniGraphSpringNodeSource){1, MANI_NO_PARAMETER, MANI_NO_PARAMETER,
                                                     MANI_NO_PARAMETER, MANI_NO_PARAMETER});
    CHECK(RefusedAlike(&source), "a spring set slot past the graph's sets");
}

static void TestSubgraph(void)
{
    // The spring graph as a subgraph of a graph without set slots,
    // mapping its parameters to the outer's, in reverse.
    maniGraphSource inner = SpringGraph(kSpring);
    static const uint32_t kMap[5] = {4, 3, 2, 1, 0};
    maniGraphParameterSource outerParameters[5];
    for (uint32_t k = 0; k < 5; ++k)
    {
        outerParameters[k] = s_parameters[4 - k];
    }
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&inner, kMap}};
    maniGraphSource outer = {.skeleton = &s_skeleton,
                             .parameters = outerParameters,
                             .nodes = &node,
                             .parameterCount = 5,
                             .nodeCount = 1};
    size_t size = 0;
    maniGraph g;
    CHECK(Build(&outer, &s_made[0], &size).result == mani_success &&
              maniOpenGraph(s_made[0].file, size, &g).result == mani_success &&
              maniGraphSpringSetCount(&g) == 1 &&
              maniGraphSpringSetIdentity(&g, 0) == maniSpringSetIdentity(&s_set),
          "the subgraph's set slot carried");
    uint32_t data = maniLoadU32(g.nodes + (size_t)SpringAt(&g) * MANI_GRAPH_NODE_SIZE + 12);
    CHECK(maniLoadU32(g.data + 4 * (size_t)(data + maniGraphSpringPosition)) == 4 &&
              maniLoadU32(g.data + 4 * (size_t)(data + maniGraphSpringRotation)) == 3 &&
              maniLoadU32(g.data + 4 * (size_t)(data + maniGraphSpringReset)) == 2 &&
              maniLoadU32(g.data + 4 * (size_t)(data + maniGraphSpringForce)) == 0,
          "its parameters the outer's");
    // Two spring nodes over the slot: one slot, the blocks one after the
    // other.
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeSpring, 1, kSecond, .spring = kSpring};
    CHECK(Build(&outer, &s_made[0], &size).result == mani_success &&
              maniOpenGraph(s_made[0].file, size, &g).result == mani_success &&
              maniGraphSpringSetCount(&g) == 1,
          "two spring nodes over one slot");
    // A slot without its set under two spring nodes: one slot, refused.
    static const maniGraphSpringSetSource kNone[1] = {{NULL, "none", 4}};
    inner.springSets = kNone;
    CHECK(RefusedFor(&outer, maniBuildReasonSpringSetSlot, 0), "a subgraph's slot without its set");
    // A slot past the subgraph's sets, where the outer lists one: refused
    // at the spring node's place, not driven by the outer's.
    inner = SpringGraph(kSpring);
    outer.springSets = inner.springSets;
    outer.springSetCount = 1;
    CHECK(Build(&outer, &s_made[0], &size).result == mani_success,
          "the subgraph's set joined to the outer's");
    inner.springSets = NULL;
    inner.springSetCount = 0;
    CHECK(RefusedFor(&outer, maniBuildReasonNode, 2), "a subgraph's slot past its sets");
    // A slot past the outer's sets, where the subgraph lists one: refused
    // at the outer spring node's place, not driven by the subgraph's.
    inner = SpringGraph(kSpring);
    static const uint32_t kSame[5] = {0, 1, 2, 3, 4};
    static const uint32_t kInner[1] = {1};
    maniGraphNodeSource nodes[2] = {
        {maniGraphNodeSpring, 1, kInner, .spring = kSpring},
        {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL, .subgraph = {&inner, kSame}}};
    maniGraphSource own = {.skeleton = &s_skeleton,
                           .parameters = s_parameters,
                           .nodes = nodes,
                           .parameterCount = 5,
                           .nodeCount = 2,
                           .springSets = inner.springSets,
                           .springSetCount = 1};
    CHECK(Build(&own, &s_made[0], &size).result == mani_success, "the outer's own set");
    own.springSets = NULL;
    own.springSetCount = 0;
    CHECK(RefusedFor(&own, maniBuildReasonNode, 0), "an outer's slot past its sets");
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
    TestTimes();
    TestRestoredTurn();
    TestSaved();
    TestRefusals();
    TestOpen();
    TestBlocksWrap();
    TestBind();
    TestJson();
    TestSubgraph();
    return s_failures == 0 ? 0 : 1;
}
