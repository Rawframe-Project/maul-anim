// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned motion matching nodes playing: each update checked
// against a reference of the node kept here from the model's own
// functions (the state stepped at the model's rate, interpolated
// between steps, projected at the period or the trigger and replaced
// past the transition cost, an inertialization requested; the root
// moved by the Decompressor's velocities and its events listed as they
// rise past 0.5) for transition costs of none, some and too much to
// pass; the pose at the interpolated state; the trigger consumed; a node
// weighed 0 left alone; a seek starting it over; a state saved and
// restored playing alike, and after the largest deltas; each word of a
// saved block checked.

#include "graph_learned_fixture.h"
#include "learned_eval.h"
#include "vecmath.h"

#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"

#include <stdio.h>

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[4096];
static _Alignas(32) uint8_t s_learnedPoseMemory[4096];
static _Alignas(16) uint8_t s_work[16384];
static maniPose* s_pose;
static maniPose* s_learnedPose;

// The node as it should be: its block's values, its time.
typedef struct Reference
{
    float before[STATE];
    float after[STATE];
    double clock;
    double time;
    float since;
    float period;
    float cost;
    uint32_t bits;
    bool has;
} Reference;

// What an update should give.
typedef struct Expected
{
    maniGraphEvent events[2];
    maniVec3 move;
    maniQuat turn;
    uint32_t count;
    bool requested;
} Expected;

static void After(Reference* r)
{
    memcpy(r->after, r->before, sizeof r->after);
    CHECK(maniLearnedModel_Step(&s_learned, r->after, s_work, sizeof s_work) == mani_success,
          "stepped");
}

static void Interpolated(const Reference* r, float* state)
{
    float fraction = (float)(r->clock * (double)s_learned.stepRate);
    for (uint32_t k = 0; k < STATE; ++k)
    {
        state[k] = r->before[k] + fraction * (r->after[k] - r->before[k]);
    }
}

// A projection of the query ahead, replacing the state past the cost.
static bool Projected(Reference* r, const float* ahead, float* state)
{
    float offsets[FEATURES];
    float scales[FEATURES];
    float query[FEATURES];
    float projected[STATE];
    CHECK(maniLearnedModel_GetNormalization(&s_learned, offsets, scales, s_learned.featureCount,
                                            NULL) == mani_success,
          "the normalization");
    for (uint32_t i = 0; i < FEATURES; ++i)
    {
        query[i] = (ahead[i] - offsets[i]) / scales[i];
    }
    CHECK(maniLearnedModel_Project(&s_learned, 0, query, projected,
                                   (s_learned.featureCount + s_learned.latentCount), NULL, s_work,
                                   sizeof s_work) == mani_success,
          "projected");
    float apart = 0.0f;
    for (uint32_t k = 0; k < FEATURES; ++k)
    {
        float d = state[k] - projected[k];
        apart += d * d;
    }
    if (r->has && !(apart > r->cost * r->cost))
    {
        return false;
    }
    memcpy(r->before, projected, sizeof r->before);
    After(r);
    memcpy(state, projected, sizeof projected);
    r->clock = 0.0;
    bool had = r->has;
    r->has = true;
    return had;
}

// The events rising at the state, listed.
static void Events(Reference* r, const float* events, Expected* x)
{
    for (uint32_t e = 0; e < 2; ++e)
    {
        bool on = events[e] > 0.5f;
        bool was = (r->bits & (1u << e)) != 0;
        r->bits = on ? r->bits | 1u << e : r->bits & ~(1u << e);
        if (on && !was)
        {
            size_t length = 0;
            const char* name = maniLearnedModel_GetEventName(&s_learned, e, &length);
            x->events[x->count++] =
                (maniGraphEvent){0,
                                 MANI_NO_TRACK,
                                 e,
                                 0,
                                 maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length),
                                 1.0f,
                                 (float)r->time,
                                 1};
        }
    }
}

static Expected Advance(Reference* r, float delta, bool triggered, const float* ahead)
{
    Expected x = {.turn = {0.0f, 0.0f, 0.0f, 1.0f}};
    double most = MANI_GRAPH_LEARNED_MAX_STEPS / (double)s_learned.stepRate;
    double step = fabs((double)delta) < most ? fabs((double)delta) : most;
    bool had = r->has;
    if (had)
    {
        double seconds = 1.0 / (double)s_learned.stepRate;
        double clock = r->clock + step;
        while (clock >= seconds)
        {
            memcpy(r->before, r->after, sizeof r->before);
            After(r);
            clock -= seconds;
        }
        r->clock = clock;
    }
    float state[STATE];
    Interpolated(r, state);
    double since = (double)r->since + step;
    bool due = !had || triggered || since >= (double)r->period;
    r->since = due ? 0.0f : (float)since;
    x.requested = due && Projected(r, ahead, state);
    if (!r->has)
    {
        return x;
    }
    r->time += step;
    float events[2];
    maniLearnedOutput output = {NULL, NULL, 0, events, 2, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    CHECK(maniLearnedModel_Decompress(&s_learned, state, &output, s_work, sizeof s_work) ==
              mani_success,
          "decompressed");
    float seconds = (float)step;
    maniVec3 v = output.rootVelocity;
    maniVec3 w = output.rootAngularVelocity;
    x.move = (maniVec3){v.x * seconds, v.y * seconds, v.z * seconds};
    x.turn = maniQuatFromRotationVector((maniVec3){w.x * seconds, w.y * seconds, w.z * seconds});
    Events(r, events, &x);
    return x;
}

static double Bits64(const uint32_t* words)
{
    uint64_t bits = (uint64_t)words[0] | (uint64_t)words[1] << 32;
    double value = 0.0;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// The node's block, time and the update's events and root motion as the
// reference has them.
static bool Matches(maniGraphInstance* instance, const maniGraph* g, const Reference* r,
                    const Expected* x, const maniGraphEvent* events, uint32_t count,
                    const maniRootMove* move)
{
    const uint32_t* block = LearnedBlock(instance, g, 0);
    bool ok = block[maniGraphLearnedHas] == (r->has ? 1u : 0u) &&
              Bits64(block + maniGraphLearnedClock) == r->clock &&
              block[maniGraphLearnedSince] == Bits(r->since) &&
              memcmp(block + maniGraphLearnedBefore, r->before, sizeof r->before) == 0 &&
              memcmp(block + maniGraphLearnedBefore + STATE, r->after, sizeof r->after) == 0 &&
              block[maniGraphLearnedBefore + 2 * STATE] == r->bits &&
              State(instance, g, 0)->time == r->time && count == x->count;
    for (uint32_t e = 0; ok && e < count; ++e)
    {
        const maniGraphEvent* a = &events[e];
        const maniGraphEvent* b = &x->events[e];
        ok = a->node == b->node && a->clip == b->clip && a->event == b->event && a->payload == 0 &&
             a->nameHash == b->nameHash && a->weight == b->weight && a->time == b->time &&
             a->direction == 1;
    }
    float dot = move->rotation.x * x->turn.x + move->rotation.y * x->turn.y +
                move->rotation.z * x->turn.z + move->rotation.w * x->turn.w;
    ok = ok && NearF(move->translation.x, (double)x->move.x) &&
         NearF(move->translation.y, (double)x->move.y) &&
         NearF(move->translation.z, (double)x->move.z) && fabs(fabs((double)dot) - 1.0) <= 1e-6;
    ok = ok && (x->requested ? Pending(instance, g, 1) == 0.25f : true);
    if (!ok)
    {
        printf("  has %u clock %.17g (want %.17g) time %.9g (want %.9g) events %u (want %u)\n",
               block[maniGraphLearnedHas], Bits64(block + maniGraphLearnedClock), r->clock,
               State(instance, g, 0)->time, r->time, count, x->count);
    }
    return ok;
}

static bool SamePose(const maniPose* a, const maniPose* b)
{
    for (uint32_t k = 0; k < maniPose_GetBoneCount(a); ++k)
    {
        maniTransform t = maniPose_GetLocal(a, k);
        maniTransform u = maniPose_GetLocal(b, k);
        if (!(NearF(t.translation.x, (double)u.translation.x) &&
              NearF(t.translation.y, (double)u.translation.y) &&
              NearF(t.rotation.w, (double)u.rotation.w) && NearF(t.scale.x, (double)u.scale.x)))
        {
            return false;
        }
    }
    return true;
}

// The pose at the node's interpolated state.
static bool Posed(maniGraphInstance* instance, const Reference* r)
{
    float state[STATE];
    Interpolated(r, state);
    maniLearnedOutput output = {s_learnedPose,     NULL, 0, NULL, 0, {0.0f, 0.0f, 0.0f},
                                {0.0f, 0.0f, 0.0f}};
    if (maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) != mani_success ||
        maniLearnedModel_Decompress(&s_learned, state, &output, s_work, sizeof s_work) !=
            mani_success)
    {
        return false;
    }
    return SamePose(s_pose, s_learnedPose);
}

// The updates of a script: its steps (either sign), the query ahead and
// the trigger as each update finds them.
typedef struct Script
{
    float delta;
    float ahead[2];
    bool trigger;
} Script;

static const Script kScript[] = {
    {0.0f, {0.5f, 0.0f}, false},   {0.01f, {0.5f, 0.0f}, false},  {0.02f, {0.4f, 0.1f}, false},
    {0.05f, {0.0f, 0.0f}, false},  {0.3f, {0.0f, 0.0f}, false},   {-0.02f, {0.2f, -0.3f}, false},
    {0.004f, {0.2f, -0.3f}, true}, {0.1f, {0.6f, 0.2f}, false},   {0.0f, {0.6f, 0.2f}, true},
    {0.07f, {-0.4f, 0.5f}, false}, {0.13f, {-0.4f, 0.5f}, false}, {0.25f, {0.5f, 0.0f}, false},
    {0.033f, {0.1f, 0.0f}, false}, {0.5f, {0.3f, 0.3f}, false},   {0.016f, {0.3f, 0.3f}, false},
    {1e30f, {0.2f, 0.1f}, false},  {0.32f, {0.2f, 0.1f}, false},  {-3.0f, {0.4f, 0.0f}, false}};

#define SCRIPTED (sizeof kScript / sizeof kScript[0])

// The script played on the node and the reference; the events listed.
static uint32_t Play(maniGraphInstance* instance, const maniGraph* g, Reference* r)
{
    uint32_t listed = 0;
    for (uint32_t k = 0; k < SCRIPTED; ++k)
    {
        const Script* s = &kScript[k];
        Ahead(instance, s->ahead[0], s->ahead[1]);
        if (s->trigger)
        {
            Set(instance, 1, maniGraphParameterTrigger, Int(1));
        }
        maniGraphEvent events[4];
        uint32_t count = 0;
        maniRootMove move = maniRootMoveIdentity();
        CHECK(maniGraphInstance_Update(instance, s->delta, events, 4, &count, &move) ==
                  mani_success,
              "updated");
        Expected x = Advance(r, s->delta, s->trigger, s->ahead);
        if (!Matches(instance, g, r, &x, events, count, &move))
        {
            printf("  at update %u\n", k);
            CHECK(false, "the node as the reference");
            return listed;
        }
        // An inertialization under way (as the evaluation finds it)
        // offsets the pose.
        bool posed = Posed(instance, r);
        const uint32_t* eased = NodeBlock(instance, g, 1, maniGraphInertiaBlock);
        bool easing = (eased[maniGraphInertiaFlags] & maniGraphInertiaActive) != 0;
        CHECK(posed != easing, "the pose at the state, or eased");
        listed += count;
    }
    return listed;
}

static void TestReference(void)
{
    const float kCosts[5] = {0.0f, 0.5f, 0.7f, 1.2f, 1e30f};
    uint32_t listed = 0;
    for (int c = 0; c < 5; ++c)
    {
        maniGraph g;
        maniGraphInstance* instance = Learning(LearnedNode(0.1f, kCosts[c]), &g);
        Reference r = {.period = 0.1f, .cost = kCosts[c]};
        listed += Play(instance, &g, &r);
    }
    CHECK(listed > 0, "events listed");
}

static bool Zero(const uint32_t* words, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (words[k] != 0)
        {
            return false;
        }
    }
    return true;
}

#define BLOCK_WORDS (4 + 2 * STATE + 1)

// The node under a mirror node: each move the plain node's reflected,
// each event the plain node's renamed (left and right paired).
static void TestMirrored(void)
{
    static _Alignas(16) uint8_t file[4096];
    static const maniMirrorNamePairSource kPairs[1] = {{"left", 4, 5, "right"}};
    const maniMirrorMapSource ms = {
        .skeleton = &s_skeleton, .namePairs = kPairs, .namePairCount = 1, .axis = maniMirrorAxisX};
    maniMirrorMap map;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, &ms).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenMirrorMap(file, size, &map).result == mani_success,
          "a mirror map");
    maniDestroyBuilder(builder);
    MatchParameters();
    static const uint32_t kNode[1] = {0};
    static const uint32_t kMirror[1] = {1};
    maniGraphNodeSource plainNodes[2] = {LearnedNode(0.1f, 0.0f), Above(0)};
    maniGraphSource source = {.parameters = s_matchParameters,
                              .nodes = plainNodes,
                              .parameterCount = 4,
                              .nodeCount = 2,
                              .output = 1};
    maniGraph g;
    maniGraphInstance* instance = MakeLearned(source, &g);
    enum
    {
        kUpdates = 12
    };
    maniRootMove plain[kUpdates];
    maniGraphEvent heard[kUpdates][2];
    uint32_t counts[kUpdates];
    uint32_t total = 0;
    for (int k = 0; k < kUpdates; ++k)
    {
        (void)maniGraphInstance_Update(instance, 0.05f, heard[k], 2, &counts[k], &plain[k]);
        total += counts[k];
    }
    maniGraphNodeSource nodes[3] = {LearnedNode(0.1f, 0.0f), Above(0), Above(0)};
    nodes[1] = (maniGraphNodeSource){maniGraphNodeMirror, 1, kNode, .clip = {0}};
    nodes[2].inputs = kMirror;
    source.nodes = nodes;
    source.nodeCount = 3;
    source.output = 2;
    source.mirrorMap = &map;
    instance = MakeLearned(source, &g);
    bool moves = true;
    bool events = total > 0;
    for (int k = 0; k < kUpdates; ++k)
    {
        maniRootMove move;
        maniGraphEvent got[2];
        uint32_t count = 0;
        maniRootMove want = maniMirrorMap_MirrorRootMotion(&map, plain[k]);
        moves = moves &&
                maniGraphInstance_Update(instance, 0.05f, got, 2, &count, &move) == mani_success &&
                NearF(move.translation.x, (double)want.translation.x) &&
                NearF(move.translation.y, (double)want.translation.y) &&
                NearF(move.translation.z, (double)want.translation.z) &&
                NearF(move.rotation.x, (double)want.rotation.x) &&
                NearF(move.rotation.y, (double)want.rotation.y) &&
                NearF(move.rotation.z, (double)want.rotation.z);
        events = events && count == counts[k];
        for (uint32_t i = 0; events && i < count && i < 2; ++i)
        {
            events = got[i].nameHash == maniMirrorMap_MirrorNameHash(&map, heard[k][i].nameHash) &&
                     got[i].nameHash != heard[k][i].nameHash;
        }
    }
    CHECK(moves, "each move the plain node's reflected");
    CHECK(events, "each event the plain node's renamed");
}

static void TestFirst(void)
{
    maniGraph g;
    // Memory not cleared: the instance's start clears the block.
    memset(s_instanceMemory, 0xA5, sizeof s_instanceMemory);
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.0f), &g);
    uint32_t* block = LearnedBlock(instance, &g, 0);
    CHECK(Zero(block, BLOCK_WORDS) &&
              maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                  mani_success &&
              maniPose_SetRest(s_learnedPose, &s_skeleton) == mani_success &&
              SamePose(s_pose, s_learnedPose),
          "no state before the first update: the rest pose");
    Update(instance, 0.0f);
    CHECK(block[maniGraphLearnedHas] == 1 && isinf(Pending(instance, &g, 1)),
          "the first projection requests nothing");
    // Projected again where it stands: no farther than a cost of 0.
    uint32_t before[BLOCK_WORDS];
    memcpy(before, block, sizeof before);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.0f);
    CHECK(memcmp(before, block, sizeof before) == 0 && isinf(Pending(instance, &g, 1)),
          "a projection at the state kept, nothing requested");
    maniGraphValue v;
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.01f);
    CHECK(maniGraphInstance_GetParameter(instance, 1, &v) == mani_success && v.b == 0,
          "the trigger consumed");
    CHECK(maniGraphLearnedFits(&g, maniLoadU32(g.nodes + 12), &s_learned), "the model fits");
}

// A model stepping 64 times a second: an update of exactly a step's
// seconds takes the step, the clock back at 0.
static void TestWholeStep(void)
{
    maniLearnedModel fast = s_learned;
    fast.stepRate = 64.0f;
    s_bound = &fast;
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(10.0f, 0.0f), &g);
    s_bound = &s_learned;
    Update(instance, 0.0f);
    const uint32_t* block = LearnedBlock(instance, &g, 0);
    uint32_t after[STATE];
    memcpy(after, block + maniGraphLearnedBefore + STATE, sizeof after);
    Update(instance, 0.015625f);
    CHECK(Bits64(block + maniGraphLearnedClock) == 0.0 &&
              memcmp(after, block + maniGraphLearnedBefore, sizeof after) == 0,
          "a whole step taken");
    for (int k = 0; k < 3; ++k)
    {
        Update(instance, 1e30f);
    }
    CHECK(Bits64(block + maniGraphLearnedClock) == 0.0 &&
              State(instance, &g, 0)->time == 0.015625 + 3.0 * 20 * 0.015625,
          "long updates: twenty steps each");
}

// Tags of no Projector's (2 required, none excluded): nothing projected,
// the block left clear however long the update.
static void TestNoProjector(void)
{
    maniGraphNodeSource node = LearnedNode(0.1f, 0.0f);
    node.learned.requiredTags = (maniGraphTags){2, 0};
    maniGraph g;
    maniGraphInstance* instance = Learning(node, &g);
    maniGraphValue v = {{0}};
    v.i = 2;
    Set(instance, 2, maniGraphParameterInt, v);
    Update(instance, 0.0f);
    Update(instance, 0.25f);
    CHECK(Zero(LearnedBlock(instance, &g, 0), BLOCK_WORDS) && State(instance, &g, 0)->time == 0.0,
          "no Projector: no state");
    CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                  mani_success &&
              maniPose_SetRest(s_learnedPose, &s_skeleton) == mani_success &&
              SamePose(s_pose, s_learnedPose),
          "weighed with no state: the rest pose");
    v.i = 1;
    Set(instance, 2, maniGraphParameterInt, v);
    Update(instance, 0.0f);
    CHECK(LearnedBlock(instance, &g, 0)[maniGraphLearnedHas] == 1, "the first Projector's tags");
}

// A direct blend of the node and a clip by weight w; node 3 inertializes.
static maniGraphInstance* Blended(float w, maniGraph* g)
{
    MatchParameters();
    static const uint32_t kInputs[2] = {0, 1};
    static maniGraphScalar weights[2];
    weights[0] = LITERAL(w);
    weights[1] = LITERAL(1.0f);
    maniGraphNodeSource nodes[4] = {LearnedNode(0.1f, 0.5f),
                                    ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    {maniGraphNodeBlend, 2, kInputs, .blend = {weights, 1}},
                                    Above(2)};
    return MakeLearned((maniGraphSource){.parameters = s_matchParameters,
                                         .nodes = nodes,
                                         .parameterCount = 4,
                                         .nodeCount = 4,
                                         .output = 3},
                       g);
}

// Weighed 0, the node is left alone; weighed below the event threshold,
// it plays but lists no events.
static void TestFaint(void)
{
    maniGraph g;
    maniGraphInstance* instance = Blended(0.0f, &g);
    Update(instance, 0.0f);
    Update(instance, 0.1f);
    CHECK(Zero(LearnedBlock(instance, &g, 0), BLOCK_WORDS), "weighed 0: no state");
    instance = Blended(1e-6f, &g);
    Reference r = {.period = 0.1f, .cost = 0.5f};
    uint32_t listed = 0;
    uint32_t rose = 0;
    for (uint32_t k = 0; k < SCRIPTED; ++k)
    {
        const Script* s = &kScript[k];
        Ahead(instance, s->ahead[0], s->ahead[1]);
        maniGraphEvent events[4];
        uint32_t count = 0;
        CHECK(maniGraphInstance_Update(instance, s->delta, events, 4, &count, NULL) == mani_success,
              "updated");
        Expected x = Advance(&r, s->delta, false, s->ahead);
        for (uint32_t e = 0; e < count && e < 4; ++e)
        {
            listed += events[e].node == 0 ? 1u : 0u;
        }
        rose += x.count;
    }
    CHECK(rose > 0 && listed == 0 &&
              LearnedBlock(instance, &g, 0)[maniGraphLearnedBefore + 2 * STATE] == r.bits,
          "events rising, none listed");
}

// A select between the node and a clip: the node unselected is left
// alone.
static void TestWeightless(void)
{
    MatchParameters();
    static const uint32_t kInputs[2] = {0, 1};
    maniGraphNodeSource nodes[4] = {LearnedNode(0.1f, 0.0f),
                                    ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    {maniGraphNodeSelect, 2, kInputs, .select = {2}},
                                    Above(2)};
    maniGraph g;
    maniGraphInstance* instance = MakeLearned((maniGraphSource){.parameters = s_matchParameters,
                                                                .nodes = nodes,
                                                                .parameterCount = 4,
                                                                .nodeCount = 4,
                                                                .output = 3},
                                              &g);
    Update(instance, 0.0f);
    Update(instance, 0.03f);
    uint32_t saved[4 + 2 * STATE + 1];
    memcpy(saved, LearnedBlock(instance, &g, 0), sizeof saved);
    double time = State(instance, &g, 0)->time;
    maniGraphValue v = {{0}};
    v.i = 1;
    Set(instance, 2, maniGraphParameterInt, v);
    Update(instance, 0.2f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.2f);
    maniGraphValue t;
    CHECK(memcmp(saved, LearnedBlock(instance, &g, 0), sizeof saved) == 0 &&
              State(instance, &g, 0)->time == time &&
              maniGraphInstance_GetParameter(instance, 1, &t) == mani_success && t.b == 1,
          "unselected: its block, time and trigger kept");
}

// A seek above the node: its state dropped, then projected anew with no
// inertialization requested.
static void TestSeek(void)
{
    MatchParameters();
    static const uint32_t kInputs[1] = {0};
    maniGraphNodeSource node = LearnedNode(10.0f, 0.0f);
    node.learned.trigger = MANI_NO_PARAMETER;
    maniGraphNodeSource nodes[3] = {
        node, {maniGraphNodeSeek, 1, kInputs, .seek = {1, LITERAL(0.0f)}}, Above(1)};
    maniGraph g;
    maniGraphInstance* instance = MakeLearned((maniGraphSource){.parameters = s_matchParameters,
                                                                .nodes = nodes,
                                                                .parameterCount = 4,
                                                                .nodeCount = 3,
                                                                .output = 2},
                                              &g);
    Update(instance, 0.0f);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.03f);
    const uint32_t* block = LearnedBlock(instance, &g, 0);
    CHECK(Bits64(block + maniGraphLearnedClock) > 0.0, "the clock moved");
    Reference r = {.period = 10.0f};
    float ahead[2] = {0.0f, 0.0f};
    (void)Advance(&r, 0.0f, false, ahead);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.05f);
    CHECK(block[maniGraphLearnedHas] == 1 && Bits64(block + maniGraphLearnedClock) == 0.0 &&
              memcmp(block + maniGraphLearnedBefore, r.before, sizeof r.before) == 0 &&
              isinf(Pending(instance, &g, 2)),
          "sought: projected anew, nothing requested");
}

static _Alignas(8) uint8_t s_other[8192];
static uint8_t s_saved[2][4096];

// The largest delta a host can pass, twice: the step is bounded, the
// clock and the time since the last projection stay below theirs, and the
// state saves and restores.
static void TestLargestDelta(void)
{
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.5f), &g);
    Update(instance, FLT_MAX);
    Update(instance, FLT_MAX);
    CHECK(Restores(instance), "saved and restored after the largest deltas");
}

static void TestSaved(void)
{
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.5f), &g);
    size_t size = SavedGraphSize(instance);
    maniGraphInstance* other = NULL;
    // The node's block is the first, after the values.
    uint32_t block = MANI_GRAPH_STATE_HEADER_SIZE + 6 * MANI_GRAPH_VALUE_SIZE;
    CHECK(size <= sizeof s_saved[0] &&
              maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) ==
                  mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success,
          "saved before its first update");
    const uint32_t kNone[3][2] = {{maniGraphLearnedClock, 0x3F800000u},
                                  {maniGraphLearnedSince, 0x3F800000u},
                                  {maniGraphLearnedBefore + 1, 0x3F800000u}};
    for (int k = 0; k < 3; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kNone[k][0], kNone[k][1]);
        maniFileResult r = maniGraphInstance_Restore(other, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kNone[k][0],
              "a block of no state holding a value");
    }
    Update(instance, 0.0f);
    Update(instance, 0.025f);
    Ahead(instance, 0.1f, 0.2f);
    Update(instance, 0.03f);
    CHECK(maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) == mani_success &&
              maniGraphInstance_Restore(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    for (int k = 0; k < 6; ++k)
    {
        Update(instance, 0.045f);
        Update(other, 0.045f);
    }
    CHECK(State(instance, &g, 0)->time == State(other, &g, 0)->time &&
              memcmp(LearnedBlock(instance, &g, 0), LearnedBlock(other, &g, 0),
                     4 * (4 + 2 * STATE + 1)) == 0,
          "playing alike");
    CHECK(maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) == mani_success,
          "saved");
    const uint32_t kBad[10][3] = {
        {maniGraphLearnedHas, 2, maniGraphLearnedHas},
        {maniGraphLearnedClock + 1, 0xBFF00000u, maniGraphLearnedClock},
        {maniGraphLearnedClock + 1, 0x3FA00000u, maniGraphLearnedClock},
        {maniGraphLearnedClock + 1, 0x7FF80000u, maniGraphLearnedClock},
        {maniGraphLearnedSince, 0x7FC00000u, maniGraphLearnedSince},
        {maniGraphLearnedSince, 0xBF800000u, maniGraphLearnedSince},
        {maniGraphLearnedSince, 0x7F800000u, maniGraphLearnedSince},
        {maniGraphLearnedBefore + 2, 0x7FC00000u, maniGraphLearnedBefore + 2},
        {maniGraphLearnedBefore + STATE, 0x7F800000u, maniGraphLearnedBefore + STATE},
        {maniGraphLearnedBefore + 2 * STATE, 4, maniGraphLearnedBefore + 2 * STATE}};
    for (int k = 0; k < 10; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kBad[k][0], kBad[k][1]);
        maniFileResult r = maniGraphInstance_Restore(other, s_saved[1], size);
        if (r.result != mani_errorFormat || r.offset != block + 4 * kBad[k][2])
        {
            printf("  saved %d refused %d at %llu, want %u\n", k, (int)r.result,
                   (unsigned long long)r.offset, block + 4 * kBad[k][2]);
        }
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kBad[k][2],
              "a saved block refused at its word");
    }
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + block + 4 * (maniGraphLearnedBefore + 2 * STATE), 3);
    CHECK(maniGraphInstance_Restore(other, s_saved[1], size).result == mani_success,
          "both events' bits");
    // A clock of a whole step, which the step would have taken.
    double whole = 1.0 / (double)s_learned.stepRate;
    uint64_t wholeBits = 0;
    memcpy(&wholeBits, &whole, sizeof wholeBits);
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + block + 4 * maniGraphLearnedClock, (uint32_t)wholeBits);
    maniStoreU32(s_saved[1] + block + 4 * (maniGraphLearnedClock + 1), (uint32_t)(wholeBits >> 32));
    maniFileResult r = maniGraphInstance_Restore(other, s_saved[1], size);
    CHECK(r.result == mani_errorFormat && r.offset == block + 4 * maniGraphLearnedClock,
          "a clock of a whole step");
}

// A reset clears every word of the node's block (the events' bits last,
// whose stale "on" would hide an event starting after a seek), and no
// word past it.
static void TestReset(void)
{
    maniGraph g;
    (void)Learning(LearnedNode(0.1f, 0.0f), &g);
    uint32_t data = maniLoadU32(g.nodes + 12);
    uint32_t words = maniGraphLearnedBlockWords(&g, data);
    uint32_t block[64];
    CHECK(words < 64, "a small block");
    memset(block, 0xFF, sizeof block);
    maniGraphResetLearned(&g, data, block);
    bool cleared = true;
    for (uint32_t k = 0; k < words; ++k)
    {
        cleared = cleared && block[k] == 0;
    }
    CHECK(cleared && block[words] == UINT32_MAX, "a reset clears its block, no more");
}

// Two poses' locals the same bits.
static bool SameBits(const maniPose* a, const maniPose* b)
{
    for (uint32_t bone = 0; bone < maniPose_GetBoneCount(a); ++bone)
    {
        maniTransform t = maniPose_GetLocal(a, bone);
        maniTransform u = maniPose_GetLocal(b, bone);
        if (memcmp(&t, &u, sizeof t) != 0)
        {
            return false;
        }
    }
    return true;
}

// The Decompressor's hidden values the update keeps with its state: none
// kept before the first update; after each update of the script the
// cache holds the interpolated state, and the evaluation from it gives
// the same bits as one computing them (the cache unkept, or kept at
// another state with other hidden values); hidden values changed under
// the same state change the pose, so the evaluation takes them.
static void TestCache(void)
{
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.5f), &g);
    size_t size = SavedGraphSize(instance);
    uint32_t hidden = maniLearnedHiddenCount(&s_learned);
    float* cache = maniGraphLearnedCache(instance, 0);
    static float kept[1 + STATE + 1024];
    size_t bytes = sizeof(float) * (1 + STATE + hidden);
    CHECK(hidden > 0 && hidden <= 1024 && cache != NULL && cache[0] == 0.0f &&
              size <= sizeof s_saved[0],
          "a cache, none kept");
    uint32_t taken = 0;
    for (uint32_t k = 0; k < SCRIPTED; ++k)
    {
        const Script* s = &kScript[k];
        Ahead(instance, s->ahead[0], s->ahead[1]);
        Set(instance, 1, maniGraphParameterTrigger, Int(s->trigger ? 1 : 0));
        CHECK(maniGraphInstance_Update(instance, s->delta, NULL, 0, NULL, NULL) == mani_success,
              "updated");
        const uint32_t* block = LearnedBlock(instance, &g, 0);
        float state[STATE];
        maniGraphLearnedState(block, STATE, s_learned.stepRate, state);
        CHECK(block[maniGraphLearnedHas] == 1 && cache[0] == 1.0f &&
                  memcmp(cache + 1, state, sizeof state) == 0,
              "the cache at the state");
        memcpy(kept, cache, bytes);
        CHECK(maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) == mani_success,
              "saved");
        // None kept, though at the state, with other hidden values: computed.
        cache[0] = 0.0f;
        for (uint32_t i = 0; i < hidden; ++i)
        {
            cache[1 + STATE + i] += 1.0f;
        }
        CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_learnedPose) ==
                  mani_success,
              "evaluated with none kept");
        // Kept at another state, with other hidden values: computed.
        CHECK(maniGraphInstance_Restore(instance, s_saved[0], size).result == mani_success,
              "restored");
        memcpy(cache, kept, bytes);
        cache[1] = nextafterf(cache[1], HUGE_VALF);
        for (uint32_t i = 0; i < hidden; ++i)
        {
            cache[1 + STATE + i] += 1.0f;
        }
        CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                      mani_success &&
                  SameBits(s_pose, s_learnedPose),
              "kept at another state, computed");
        CHECK(maniGraphInstance_Restore(instance, s_saved[0], size).result == mani_success,
              "restored");
        memcpy(cache, kept, bytes);
        for (uint32_t i = 0; i < hidden; ++i)
        {
            cache[1 + STATE + i] += 1.0f;
        }
        CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                  mani_success,
              "evaluated at other hidden values");
        taken += SameBits(s_pose, s_learnedPose) ? 0u : 1u;
        CHECK(maniGraphInstance_Restore(instance, s_saved[0], size).result == mani_success,
              "restored");
        memcpy(cache, kept, bytes);
        CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                      mani_success &&
                  SameBits(s_pose, s_learnedPose),
              "the cache's evaluation the same bits");
    }
    CHECK(taken == SCRIPTED, "the cache taken");
}

// A model whose Stepper takes the state past the float range on its
// first step, though every weight is finite and opening checked them all:
// the update completes and reports the model's data with
// mani_errorFormat, the node held at its last finite state and named in
// its view, and every output stays finite; a save of it restores. Last, as it rebuilds the
// fixture's model.
static void TestOverflow(void)
{
    MakeNetworks();
    maniLearnedLayerSource* layers = (maniLearnedLayerSource*)s_stepper.layers;
    float* weights = (float*)layers[0].weights;
    float* biases = (float*)layers[0].biases;
    for (uint32_t k = 0; k < layers[0].inputCount * layers[0].outputCount; ++k)
    {
        weights[k] = 0.0f;
    }
    for (uint32_t k = 0; k < layers[0].outputCount; ++k)
    {
        biases[k] = 1e30f;
    }
    weights = (float*)layers[1].weights;
    for (uint32_t k = 0; k < layers[1].inputCount * layers[1].outputCount; ++k)
    {
        weights[k] = 1e30f;
    }
    maniLearnedModelSource source = Source();
    CHECK(Opened(&source, &s_learned), "a model of finite weights that overflows");
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.0f), &g);
    maniGraphEvent events[8];
    uint32_t count = 0;
    maniRootMove move;
    // The first update projects, then steps from the projection.
    CHECK(maniGraphInstance_Update(instance, 0.0f, events, 8, &count, &move) == mani_errorFormat,
          "the first step past the float range refused as the model's data");
    maniResult r = maniGraphInstance_Update(instance, 0.05f, events, 8, &count, &move);
    CHECK(r == mani_errorFormat, "and every step after it");
    bool finite = isfinite(move.translation.x) && isfinite(move.translation.y) &&
                  isfinite(move.translation.z) && isfinite(move.rotation.x) &&
                  isfinite(move.rotation.y) && isfinite(move.rotation.z) &&
                  isfinite(move.rotation.w);
    for (uint32_t e = 0; e < count && e < 8; ++e)
    {
        finite = finite && isfinite(events[e].weight) && isfinite(events[e].time);
    }
    CHECK(finite, "the update's root motion and events finite");
    maniGraphNodeView view;
    CHECK(maniGraphInstance_InspectNode(instance, 0, &view) == mani_success && view.failed == 1 &&
              maniGraphInstance_InspectNode(instance, 1, &view) == mani_success && view.failed == 0,
          "the learned node named as failed, the node above it not");
    const uint32_t* block = LearnedBlock(instance, &g, 0);
    bool held = block[maniGraphLearnedHas] == 1;
    for (uint32_t k = 0; k < 2 * STATE; ++k)
    {
        float v = 0.0f;
        memcpy(&v, &block[maniGraphLearnedBefore + k], sizeof v);
        held = held && isfinite(v);
    }
    CHECK(held, "the node held at a finite state");
    maniTransform locals[64];
    uint32_t bones = maniSkeleton_GetBoneCount(&s_skeleton);
    bool posed =
        bones <= 64 &&
        maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success &&
        maniPose_ReadLocals(s_pose, 0, bones, locals) == mani_success;
    for (uint32_t b = 0; posed && b < bones; ++b)
    {
        float f[10];
        memcpy(f, &locals[b], sizeof f);
        for (int k = 0; k < 10; ++k)
        {
            posed = posed && isfinite(f[k]);
        }
    }
    CHECK(posed, "the pose finite");
    maniGraphInstance* other = NULL;
    size_t size = SavedGraphSize(instance);
    CHECK(size <= sizeof s_saved[0] &&
              maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) ==
                  mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniGraphInstance_Restore(other, s_saved[0], size).result == mani_success,
          "its saved state restores");
    // A Decompressor that overflows too: the update gives no move or event
    // from the node, and the evaluation the neutral pose, both reported.
    layers = (maniLearnedLayerSource*)s_decompressor.layers;
    biases = (float*)layers[0].biases;
    for (uint32_t k = 0; k < layers[0].outputCount; ++k)
    {
        biases[k] = 1e30f;
    }
    weights = (float*)layers[1].weights;
    for (uint32_t k = 0; k < layers[1].inputCount * layers[1].outputCount; ++k)
    {
        weights[k] = 1e30f;
    }
    source = Source();
    CHECK(Opened(&source, &s_learned), "a model whose Decompressor overflows");
    instance = Learning(LearnedNode(0.1f, 0.0f), &g);
    CHECK(maniGraphInstance_Update(instance, 0.0f, events, 8, &count, &move) == mani_errorFormat &&
              count == 0 && move.translation.x == 0.0f && move.rotation.w == 1.0f,
          "no move and no event from the node");
    posed = maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                mani_errorFormat &&
            maniPose_ReadLocals(s_pose, 0, bones, locals) == mani_success;
    for (uint32_t b = 0; posed && b < bones; ++b)
    {
        float f[10];
        memcpy(f, &locals[b], sizeof f);
        for (int k = 0; k < 10; ++k)
        {
            posed = posed && isfinite(f[k]);
        }
    }
    CHECK(posed && maniGraphInstance_InspectNode(instance, 0, &view) == mani_success &&
              view.failed == 1,
          "the evaluation reported, its pose finite, the node named");
}

// The fixture's model at 0.5 steps a second, its Decompressor's root
// outputs from first (the velocity at 30, the angular velocity at 33)
// value whatever the state; the other outputs random.
static void RootOutputs(uint32_t first, float value)
{
    MakeNetworks();
    maniLearnedLayerSource* last =
        (maniLearnedLayerSource*)&s_decompressor.layers[s_decompressor.layerCount - 1];
    float* weights = (float*)last->weights;
    float* biases = (float*)last->biases;
    for (uint32_t o = first; o < first + 3; ++o)
    {
        for (uint32_t i = 0; i < last->inputCount; ++i)
        {
            weights[(size_t)i * last->outputCount + o] = 0.0f;
        }
        biases[o] = 0.0f;
        ((float*)s_decompressor.outputScales)[o] = 1.0f;
        ((float*)s_decompressor.outputOffsets)[o] = value;
    }
    maniLearnedModelSource source = Source();
    source.stepRate = 0.5f;
    CHECK(Opened(&source, &s_learned), "a model of finite root outputs");
}

// Finite root outputs whose move over a long step passes the float range:
// a velocity of 3e38 over 2 seconds, and an angular velocity of 2.5e38 on
// each axis over 1 (each product finite, the angle not). The update
// reports the model's data with mani_errorFormat, its move finite and the
// node named in its view. An angular velocity of 1e-40 over 1/30 second,
// an angle whose reciprocal passes the float range, turns by its first
// order term (v / 2, 1), checked in binary64. Last, as it rebuilds the
// fixture's model.
static void TestRootOverflow(void)
{
    static const struct
    {
        uint32_t first;
        float value;
        float delta;
        const char* name;
    } kCases[2] = {{30, 3e38f, 2.0f, "a velocity past the float range over the step"},
                   {33, 2.5e38f, 1.0f, "a rotation angle past the float range"}};
    for (int c = 0; c < 2; ++c)
    {
        RootOutputs(kCases[c].first, kCases[c].value);
        maniGraph g;
        maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.0f), &g);
        maniGraphEvent events[8];
        uint32_t count = 0;
        maniRootMove move;
        CHECK(maniGraphInstance_Update(instance, 0.0f, events, 8, &count, &move) == mani_success,
              "no move over no time");
        maniResult r =
            maniGraphInstance_Update(instance, kCases[c].delta, events, 8, &count, &move);
        maniGraphNodeView view;
        CHECK(r == mani_errorFormat && isfinite(move.translation.x) &&
                  isfinite(move.translation.y) && isfinite(move.translation.z) &&
                  isfinite(move.rotation.x) && isfinite(move.rotation.y) &&
                  isfinite(move.rotation.z) && isfinite(move.rotation.w) &&
                  maniGraphInstance_InspectNode(instance, 0, &view) == mani_success &&
                  view.failed == 1,
              kCases[c].name);
    }
    RootOutputs(33, 1e-40f);
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.0f), &g);
    maniGraphEvent events[8];
    uint32_t count = 0;
    maniRootMove move;
    float seconds = 1.0f / 30.0f;
    CHECK(maniGraphInstance_Update(instance, 0.0f, events, 8, &count, &move) == mani_success,
          "no move over no time");
    maniResult r = maniGraphInstance_Update(instance, seconds, events, 8, &count, &move);
    // Each axis turns by half its angle; the quaternion's length is 1.
    double half = (double)1e-40f * (double)seconds * 0.5;
    maniGraphNodeView view;
    CHECK(r == mani_success && move.rotation.w == 1.0f && move.rotation.x == move.rotation.y &&
              move.rotation.y == move.rotation.z &&
              fabs((double)move.rotation.x - half) < 0.01 * half &&
              maniGraphInstance_InspectNode(instance, 0, &view) == mani_success && view.failed == 0,
          "an angle whose reciprocal passes the float range turned by its first order");
}

int main(void)
{
    Open();
    OpenDatabase();
    OpenModel();
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_learnedPoseMemory, sizeof s_learnedPoseMemory,
                           &s_learnedPose) == mani_success,
          "poses");
    TestFirst();
    TestReference();
    TestWholeStep();
    TestNoProjector();
    TestFaint();
    TestWeightless();
    TestSeek();
    TestSaved();
    TestLargestDelta();
    TestReset();
    TestCache();
    TestMirrored();
    TestOverflow();
    TestRootOverflow();
    return s_failures == 0 ? 0 : 1;
}
