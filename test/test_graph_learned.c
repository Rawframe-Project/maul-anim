// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned motion matching nodes playing (mani-0025): each update checked
// against a reference of the node kept here from the model's own
// functions (the state stepped at the model's rate, interpolated
// between steps, projected at the period or the trigger and replaced
// past the transition cost, an inertialization requested; the root
// moved by the Decompressor's velocities and its events listed as they
// rise past 0.5) for transition costs of none, some and too much to
// pass; the pose at the interpolated state; the trigger consumed; a node
// weighed 0 left alone; a seek starting it over; a state saved and
// restored playing alike, and each word of a saved block checked.

#include "graph_learned_fixture.h"
#include "vecmath.h"

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
    CHECK(maniLearnedStep(&s_learned, r->after, s_work, sizeof s_work) == mani_success, "stepped");
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
    CHECK(maniLearnedModelNormalization(&s_learned, offsets, scales) == mani_success,
          "the normalization");
    for (uint32_t i = 0; i < FEATURES; ++i)
    {
        query[i] = (ahead[i] - offsets[i]) / scales[i];
    }
    CHECK(maniLearnedProject(&s_learned, 0, query, projected, s_work, sizeof s_work) ==
              mani_success,
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
            uint32_t length = 0;
            const char* name = maniLearnedModelEventName(&s_learned, e, &length);
            x->events[x->count++] =
                (maniGraphEvent){0,
                                 MANI_NO_TRACK,
                                 e,
                                 0,
                                 maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length),
                                 1.0f,
                                 (float)r->time,
                                 1,
                                 0};
        }
    }
}

static Expected Advance(Reference* r, float delta, bool triggered, const float* ahead)
{
    Expected x = {.turn = {0.0f, 0.0f, 0.0f, 1.0f}};
    double step = fabs((double)delta);
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
    maniLearnedOutput output = {NULL, NULL, events, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    CHECK(maniLearnedDecompress(&s_learned, state, &output, s_work, sizeof s_work) == mani_success,
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

// The pose at the node's interpolated state.
static bool Posed(maniGraphInstance* instance, const Reference* r)
{
    float state[STATE];
    Interpolated(r, state);
    maniLearnedOutput output = {s_learnedPose, NULL, NULL, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    if (maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) != mani_success ||
        maniLearnedDecompress(&s_learned, state, &output, s_work, sizeof s_work) != mani_success)
    {
        return false;
    }
    for (uint32_t b = 0; b < maniPoseBoneCount(s_pose); ++b)
    {
        maniTransform t = maniPoseLocal(s_pose, b);
        maniTransform u = maniPoseLocal(s_learnedPose, b);
        if (!(NearF(t.translation.x, (double)u.translation.x) &&
              NearF(t.translation.y, (double)u.translation.y) &&
              NearF(t.rotation.w, (double)u.rotation.w) && NearF(t.scale.x, (double)u.scale.x)))
        {
            return false;
        }
    }
    return true;
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
    {0.033f, {0.1f, 0.0f}, false}, {0.5f, {0.3f, 0.3f}, false},   {0.016f, {0.3f, 0.3f}, false}};

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
        CHECK(maniUpdateGraph(instance, s->delta, events, 4, &count, &move) == mani_success,
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
    const float kCosts[3] = {0.0f, 0.5f, 1e30f};
    uint32_t listed = 0;
    for (int c = 0; c < 3; ++c)
    {
        maniGraph g;
        maniGraphInstance* instance = Learning(LearnedNode(0.1f, kCosts[c]), &g);
        Reference r = {.period = 0.1f, .cost = kCosts[c]};
        listed += Play(instance, &g, &r);
    }
    CHECK(listed > 0, "events listed");
}

static void TestFirst(void)
{
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.0f), &g);
    const uint32_t* block = LearnedBlock(instance, &g, 0);
    CHECK(block[maniGraphLearnedHas] == 0 &&
              maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "no state before the first update");
    Update(instance, 0.0f);
    CHECK(block[maniGraphLearnedHas] == 1 && isinf(Pending(instance, &g, 1)),
          "the first projection requests nothing");
    maniGraphValue v;
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.01f);
    CHECK(maniGetGraphParameter(instance, 1, &v) == mani_success && v.b == 0,
          "the trigger consumed");
    CHECK(maniGraphLearnedFits(&g, maniLoadU32(g.nodes + 12), &s_learned), "the model fits");
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
              maniGetGraphParameter(instance, 1, &t) == mani_success && t.b == 1,
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

static void TestSaved(void)
{
    maniGraph g;
    maniGraphInstance* instance = Learning(LearnedNode(0.1f, 0.5f), &g);
    size_t size = maniGraphStateSize(instance->binding);
    maniGraphInstance* other = NULL;
    // The node's block is the first, after the values.
    uint32_t block = MANI_GRAPH_STATE_HEADER_SIZE + 6 * MANI_GRAPH_VALUE_SIZE;
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
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
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kNone[k][0],
              "a block of no state holding a value");
    }
    Update(instance, 0.0f);
    Update(instance, 0.025f);
    Ahead(instance, 0.1f, 0.2f);
    Update(instance, 0.03f);
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniRestoreGraphInstance(other, s_saved[0], size).result == mani_success,
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
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success, "saved");
    const uint32_t kBad[9][3] = {
        {maniGraphLearnedHas, 2, maniGraphLearnedHas},
        {maniGraphLearnedClock + 1, 0xBFF00000u, maniGraphLearnedClock},
        {maniGraphLearnedClock + 1, 0x3FA00000u, maniGraphLearnedClock},
        {maniGraphLearnedClock + 1, 0x7FF80000u, maniGraphLearnedClock},
        {maniGraphLearnedSince, 0x7FC00000u, maniGraphLearnedSince},
        {maniGraphLearnedSince, 0xBF800000u, maniGraphLearnedSince},
        {maniGraphLearnedBefore + 2, 0x7FC00000u, maniGraphLearnedBefore + 2},
        {maniGraphLearnedBefore + STATE, 0x7F800000u, maniGraphLearnedBefore + STATE},
        {maniGraphLearnedBefore + 2 * STATE, 4, maniGraphLearnedBefore + 2 * STATE}};
    for (int k = 0; k < 9; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kBad[k][0], kBad[k][1]);
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
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
    CHECK(maniRestoreGraphInstance(other, s_saved[1], size).result == mani_success,
          "both events' bits");
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
    TestWeightless();
    TestSeek();
    TestSaved();
    return s_failures == 0 ? 0 : 1;
}
