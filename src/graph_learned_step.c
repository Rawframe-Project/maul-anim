// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node reached with a weight moves by the magnitude of its time factor
// times the update's delta, at most MANI_GRAPH_LEARNED_MAX_STEPS steps'
// seconds (1 / the model's rate, in binary64): its clock moves on, and
// each time a step's seconds pass, the state after the last step becomes
// the state before it and the Stepper gives the next.
// It then projects when it has no state, its trigger is set (consumed)
// or its period passed since the last projection: the query is the
// state's features with its trajectory and float channels taken from its
// parameters, normalized as the model's database; the Projector is the
// one whose filter equals the node's tags (none: no projection). With no
// state, or a projection whose features' squared distance from the
// state's passes the transition cost's square (Holden's lmm.h), the
// projection becomes the state before the next step and the Stepper gives
// the one after, the clock at 0; a node that had a state then requests
// its duration of inertialization above. Last, the Decompressor's
// motion outputs at the state give the root's move over the step (the
// velocity times the step, the rotation by the angular velocity times
// the step) at the node's root weight, and each event whose state rose
// past 0.5 is listed (the node's weight past MANI_GRAPH_EVENT_THRESHOLD).

#include "graph_learned_step.h"

#include "bytes.h"
#include "container.h"
#include "graph_emit.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_learned.h"
#include "learned_eval.h"
#include "match_file.h"
#include "root_motion.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/rootmotion.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float FloatOf(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static uint32_t BitsOf(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

// The node being moved: its words, block, state, model and room (the
// model's scratch, a state, the motion outputs).
typedef struct Learned
{
    maniGraphUpdate* u;
    maniGraphNodeState* s;
    uint32_t* block;
    const maniLearnedModel* m;
    float* scratch;
    float* state;
    float* motion;
    size_t scratchSize;
    uint32_t p;
    uint32_t data;
    uint32_t states;
    uint32_t reserved;
} Learned;

// The model's data failed (a value past the float range): the node is
// named in its view and the update reports it.
static void Fail(const Learned* l)
{
    l->s->flags |= MANI_GRAPH_FAILED;
    l->u->failed = 1;
}

static uint32_t Word(const Learned* l, uint32_t k)
{
    return maniGraphUpdateWord(l->u, l->data + k);
}

static double Clock(const Learned* l)
{
    uint64_t bits = (uint64_t)l->block[maniGraphLearnedClock] |
                    (uint64_t)l->block[maniGraphLearnedClock + 1] << 32;
    double clock = 0.0;
    memcpy(&clock, &bits, sizeof clock);
    return clock;
}

static void SetClock(const Learned* l, double clock)
{
    uint64_t bits = 0;
    memcpy(&bits, &clock, sizeof bits);
    l->block[maniGraphLearnedClock] = (uint32_t)bits;
    l->block[maniGraphLearnedClock + 1] = (uint32_t)(bits >> 32);
}

// The state after from: from's bits in the after state, then stepped. A
// step the model's data refuses leaves from's state there, held.
static void After(const Learned* l, const uint32_t* from)
{
    uint32_t* after = l->block + maniGraphLearnedBefore + l->states;
    for (uint32_t k = 0; k < l->states; ++k)
    {
        l->state[k] = FloatOf(from[k]);
    }
    if (maniLearnedModel_Step(l->m, l->state, l->scratch, l->scratchSize) != mani_success)
    {
        Fail(l);
    }
    for (uint32_t k = 0; k < l->states; ++k)
    {
        after[k] = BitsOf(l->state[k]);
    }
}

// The clock moved by step, the state stepped each time a step passes.
static void Move(const Learned* l, double step)
{
    double seconds = 1.0 / (double)l->m->stepRate;
    double clock = Clock(l) + step;
    uint32_t* before = l->block + maniGraphLearnedBefore;
    while (clock >= seconds)
    {
        memcpy(before, before + l->states, sizeof(uint32_t) * l->states);
        After(l, before);
        clock -= seconds;
    }
    SetClock(l, clock);
}

// The query: the state's features, the parameters' trajectory and float
// channels normalized as the model's database.
static void Query(const Learned* l, float* out)
{
    const maniLearnedModel* m = l->m;
    uint32_t n = m->featureCount;
    memcpy(out, l->state, sizeof(float) * n);
    uint32_t q = 0;
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        maniMatchChannel ch;
        (void)maniLearnedModel_GetChannel(m, c, &ch);
        uint32_t width = maniMatchChannelWidth(ch.kind);
        bool vector =
            ch.kind == maniMatchTrajectoryPosition || ch.kind == maniMatchTrajectoryFacing;
        for (uint32_t t = 0; t < (vector || ch.kind == maniMatchFloatTrack ? ch.timeCount : 0u);
             ++t, ++q)
        {
            const maniGraphValue* v =
                &l->u->instance->cells[Word(l, maniGraphLearnedQuery + q)].value;
            for (uint32_t i = 0; i < width; ++i)
            {
                uint32_t j = ch.firstFeature + t * width + i;
                float offset = FloatOf(maniLoadU32(m->normalization + 4 * (size_t)j));
                float scale = FloatOf(maniLoadU32(m->normalization + 4 * (size_t)(n + j)));
                out[j] = (v->f[i] - offset) / scale;
            }
        }
    }
}

// Tags: an int parameter's bits, or the literal.
static uint32_t Tags(const Learned* l, uint32_t k)
{
    uint32_t parameter = Word(l, k);
    return parameter == MANI_NO_PARAMETER ? Word(l, k + 1)
                                          : (uint32_t)l->u->instance->cells[parameter].value.i;
}

// The Projector whose filter is the node's tags, or UINT32_MAX.
static uint32_t Projector(const Learned* l)
{
    uint32_t required = Tags(l, maniGraphLearnedRequiredTags);
    uint32_t excluded = Tags(l, maniGraphLearnedExcludedTags);
    for (uint32_t p = 0; p < l->m->projectorCount; ++p)
    {
        uint32_t r = 0;
        uint32_t e = 0;
        (void)maniLearnedModel_GetProjector(l->m, p, &r, &e);
        if (r == required && e == excluded)
        {
            return p;
        }
    }
    return UINT32_MAX;
}

static void Project(const Learned* l)
{
    uint32_t projector = Projector(l);
    if (projector == UINT32_MAX)
    {
        return;
    }
    float query[MANI_MAX_MATCH_FEATURES];
    float projected[MANI_GRAPH_LEARNED_MAX_STATES];
    Query(l, query);
    maniResult r =
        maniLearnedModel_Project(l->m, projector, query, projected, MANI_GRAPH_LEARNED_MAX_STATES,
                                 NULL, l->scratch, l->scratchSize);
    if (r != mani_success)
    {
        if (r == mani_errorFormat)
        {
            Fail(l);
        }
        return;
    }
    bool had = l->block[maniGraphLearnedHas] != 0;
    float cost = FloatOf(Word(l, maniGraphLearnedTransitionCost));
    float apart = 0.0f;
    for (uint32_t k = 0; k < l->m->featureCount; ++k)
    {
        float d = l->state[k] - projected[k];
        apart += d * d;
    }
    if (had && !(apart > cost * cost))
    {
        return;
    }
    uint32_t* before = l->block + maniGraphLearnedBefore;
    for (uint32_t k = 0; k < l->states; ++k)
    {
        before[k] = BitsOf(projected[k]);
    }
    After(l, before);
    // The state at the clock's 0: the projection.
    memcpy(l->state, projected, sizeof(float) * l->states);
    SetClock(l, 0.0);
    l->block[maniGraphLearnedHas] = 1;
    if (had)
    {
        maniGraphRequest(l->u, l->s->inertia,
                         maniGraphUpdateScalar(l->u, l->data + maniGraphLearnedDurationScalar));
    }
}

// The root's move over step from the motion outputs: the velocity times
// the step, the rotation by the angular velocity times the step. False
// when a product or the rotation's angle passes the float range (finite
// outputs over a step of a slow model's seconds can).
static bool RootMove(const Learned* l, double step, maniRootMove* move)
{
    const float* motion = l->motion;
    float seconds = (float)step;
    maniVec3 v = {motion[0] * seconds, motion[1] * seconds, motion[2] * seconds};
    maniVec3 w = {motion[3] * seconds, motion[4] * seconds, motion[5] * seconds};
    if (!(fabsf(v.x) < HUGE_VALF && fabsf(v.y) < HUGE_VALF && fabsf(v.z) < HUGE_VALF) ||
        !(maniVec3Length(w) < HUGE_VALF))
    {
        return false;
    }
    *move = (maniRootMove){v, maniQuatFromRotationVector(w)};
    return true;
}

// The root's move and the events at the state; the Decompressor's hidden
// values kept with the state for the evaluation.
static void Outputs(const Learned* l, double step)
{
    const maniLearnedModel* m = l->m;
    float* cache = maniGraphLearnedCache(l->u->instance, l->p);
    maniRootMove move = maniGetIdentityRootMove();
    if (!maniLearnedDecompressMotion(m, l->state, l->motion, l->scratch,
                                     cache != NULL ? cache + 1 + l->states : NULL) ||
        !RootMove(l, step, &move))
    {
        // No move and no event from the node; the cache kept for no state.
        if (cache != NULL)
        {
            cache[0] = 0.0f;
        }
        Fail(l);
        return;
    }
    if (cache != NULL)
    {
        cache[0] = 1.0f;
        memcpy(cache + 1, l->state, sizeof(float) * l->states);
    }
    maniGraphNodeState* s = l->s;
    if (s->rootWeight > 0.0f)
    {
        move = maniGraphNodeMove(l->u, l->p, move);
        maniRootMotionSumAdd(&l->u->root, &move, (double)s->rootWeight);
    }
    uint32_t* bits = l->block + maniGraphLearnedBefore + 2 * l->states;
    const float* events = l->motion + 6 + m->floatTrackCount;
    bool listed = s->weight > MANI_GRAPH_EVENT_THRESHOLD;
    for (uint32_t e = 0; e < m->eventCount; ++e)
    {
        uint32_t bit = 1u << (e % 32);
        bool on = events[e] > 0.5f;
        bool was = (bits[e / 32] & bit) != 0;
        bits[e / 32] = on ? bits[e / 32] | bit : bits[e / 32] & ~bit;
        if (!on || was || !listed)
        {
            continue;
        }
        maniGraphUpdate* u = l->u;
        if (u->count < u->capacity)
        {
            size_t length = 0;
            const char* name = maniLearnedModel_GetEventName(m, e, &length);
            uint64_t hash = maniGraphNodeName(
                u, l->p, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length));
            maniGraphEvent* out = &u->events[u->count];
            memset(out, 0, sizeof *out);
            out->node = l->p;
            out->clip = MANI_NO_TRACK;
            out->event = e;
            out->nameHash = hash;
            out->weight = s->weight;
            out->time = (float)s->time;
            out->direction = 1;
        }
        u->count += u->count < UINT32_MAX ? 1u : 0u;
        s->flags |= MANI_GRAPH_LISTED;
    }
}

void maniGraphAdvanceLearned(maniGraphUpdate* u, uint32_t p)
{
    maniGraphNodeState* s = maniGraphUpdateNode(u, p);
    s->from = s->time;
    s->step = 0.0;
    if ((s->flags & MANI_GRAPH_REACHED) == 0)
    {
        return;
    }
    uint32_t data = maniLoadU32(u->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)u->instance + u->instance->machinesAt) +
                      maniGraphUpdateWord(u, data + maniGraphLearnedBlock);
    const maniLearnedModel* m =
        maniGraphBoundModel(u->binding, maniGraphUpdateWord(u, data + maniGraphLearnedModel));
    float* room = (float*)((uint8_t*)u->instance + u->instance->roomAt);
    size_t scratchSize = maniLearnedModel_GetScratchSize(m);
    // The scratch is whole floats; the state follows it.
    size_t stateAt = scratchSize / 4;
    uint32_t states = m->featureCount + m->latentCount;
    Learned l = {u,           s, block, m,      room, room + stateAt, room + stateAt + states,
                 scratchSize, p, data,  states, 0};
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphResetLearned(u->graph, data, block);
    }
    if (!(s->weight > 0.0f))
    {
        return;
    }
    // Bounded: an update costs at most the limit's Stepper evaluations,
    // and its root motion stays finite.
    double most = MANI_GRAPH_LEARNED_MAX_STEPS / (double)m->stepRate;
    double step = fabs(u->delta * (double)s->factor);
    step = step < most ? step : most;
    bool has = block[maniGraphLearnedHas] != 0;
    if (has)
    {
        Move(&l, step);
    }
    maniGraphLearnedState(block, states, m->stepRate, l.state);
    double since = (double)FloatOf(block[maniGraphLearnedSince]) + step;
    uint32_t trigger = Word(&l, maniGraphLearnedTrigger);
    bool triggered = trigger != MANI_NO_PARAMETER && u->instance->cells[trigger].value.b != 0;
    if (triggered)
    {
        u->instance->cells[trigger].value.b = 0;
    }
    bool due = !has || triggered || since >= (double)FloatOf(Word(&l, maniGraphLearnedPeriod));
    block[maniGraphLearnedSince] = BitsOf(due ? 0.0f : (float)since);
    if (due)
    {
        Project(&l);
    }
    if (block[maniGraphLearnedHas] != 0)
    {
        s->time += step;
        s->step = step;
        Outputs(&l, step);
    }
}
