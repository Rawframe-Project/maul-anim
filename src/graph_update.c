// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Updating a graph instance (mani-0023): weights shared down from the
// output in reverse node order, then every clip node's playhead moved in
// node order, its events listed and its root motion blended.

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "root_motion.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The most whole loops a step lists or counts, as moves bound them.
#define MAX_WRAPS 4294967296.0

typedef struct Update
{
    maniGraphInstance* instance;
    const maniGraph* graph;
    const maniGraphBinding* binding;
    double delta;
    maniGraphEvent* events;
    uint32_t capacity;
    uint32_t reserved;
    uint64_t count;
    maniRootMotionSum root;
} Update;

static maniGraphNodeState* Node(const Update* u, uint32_t p)
{
    return &u->instance->cells[u->graph->parameterCount + p].node;
}

static const uint8_t* Record(const Update* u, uint32_t p)
{
    return u->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
}

static uint32_t Word(const Update* u, uint32_t at)
{
    return maniLoadU32(u->graph->data + 4 * (size_t)at);
}

static uint32_t InputOf(const Update* u, const uint8_t* record, uint32_t k)
{
    return maniLoadU32(u->graph->inputs + 4 * (size_t)(maniLoadU32(record + 4) + k));
}

// A scalar's value: its float parameter's, or its literal.
static float Scalar(const Update* u, uint32_t at)
{
    uint32_t parameter = Word(u, at);
    if (parameter != MANI_NO_PARAMETER)
    {
        return u->instance->cells[parameter].value.f[0];
    }
    uint32_t bits = Word(u, at + 1);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// What a parent hands an input: shares of its weights, and its time
// factor and seek when the input has none yet.
typedef struct Share
{
    float weight;
    float rootWeight;
    float factor;
    float seekTo;
    uint32_t flags;
} Share;

static void Give(const Update* u, uint32_t input, Share share)
{
    maniGraphNodeState* s = Node(u, input);
    s->weight += share.weight;
    s->rootWeight += share.rootWeight;
    if ((s->flags & MANI_GRAPH_REACHED) == 0)
    {
        s->factor = share.factor;
        s->seekTo = share.seekTo;
        s->flags = MANI_GRAPH_REACHED | (share.flags & MANI_GRAPH_SEEKING);
    }
}

static Share Pass(const maniGraphNodeState* s, float part, float rootPart)
{
    return (Share){s->weight * part, s->rootWeight * rootPart, s->factor, s->seekTo, s->flags};
}

static void ShareBlend(const Update* u, const uint8_t* record, const maniGraphNodeState* s)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t n = maniLoadU32(record + 8);
    bool direct = Word(u, d) != 0;
    float total = 0.0f;
    for (uint32_t k = 0; k < n; ++k)
    {
        float v = Scalar(u, d + 1 + 2 * k);
        total += v > 0.0f ? v : 0.0f;
    }
    float scale = direct ? 1.0f : 1.0f / (total > MANI_GRAPH_EVENT_THRESHOLD ? total : 1e-5f);
    for (uint32_t k = 0; k < n; ++k)
    {
        float v = Scalar(u, d + 1 + 2 * k);
        v = v > 0.0f ? v : 0.0f;
        v = direct && v > 1.0f ? 1.0f : v;
        Give(u, InputOf(u, record, k), Pass(s, v * scale, v * scale));
    }
}

static void ShareMask(const Update* u, const uint8_t* record, const maniGraphNodeState* s)
{
    uint32_t bones = u->graph->skeletonBoneCount;
    uint32_t mask = Word(u, maniLoadU32(record + 12));
    const uint8_t* weights = u->graph->maskWeights + 4 * (size_t)mask * bones;
    float inside = 0.0f;
    float outside = 0.0f;
    for (uint32_t b = 0; b < bones; ++b)
    {
        float m = 0.0f;
        memcpy(&m, weights + 4 * (size_t)b, sizeof m);
        inside = m > inside ? m : inside;
        outside = 1.0f - m > outside ? 1.0f - m : outside;
    }
    float root = 0.0f;
    uint32_t bone = u->binding->rootMotionBone;
    if (bone < bones)
    {
        memcpy(&root, weights + 4 * (size_t)bone, sizeof root);
    }
    Give(u, InputOf(u, record, 0), Pass(s, inside, root));
    Give(u, InputOf(u, record, 1), Pass(s, outside, 1.0f - root));
}

static void ShareAdditive(const Update* u, const uint8_t* record, const maniGraphNodeState* s)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t n = maniLoadU32(record + 8);
    Give(u, InputOf(u, record, 0), Pass(s, 1.0f, 1.0f));
    for (uint32_t k = 1; k < n; ++k)
    {
        float v = Scalar(u, d + 2 * (k - 1));
        v = v > 0.0f ? (v < 1.0f ? v : 1.0f) : 0.0f;
        Give(u, InputOf(u, record, k), Pass(s, v, 0.0f));
    }
}

// The input a select's parameter chooses, clamped to its inputs.
static uint32_t Chosen(const Update* u, const uint8_t* record)
{
    uint32_t parameter = Word(u, maniLoadU32(record + 12));
    uint32_t n = maniLoadU32(record + 8);
    const uint8_t* entry = u->graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE;
    const maniGraphValue* v = &u->instance->cells[parameter].value;
    int64_t i = maniLoadU32(entry) == maniGraphParameterInt ? (int64_t)v->i : (int64_t)v->b;
    return i < 0 ? 0u : (i >= (int64_t)n ? n - 1 : (uint32_t)i);
}

static void ShareOne(const Update* u, const uint8_t* record, maniGraphNodeState* s)
{
    uint32_t kind = maniLoadU16(record);
    uint32_t d = maniLoadU32(record + 12);
    Share share = Pass(s, 1.0f, 1.0f);
    if (kind == maniGraphNodeSelect)
    {
        s->word = Chosen(u, record);
        Give(u, InputOf(u, record, s->word), share);
        return;
    }
    if (kind == maniGraphNodeTimeScale)
    {
        share.factor *= Scalar(u, d);
    }
    else if (kind == maniGraphNodeSeek)
    {
        maniGraphValue* trigger = &u->instance->cells[Word(u, d)].value;
        if (trigger->b != 0)
        {
            trigger->b = 0;
            share.flags |= MANI_GRAPH_SEEKING;
            share.seekTo = Scalar(u, d + 1);
        }
    }
    Give(u, InputOf(u, record, 0), share);
}

// Weights from the output down: each node, last first, shares its own.
static void Weigh(const Update* u)
{
    uint32_t n = u->graph->nodeCount;
    for (uint32_t p = 0; p < n; ++p)
    {
        maniGraphNodeState* s = Node(u, p);
        s->weight = 0.0f;
        s->rootWeight = 0.0f;
        s->factor = 0.0f;
        s->flags = 0;
        s->seekTo = 0.0f;
    }
    maniGraphNodeState* output = Node(u, n - 1);
    *output = (maniGraphNodeState){1.0f, output->word,       output->time, 1.0f,
                                   1.0f, MANI_GRAPH_REACHED, 0.0f};
    for (uint32_t p = n; p-- > 0;)
    {
        const uint8_t* record = Record(u, p);
        maniGraphNodeState* s = Node(u, p);
        uint32_t kind = maniLoadU16(record);
        if (kind == maniGraphNodeBlend)
        {
            ShareBlend(u, record, s);
        }
        else if (kind == maniGraphNodeMask)
        {
            ShareMask(u, record, s);
        }
        else if (kind == maniGraphNodeAdditive)
        {
            ShareAdditive(u, record, s);
        }
        else if (kind != maniGraphNodeClip)
        {
            ShareOne(u, record, s);
        }
    }
}

// A clip node's clip, its loop as the node says.
static maniClip ClipOf(const Update* u, uint32_t data)
{
    maniClip clip = *u->binding->clips[Word(u, data)];
    uint32_t loop = Word(u, data + 2);
    clip.loop = loop == 0 ? clip.loop : (loop == 1 ? maniLoopClamp : maniLoopRepeat);
    return clip;
}

static bool Repeats(const maniClip* clip)
{
    return clip->loop == maniLoopRepeat && clip->duration > 0.0f;
}

// A time placed in the clip: wrapped into it, or clamped to it.
static double Place(const maniClip* clip, double t)
{
    double d = (double)clip->duration;
    if (!Repeats(clip))
    {
        return t < 0.0 ? 0.0 : (t > d ? d : t);
    }
    double w = fmod(t, d);
    w = w < 0.0 ? w + d : w;
    return w < d ? w : 0.0;
}

// The events of one stretch within the clip, listed while the buffer
// lasts and counted.
static void Emit(Update* u, uint32_t node, const maniClip* clip, double from, double to)
{
    uint32_t first = 0;
    uint32_t count = 0;
    maniClipEventRange(clip, (float)from, (float)to, &first, &count);
    const maniGraphNodeState* s = Node(u, node);
    for (uint32_t k = 0; k < count; ++k)
    {
        if (u->count < u->capacity)
        {
            maniClipEvent e = maniClipEventAt(clip, first + k);
            u->events[u->count] =
                (maniGraphEvent){node,       Word(u, maniLoadU32(Record(u, node) + 12)),
                                 first + k,  e.payload,
                                 e.nameHash, s->weight,
                                 e.time,     to >= from ? 1 : -1,
                                 0};
        }
        u->count += u->count < UINT32_MAX ? 1u : 0u;
    }
}

// Whole loops between the ends: listed while the buffer lasts, counted
// beyond it.
static void EmitLoops(Update* u, uint32_t node, const maniClip* clip, double loops, bool forward)
{
    double d = (double)clip->duration;
    double from = forward ? 0.0 : d;
    double to = forward ? d : 0.0;
    uint32_t first = 0;
    uint32_t perLoop = 0;
    maniClipEventRange(clip, (float)from, (float)to, &first, &perLoop);
    uint64_t n = (uint64_t)(loops < MAX_WRAPS ? loops : MAX_WRAPS);
    for (uint64_t k = 0; k < n; ++k)
    {
        if (u->count >= u->capacity || perLoop == 0)
        {
            // At most 2^32 loops of fewer than 2^32 events: no overflow.
            u->count += (n - k) * perLoop;
            u->count = u->count < UINT32_MAX ? u->count : UINT32_MAX;
            return;
        }
        Emit(u, node, clip, from, to);
    }
}

// The events a playhead passes from t0 over step, across wraps.
static void Events(Update* u, uint32_t node, const maniClip* clip, double t0, double step)
{
    double t1 = t0 + step;
    double d = (double)clip->duration;
    if (!Repeats(clip))
    {
        Emit(u, node, clip, t0, Place(clip, t1));
        return;
    }
    double wraps = floor(t1 / d);
    if (wraps == 0.0)
    {
        Emit(u, node, clip, t0, t1);
        return;
    }
    bool forward = wraps > 0.0;
    double loops = fabs(wraps) - 1.0;
    Emit(u, node, clip, t0, forward ? d : 0.0);
    EmitLoops(u, node, clip, loops, forward);
    double rest = t1 - wraps * d;
    Emit(u, node, clip, forward ? 0.0 : d, rest);
}

static void Advance(Update* u, uint32_t p, const uint8_t* record)
{
    maniGraphNodeState* s = Node(u, p);
    uint32_t data = maniLoadU32(record + 12);
    maniClip clip = ClipOf(u, data);
    maniRootMove move = maniRootMoveIdentity();
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        s->time = Place(&clip, (double)s->seekTo);
    }
    else
    {
        double step = u->delta * (double)Scalar(u, data + 4) * (double)s->factor;
        double t0 = s->time;
        if (s->weight > MANI_GRAPH_EVENT_THRESHOLD)
        {
            Events(u, p, &clip, t0, step);
        }
        if (s->rootWeight > 0.0f)
        {
            (void)maniClipRootMotion(&clip, (float)t0, (float)step, &move);
        }
        s->time = Place(&clip, t0 + step);
    }
    if (s->rootWeight > 0.0f)
    {
        maniRootMotionSumAdd(&u->root, &move, (double)s->rootWeight);
    }
}

maniResult maniUpdateGraph(maniGraphInstance* instance, float delta, maniGraphEvent* events,
                           uint32_t capacity, uint32_t* eventCountOut, maniRootMove* rootMotionOut)
{
    if (instance == NULL || instance->stamp != MANI_GRAPH_INSTANCE_STAMP || !isfinite(delta) ||
        (events == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    const maniGraphBinding* binding = instance->binding;
    Update u = {.instance = instance,
                .graph = &binding->graph,
                .binding = binding,
                .delta = (double)delta,
                .events = events,
                .capacity = capacity};
    Weigh(&u);
    for (uint32_t p = 0; p < u.graph->nodeCount; ++p)
    {
        const uint8_t* record = Record(&u, p);
        if (maniLoadU16(record) == maniGraphNodeClip)
        {
            Advance(&u, p, record);
        }
    }
    if (eventCountOut != NULL)
    {
        *eventCountOut = u.count < UINT32_MAX ? (uint32_t)u.count : UINT32_MAX;
    }
    if (rootMotionOut != NULL)
    {
        *rootMotionOut = maniRootMotionSumResult(&u.root);
    }
    return mani_success;
}
