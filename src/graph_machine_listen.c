// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Positions are normalized playheads; a clip of no duration stays at 0.
// A one-shot records its leading clip's clip seconds left. The whole
// loops a repeating clip wrapped come from where its playhead
// landed, so a crossing at a loop's end counts in one update only. A
// clip's events heard are the stretches its update listed: the same
// ranges, the same whole loops.

#include "graph_machine_listen.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_machine.h"
#include "graph_shot.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct Listen
{
    const maniGraph* graph;
    const maniClip* const* clips;
    maniGraphCell* cells;
    uint32_t* blocks;
} Listen;

static const uint8_t* Record(const Listen* l, uint32_t p)
{
    return l->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
}

static maniGraphNodeState* Node(const Listen* l, uint32_t p)
{
    return &l->cells[l->graph->parameterCount + p].node;
}

static uint32_t Word(const Listen* l, uint32_t at)
{
    return maniLoadU32(l->graph->data + 4 * (size_t)at);
}

static maniGraphMachine MachineAt(const Listen* l, uint32_t p)
{
    const uint8_t* record = Record(l, p);
    return (maniGraphMachine){l->graph, maniLoadU32(record + 12), maniLoadU32(record + 8)};
}

// A clip node's clip, its loop as the node says.
static maniClip ClipOf(const Listen* l, uint32_t p)
{
    uint32_t data = maniLoadU32(Record(l, p) + 12);
    maniClip clip = *l->clips[Word(l, data)];
    uint32_t loop = Word(l, data + 2);
    clip.loop = loop == 0 ? clip.loop : (loop == 1 ? maniLoopClamp : maniLoopRepeat);
    return clip;
}

static bool Repeats(const maniClip* clip)
{
    return clip->loop == maniLoopRepeat && clip->duration > 0.0f;
}

// Whether an event of the name lies in one stretch the playhead passed.
static bool InRange(const maniClip* clip, double from, double to, uint64_t hash)
{
    uint32_t first = 0;
    uint32_t count = 0;
    maniClipEventRange(clip, (float)from, (float)to, &first, &count);
    bool heard = false;
    for (uint32_t k = 0; k < count && !heard; ++k)
    {
        heard = maniClipEventAt(clip, first + k).nameHash == hash;
    }
    return heard;
}

// Whether the playhead from t0 over step passed an event of the name,
// across wraps (a whole loop passes every event).
static bool Heard(const maniClip* clip, double t0, double step, uint64_t hash)
{
    double t1 = t0 + step;
    double d = (double)clip->duration;
    if (!Repeats(clip))
    {
        return InRange(clip, t0, t1 < 0.0 ? 0.0 : (t1 > d ? d : t1), hash);
    }
    double wraps = floor(t1 / d);
    if (wraps == 0.0)
    {
        return InRange(clip, t0, t1, hash);
    }
    bool forward = wraps > 0.0;
    double start = forward ? 0.0 : d;
    double end = forward ? d : 0.0;
    return InRange(clip, t0, end, hash) || (fabs(wraps) > 1.0 && InRange(clip, start, end, hash)) ||
           InRange(clip, start, t1 - wraps * d, hash);
}

// State k's clip of greatest effective weight, the first of equals, or
// MANI_GRAPH_NO_NODE when it has none.
static uint32_t Leading(const Listen* l, const maniGraphMachine* m, uint32_t k)
{
    uint32_t leading = MANI_GRAPH_NO_NODE;
    float most = 0.0f;
    for (uint32_t i = 0; i < maniGraphMachineState(m, k, 3); ++i)
    {
        uint32_t clip = maniGraphMachineClip(m, k, i);
        float weight = Node(l, clip)->weight;
        if (leading == MANI_GRAPH_NO_NODE || weight > most)
        {
            leading = clip;
            most = weight;
        }
    }
    return leading;
}

// The leading clip's move: positions before and after, whole wraps.
static void Move(const Listen* l, uint32_t clip, uint32_t* block)
{
    const maniGraphNodeState* s = Node(l, clip);
    maniClip c = ClipOf(l, clip);
    double d = (double)c.duration;
    double before = d > 0.0 ? s->from / d : 0.0;
    double after = d > 0.0 ? s->time / d : 0.0;
    double wraps = Repeats(&c) ? floor((s->from + s->step - s->time) / d + 0.5) : 0.0;
    maniGraphSetBlockDouble(block, maniGraphBlockBefore, before < 1.0 ? before : 1.0);
    maniGraphSetBlockDouble(block, maniGraphBlockPosition, after < 1.0 ? after : 1.0);
    maniGraphSetBlockDouble(block, maniGraphBlockWraps, wraps);
}

// Whether the current state finished: its nearest machine final and
// settled, or with none its leading clip, not repeating, at the end it
// moves toward (its end when still).
static bool Finished(const Listen* l, const maniGraphMachine* m, uint32_t k, uint32_t leading)
{
    uint32_t nearest = maniGraphMachineState(m, k, 1);
    if (nearest != MANI_GRAPH_NO_NODE)
    {
        maniGraphMachine inner = MachineAt(l, nearest);
        const uint32_t* block = l->blocks + maniGraphMachineBlockAt(&inner);
        return maniGraphMachineState(&inner, block[maniGraphBlockCurrent], 0) != 0 &&
               block[maniGraphBlockRunning] == MANI_GRAPH_NO_TRANSITION;
    }
    if (leading == MANI_GRAPH_NO_NODE)
    {
        return false;
    }
    const maniGraphNodeState* s = Node(l, leading);
    maniClip c = ClipOf(l, leading);
    return !Repeats(&c) && (s->step < 0.0 ? s->time <= 0.0 : s->time >= (double)c.duration);
}

// Each event condition of the current state's candidates heard from a
// clip beneath the state that listed its events.
static void Hear(const Listen* l, const maniGraphMachine* m, uint32_t* block, uint32_t k)
{
    for (uint32_t t = 0; t < maniGraphMachineCount(m); ++t)
    {
        uint32_t from = maniGraphMachineTransition(m, t, 0);
        uint32_t first = maniGraphMachineTransition(m, t, 6);
        uint32_t count =
            (from == k || from == MANI_GRAPH_ANY_STATE) ? maniGraphMachineTransition(m, t, 7) : 0u;
        for (uint32_t c = first; c < first + count; ++c)
        {
            uint32_t at = maniGraphMachineCondition(m, c);
            if (maniGraphMachineWord(m, at) != maniGraphConditionEvent)
            {
                continue;
            }
            uint64_t hash =
                maniGraphMachineWord(m, at + 1) + ((uint64_t)maniGraphMachineWord(m, at + 2) << 32);
            bool heard = false;
            for (uint32_t i = 0; i < maniGraphMachineState(m, k, 3) && !heard; ++i)
            {
                uint32_t clip = maniGraphMachineClip(m, k, i);
                const maniGraphNodeState* s = Node(l, clip);
                maniClip view = ClipOf(l, clip);
                heard = (s->flags & MANI_GRAPH_LISTED) != 0 && Heard(&view, s->from, s->step, hash);
            }
            if (heard)
            {
                maniGraphSetMachineHeard(m, block, c);
            }
        }
    }
}

// One machine's record: the loops of the move before added, then this
// update's move, finish and events; a machine no parent reached records
// no move and keeps its finish.
static void Keep(const Listen* l, uint32_t p)
{
    maniGraphMachine m = MachineAt(l, p);
    uint32_t* block = l->blocks + maniGraphMachineBlockAt(&m);
    double loops = maniGraphBlockDouble(block, maniGraphBlockLoops);
    bool finished = block[maniGraphBlockFinished] != 0;
    loops += fabs(maniGraphRecordMove(block));
    maniGraphClearRecord(&m, block);
    maniGraphSetBlockDouble(block, maniGraphBlockLoops, loops);
    block[maniGraphBlockFinished] = finished ? 1u : 0u;
    if ((Node(l, p)->flags & MANI_GRAPH_REACHED) == 0)
    {
        return;
    }
    uint32_t k = block[maniGraphBlockCurrent];
    uint32_t leading = Leading(l, &m, k);
    if (leading != MANI_GRAPH_NO_NODE)
    {
        Move(l, leading, block);
    }
    block[maniGraphBlockFinished] = Finished(l, &m, k, leading) ? 1u : 0u;
    Hear(l, &m, block, k);
}

// A one-shot a parent reached: the clip seconds its leading clip has left
// toward the end it moves to (+inf for none or repeating).
static void KeepShot(const Listen* l, uint32_t p)
{
    if ((Node(l, p)->flags & MANI_GRAPH_REACHED) == 0)
    {
        return;
    }
    maniGraphShot shot = {l->graph, maniLoadU32(Record(l, p) + 12), 0};
    uint32_t leading = MANI_GRAPH_NO_NODE;
    float most = 0.0f;
    for (uint32_t i = 0; i < maniGraphShotWord(&shot, maniGraphShotClips); ++i)
    {
        uint32_t clip = maniGraphShotClip(&shot, i);
        float weight = Node(l, clip)->weight;
        if (leading == MANI_GRAPH_NO_NODE || weight > most)
        {
            leading = clip;
            most = weight;
        }
    }
    double left = HUGE_VAL;
    if (leading != MANI_GRAPH_NO_NODE)
    {
        const maniGraphNodeState* s = Node(l, leading);
        maniClip c = ClipOf(l, leading);
        left = Repeats(&c) ? HUGE_VAL : (s->step < 0.0 ? s->time : (double)c.duration - s->time);
    }
    uint32_t* block = l->blocks + maniGraphShotWord(&shot, maniGraphShotBlock);
    maniGraphSetBlockDouble(block, maniGraphShotRemaining, left > 0.0 ? left : 0.0);
}

void maniGraphListen(const maniGraph* graph, const maniClip* const* clips,
                     maniGraphInstance* instance)
{
    Listen l = {graph, clips, instance->cells,
                (uint32_t*)((uint8_t*)instance + instance->machinesAt)};
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        uint32_t kind = maniLoadU16(Record(&l, p));
        maniGraphMachine m = MachineAt(&l, p);
        if (kind == maniGraphNodeMachine && maniGraphMachineListening(&m))
        {
            Keep(&l, p);
        }
        else if (kind == maniGraphNodeOneShot)
        {
            KeepShot(&l, p);
        }
    }
}
