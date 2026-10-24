// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Updating a graph instance: weights shared down from the
// output in reverse node order, then every clip node's and motion
// matching node's playhead moved in node order, its events listed and its
// root motion blended.

#include "bytes.h"
#include "fpenv.h"
#include "graph_emit.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_instance.h"
#include "graph_learned_step.h"
#include "graph_machine.h"
#include "graph_machine_listen.h"
#include "graph_machine_step.h"
#include "graph_match_step.h"
#include "graph_shot.h"
#include "graph_shot_step.h"
#include "graph_space.h"
#include "graph_spring.h"
#include "graph_turn.h"
#include "retarget_motion.h"
#include "root_motion.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef maniGraphUpdate Update;

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
    return maniGraphUpdateWord(u, at);
}

static uint32_t InputOf(const Update* u, const uint8_t* record, uint32_t k)
{
    return maniLoadU32(u->graph->inputs + 4 * (size_t)(maniLoadU32(record + 4) + k));
}

static float Scalar(const Update* u, uint32_t at)
{
    return maniGraphUpdateScalar(u, at);
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
    uint32_t inertia;
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
        s->inertia = share.inertia;
        s->flags = MANI_GRAPH_REACHED | (share.flags & (MANI_GRAPH_SEEKING | MANI_GRAPH_PHASE));
    }
}

// Whether a parent reached a node this update.
static bool Reached(const maniGraphNodeState* s)
{
    return (s->flags & MANI_GRAPH_REACHED) != 0;
}

static Share Pass(const maniGraphNodeState* s, float part, float rootPart)
{
    return (Share){s->weight * part, s->rootWeight * rootPart, s->factor, s->seekTo, s->flags,
                   s->inertia};
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

// A blend space's weights, shared as a normalized blend shares them.
static void ShareSpace(const Update* u, const uint8_t* record, const maniGraphNodeState* s)
{
    uint32_t n = maniLoadU32(record + 8);
    float weights[MANI_GRAPH_MAX_SPACE_POINTS];
    maniGraphSpaceWeights(u->graph, u->instance->cells, maniLoadU16(record),
                          maniLoadU32(record + 12), n, weights);
    for (uint32_t k = 0; k < n; ++k)
    {
        Give(u, InputOf(u, record, k), Pass(s, weights[k], weights[k]));
    }
}

static uint32_t* BlockAt(const Update* u, uint32_t first)
{
    return (uint32_t*)((uint8_t*)u->instance + u->instance->machinesAt) + first;
}

// A machine a parent reached stepped by its own time, then its states
// shared by their weights; a state it entered at weight 0 this step by a
// restart or a match reached even so, seeking 0 or the leading clip's
// phase, so its playheads start over or match. A seek above the machine
// starts it over too.
static void ShareMachine(const Update* u, const uint8_t* record, maniGraphNodeState* s)
{
    maniGraphMachine m = {u->graph, maniLoadU32(record + 12), maniLoadU32(record + 8)};
    uint32_t* block =
        (uint32_t*)((uint8_t*)u->instance + u->instance->machinesAt) + maniGraphMachineBlockAt(&m);
    if (!Reached(s))
    {
        return;
    }
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphResetMachine(&m, block);
    }
    maniGraphEntered entered;
    maniGraphStepMachine(&m, u->instance->cells, block, fabs(u->delta * (double)s->factor),
                         &entered);
    maniGraphRequest(u, s->inertia, entered.inertia);
    for (uint32_t k = 0; k < m.states; ++k)
    {
        float weight = maniGraphMachineWeight(block, k);
        Share share = Pass(s, weight, weight);
        uint32_t entry = maniGraphEntryKeep;
        for (uint32_t e = 0; e < entered.count; ++e)
        {
            entry = entered.states[e] == k ? entered.entries[e] : entry;
        }
        bool restart = entry != maniGraphEntryKeep;
        bool match = entry == maniGraphEntryMatch;
        double position = maniGraphBlockDouble(block, maniGraphBlockPosition);
        share.flags &= restart ? ~MANI_GRAPH_PHASE : ~0u;
        share.flags |= restart ? MANI_GRAPH_SEEKING : 0u;
        share.flags |= match ? MANI_GRAPH_PHASE : 0u;
        share.seekTo = restart ? (match ? (float)position : 0.0f) : share.seekTo;
        if (weight > 0.0f || restart)
        {
            Give(u, InputOf(u, record, k), share);
        }
    }
}

// A one-shot a parent reached stepped by its own time, then its base and
// shot shared as a mask of the shot's weight times the mask's per-bone
// weights (1 without one) shares them; a shot fire restarted reached
// even at weight 0, seeking 0. A seek above it stops the shot.
static void ShareShot(const Update* u, const uint8_t* record, const maniGraphNodeState* s)
{
    if (!Reached(s))
    {
        return;
    }
    maniGraphShot shot = {u->graph, maniLoadU32(record + 12), 0};
    uint32_t* block = (uint32_t*)((uint8_t*)u->instance + u->instance->machinesAt) +
                      maniGraphShotWord(&shot, maniGraphShotBlock);
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphResetShot(block);
    }
    bool restart =
        maniGraphStepShot(&shot, u->instance->cells, block, fabs(u->delta * (double)s->factor));
    float w = 0.0f;
    memcpy(&w, &block[maniGraphShotWeight], sizeof w);
    float most = 1.0f;
    float least = 1.0f;
    float root = 1.0f;
    uint32_t mask = maniGraphShotWord(&shot, maniGraphShotMask);
    if (mask != MANI_GRAPH_NO_MASK)
    {
        uint32_t bones = u->graph->skeletonBoneCount;
        const uint8_t* weights = u->graph->maskWeights + 4 * (size_t)mask * bones;
        most = 0.0f;
        for (uint32_t b = 0; b < bones; ++b)
        {
            float m = 0.0f;
            memcpy(&m, weights + 4 * (size_t)b, sizeof m);
            most = m > most ? m : most;
            least = m < least ? m : least;
            root = b == u->binding->rootMotionBone ? m : root;
        }
    }
    Give(u, InputOf(u, record, 0), Pass(s, 1.0f - w * least, 1.0f - w * root));
    if (w > 0.0f || restart)
    {
        Share share = Pass(s, w * most, w * root);
        share.flags =
            restart ? (share.flags | MANI_GRAPH_SEEKING) & ~MANI_GRAPH_PHASE : share.flags;
        share.seekTo = restart ? 0.0f : share.seekTo;
        Give(u, InputOf(u, record, 1), share);
    }
}

// An inertialize node a parent reached: its count and step for the
// evaluation, its clock advanced, its trigger read; its input reached as
// it was, with this node the nearest above. A seek above clears it.
static void ShareInertia(const Update* u, const uint8_t* record, const maniGraphNodeState* s,
                         uint32_t p)
{
    if (!Reached(s))
    {
        return;
    }
    uint32_t d = maniLoadU32(record + 12);
    uint32_t* block = BlockAt(u, Word(u, d + maniGraphInertiaBlock));
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphInertia node = maniGraphInertiaOf(u->graph, d);
        maniGraphResetInertia(block, &node);
    }
    float step = (float)fabs(u->delta * (double)s->factor);
    block[maniGraphInertiaSerial] += 1u;
    maniGraphInertiaSet(block, maniGraphInertiaStep, step);
    double elapsed = (double)maniGraphInertiaGet(block, maniGraphInertiaElapsed) + (double)step;
    maniGraphInertiaSet(block, maniGraphInertiaElapsed, (float)elapsed);
    uint32_t trigger = Word(u, d);
    if (trigger != MANI_NO_PARAMETER && u->instance->cells[trigger].value.b != 0)
    {
        u->instance->cells[trigger].value.b = 0;
        maniGraphRequest(u, p, Scalar(u, d + maniGraphInertiaDurationScalar));
    }
    Share share = Pass(s, 1.0f, 1.0f);
    share.inertia = p;
    Give(u, InputOf(u, record, 0), share);
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
            share.flags = (share.flags | MANI_GRAPH_SEEKING) & ~MANI_GRAPH_PHASE;
            share.seekTo = Scalar(u, d + 1);
        }
    }
    Give(u, InputOf(u, record, 0), share);
}

// Whether a reset trigger fired; a fired one is cleared.
static bool Consumed(const Update* u, uint32_t reset)
{
    bool fired = reset != MANI_NO_PARAMETER && u->instance->cells[reset].value.b != 0;
    if (fired)
    {
        u->instance->cells[reset].value.b = 0;
    }
    return fired;
}

// A spring node a parent reached: the time it moves by kept for its
// evaluation, its motion dropped by a seek above it or its reset trigger;
// its input reached as it was.
static void ShareSpring(const Update* u, const uint8_t* record, maniGraphNodeState* s)
{
    if (!Reached(s))
    {
        return;
    }
    uint32_t d = maniLoadU32(record + 12);
    uint32_t* block = BlockAt(u, Word(u, d + maniGraphSpringBlock));
    if (Consumed(u, Word(u, d + maniGraphSpringReset)) || (s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphResetSpring(u->graph, d, block);
    }
    maniGraphSpringAdd(block, fabs(u->delta * (double)s->factor));
    ShareOne(u, record, s);
}

// A turn limit node: not reached, what it holds dropped (each bone takes
// its input as it is when next evaluated); reached, the time it moves by kept
// for its evaluation, its hold dropped by a seek above it or its reset
// trigger; its input reached as it was.
static void ShareTurn(const Update* u, const uint8_t* record, maniGraphNodeState* s)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t* block = BlockAt(u, Word(u, d + maniGraphTurnBlock));
    if (!Reached(s))
    {
        maniGraphResetTurn(block, u->graph->skeletonBoneCount);
        return;
    }
    if (Consumed(u, Word(u, d + maniGraphTurnReset)) || (s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphResetTurn(block, u->graph->skeletonBoneCount);
    }
    maniGraphTurnAdd(block, fabs(u->delta * (double)s->factor));
    ShareOne(u, record, s);
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
        s->inertia = MANI_GRAPH_NO_NODE;
    }
    maniGraphNodeState* output = Node(u, n - 1);
    output->weight = 1.0f;
    output->rootWeight = 1.0f;
    output->factor = 1.0f;
    output->flags = MANI_GRAPH_REACHED;
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
        else if (kind == maniGraphNodeBlendSpace1D || kind == maniGraphNodeBlendSpaceTriangles ||
                 kind == maniGraphNodeBlendSpaceBands)
        {
            ShareSpace(u, record, s);
        }
        else if (kind == maniGraphNodeMachine)
        {
            ShareMachine(u, record, s);
        }
        else if (kind == maniGraphNodeOneShot)
        {
            ShareShot(u, record, s);
        }
        else if (kind == maniGraphNodeInertialize)
        {
            ShareInertia(u, record, s, p);
        }
        else if (kind == maniGraphNodeSpring)
        {
            ShareSpring(u, record, s);
        }
        else if (kind == maniGraphNodeTurnLimit)
        {
            ShareTurn(u, record, s);
        }
        else if (kind != maniGraphNodeClip && kind != maniGraphNodeMatch &&
                 kind != maniGraphNodeLearned && kind != maniGraphNodePose)
        {
            ShareOne(u, record, s);
        }
    }
}

// A mapped clip's move carried as the map carries its root motion bone's
// into the binding's; any other move as it is.
static maniRootMove Carried(const Update* u, uint32_t slot, const maniClip* clip, maniRootMove move)
{
    const maniGraphBoundMapPair* pair = maniGraphClipMapPair(u->binding, slot);
    maniRootMotionSource source;
    if (pair == NULL || !maniClipRootMotionSource(clip, &source))
    {
        return move;
    }
    maniRetargetMotion motion = maniRetargetMotionOf(
        pair->map, pair->source, source.bone, u->binding->skeleton, u->binding->rootMotionBone);
    return maniRetargetMove(&motion, move);
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

// The events a playhead passes from t0 over step, across wraps.
static void Events(Update* u, uint32_t node, uint32_t slot, const maniClip* clip, double t0,
                   double step)
{
    double t1 = t0 + step;
    double d = (double)clip->duration;
    if (!Repeats(clip))
    {
        maniGraphEmit(u, node, slot, clip, t0, Place(clip, t1));
        return;
    }
    double wraps = floor(t1 / d);
    if (wraps == 0.0)
    {
        maniGraphEmit(u, node, slot, clip, t0, t1);
        return;
    }
    bool forward = wraps > 0.0;
    double loops = fabs(wraps) - 1.0;
    maniGraphEmit(u, node, slot, clip, t0, forward ? d : 0.0);
    maniGraphEmitLoops(u, node, slot, clip, forward ? 0.0 : d, forward ? d : 0.0, loops);
    double rest = t1 - wraps * d;
    maniGraphEmit(u, node, slot, clip, forward ? 0.0 : d, rest);
}

// How a clip node moves this update: from t0 by step, or set to t0 by a
// seek (nothing passed); with its events or not.
typedef struct Motion
{
    double t0;
    double step;
    bool seeking;
    bool events;
} Motion;

// A node's own motion: its playhead by its speed and time factor.
static Motion OwnMotion(const Update* u, const maniGraphNodeState* s, const maniClip* clip,
                        uint32_t data)
{
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        double scale = (s->flags & MANI_GRAPH_PHASE) != 0 ? (double)clip->duration : 1.0;
        return (Motion){Place(clip, (double)s->seekTo * scale), 0.0, true, false};
    }
    return (Motion){s->time, u->delta * (double)Scalar(u, data + 4) * (double)s->factor, false,
                    true};
}

static maniGraphGroupState* Group(const Update* u, uint32_t group)
{
    return &u->instance->cells[u->graph->parameterCount + u->graph->nodeCount + group].group;
}

static uint32_t GroupOf(const Update* u, const uint8_t* record)
{
    return Word(u, maniLoadU32(record + 12) + 1);
}

// The next clip node of group g from node from on, or UINT32_MAX.
static uint32_t NextMember(const Update* u, uint32_t g, uint32_t from)
{
    for (uint32_t p = from; p < u->graph->nodeCount; ++p)
    {
        const uint8_t* record = Record(u, p);
        if (maniLoadU16(record) == maniGraphNodeClip && GroupOf(u, record) == g)
        {
            return p;
        }
    }
    return UINT32_MAX;
}

// A marker's name as node p plays its clip: its partner's under a
// mirror.
static uint64_t MarkerName(const Update* u, uint32_t p, const maniClip* clip, uint32_t k)
{
    return maniGraphNodeName(u, p, maniClipMarkerAt(clip, k).nameHash);
}

// Whether every member's clip has a marker of the name.
static bool InAll(const Update* u, uint32_t g, uint64_t hash)
{
    for (uint32_t p = NextMember(u, g, 0); p != UINT32_MAX; p = NextMember(u, g, p + 1))
    {
        const maniClip* clip = u->binding->clips[Word(u, maniLoadU32(Record(u, p) + 12))];
        bool found = false;
        for (uint32_t k = 0; k < clip->markerCount && !found; ++k)
        {
            found = MarkerName(u, p, clip, k) == hash;
        }
        if (!found)
        {
            return false;
        }
    }
    return true;
}

static bool Common(const Update* u, uint32_t g, uint32_t p, const maniClip* clip, uint32_t k)
{
    return InAll(u, g, MarkerName(u, p, clip, k));
}

// A group syncs by markers when every member repeats and the leader has
// a marker every member has.
static bool ByMarkers(const Update* u, uint32_t g, uint32_t lead, const maniClip* leader)
{
    for (uint32_t p = NextMember(u, g, 0); p != UINT32_MAX; p = NextMember(u, g, p + 1))
    {
        maniClip clip = ClipOf(u, maniLoadU32(Record(u, p) + 12));
        if (!Repeats(&clip))
        {
            return false;
        }
    }
    for (uint32_t k = 0; k < leader->markerCount; ++k)
    {
        if (Common(u, g, lead, leader, k))
        {
            return true;
        }
    }
    return false;
}

// The distance from a to b in the direction of play, within a loop
// (0 up to the duration).
static double Along(double a, double b, bool backward, double d)
{
    double x = fmod(backward ? a - b : b - a, d);
    return x < 0.0 ? x + d : x;
}

// The common marker after k in the direction of play, cyclic.
static uint32_t NextCommon(const Update* u, uint32_t g, uint32_t p, const maniClip* clip,
                           uint32_t k, bool backward)
{
    uint32_t n = clip->markerCount;
    for (uint32_t step = 1; step <= n; ++step)
    {
        uint32_t i = backward ? (k + n - step % n) % n : (k + step) % n;
        if (Common(u, g, p, clip, i))
        {
            return i;
        }
    }
    return k;
}

// The common marker last passed at t in the direction of play: before t
// going forward, after it going back, cyclic (one at t is still ahead,
// as the event windows have it).
static uint32_t Passed(const Update* u, uint32_t g, uint32_t p, const maniClip* clip, double t,
                       bool backward)
{
    uint32_t found = UINT32_MAX;
    double best = 0.0;
    for (uint32_t k = 0; k < clip->markerCount; ++k)
    {
        if (!Common(u, g, p, clip, k))
        {
            continue;
        }
        double d = (double)clip->duration;
        double behind = Along((double)maniClipMarkerAt(clip, k).time, t, backward, d);
        behind = behind > 0.0 ? behind : d;
        if (found == UINT32_MAX || behind < best)
        {
            found = k;
            best = behind;
        }
    }
    return found;
}

static double MarkerTime(const maniClip* clip, uint32_t k)
{
    return (double)maniClipMarkerAt(clip, k).time;
}

// The common markers a playhead passes from t0 over step: forward those
// at times in [t0, t1), backward in (t1, t0], across wraps.
static double CountPassed(const Update* u, uint32_t g, uint32_t p, const maniClip* clip, double t0,
                          double step)
{
    double d = (double)clip->duration;
    bool backward = step < 0.0;
    double distance = fabs(step);
    double loops = floor(distance / d);
    double perLoop = 0.0;
    double count = 0.0;
    for (uint32_t k = 0; k < clip->markerCount; ++k)
    {
        if (!Common(u, g, p, clip, k))
        {
            continue;
        }
        perLoop += 1.0;
        // How far ahead the marker lies; passed when within the rest.
        double ahead = Along(t0, MarkerTime(clip, k), backward, d);
        count += ahead < distance - loops * d ? 1.0 : 0.0;
    }
    return count + loops * perLoop;
}

// The leader's motion as a group sees it: its phase and how far it
// moves through it; by markers, its playhead and step in seconds.
static void LeaderMotion(const Update* u, uint32_t g)
{
    maniGraphGroupState* group = Group(u, g);
    uint32_t data = maniLoadU32(Record(u, group->leader) + 12);
    maniClip clip = ClipOf(u, data);
    Motion m = OwnMotion(u, Node(u, group->leader), &clip, data);
    double d = (double)clip.duration;
    group->flags |= m.seeking ? MANI_GRAPH_LEADER_SEEKING : 0u;
    group->phase = d > 0.0 ? m.t0 / d : 0.0;
    group->delta = d > 0.0 ? m.step / d : 0.0;
    if (!m.seeking && ByMarkers(u, g, group->leader, &clip))
    {
        group->flags |= MANI_GRAPH_MARKERS;
        group->phase = m.t0;
        group->delta = m.step;
    }
}

// Each group's leader: the first declared, else the member weighed most
// (the first of equals); then how it moves.
static void Lead(const Update* u)
{
    for (uint32_t g = 0; g < u->graph->syncGroupCount; ++g)
    {
        *Group(u, g) = (maniGraphGroupState){UINT32_MAX, 0, 0.0, 0.0};
    }
    for (uint32_t p = 0; p < u->graph->nodeCount; ++p)
    {
        const uint8_t* record = Record(u, p);
        uint32_t g = maniLoadU16(record) == maniGraphNodeClip ? GroupOf(u, record) : UINT32_MAX;
        if (g >= u->graph->syncGroupCount)
        {
            continue;
        }
        maniGraphGroupState* group = Group(u, g);
        bool declared = Word(u, maniLoadU32(record + 12) + 3) != 0;
        bool heavier =
            group->leader == UINT32_MAX || Node(u, p)->weight > Node(u, group->leader)->weight;
        if ((group->flags & MANI_GRAPH_DECLARED) == 0 && (declared || heavier))
        {
            group->leader = p;
            group->flags = declared ? MANI_GRAPH_DECLARED : 0u;
        }
    }
    for (uint32_t g = 0; g < u->graph->syncGroupCount; ++g)
    {
        if (Group(u, g)->leader != UINT32_MAX)
        {
            LeaderMotion(u, g);
        }
    }
}

// The span between two markers in the direction of play (a whole loop
// for one marker).
static double Span(const maniClip* clip, uint32_t a, uint32_t b, bool backward)
{
    double d = (double)clip->duration;
    double span = Along(MarkerTime(clip, a), MarkerTime(clip, b), backward, d);
    return span > 0.0 ? span : d;
}

// A place between markers: the common marker last passed, the next, and
// the fraction of the way between them.
typedef struct Between
{
    uint32_t previous;
    uint32_t next;
    double fraction;
} Between;

static Between PositionAt(const Update* u, uint32_t g, uint32_t p, const maniClip* clip, double t,
                          bool backward)
{
    uint32_t previous = Passed(u, g, p, clip, t, backward);
    uint32_t next = NextCommon(u, g, p, clip, previous, backward);
    double behind = Along(MarkerTime(clip, previous), t, backward, (double)clip->duration);
    return (Between){previous, next, behind / Span(clip, previous, next, backward)};
}

// The follower's place matching the leader's: of its pairs of markers
// with the leader's two names, the place at the leader's fraction nearest
// to t0; t0 when it has none.
static double Match(const Update* u, uint32_t g, uint32_t p, const maniClip* clip,
                    const maniClip* leader, Between want, double t0, bool backward)
{
    uint32_t lead = Group(u, g)->leader;
    double d = (double)clip->duration;
    double best = t0;
    double nearest = d;
    for (uint32_t i = 0; i < clip->markerCount; ++i)
    {
        uint32_t j = NextCommon(u, g, p, clip, i, backward);
        if (!Common(u, g, p, clip, i) ||
            MarkerName(u, p, clip, i) != MarkerName(u, lead, leader, want.previous) ||
            MarkerName(u, p, clip, j) != MarkerName(u, lead, leader, want.next))
        {
            continue;
        }
        double offset = want.fraction * Span(clip, i, j, backward);
        double t = Place(clip, MarkerTime(clip, i) + (backward ? -offset : offset));
        double ahead = Along(t0, t, false, d);
        double behind = Along(t, t0, false, d);
        double away = ahead < behind ? ahead : behind;
        if (away < nearest)
        {
            nearest = away;
            best = t;
        }
    }
    return best;
}

// A follower by markers: placed where the leader began (by names and
// fraction), past as many common markers as the leader passed, then on
// (at most a loop) until its markers carry the leader's final names, to
// the leader's final fraction. Its start, and the distance it moves in
// the direction of play.
static double FollowMarkers(const Update* u, uint32_t g, uint32_t p, const maniClip* clip,
                            double* t0)
{
    const maniGraphGroupState* group = Group(u, g);
    uint32_t lead = group->leader;
    maniClip leader = ClipOf(u, maniLoadU32(Record(u, lead) + 12));
    bool backward = group->delta < 0.0;
    Between start = PositionAt(u, g, lead, &leader, group->phase, backward);
    Between end =
        PositionAt(u, g, lead, &leader, Place(&leader, group->phase + group->delta), backward);
    double pass = CountPassed(u, g, lead, &leader, group->phase, group->delta);
    *t0 = Match(u, g, p, clip, &leader, start, *t0, backward);
    Between at = PositionAt(u, g, p, clip, *t0, backward);
    double travelled = -at.fraction * Span(clip, at.previous, at.next, backward);
    double perLoop = 0.0;
    for (uint32_t k = 0; k < clip->markerCount; ++k)
    {
        perLoop += Common(u, g, p, clip, k) ? 1.0 : 0.0;
    }
    if (perLoop > 0.0 && pass > 2.0 * perLoop)
    {
        double loops = floor((pass - perLoop) / perLoop);
        pass -= loops * perLoop;
        travelled += loops * (double)clip->duration;
    }
    // At most three loops of markers remain to walk.
    uint64_t steps = (uint64_t)(pass < 9.0e15 ? pass : 9.0e15);
    uint64_t limit = steps + (uint64_t)perLoop;
    for (uint64_t done = 0; done < limit; ++done)
    {
        bool match =
            MarkerName(u, p, clip, at.previous) == MarkerName(u, lead, &leader, end.previous) &&
            MarkerName(u, p, clip, at.next) == MarkerName(u, lead, &leader, end.next);
        if (done >= steps && match)
        {
            break;
        }
        travelled += Span(clip, at.previous, at.next, backward);
        at.previous = at.next;
        at.next = NextCommon(u, g, p, clip, at.next, backward);
    }
    return travelled + end.fraction * Span(clip, at.previous, at.next, backward);
}

// A follower's motion: by markers, or as far through its clip as the
// leader through its own from the leader's phase; its events only when
// the group asks.
static Motion Follow(const Update* u, uint32_t g, uint32_t p, const maniClip* clip, double t0)
{
    const maniGraphGroupState* group = Group(u, g);
    double d = (double)clip->duration;
    const uint8_t* entry = u->graph->syncGroups + (size_t)g * MANI_GRAPH_SYNC_GROUP_SIZE;
    bool all = (maniLoadU32(entry + 16) & 1u) != 0;
    bool seeking = (group->flags & MANI_GRAPH_LEADER_SEEKING) != 0;
    if ((group->flags & MANI_GRAPH_MARKERS) != 0)
    {
        double start = t0;
        double distance = FollowMarkers(u, g, p, clip, &start);
        bool backward = group->delta < 0.0;
        return (Motion){start, backward ? -distance : distance, false, all && distance >= 0.0};
    }
    return (Motion){Place(clip, group->phase * d), group->delta * d, seeking, all && !seeking};
}

// A clip node's move. One no parent reached and in no sync group stays
// where it is: its time factor 0, no seek, no weight, so no events and
// no root motion.
static void Advance(Update* u, uint32_t p, const uint8_t* record)
{
    maniGraphNodeState* s = Node(u, p);
    uint32_t group = GroupOf(u, record);
    if (!Reached(s) && group >= u->graph->syncGroupCount)
    {
        s->from = s->time;
        s->step = 0.0;
        return;
    }
    uint32_t data = maniLoadU32(record + 12);
    maniClip clip = ClipOf(u, data);
    Motion m = OwnMotion(u, s, &clip, data);
    if (group < u->graph->syncGroupCount && Group(u, group)->leader != p &&
        Group(u, group)->leader != UINT32_MAX)
    {
        m = Follow(u, group, p, &clip, s->time);
    }
    maniRootMove move = maniRootMoveIdentity();
    s->from = m.t0;
    s->step = m.seeking ? 0.0 : m.step;
    if (!m.seeking)
    {
        if (m.events && s->weight > MANI_GRAPH_EVENT_THRESHOLD)
        {
            s->flags |= MANI_GRAPH_LISTED;
            Events(u, p, Word(u, data), &clip, m.t0, m.step);
        }
        if (s->rootWeight > 0.0f)
        {
            (void)maniClipRootMotionEntered(&clip, (float)m.t0, (float)m.step, &move);
            move = maniGraphNodeMove(u, p, Carried(u, Word(u, data), &clip, move));
        }
    }
    s->time = Place(&clip, m.t0 + m.step);
    if (s->rootWeight > 0.0f)
    {
        maniRootMotionSumAdd(&u->root, &move, (double)s->rootWeight);
    }
}

static MANI_NOINLINE maniResult UpdateGraphWork(maniGraphInstance* instance, float delta,
                                                maniGraphEvent* events, uint32_t capacity,
                                                uint32_t* eventCountOut,
                                                maniRootMove* rootMotionOut)
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
    Lead(&u);
    for (uint32_t p = 0; p < u.graph->nodeCount; ++p)
    {
        const uint8_t* record = Record(&u, p);
        if (maniLoadU16(record) == maniGraphNodeClip)
        {
            Advance(&u, p, record);
        }
        else if (maniLoadU16(record) == maniGraphNodeMatch)
        {
            maniGraphAdvanceMatch(&u, p);
        }
        else if (maniLoadU16(record) == maniGraphNodeLearned)
        {
            maniGraphAdvanceLearned(&u, p);
        }
    }
    maniGraphListen(u.graph, binding->clips, instance);
    if (eventCountOut != NULL)
    {
        *eventCountOut = u.count < UINT32_MAX ? (uint32_t)u.count : UINT32_MAX;
    }
    if (rootMotionOut != NULL)
    {
        *rootMotionOut = maniRootMotionSumResult(&u.root);
    }
    return u.failed != 0 ? mani_errorFormat : mani_success;
}

maniResult maniUpdateGraph(maniGraphInstance* instance, float delta, maniGraphEvent* events,
                           uint32_t capacity, uint32_t* eventCountOut, maniRootMove* rootMotionOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result =
        UpdateGraphWork(instance, delta, events, capacity, eventCountOut, rootMotionOut);
    maniFloatEnvLeave(&env);
    return result;
}
