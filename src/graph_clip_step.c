// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clip node's step in an update: its playhead moved by its own speed
// and time factor or set by a seek, or as its sync group's leader moves,
// by phase or by markers; its events listed and its root motion added.
// A group's leader is the first member declared one, else the member
// weighed most.

#include "graph_clip_step.h"

#include "bytes.h"
#include "graph_emit.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "invariant.h"
#include "retarget_motion.h"
#include "root_motion.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/rootmotion.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef maniGraphUpdate Update;

static maniGraphNodeState* Node(const Update* u, uint32_t p)
{
    MANI_ASSERT(p < u->graph->nodeCount);
    return maniGraphUpdateNode(u, p);
}

static const uint8_t* Record(const Update* u, uint32_t p)
{
    return u->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
}

static uint32_t Word(const Update* u, uint32_t at)
{
    return maniGraphUpdateWord(u, at);
}

static float Scalar(const Update* u, uint32_t at)
{
    return maniGraphUpdateScalar(u, at);
}

// Whether a parent reached a node this update.
static bool Reached(const maniGraphNodeState* s)
{
    return (s->flags & MANI_GRAPH_REACHED) != 0;
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
    MANI_ASSERT(Word(u, data) < u->graph->clipCount);
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
    // A step that is not finite moves nothing.
    double step = u->delta * (double)Scalar(u, data + 4) * (double)s->factor;
    return (Motion){s->time, isfinite(step) ? step : 0.0, false, true};
}

static maniGraphGroupState* Group(const Update* u, uint32_t group)
{
    MANI_ASSERT(group < u->graph->syncGroupCount);
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
    // The leader is a clip node of the group.
    MANI_ASSERT(group->leader < u->graph->nodeCount &&
                maniLoadU16(Record(u, group->leader)) == maniGraphNodeClip &&
                GroupOf(u, Record(u, group->leader)) == g);
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
void maniGraphLead(const maniGraphUpdate* u)
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
    // A group follows by markers only when every member's clip has one of
    // the leader's markers, so a common marker is always found.
    MANI_ASSERT(previous < clip->markerCount);
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
    // At most two loops of markers remain to walk: the reduction above
    // leaves pass at most 2 perLoop, and the walk's count is bounded by it.
    double bounded = pass < 2.0 * perLoop ? pass : 2.0 * perLoop;
    uint64_t steps = (uint64_t)(bounded > 0.0 ? bounded : 0.0);
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
void maniGraphAdvanceClip(maniGraphUpdate* u, uint32_t p)
{
    const uint8_t* record = Record(u, p);
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
    // Every step is finite, so the playhead lands within the clip.
    MANI_ASSERT(s->time >= 0.0 && s->time <= (double)clip.duration);
    if (s->rootWeight > 0.0f)
    {
        maniRootMotionSumAdd(&u->root, &move, (double)s->rootWeight);
    }
}
