// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Sync groups in an update: each group's leader, the first declared or
// else the member weighed most, and how it moves; then each member's
// motion, by phase or by markers, as far as the leader's.

#include "graph_sync.h"

#include "bytes.h"
#include "graph_emit.h"
#include "graph_file.h"
#include "graph_instance.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef maniGraphUpdate Update;
typedef maniGraphMotion Motion;

static maniGraphNodeState* Node(const Update* u, uint32_t p)
{
    return maniGraphUpdateNode(u, p);
}

static uint32_t Word(const Update* u, uint32_t at)
{
    return maniGraphUpdateWord(u, at);
}

static float Scalar(const Update* u, uint32_t at)
{
    return maniGraphUpdateScalar(u, at);
}

// A node's own motion: its playhead by its speed and time factor.
static Motion OwnMotion(const Update* u, const maniGraphNodeState* s, const maniClip* clip,
                        uint32_t data)
{
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        double scale = (s->flags & MANI_GRAPH_PHASE) != 0 ? (double)clip->duration : 1.0;
        return (Motion){maniGraphClipPlace(clip, (double)s->seekTo * scale), 0.0, true, false};
    }
    return (Motion){s->time, u->delta * (double)Scalar(u, data + 4) * (double)s->factor, false,
                    true};
}

static maniGraphGroupState* Group(const Update* u, uint32_t group)
{
    return &u->instance->cells[u->graph->parameterCount + u->graph->nodeCount + group].group;
}

// The next clip node of group g from node from on, or UINT32_MAX.
static uint32_t NextMember(const Update* u, uint32_t g, uint32_t from)
{
    for (uint32_t p = from; p < u->graph->nodeCount; ++p)
    {
        const uint8_t* record = maniGraphUpdateRecord(u, p);
        if (maniLoadU16(record) == maniGraphNodeClip && maniGraphUpdateGroupOf(u, record) == g)
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
        const maniClip* clip =
            u->binding->clips[Word(u, maniLoadU32(maniGraphUpdateRecord(u, p) + 12))];
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
        maniClip clip = maniGraphUpdateClip(u, maniLoadU32(maniGraphUpdateRecord(u, p) + 12));
        if (!maniGraphClipRepeats(&clip))
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
    uint32_t data = maniLoadU32(maniGraphUpdateRecord(u, group->leader) + 12);
    maniClip clip = maniGraphUpdateClip(u, data);
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
        const uint8_t* record = maniGraphUpdateRecord(u, p);
        uint32_t g = maniLoadU16(record) == maniGraphNodeClip ? maniGraphUpdateGroupOf(u, record)
                                                              : UINT32_MAX;
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
        double t = maniGraphClipPlace(clip, MarkerTime(clip, i) + (backward ? -offset : offset));
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
    maniClip leader = maniGraphUpdateClip(u, maniLoadU32(maniGraphUpdateRecord(u, lead) + 12));
    bool backward = group->delta < 0.0;
    Between start = PositionAt(u, g, lead, &leader, group->phase, backward);
    Between end = PositionAt(u, g, lead, &leader,
                             maniGraphClipPlace(&leader, group->phase + group->delta), backward);
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
    return (Motion){maniGraphClipPlace(clip, group->phase * d), group->delta * d, seeking,
                    all && !seeking};
}

maniGraphMotion maniGraphClipMotion(const maniGraphUpdate* u, uint32_t p, uint32_t group,
                                    const maniClip* clip, uint32_t data)
{
    maniGraphNodeState* s = Node(u, p);
    if (group < u->graph->syncGroupCount && Group(u, group)->leader != p &&
        Group(u, group)->leader != UINT32_MAX)
    {
        return Follow(u, group, p, clip, s->time);
    }
    return OwnMotion(u, s, clip, data);
}
