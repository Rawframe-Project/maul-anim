// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's nodes checked on open: records laid out in turn,
// inputs before their reader and in other slots, what each gives (poses
// or differences) as its inputs make it, each kind's data in range, and
// each node's time factor bounding its inputs'.

#include "graph_nodes.h"

#include "bytes.h"
#include "graph_body.h"
#include "graph_chain.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_learned.h"
#include "graph_machine_file.h"
#include "graph_match.h"
#include "graph_shot.h"
#include "graph_spring.h"
#include "graph_turn.h"
#include "invariant.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/ik.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static const maniFileResult kOk = {mani_success, 0, 0};

// A node's record and where it is.
typedef struct Node
{
    const uint8_t* entry;
    uint64_t at;
    uint32_t kind;
    uint32_t flags;
    uint32_t first;
    uint32_t count;
    uint32_t data;
    uint32_t words;
    uint32_t slot;
} Node;

static Node LoadNode(const maniGraph* g, uint32_t p, uint64_t nodesAt)
{
    const uint8_t* e = g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
    return (Node){e,
                  nodesAt + (uint64_t)p * MANI_GRAPH_NODE_SIZE,
                  maniLoadU16(e),
                  maniLoadU16(e + 2),
                  maniLoadU32(e + 4),
                  maniLoadU32(e + 8),
                  maniLoadU32(e + 12),
                  maniLoadU32(e + 16),
                  maniLoadU32(e + 20)};
}

// A data word of a node whose record RecordValid took: within the data.
static uint32_t Word(const maniGraph* g, uint32_t at)
{
    MANI_ASSERT(at < g->dataWords);
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static uint32_t ParameterType(const maniGraph* g, uint32_t parameter)
{
    MANI_ASSERT(parameter < g->parameterCount);
    return maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE);
}

// A scalar at data word at: a float parameter with a literal of 0 bits,
// or a finite literal from low to high.
static bool ScalarValid(const maniGraph* g, uint32_t at, float low, float high)
{
    uint32_t parameter = Word(g, at);
    uint32_t bits = Word(g, at + 1);
    if (parameter != MANI_NO_PARAMETER)
    {
        return parameter < g->parameterCount &&
               ParameterType(g, parameter) == maniGraphParameterFloat && bits == 0;
    }
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return isfinite(value) && value >= low && value <= high;
}

// Scalars from data word at, count of them, each from low to high; the
// first wrong one's word, or UINT32_MAX.
static uint32_t ScalarsValid(const maniGraph* g, uint32_t at, uint32_t count, float low, float high)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!ScalarValid(g, at + 2 * k, low, high))
        {
            return at + 2 * k;
        }
    }
    return UINT32_MAX;
}

static uint32_t ClipData(const maniGraph* g, const Node* n)
{
    uint32_t d = n->data;
    uint32_t clip = Word(g, d);
    uint32_t group = Word(g, d + 1);
    bool grouped = group < g->syncGroupCount;
    if (clip >= g->clipCount)
    {
        return d;
    }
    if (!grouped && group != MANI_NO_SYNC_GROUP)
    {
        return d + 1;
    }
    if (Word(g, d + 2) > 2u)
    {
        return d + 2;
    }
    uint32_t flags = Word(g, d + 3);
    if (flags > 1u || (!grouped && flags != 0))
    {
        return d + 3;
    }
    return ScalarsValid(g, d + 4, 1, -HUGE_VALF, HUGE_VALF);
}

static float WordFloat(const maniGraph* g, uint32_t at)
{
    uint32_t bits = Word(g, at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static float EntryFloat(const uint8_t* entry, uint32_t at)
{
    uint32_t bits = maniLoadU32(entry + at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// A time scale: a finite literal, or a float parameter with bounds.
static uint32_t ScaleValid(const maniGraph* g, uint32_t d)
{
    uint32_t parameter = Word(g, d);
    bool bounded =
        parameter == MANI_NO_PARAMETER ||
        (parameter < g->parameterCount &&
         (maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE + 4) &
          MANI_GRAPH_BOUNDED) != 0);
    return bounded ? ScalarsValid(g, d, 1, -HUGE_VALF, HUGE_VALF) : d;
}

// The largest size a valid time scale at data word d takes: its literal's,
// or the larger of its parameter's bounds.
static float ScaleSize(const maniGraph* g, uint32_t d)
{
    uint32_t parameter = Word(g, d);
    if (parameter == MANI_NO_PARAMETER)
    {
        return fabsf(WordFloat(g, d + 1));
    }
    // ScaleValid took only a parameter within the parameters.
    MANI_ASSERT(parameter < g->parameterCount);
    const uint8_t* e = g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE;
    float low = fabsf(EntryFloat(e, 40));
    float high = fabsf(EntryFloat(e, 44));
    return low > high ? low : high;
}

// Whether the position's words from k on are literal zeros.
static uint32_t ZerosFrom(const maniGraph* g, uint32_t d, uint32_t k)
{
    for (; k < MANI_GRAPH_POSITION_WORDS; ++k)
    {
        if (Word(g, d + k) != (k % 2 == 1 ? MANI_NO_PARAMETER : 0u))
        {
            return d + k;
        }
    }
    return UINT32_MAX;
}

// A blend space's position: a vec2 parameter with literal zeros after
// it, or two scalars (a 1D space's y a literal zero).
static uint32_t PositionValid(const maniGraph* g, uint32_t d, bool line)
{
    uint32_t vector = Word(g, d);
    if (vector == MANI_NO_PARAMETER)
    {
        uint32_t wrong = ScalarsValid(g, d + 1, line ? 1u : 2u, -HUGE_VALF, HUGE_VALF);
        return wrong == UINT32_MAX && line ? ZerosFrom(g, d, 3) : wrong;
    }
    if (vector >= g->parameterCount || ParameterType(g, vector) != maniGraphParameterVec2)
    {
        return d;
    }
    return ZerosFrom(g, d, 1);
}

// A 1D space: its period finite and not negative, its points finite, its
// order every input once by strictly ascending point, within a period
// when it wraps.
static uint32_t LineValid(const maniGraph* g, uint32_t d, uint32_t n)
{
    float period = WordFloat(g, d + MANI_GRAPH_POSITION_WORDS);
    if (!isfinite(period) || period < 0.0f)
    {
        return d + MANI_GRAPH_POSITION_WORDS;
    }
    uint32_t points = d + MANI_GRAPH_SPACE_WORDS;
    for (uint32_t k = 0; k < n; ++k)
    {
        if (!isfinite(WordFloat(g, points + k)))
        {
            return points + k;
        }
    }
    uint32_t order = points + n;
    for (uint32_t r = 0; r < n; ++r)
    {
        uint32_t k = Word(g, order + r);
        bool ascending = k < n && (r == 0 || WordFloat(g, points + Word(g, order + r - 1)) <
                                                 WordFloat(g, points + k));
        if (!ascending)
        {
            return order + r;
        }
    }
    float span =
        WordFloat(g, points + Word(g, order + n - 1)) - WordFloat(g, points + Word(g, order));
    return period > 0.0f && !(span < period) ? d + MANI_GRAPH_POSITION_WORDS : UINT32_MAX;
}

// Points in a plane from word at: finite, no two equal.
static uint32_t PairsValid(const maniGraph* g, uint32_t points, uint32_t n)
{
    for (uint32_t k = 0; k < n; ++k)
    {
        float x = WordFloat(g, points + 2 * k);
        float y = WordFloat(g, points + 2 * k + 1);
        bool repeated = false;
        for (uint32_t j = 0; j < k && !repeated; ++j)
        {
            repeated = WordFloat(g, points + 2 * j) == x && WordFloat(g, points + 2 * j + 1) == y;
        }
        if (!isfinite(x) || !isfinite(y) || repeated)
        {
            return points + 2 * k;
        }
    }
    return UINT32_MAX;
}

// A banded space: its flag 0 or 1, then its points.
static uint32_t BandsValid(const maniGraph* g, uint32_t d, uint32_t n)
{
    if (Word(g, d + MANI_GRAPH_POSITION_WORDS) > 1u)
    {
        return d + MANI_GRAPH_POSITION_WORDS;
    }
    return PairsValid(g, d + MANI_GRAPH_SPACE_WORDS, n);
}

// A triangulated space: its count 1 to the most, its points, each
// counted triangle's corners distinct inputs, the reserved ones zero.
static uint32_t TrianglesValid(const maniGraph* g, uint32_t d, uint32_t n)
{
    uint32_t count = Word(g, d + MANI_GRAPH_POSITION_WORDS);
    if (count == 0 || count > MANI_GRAPH_MAX_TRIANGLES(n))
    {
        return d + MANI_GRAPH_POSITION_WORDS;
    }
    uint32_t wrong = PairsValid(g, d + MANI_GRAPH_SPACE_WORDS, n);
    uint32_t corners = d + MANI_GRAPH_SPACE_WORDS + 2 * n;
    for (uint32_t t = 0; t < MANI_GRAPH_MAX_TRIANGLES(n) && wrong == UINT32_MAX; ++t)
    {
        uint32_t a = Word(g, corners + 3 * t);
        uint32_t b = Word(g, corners + 3 * t + 1);
        uint32_t c = Word(g, corners + 3 * t + 2);
        bool ok = t < count ? a < n && b < n && c < n && a != b && b != c && a != c
                            : a == 0 && b == 0 && c == 0;
        wrong = ok ? UINT32_MAX : corners + 3 * t;
    }
    return wrong;
}

static uint32_t SpaceValid(const maniGraph* g, const Node* n)
{
    bool line = n->kind == maniGraphNodeBlendSpace1D;
    uint32_t wrong = PositionValid(g, n->data, line);
    if (wrong != UINT32_MAX)
    {
        return wrong;
    }
    if (line)
    {
        return LineValid(g, n->data, n->count);
    }
    return n->kind == maniGraphNodeBlendSpaceBands ? BandsValid(g, n->data, n->count)
                                                   : TrianglesValid(g, n->data, n->count);
}

// The first wrong data word of a node, or UINT32_MAX.
// A vec3 parameter at data word at, or none when optional.
static bool VectorValid(const maniGraph* g, uint32_t at, bool optional)
{
    uint32_t parameter = Word(g, at);
    return (optional && parameter == MANI_NO_PARAMETER) ||
           (parameter < g->parameterCount && ParameterType(g, parameter) == maniGraphParameterVec3);
}

// The first of count binary32 words from at not finite, or UINT32_MAX.
static uint32_t FiniteFrom(const maniGraph* g, uint32_t at, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!isfinite(WordFloat(g, at + k)))
        {
            return at + k;
        }
    }
    return UINT32_MAX;
}

// A two-bone node's words: bones of the skeleton, a vec3 goal, a vec3
// pole or none, a finite hinge, a finite twist, soften and weight in
// [0, 1]. Whether each bone is an ancestor of the next waits for the
// skeleton: a solve refusing them passes the input.
static uint32_t TwoBoneValid(const maniGraph* g, uint32_t d)
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (Word(g, d + k) >= g->skeletonBoneCount)
        {
            return d + k;
        }
    }
    if (!VectorValid(g, d + maniGraphTwoBoneGoal, false))
    {
        return d + maniGraphTwoBoneGoal;
    }
    if (!VectorValid(g, d + maniGraphTwoBonePole, true))
    {
        return d + maniGraphTwoBonePole;
    }
    uint32_t wrong = FiniteFrom(g, d + maniGraphTwoBoneHinge, 3);
    wrong = wrong != UINT32_MAX
                ? wrong
                : ScalarsValid(g, d + maniGraphTwoBoneTwist, 1, -HUGE_VALF, HUGE_VALF);
    return wrong != UINT32_MAX ? wrong : ScalarsValid(g, d + maniGraphTwoBoneSoften, 2, 0.0f, 1.0f);
}

// An aim node's bones and shares: count bones of the skeleton and shares
// at least 0, finite and not all 0 (their sum finite), the words past
// the count 0.
static uint32_t AimChainValid(const maniGraph* g, uint32_t d, uint32_t count)
{
    for (uint32_t k = 0; k < MANI_MAX_AIM_BONES; ++k)
    {
        uint32_t bone = Word(g, d + maniGraphAimBones + k);
        if (k < count ? bone >= g->skeletonBoneCount : bone != 0)
        {
            return d + maniGraphAimBones + k;
        }
    }
    float total = 0.0f;
    for (uint32_t k = 0; k < MANI_MAX_AIM_BONES; ++k)
    {
        float share = WordFloat(g, d + maniGraphAimShares + k);
        bool ok =
            k < count ? share >= 0.0f && isfinite(share) : Word(g, d + maniGraphAimShares + k) == 0;
        if (!ok)
        {
            return d + maniGraphAimShares + k;
        }
        total += share;
    }
    return total > 0.0f && isfinite(total) ? UINT32_MAX : d + maniGraphAimShares;
}

// An aim node's words: a count from 1 to MANI_MAX_AIM_BONES, a vec3 goal,
// a vec3 pole or none, its bones and shares, a forward of length 1 within
// a thousandth, offset and up finite, a finite twist, a cone in [0,
// MANI_AIM_MAX_CONE] and a weight in [0, 1].
static uint32_t AimValid(const maniGraph* g, uint32_t d)
{
    uint32_t count = Word(g, d);
    if (count == 0 || count > MANI_MAX_AIM_BONES)
    {
        return d;
    }
    if (!VectorValid(g, d + maniGraphAimGoal, false))
    {
        return d + maniGraphAimGoal;
    }
    if (!VectorValid(g, d + maniGraphAimPole, true))
    {
        return d + maniGraphAimPole;
    }
    uint32_t wrong = AimChainValid(g, d, count);
    wrong = wrong != UINT32_MAX ? wrong : FiniteFrom(g, d + maniGraphAimForward, 9);
    if (wrong != UINT32_MAX)
    {
        return wrong;
    }
    uint32_t f = d + maniGraphAimForward;
    maniVec3 forward = {WordFloat(g, f), WordFloat(g, f + 1), WordFloat(g, f + 2)};
    if (!(fabsf(maniVec3Length(forward) - 1.0f) <= 1e-3f))
    {
        return f;
    }
    wrong = ScalarsValid(g, d + maniGraphAimTwist, 1, -HUGE_VALF, HUGE_VALF);
    wrong = wrong != UINT32_MAX ? wrong
                                : ScalarsValid(g, d + maniGraphAimCone, 1, 0.0f, MANI_AIM_MAX_CONE);
    return wrong != UINT32_MAX ? wrong : ScalarsValid(g, d + maniGraphAimWeight, 1, 0.0f, 1.0f);
}

static uint32_t DataValid(const maniGraph* g, const Node* n)
{
    uint32_t d = n->data;
    uint32_t type = 0;
    switch (n->kind)
    {
    case maniGraphNodeClip:
        return ClipData(g, n);
    case maniGraphNodeBlend:
        return Word(g, d) > 1u
                   ? d
                   : ScalarsValid(g, d + 1, n->count, 0.0f, Word(g, d) != 0 ? 1.0f : HUGE_VALF);
    case maniGraphNodeMask:
        return Word(g, d) < g->maskCount ? UINT32_MAX : d;
    case maniGraphNodeAdditive:
        return ScalarsValid(g, d, n->count - 1, 0.0f, 1.0f);
    case maniGraphNodeSelect:
        type = Word(g, d) < g->parameterCount ? ParameterType(g, Word(g, d)) : UINT32_MAX;
        return type == maniGraphParameterInt || type == maniGraphParameterBool ? UINT32_MAX : d;
    case maniGraphNodeTimeScale:
        return ScaleValid(g, d);
    case maniGraphNodeMirror:
        return UINT32_MAX;
    case maniGraphNodePose:
        type = Word(g, d) < g->parameterCount ? ParameterType(g, Word(g, d)) : UINT32_MAX;
        return type == maniGraphParameterPose ? UINT32_MAX : d;
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceTriangles:
    case maniGraphNodeBlendSpaceBands:
        return SpaceValid(g, n);
    case maniGraphNodeTwoBone:
        return TwoBoneValid(g, d);
    case maniGraphNodeAim:
        return AimValid(g, d);
    case maniGraphNodeCcd:
    case maniGraphNodeFabrik:
        return maniGraphChainValid(g, d, n->kind == maniGraphNodeFabrik);
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        return maniGraphBodyValid(g, d, n->kind == maniGraphNodeBodySdls);
    default:
        type = Word(g, d) < g->parameterCount ? ParameterType(g, Word(g, d)) : UINT32_MAX;
        return type == maniGraphParameterTrigger ? ScalarsValid(g, d + 1, 1, -HUGE_VALF, HUGE_VALF)
                                                 : d;
    }
}

// An input of a node whose record RecordValid took: within the inputs.
static uint32_t Input(const maniGraph* g, uint32_t at)
{
    MANI_ASSERT(at < g->inputCount);
    return maniLoadU32(g->inputs + 4 * (size_t)at);
}

// A node's difference flag.
static uint32_t Difference(const maniGraph* g, uint32_t p)
{
    MANI_ASSERT(p < g->nodeCount);
    return maniLoadU16(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 2) & MANI_GRAPH_DIFFERENCE;
}

// Whether a node gives differences as its inputs (or clip) make it.
static bool DifferenceValid(const maniGraph* g, const Node* n)
{
    uint32_t own = n->flags & MANI_GRAPH_DIFFERENCE;
    if (n->kind == maniGraphNodeMatch || n->kind == maniGraphNodeLearned ||
        n->kind == maniGraphNodePose)
    {
        return own == 0;
    }
    // The kinds that take and give poses only: springs, IK, mirrors and
    // turn limits.
    if (n->kind == maniGraphNodeSpring || n->kind == maniGraphNodeMirror ||
        n->kind == maniGraphNodeTurnLimit ||
        ((uint32_t)n->kind >= maniGraphNodeTwoBone && (uint32_t)n->kind <= maniGraphNodeBodySdls))
    {
        return own == 0 && Difference(g, Input(g, n->first)) == 0;
    }
    if (n->kind == maniGraphNodeClip)
    {
        // ClipData took only a clip slot within the clips.
        MANI_ASSERT(Word(g, n->data) < g->clipCount);
        const uint8_t* clip = g->clips + (size_t)Word(g, n->data) * MANI_GRAPH_CLIP_SIZE;
        return own == maniLoadU32(clip + 24);
    }
    bool additive = n->kind == maniGraphNodeAdditive;
    uint32_t first = Difference(g, Input(g, n->first));
    if (additive ? first != 0 || own != 0 : own != first)
    {
        return false;
    }
    for (uint32_t k = 1; k < n->count; ++k)
    {
        uint32_t d = Difference(g, Input(g, n->first + k));
        if (additive ? d == 0 : d != first)
        {
            return false;
        }
    }
    return true;
}

// Whether each input is mirrored as the node is, a mirror node's the
// other way.
static bool MirroredValid(const maniGraph* g, const Node* n)
{
    uint32_t want = (n->flags & MANI_GRAPH_MIRRORED) ^
                    (n->kind == maniGraphNodeMirror ? MANI_GRAPH_MIRRORED : 0u);
    for (uint32_t k = 0; k < n->count; ++k)
    {
        uint32_t u = Input(g, n->first + k);
        uint32_t flags = maniLoadU16(g->nodes + (size_t)u * MANI_GRAPH_NODE_SIZE + 2);
        if ((flags & MANI_GRAPH_MIRRORED) != want)
        {
            return false;
        }
    }
    return true;
}

static bool KindValid(const maniGraph* g, const Node* n)
{
    return DifferenceValid(g, n) && MirroredValid(g, n);
}

static float FactorOf(const maniGraph* g, uint32_t p)
{
    MANI_ASSERT(p < g->nodeCount);
    uint32_t bits = maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 28);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// The node's time factor size: from 0 to MANI_MAX_GRAPH_TIME_FACTOR, 1 at
// the output, and each input's at least the node's times its scale's.
static maniFileResult FactorValid(const maniGraph* g, const Node* n, bool last, uint64_t nodesAt)
{
    uint32_t bits = maniLoadU32(n->entry + 28);
    float factor = 0.0f;
    memcpy(&factor, &bits, sizeof factor);
    if (bits >= 0x80000000u || !(factor <= MANI_MAX_GRAPH_TIME_FACTOR) || (last && factor != 1.0f))
    {
        return Fail(n->at + 28);
    }
    float need = n->kind == maniGraphNodeTimeScale ? factor * ScaleSize(g, n->data) : factor;
    for (uint32_t k = 0; k < n->count; ++k)
    {
        uint32_t u = Input(g, n->first + k);
        if (!(FactorOf(g, u) >= need))
        {
            return Fail(nodesAt + (uint64_t)u * MANI_GRAPH_NODE_SIZE + 28);
        }
    }
    return kOk;
}

// The record's fields, laid out after the nodes before it (inputs and
// data at the running counts).
static maniFileResult RecordValid(const maniGraph* g, const Node* n, uint32_t inputAt,
                                  uint32_t dataAt)
{
    uint32_t low = 0;
    uint32_t high = maniGraphInputRange(n->kind, &low);
    if (n->kind != maniGraphNodeClip && n->kind != maniGraphNodeMatch &&
        n->kind != maniGraphNodeLearned && n->kind != maniGraphNodePose && high == 0)
    {
        return Fail(n->at);
    }
    if (n->flags > MANI_GRAPH_NODE_FLAGS)
    {
        return Fail(n->at + 2);
    }
    if (n->kind == maniGraphNodeMirror && g->mirrorMapIdentity == 0)
    {
        return Fail(n->at);
    }
    if (n->first != inputAt)
    {
        return Fail(n->at + 4);
    }
    if (n->count < low || n->count > high || n->count > g->inputCount - inputAt)
    {
        return Fail(n->at + 8);
    }
    if (n->data != dataAt)
    {
        return Fail(n->at + 12);
    }
    uint32_t available = g->dataWords - dataAt;
    uint32_t words = maniGraphDataWords(n->kind, n->count);
    words = n->kind == maniGraphNodeMachine
                ? maniGraphMachineDataWords(g, n->data, n->count, available)
                : (n->kind == maniGraphNodeOneShot ? maniGraphShotDataWords(g, n->data, available)
                                                   : words);
    words = n->kind == maniGraphNodeMatch ? maniGraphMatchDataWords(g, n->data, available) : words;
    words =
        n->kind == maniGraphNodeLearned ? maniGraphLearnedDataWords(g, n->data, available) : words;
    words = n->kind == maniGraphNodeCcd || n->kind == maniGraphNodeFabrik
                ? maniGraphChainDataWords(g, n->data, available)
                : words;
    words = n->kind == maniGraphNodeBodyXpbd || n->kind == maniGraphNodeBodySdls
                ? maniGraphBodyDataWords(g, n->data, available)
                : words;
    if (n->words != words || n->words > g->dataWords - dataAt)
    {
        return Fail(n->at + 16);
    }
    if (n->slot >= g->slotCount)
    {
        return Fail(n->at + 20);
    }
    if (maniLoadU32(n->entry + 24) >= g->nodeCount)
    {
        return Fail(n->at + 24);
    }
    return kOk;
}

// Every input before the node, in another slot.
static maniFileResult InputsValid(const maniGraph* g, const Node* n, uint32_t p, uint64_t inputsAt)
{
    for (uint32_t k = 0; k < n->count; ++k)
    {
        uint32_t u = Input(g, n->first + k);
        uint64_t at = inputsAt + 4 * (uint64_t)(n->first + k);
        if (u >= p)
        {
            return Fail(at);
        }
        if (maniLoadU32(g->nodes + (size_t)u * MANI_GRAPH_NODE_SIZE + 20) == n->slot)
        {
            return Fail(n->at + 20);
        }
    }
    return kOk;
}

// The first wrong data word of node p, whose block (if it keeps one)
// starts at blockAt, or UINT32_MAX.
static uint32_t WordsValid(const maniGraph* g, const Node* n, uint32_t p, uint32_t blockAt)
{
    switch (n->kind)
    {
    case maniGraphNodeMachine:
        return maniGraphMachineValid(g, p, n->data, n->count, blockAt);
    case maniGraphNodeOneShot:
        return maniGraphShotValid(g, p, n->data, blockAt);
    case maniGraphNodeInertialize:
        return maniGraphInertiaValid(g, n->data, blockAt);
    case maniGraphNodeMatch:
        return maniGraphMatchValid(g, n->data, blockAt);
    case maniGraphNodeLearned:
        return maniGraphLearnedValid(g, n->data, blockAt);
    case maniGraphNodeSpring:
        return maniGraphSpringValid(g, n->data, blockAt);
    case maniGraphNodeTurnLimit:
        return maniGraphTurnValid(g, n->data, blockAt);
    default:
        return DataValid(g, n);
    }
}

// The words of the block node n keeps in an instance.
static uint32_t BlockWords(const maniGraph* g, const Node* n)
{
    switch (n->kind)
    {
    case maniGraphNodeMachine:
        return MANI_GRAPH_BLOCK_SIZE(n->count, Word(g, n->data + 2), Word(g, n->data + 4));
    case maniGraphNodeOneShot:
        return MANI_GRAPH_SHOT_BLOCK_WORDS;
    case maniGraphNodeInertialize:
        return MANI_GRAPH_INERTIA_BLOCK(g->skeletonBoneCount,
                                        Word(g, n->data + maniGraphInertiaMethod));
    case maniGraphNodeMatch:
        return MANI_GRAPH_MATCH_BLOCK_WORDS;
    case maniGraphNodeLearned:
        return maniGraphLearnedBlockWords(g, n->data);
    case maniGraphNodeSpring:
        return maniGraphSpringBlockWords(g, n->data);
    case maniGraphNodeTurnLimit:
        return maniGraphTurnBlockWords(g->skeletonBoneCount);
    default:
        return 0;
    }
}

maniFileResult maniCheckGraphNodes(const maniGraph* g, uint64_t nodesAt, uint64_t inputsAt,
                                   uint64_t dataAt, uint32_t* machineWordsOut)
{
    uint32_t inputAt = 0;
    uint32_t wordAt = 0;
    uint32_t blockAt = 0;
    for (uint32_t p = 0; p < g->nodeCount; ++p)
    {
        Node n = LoadNode(g, p, nodesAt);
        maniFileResult r = RecordValid(g, &n, inputAt, wordAt);
        r = r.result == mani_success ? InputsValid(g, &n, p, inputsAt) : r;
        if (r.result != mani_success)
        {
            return r;
        }
        MANI_ASSERT(n.first == inputAt && n.count <= g->inputCount - inputAt && n.data == wordAt &&
                    n.words <= g->dataWords - wordAt);
        uint32_t wrong = WordsValid(g, &n, p, blockAt);
        if (wrong != UINT32_MAX)
        {
            return Fail(dataAt + 4 * (uint64_t)wrong);
        }
        bool last = p + 1 == g->nodeCount;
        if (!KindValid(g, &n) || (last && n.flags != 0))
        {
            return Fail(n.at + 2);
        }
        r = FactorValid(g, &n, last, nodesAt);
        if (r.result != mani_success)
        {
            return r;
        }
        inputAt += n.count;
        wordAt += n.words;
        uint32_t block = BlockWords(g, &n);
        if (block > UINT32_MAX - blockAt)
        {
            return Fail(dataAt + 4 * (uint64_t)n.data);
        }
        blockAt += block;
    }
    *machineWordsOut = blockAt;
    if (inputAt != g->inputCount)
    {
        return Fail(inputsAt);
    }
    return wordAt == g->dataWords ? kOk : Fail(dataAt);
}
