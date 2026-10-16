// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's nodes checked on open (mani-0023): records laid out in turn,
// inputs before their reader and in other slots, what each gives (poses
// or differences) as its inputs make it, and each kind's data in range.

#include "graph_nodes.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"

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

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static uint32_t ParameterType(const maniGraph* g, uint32_t parameter)
{
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
        return ScalarsValid(g, d, 1, -HUGE_VALF, HUGE_VALF);
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceTriangles:
    case maniGraphNodeBlendSpaceBands:
        return SpaceValid(g, n);
    default:
        type = Word(g, d) < g->parameterCount ? ParameterType(g, Word(g, d)) : UINT32_MAX;
        return type == maniGraphParameterTrigger ? ScalarsValid(g, d + 1, 1, -HUGE_VALF, HUGE_VALF)
                                                 : d;
    }
}

static uint32_t Input(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->inputs + 4 * (size_t)at);
}

static uint32_t NodeFlags(const maniGraph* g, uint32_t p)
{
    return maniLoadU16(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 2);
}

// Whether a node gives differences as its inputs (or clip) make it.
static bool KindValid(const maniGraph* g, const Node* n)
{
    if (n->kind == maniGraphNodeClip)
    {
        const uint8_t* clip = g->clips + (size_t)Word(g, n->data) * MANI_GRAPH_CLIP_SIZE;
        return n->flags == maniLoadU32(clip + 24);
    }
    bool additive = n->kind == maniGraphNodeAdditive;
    uint32_t first = NodeFlags(g, Input(g, n->first));
    if (additive ? first != 0 || n->flags != 0 : n->flags != first)
    {
        return false;
    }
    for (uint32_t k = 1; k < n->count; ++k)
    {
        uint32_t d = NodeFlags(g, Input(g, n->first + k));
        if (additive ? d == 0 : d != first)
        {
            return false;
        }
    }
    return true;
}

// The record's fields, laid out after the nodes before it (inputs and
// data at the running counts).
static maniFileResult RecordValid(const maniGraph* g, const Node* n, uint32_t inputAt,
                                  uint32_t dataAt)
{
    uint32_t low = 0;
    uint32_t high = maniGraphInputRange(n->kind, &low);
    if (n->kind != maniGraphNodeClip && high == 0)
    {
        return Fail(n->at);
    }
    if (n->flags > MANI_GRAPH_DIFFERENCE)
    {
        return Fail(n->at + 2);
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
    if (n->words != maniGraphDataWords(n->kind, n->count) || n->words > g->dataWords - dataAt)
    {
        return Fail(n->at + 16);
    }
    if (n->slot >= g->slotCount)
    {
        return Fail(n->at + 20);
    }
    if (maniLoadU64(n->entry + 24) != 0)
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

maniFileResult maniCheckGraphNodes(const maniGraph* g, uint64_t nodesAt, uint64_t inputsAt,
                                   uint64_t dataAt)
{
    uint32_t inputAt = 0;
    uint32_t wordAt = 0;
    for (uint32_t p = 0; p < g->nodeCount; ++p)
    {
        Node n = LoadNode(g, p, nodesAt);
        maniFileResult r = RecordValid(g, &n, inputAt, wordAt);
        r = r.result == mani_success ? InputsValid(g, &n, p, inputsAt) : r;
        if (r.result != mani_success)
        {
            return r;
        }
        uint32_t wrong = DataValid(g, &n);
        if (wrong != UINT32_MAX)
        {
            return Fail(dataAt + 4 * (uint64_t)wrong);
        }
        bool last = p + 1 == g->nodeCount;
        if (!KindValid(g, &n) || (last && n.flags != 0))
        {
            return Fail(n.at + 2);
        }
        inputAt += n.count;
        wordAt += n.words;
    }
    if (inputAt != g->inputCount)
    {
        return Fail(inputsAt);
    }
    return wordAt == g->dataWords ? kOk : Fail(dataAt);
}
