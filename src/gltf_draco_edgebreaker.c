// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Edgebreaker decoding in Google's order (research 59): the counts, the
// topology split events, the traversal's symbols (bits, or for valence
// six contexts of symbols chosen by the active vertex's valence), the
// start faces' and every attribute data's seam decisions (rABS), then
// the symbols rebuild the corner table face by face from an active
// corner stack. Where Google's decoder would index with an invalid
// vertex or corner, or walk a fan without end, the stream is invalid.

#include "gltf_draco_edgebreaker.h"

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NONE MANI_DRACO_NONE

enum
{
    SymbolC = 0,
    SymbolS = 1,
    SymbolL = 3,
    SymbolR = 5,
    SymbolE = 7,
    SymbolInvalid = 8,
};

enum
{
    Standard = 0,
    Valence = 2,
    // Valences 2 to 7 make the contexts.
    Contexts = 6,
};

typedef struct Split
{
    uint32_t source;
    uint32_t split;
    uint8_t edge;
} Split;

typedef struct State
{
    maniDracoReader* r;
    maniArena* arena;
    maniDracoTable* t;
    uint8_t* hole;
    uint32_t vertexMax;
    uint32_t symbols;
    uint32_t faces;
    uint32_t face;
    uint8_t method;
    // Standard: the symbols' bits. Valence: per vertex valences, the
    // contexts' symbols and how many of each remain, the active one.
    maniDracoBits bits;
    uint32_t* valences;
    uint32_t* contexts[Contexts];
    uint32_t remaining[Contexts];
    int32_t active;
    uint32_t last;
    maniDracoRabs start;
    maniDracoRabs* seams;
    uint32_t dataCount;
    Split* splits;
    uint32_t splitCount;
    // Per decoder symbol, the corner a split makes active, or none.
    uint32_t* splitCorner;
    uint32_t* stack;
    uint32_t top;
    uint32_t* invalid;
    uint32_t invalidCount;
} State;

static bool ValidVertex(const State* s, uint32_t v)
{
    return v < s->t->vertices;
}

static bool AddVertex(State* s, uint32_t* out)
{
    if (s->t->vertices >= s->vertexMax)
    {
        return false;
    }
    *out = s->t->vertices++;
    s->t->leftMost[*out] = NONE;
    return true;
}

static void SetOpposites(maniDracoTable* t, uint32_t a, uint32_t b)
{
    t->opposite[a] = b;
    t->opposite[b] = a;
}

static void SetLeftMost(State* s, uint32_t v, uint32_t c)
{
    if (ValidVertex(s, v))
    {
        s->t->leftMost[v] = c;
    }
}

// The counts and the split events (bitstream 2.2: no hole events).
static maniDracoStatus Counts(State* s, uint32_t* vertices, uint32_t* splitSymbols)
{
    uint8_t data = 0;
    if (!maniDracoU8(s->r, &s->method) || (s->method != Standard && s->method != Valence) ||
        !maniDracoLeb32(s->r, vertices) || !maniDracoLeb32(s->r, &s->faces) ||
        s->faces > (uint32_t)INT32_MAX / 3u || *vertices > s->faces * 3u ||
        (uint64_t)*vertices * ((uint64_t)*vertices - 1u) / 2u < (uint64_t)s->faces * 3u / 2u ||
        !maniDracoU8(s->r, &data) || !maniDracoLeb32(s->r, &s->symbols) || s->faces < s->symbols ||
        s->faces > s->symbols + s->symbols / 3u || !maniDracoLeb32(s->r, splitSymbols) ||
        *splitSymbols > s->symbols)
    {
        return maniDracoInvalid;
    }
    s->dataCount = data;
    s->vertexMax = *vertices + *splitSymbols;
    return maniDracoDone;
}

static maniDracoStatus SplitEvents(State* s)
{
    uint32_t count = 0;
    if (!maniDracoLeb32(s->r, &count) || count > s->faces)
    {
        return maniDracoInvalid;
    }
    s->splits = maniArenaTake(s->arena, count, sizeof(Split));
    if (s->splits == NULL)
    {
        return maniDracoCapacity;
    }
    uint32_t last = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t delta = 0;
        uint32_t back = 0;
        if (!maniDracoLeb32(s->r, &delta) || (uint64_t)delta + last > INT32_MAX ||
            !maniDracoLeb32(s->r, &back) || back > delta + last)
        {
            return maniDracoInvalid;
        }
        s->splits[i].source = delta + last;
        s->splits[i].split = s->splits[i].source - back;
        last = s->splits[i].source;
    }
    maniDracoBits bits = {s->r->at, 0, maniDracoLeft(s->r)};
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t edge = 0;
        if (!maniDracoBitsRead(&bits, 1, &edge))
        {
            return maniDracoInvalid;
        }
        s->splits[i].edge = (uint8_t)edge;
    }
    s->r->at += (bits.bit + 7u) / 8u;
    s->splitCount = count;
    return maniDracoDone;
}

// The valence contexts: each a LEB128 count of at most the faces, then
// that many symbols.
static maniDracoStatus ValenceContexts(State* s)
{
    s->valences = maniArenaTake(s->arena, s->vertexMax, sizeof(uint32_t));
    if (s->valences == NULL)
    {
        return maniDracoCapacity;
    }
    for (uint32_t i = 0; i < Contexts; ++i)
    {
        uint32_t n = 0;
        if (!maniDracoLeb32(s->r, &n) || n > s->faces)
        {
            return maniDracoInvalid;
        }
        s->contexts[i] = maniArenaTake(s->arena, n, sizeof(uint32_t));
        if (s->contexts[i] == NULL)
        {
            return maniDracoCapacity;
        }
        maniDracoStatus st = maniDracoSymbols(s->r, s->arena, n, 1, s->contexts[i]);
        if (st != maniDracoDone)
        {
            return st;
        }
        s->remaining[i] = n;
    }
    s->active = -1;
    return maniDracoDone;
}

// The traversal's data: standard symbols' bits, the start faces' and the
// seams' rABS, then valence's contexts.
static maniDracoStatus Traversal(State* s)
{
    if (s->method == Standard)
    {
        uint64_t size = 0;
        const uint8_t* bytes = NULL;
        if (!maniDracoLeb64(s->r, &size) || size > maniDracoLeft(s->r) ||
            !maniDracoBytes(s->r, (size_t)size, &bytes))
        {
            return maniDracoInvalid;
        }
        s->bits = (maniDracoBits){bytes, 0, size};
    }
    s->seams = maniArenaTake(s->arena, s->dataCount, sizeof(maniDracoRabs));
    if (s->seams == NULL)
    {
        return maniDracoCapacity;
    }
    bool read = maniDracoRabsStart(s->r, &s->start);
    for (uint32_t i = 0; read && i < s->dataCount; ++i)
    {
        read = maniDracoRabsStart(s->r, &s->seams[i]);
    }
    if (!read)
    {
        return maniDracoInvalid;
    }
    return s->method == Valence ? ValenceContexts(s) : maniDracoDone;
}

static uint32_t NextSymbol(State* s)
{
    static const uint32_t kSymbols[5] = {SymbolC, SymbolS, SymbolL, SymbolR, SymbolE};
    uint32_t v = 0;
    if (s->method == Standard)
    {
        uint32_t suffix = 0;
        if (!maniDracoBitsRead(&s->bits, 1, &v))
        {
            return SymbolInvalid;
        }
        return v == 0
                   ? SymbolC
                   : (maniDracoBitsRead(&s->bits, 2, &suffix) ? 1u | suffix << 1 : SymbolInvalid);
    }
    if (s->active < 0)
    {
        s->last = SymbolE;
        return s->last;
    }
    if (s->remaining[s->active] == 0)
    {
        return SymbolInvalid;
    }
    v = s->contexts[s->active][--s->remaining[s->active]];
    s->last = v > 4u ? SymbolInvalid : kSymbols[v];
    return s->last;
}

// Valence: the vertices of the new active corner gain the valence its
// symbol adds; the context is the next vertex's, clamped to 2 to 7.
static bool ActiveCorner(State* s, uint32_t corner)
{
    if (s->method != Valence)
    {
        return true;
    }
    uint32_t c = maniDracoVertex(s->t, corner);
    uint32_t n = maniDracoVertex(s->t, maniDracoNext(corner));
    uint32_t p = maniDracoVertex(s->t, maniDracoPrevious(corner));
    if (c >= s->vertexMax || n >= s->vertexMax || p >= s->vertexMax)
    {
        return false;
    }
    // What C and S, R, L and E add to the corner's, next and previous
    // vertices.
    static const uint8_t kAdd[4][3] = {{0, 1, 1}, {1, 1, 2}, {1, 2, 1}, {2, 2, 2}};
    uint32_t k =
        s->last == SymbolR ? 1u : (s->last == SymbolL ? 2u : (s->last == SymbolE ? 3u : 0u));
    s->valences[c] += kAdd[k][0];
    s->valences[n] += kAdd[k][1];
    s->valences[p] += kAdd[k][2];
    uint32_t v = s->valences[n];
    s->active = (int32_t)(v < 2u ? 2u : (v > 7u ? 7u : v)) - 2;
    return true;
}

// C: closes the active corner's gap with the next vertex's left-most
// corner's next.
static bool SymbolCFace(State* s, uint32_t corner)
{
    if (s->top == 0)
    {
        return false;
    }
    maniDracoTable* t = s->t;
    uint32_t a = s->stack[s->top - 1u];
    uint32_t x = maniDracoVertex(t, maniDracoNext(a));
    uint32_t b = maniDracoNext(maniDracoLeftMost(t, x));
    if (!ValidVertex(s, x) || b == NONE || a == b || maniDracoOpposite(t, a) != NONE ||
        maniDracoOpposite(t, b) != NONE)
    {
        return false;
    }
    SetOpposites(t, a, corner + 1u);
    SetOpposites(t, b, corner + 2u);
    uint32_t ap = maniDracoVertex(t, maniDracoPrevious(a));
    uint32_t bn = maniDracoVertex(t, maniDracoNext(b));
    if (x == ap || x == bn || !ValidVertex(s, ap) || !ValidVertex(s, bn))
    {
        return false;
    }
    t->vertex[corner] = x;
    t->vertex[corner + 1u] = bn;
    t->vertex[corner + 2u] = ap;
    SetLeftMost(s, ap, corner + 2u);
    s->hole[x] = 0;
    s->stack[s->top - 1u] = corner;
    return true;
}

// R and L: a new vertex opposite the active corner's edge.
static bool SymbolRLFace(State* s, uint32_t corner, uint32_t symbol)
{
    if (s->top == 0)
    {
        return false;
    }
    maniDracoTable* t = s->t;
    uint32_t a = s->stack[s->top - 1u];
    uint32_t v = 0;
    if (maniDracoOpposite(t, a) != NONE || !AddVertex(s, &v))
    {
        return false;
    }
    uint32_t opp = symbol == SymbolR ? corner + 2u : corner + 1u;
    uint32_t l = symbol == SymbolR ? corner + 1u : corner;
    uint32_t r = symbol == SymbolR ? corner : corner + 2u;
    SetOpposites(t, opp, a);
    t->vertex[opp] = v;
    t->leftMost[v] = opp;
    uint32_t vr = maniDracoVertex(t, maniDracoPrevious(a));
    uint32_t vl = maniDracoVertex(t, maniDracoNext(a));
    if (!ValidVertex(s, vr) || !ValidVertex(s, vl))
    {
        return false;
    }
    t->vertex[r] = vr;
    SetLeftMost(s, vr, r);
    t->vertex[l] = vl;
    s->stack[s->top - 1u] = corner;
    return true;
}

// S: joins the active corner with the one below it on the stack (or the
// corner a split left for this symbol), merging two vertices.
static bool SymbolSFace(State* s, uint32_t corner, uint32_t symbol)
{
    maniDracoTable* t = s->t;
    if (s->top == 0)
    {
        return false;
    }
    uint32_t b = s->stack[--s->top];
    if (s->splitCorner[symbol] != NONE)
    {
        s->stack[s->top++] = s->splitCorner[symbol];
    }
    if (s->top == 0)
    {
        return false;
    }
    uint32_t a = s->stack[s->top - 1u];
    if (a == b || maniDracoOpposite(t, a) != NONE || maniDracoOpposite(t, b) != NONE)
    {
        return false;
    }
    SetOpposites(t, a, corner + 2u);
    SetOpposites(t, b, corner + 1u);
    uint32_t p = maniDracoVertex(t, maniDracoPrevious(a));
    uint32_t an = maniDracoVertex(t, maniDracoNext(a));
    uint32_t bp = maniDracoVertex(t, maniDracoPrevious(b));
    uint32_t cn = maniDracoNext(b);
    uint32_t n = maniDracoVertex(t, cn);
    if (!ValidVertex(s, p) || !ValidVertex(s, an) || !ValidVertex(s, bp) || !ValidVertex(s, n))
    {
        return false;
    }
    t->vertex[corner] = p;
    t->vertex[corner + 1u] = an;
    t->vertex[corner + 2u] = bp;
    SetLeftMost(s, bp, corner + 2u);
    if (s->method == Valence)
    {
        s->valences[p] += s->valences[n];
    }
    t->leftMost[p] = t->leftMost[n];
    uint32_t first = cn;
    for (uint32_t k = 0; cn != NONE; ++k)
    {
        t->vertex[cn] = p;
        cn = maniDracoSwingLeft(t, cn);
        if (cn == first || k > t->corners)
        {
            return false;
        }
    }
    t->leftMost[n] = NONE;
    if (s->dataCount == 0)
    {
        s->invalid[s->invalidCount++] = n;
    }
    s->stack[s->top - 1u] = corner;
    return true;
}

// E: a new face of three new vertices, a new active corner.
static bool SymbolEFace(State* s, uint32_t corner)
{
    maniDracoTable* t = s->t;
    uint32_t v[3] = {0};
    if (s->t->vertices + 3u > s->vertexMax)
    {
        return false;
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        (void)AddVertex(s, &v[k]);
        t->vertex[corner + k] = v[k];
        t->leftMost[v[k]] = corner + k;
    }
    s->stack[s->top++] = corner;
    return true;
}

// The splits whose source is this symbol: each leaves a corner active for
// its split symbol; a source past it is invalid.
static bool Splits(State* s, uint32_t symbol)
{
    uint32_t encoder = s->symbols - symbol - 1u;
    while (s->splitCount > 0)
    {
        const Split* e = &s->splits[s->splitCount - 1u];
        if (e->source > encoder)
        {
            return false;
        }
        if (e->source != encoder)
        {
            return true;
        }
        uint32_t top = s->stack[s->top - 1u];
        uint32_t corner = e->edge == 1 ? maniDracoNext(top) : maniDracoPrevious(top);
        s->splitCorner[s->symbols - e->split - 1u] = corner;
        --s->splitCount;
    }
    return true;
}

static bool Face(State* s, uint32_t symbol)
{
    uint32_t corner = 3u * s->face++;
    uint32_t kind = NextSymbol(s);
    bool split = kind == SymbolR || kind == SymbolL || kind == SymbolE;
    bool ok = kind == SymbolC                      ? SymbolCFace(s, corner)
              : kind == SymbolR || kind == SymbolL ? SymbolRLFace(s, corner, kind)
              : kind == SymbolS                    ? SymbolSFace(s, corner, symbol)
              : kind == SymbolE                    ? SymbolEFace(s, corner)
                                                   : false;
    return ok && ActiveCorner(s, s->stack[s->top - 1u]) && (!split || Splits(s, symbol));
}

// The faces the stack's corners close: per corner a start-face decision,
// an interior face joining it with its two neighbours' gaps.
static bool StartFaces(State* s)
{
    maniDracoTable* t = s->t;
    while (s->top > 0)
    {
        uint32_t corner = s->stack[--s->top];
        if (!maniDracoRabsRead(&s->start))
        {
            continue;
        }
        uint32_t n = maniDracoVertex(t, maniDracoNext(corner));
        uint32_t b = maniDracoNext(maniDracoLeftMost(t, n));
        uint32_t x = maniDracoVertex(t, maniDracoNext(b));
        uint32_t c = maniDracoNext(maniDracoLeftMost(t, x));
        uint32_t p = maniDracoVertex(t, maniDracoNext(c));
        if (s->face >= s->faces || b == NONE || c == NONE || corner == b || corner == c || b == c ||
            maniDracoOpposite(t, corner) != NONE || maniDracoOpposite(t, b) != NONE ||
            maniDracoOpposite(t, c) != NONE || !ValidVertex(s, n) || !ValidVertex(s, x) ||
            !ValidVertex(s, p))
        {
            return false;
        }
        uint32_t f = 3u * s->face++;
        SetOpposites(t, f, corner);
        SetOpposites(t, f + 1u, b);
        SetOpposites(t, f + 2u, c);
        t->vertex[f] = x;
        t->vertex[f + 1u] = p;
        t->vertex[f + 2u] = n;
        s->hole[x] = 0;
        s->hole[p] = 0;
        s->hole[n] = 0;
    }
    return s->face == s->faces;
}

// Without attribute data, the vertices S merged away are filled from the
// end of the table; the vertices that remain.
static bool RemoveMerged(State* s, uint32_t* remaining)
{
    maniDracoTable* t = s->t;
    uint32_t count = t->vertices;
    for (uint32_t i = 0; i < s->invalidCount; ++i)
    {
        uint32_t gone = s->invalid[i];
        while (count > 0 && t->leftMost[count - 1u] == NONE)
        {
            --count;
        }
        if (count == 0)
        {
            return false;
        }
        uint32_t source = count - 1u;
        if (source < gone)
        {
            continue;
        }
        // Every corner of the source vertex, swinging left then right.
        uint32_t start = t->leftMost[source];
        uint32_t c = start;
        bool left = true;
        for (uint32_t k = 0; c != NONE; ++k)
        {
            if (t->vertex[c] != source || k > t->corners)
            {
                return false;
            }
            t->vertex[c] = gone;
            uint32_t next = left ? maniDracoSwingLeft(t, c) : maniDracoSwingRight(t, c);
            if (left && next == NONE)
            {
                left = false;
                next = maniDracoSwingRight(t, start);
            }
            c = left && next == start ? NONE : next;
        }
        t->leftMost[gone] = t->leftMost[source];
        t->leftMost[source] = NONE;
        s->hole[gone] = s->hole[source];
        s->hole[source] = 0;
        --count;
    }
    *remaining = count;
    return true;
}

// Each face's three edges: an open one is a seam for every attribute
// data; an edge shared with a later face (or this one) reads one rABS
// bit per data; one shared with an earlier face was read there.
static bool Seams(State* s, maniDracoTable* data)
{
    const maniDracoTable* t = s->t;
    for (uint32_t f = 0; f < s->faces; ++f)
    {
        // The corner, its next and its previous.
        for (uint32_t c = 3u * f; c < 3u * f + 3u; ++c)
        {
            uint32_t o = maniDracoOpposite(t, c);
            if (o != NONE && o / 3u < f)
            {
                continue;
            }
            for (uint32_t i = 0; i < s->dataCount; ++i)
            {
                if ((o == NONE || maniDracoRabsRead(&s->seams[i])) &&
                    !maniDracoAddSeam(&data[i], t, c))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

// A vertex's first point corner: its left-most corner, or for an
// interior vertex on a seam, the first corner swinging right where the
// seam's attribute vertex changes.
static bool FirstPointCorner(const State* s, const maniDracoTable* data, uint32_t v,
                             uint32_t* first)
{
    const maniDracoTable* t = s->t;
    uint32_t c = t->leftMost[v];
    *first = c;
    if (s->hole[v] != 0)
    {
        return true;
    }
    for (uint32_t i = 0; i < s->dataCount; ++i)
    {
        if (data[i].vertexSeam[v] == 0)
        {
            continue;
        }
        uint32_t id = data[i].vertex[c];
        uint32_t act = maniDracoSwingRight(t, c);
        for (uint32_t k = 0; act != c; ++k)
        {
            if (act == NONE || k > t->corners)
            {
                return false;
            }
            if (data[i].vertex[act] != id)
            {
                *first = act;
                return true;
            }
            act = maniDracoSwingRight(t, act);
        }
    }
    return true;
}

static bool SameAttributeVertex(const State* s, const maniDracoTable* data, uint32_t a, uint32_t b)
{
    for (uint32_t i = 0; i < s->dataCount; ++i)
    {
        if (data[i].vertex[a] != data[i].vertex[b])
        {
            return false;
        }
    }
    return true;
}

// Google's AssignPointsToCorners: without attribute data a corner's point
// is its vertex; with it, a vertex's corners swinging right share a point
// until an attribute vertex changes. A corner no walk reaches keeps point
// 0, as in Google's decoder.
static bool AssignPoints(State* s, maniDracoEdgebreaker* out, uint32_t vertices)
{
    const maniDracoTable* t = s->t;
    out->cornerPoint = maniArenaTake(s->arena, t->corners, sizeof(uint32_t));
    if (out->cornerPoint == NULL)
    {
        return false;
    }
    if (s->dataCount == 0)
    {
        memcpy(out->cornerPoint, t->vertex, (size_t)t->corners * sizeof(uint32_t));
        out->pointCount = vertices;
        return true;
    }
    uint32_t count = 0;
    for (uint32_t v = 0; v < t->vertices; ++v)
    {
        uint32_t first = NONE;
        if (t->leftMost[v] == NONE)
        {
            continue;
        }
        if (t->leftMost[v] >= t->corners || !FirstPointCorner(s, out->data, v, &first) ||
            count >= t->corners)
        {
            return false;
        }
        out->cornerPoint[first] = count++;
        uint32_t previous = first;
        uint32_t c = maniDracoSwingRight(t, first);
        for (uint32_t k = 0; c != NONE && c != first; ++k)
        {
            if (k > t->corners || count >= t->corners)
            {
                return false;
            }
            out->cornerPoint[c] = SameAttributeVertex(s, out->data, c, previous)
                                      ? out->cornerPoint[previous]
                                      : count++;
            previous = c;
            c = maniDracoSwingRight(t, c);
        }
    }
    out->pointCount = count;
    return true;
}

static maniDracoStatus Allocate(State* s)
{
    s->hole = maniArenaTake(s->arena, s->vertexMax, 1);
    s->stack = maniArenaTake(s->arena, (size_t)s->symbols + 1u, sizeof(uint32_t));
    s->splitCorner = maniArenaTake(s->arena, s->symbols, sizeof(uint32_t));
    s->invalid = maniArenaTake(s->arena, s->symbols, sizeof(uint32_t));
    if (s->hole == NULL || s->stack == NULL || s->splitCorner == NULL || s->invalid == NULL ||
        !maniDracoTableInit(s->arena, s->faces, s->vertexMax, s->t))
    {
        return maniDracoCapacity;
    }
    memset(s->hole, 1, s->vertexMax);
    memset(s->splitCorner, 0xFF, (size_t)s->symbols * sizeof(uint32_t));
    return maniDracoDone;
}

// The symbols, the start faces, then the merged vertices' removal; the
// vertices the connectivity keeps.
static maniDracoStatus Connectivity(State* s, uint32_t* vertices)
{
    for (uint32_t i = 0; i < s->symbols; ++i)
    {
        if (!Face(s, i))
        {
            return maniDracoInvalid;
        }
    }
    if (!StartFaces(s))
    {
        return maniDracoInvalid;
    }
    *vertices = s->t->vertices;
    return s->dataCount == 0 && !RemoveMerged(s, vertices) ? maniDracoInvalid : maniDracoDone;
}

// Every attribute data's table: its seams, then its vertices.
static maniDracoStatus AttributeTables(State* s, maniDracoEdgebreaker* out)
{
    out->data = maniArenaTake(s->arena, s->dataCount, sizeof(maniDracoTable));
    if (out->data == NULL)
    {
        return maniDracoCapacity;
    }
    out->dataCount = s->dataCount;
    for (uint32_t i = 0; i < s->dataCount; ++i)
    {
        if (!maniDracoSeamsInit(s->arena, s->t, &out->data[i]))
        {
            return maniDracoCapacity;
        }
    }
    if (!Seams(s, out->data))
    {
        return maniDracoInvalid;
    }
    for (uint32_t i = 0; i < s->dataCount; ++i)
    {
        if (!maniDracoRecompute(s->arena, &out->data[i], s->t))
        {
            return maniDracoInvalid;
        }
    }
    return maniDracoDone;
}

maniDracoStatus maniDracoReadEdgebreaker(maniDracoReader* r, maniArena* arena, uint32_t maxPoints,
                                         maniDracoEdgebreaker* out)
{
    *out = (maniDracoEdgebreaker){0};
    State s = {0};
    s.r = r;
    s.arena = arena;
    s.t = &out->mesh;
    uint32_t vertices = 0;
    uint32_t splitSymbols = 0;
    maniDracoStatus st = Counts(&s, &vertices, &splitSymbols);
    st = st == maniDracoDone ? Allocate(&s) : st;
    st = st == maniDracoDone ? SplitEvents(&s) : st;
    st = st == maniDracoDone ? Traversal(&s) : st;
    st = st == maniDracoDone ? Connectivity(&s, &vertices) : st;
    st = st == maniDracoDone ? AttributeTables(&s, out) : st;
    if (st == maniDracoDone && !AssignPoints(&s, out, vertices))
    {
        st = maniDracoInvalid;
    }
    if (st == maniDracoDone && out->pointCount > maxPoints)
    {
        st = maniDracoInvalid;
    }
    out->hole = s.hole;
    return st;
}
