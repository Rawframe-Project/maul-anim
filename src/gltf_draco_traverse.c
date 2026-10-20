// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Google's DepthFirstTraverser and MaxPredictionDegreeTraverser with its
// MeshAttributeIndicesEncodingObserver (mani-0021). A face is visited
// once and a vertex reached once, so a traversal is bounded by the
// table; a corner whose vertex is none, where Google's would index with
// it, makes the stream invalid.

#include "gltf_draco_traverse.h"

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"

#include <stdbool.h>
#include <stdint.h>

#define NONE MANI_DRACO_NONE

enum
{
    // Prediction degree's priorities.
    Priorities = 3,
};

typedef struct Walk
{
    const maniDracoTable* t;
    const uint32_t* cornerPoint;
    maniDracoOrder* out;
    uint8_t* faceSeen;
    uint8_t* vertexSeen;
    // Depth first's stack; prediction degree's three and the degrees.
    uint32_t* stack;
    uint32_t top;
    uint32_t* stacks[Priorities];
    uint32_t tops[Priorities];
    uint32_t best;
    uint32_t* degree;
} Walk;

static bool FaceSeen(const Walk* w, uint32_t corner)
{
    return corner == NONE || w->faceSeen[corner / 3u] != 0;
}

// Reaches the vertex at corner if it is new; false for a vertex of none.
static bool Reach(Walk* w, uint32_t corner)
{
    uint32_t v = maniDracoVertex(w->t, corner);
    if (v >= w->t->vertices)
    {
        return false;
    }
    if (w->vertexSeen[v] == 0)
    {
        maniDracoOrder* o = w->out;
        w->vertexSeen[v] = 1;
        o->points[o->count] = w->cornerPoint[corner];
        o->valueCorner[o->count] = corner;
        o->vertexValue[v] = o->count++;
    }
    return true;
}

static bool OnBoundary(const maniDracoTable* t, uint32_t v)
{
    return maniDracoSwingLeft(t, maniDracoLeftMost(t, v)) == NONE;
}

static uint32_t RightCorner(const maniDracoTable* t, uint32_t c)
{
    return maniDracoOpposite(t, maniDracoNext(c));
}

static uint32_t LeftCorner(const maniDracoTable* t, uint32_t c)
{
    return maniDracoOpposite(t, maniDracoPrevious(c));
}

// One face's step of depth first: false on a vertex of none; *done when
// the stack's top is finished.
static bool DepthStep(Walk* w, uint32_t* corner, bool* done)
{
    const maniDracoTable* t = w->t;
    uint32_t c = *corner;
    w->faceSeen[c / 3u] = 1;
    uint32_t v = maniDracoVertex(t, c);
    if (v >= t->vertices)
    {
        return false;
    }
    if (w->vertexSeen[v] == 0)
    {
        bool boundary = OnBoundary(t, v);
        (void)Reach(w, c);
        if (!boundary)
        {
            *corner = RightCorner(t, c);
            // Google's would mark the face of none.
            return *corner != NONE;
        }
    }
    uint32_t right = RightCorner(t, c);
    uint32_t left = LeftCorner(t, c);
    bool rightSeen = FaceSeen(w, right);
    bool leftSeen = FaceSeen(w, left);
    *done = true;
    if (rightSeen && leftSeen)
    {
        --w->top;
    }
    else if (rightSeen || leftSeen)
    {
        *corner = rightSeen ? left : right;
        *done = false;
    }
    else
    {
        w->stack[w->top - 1u] = left;
        w->stack[w->top++] = right;
    }
    return true;
}

static bool DepthFirst(Walk* w, uint32_t corner)
{
    if (FaceSeen(w, corner))
    {
        return true;
    }
    w->top = 0;
    w->stack[w->top++] = corner;
    if (!Reach(w, maniDracoNext(corner)) || !Reach(w, maniDracoPrevious(corner)))
    {
        return false;
    }
    while (w->top > 0)
    {
        uint32_t c = w->stack[w->top - 1u];
        if (FaceSeen(w, c))
        {
            --w->top;
            continue;
        }
        bool done = false;
        while (!done)
        {
            if (!DepthStep(w, &c, &done))
            {
                return false;
            }
        }
    }
    return true;
}

// A corner's priority: 0 when its tip was reached, else 2 the first time
// the tip is counted and 1 after; false for a tip of none.
static bool Priority(Walk* w, uint32_t corner, uint32_t* priority)
{
    uint32_t v = maniDracoVertex(w->t, corner);
    if (v >= w->t->vertices)
    {
        return false;
    }
    *priority = 0;
    if (w->vertexSeen[v] == 0)
    {
        *priority = ++w->degree[v] > 1u ? 1u : 2u;
    }
    return true;
}

static void Push(Walk* w, uint32_t corner, uint32_t priority)
{
    w->stacks[priority][w->tops[priority]++] = corner;
    w->best = priority < w->best ? priority : w->best;
}

static uint32_t Pop(Walk* w)
{
    for (uint32_t i = w->best; i < Priorities; ++i)
    {
        if (w->tops[i] > 0)
        {
            w->best = i;
            return w->stacks[i][--w->tops[i]];
        }
    }
    return NONE;
}

// One face's step by prediction degree: the next face to step to, or none
// when the step ends.
static bool DegreeStep(Walk* w, uint32_t* corner)
{
    const maniDracoTable* t = w->t;
    uint32_t c = *corner;
    w->faceSeen[c / 3u] = 1;
    if (!Reach(w, c))
    {
        return false;
    }
    uint32_t right = RightCorner(t, c);
    uint32_t left = LeftCorner(t, c);
    bool rightSeen = FaceSeen(w, right);
    uint32_t priority = 0;
    *corner = NONE;
    if (!FaceSeen(w, left))
    {
        if (!Priority(w, left, &priority))
        {
            return false;
        }
        if (rightSeen && priority <= w->best)
        {
            *corner = left;
            return true;
        }
        Push(w, left, priority);
    }
    if (!rightSeen)
    {
        if (!Priority(w, right, &priority))
        {
            return false;
        }
        if (priority <= w->best)
        {
            *corner = right;
            return true;
        }
        Push(w, right, priority);
    }
    return true;
}

static bool ByDegree(Walk* w, uint32_t corner)
{
    if (w->t->vertices == 0)
    {
        return true;
    }
    w->stacks[0][w->tops[0]++] = corner;
    w->best = 0;
    if (!Reach(w, maniDracoNext(corner)) || !Reach(w, maniDracoPrevious(corner)) ||
        !Reach(w, corner))
    {
        return false;
    }
    for (uint32_t c = Pop(w); c != NONE; c = Pop(w))
    {
        if (FaceSeen(w, c))
        {
            continue;
        }
        while (c != NONE)
        {
            if (!DegreeStep(w, &c))
            {
                return false;
            }
        }
    }
    return true;
}

static bool Allocate(maniArena* arena, Walk* w, uint32_t mapSize, uint8_t method)
{
    const maniDracoTable* t = w->t;
    uint32_t faces = t->corners / 3u;
    maniDracoOrder* o = w->out;
    w->faceSeen = maniArenaTake(arena, faces, 1);
    w->vertexSeen = maniArenaTake(arena, t->vertices, 1);
    o->points = maniArenaTake(arena, t->vertices, sizeof(uint32_t));
    o->valueCorner = maniArenaTake(arena, t->vertices, sizeof(uint32_t));
    o->vertexValue = maniArenaTake(arena, mapSize, sizeof(uint32_t));
    o->vertexCount = mapSize;
    bool taken = w->faceSeen != NULL && w->vertexSeen != NULL && o->points != NULL &&
                 o->valueCorner != NULL && o->vertexValue != NULL;
    if (method == maniDracoDepthFirst)
    {
        // A step marks a new face or follows a new vertex: at most one
        // push each.
        w->stack = maniArenaTake(arena, (size_t)faces + t->vertices + 1u, sizeof(uint32_t));
        return taken && w->stack != NULL;
    }
    // Each face pushes at most its two neighbours.
    w->degree = maniArenaTake(arena, t->vertices, sizeof(uint32_t));
    for (uint32_t i = 0; i < Priorities; ++i)
    {
        w->stacks[i] = maniArenaTake(arena, 2u * (size_t)faces + 1u, sizeof(uint32_t));
        taken = taken && w->stacks[i] != NULL;
    }
    return taken && w->degree != NULL;
}

maniDracoStatus maniDracoTraverse(maniArena* arena, const maniDracoTable* table,
                                  const uint32_t* cornerPoint, uint32_t mapSize, uint8_t method,
                                  maniDracoOrder* out)
{
    *out = (maniDracoOrder){0};
    if (method > maniDracoPredictionDegree || mapSize < table->vertices)
    {
        return maniDracoInvalid;
    }
    Walk w = {0};
    w.t = table;
    w.cornerPoint = cornerPoint;
    w.out = out;
    if (!Allocate(arena, &w, mapSize, method))
    {
        return maniDracoCapacity;
    }
    for (uint32_t c = 0; c < table->corners; c += 3u)
    {
        bool ok = method == maniDracoDepthFirst ? DepthFirst(&w, c) : ByDegree(&w, c);
        if (!ok)
        {
            return maniDracoInvalid;
        }
    }
    return maniDracoDone;
}

bool maniDracoPointValues(const maniDracoTable* table, const maniDracoOrder* order,
                          const uint32_t* cornerPoint, uint32_t pointCount, uint32_t* pointToValue)
{
    for (uint32_t c = 0; c < table->corners; ++c)
    {
        uint32_t v = maniDracoVertex(table, c);
        if (v >= order->vertexCount)
        {
            return false;
        }
        uint32_t value = order->vertexValue[v];
        if (cornerPoint[c] >= pointCount || value >= pointCount)
        {
            return false;
        }
        pointToValue[cornerPoint[c]] = value;
    }
    return true;
}
