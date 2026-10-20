// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Corner tables as Google's decoder builds them (mani-0021). A walk
// around a vertex visits each corner once in a valid table; a corner
// reached twice, by any vertex's walk, makes the table invalid, so a
// crafted one cannot loop or cost more than its corners.

#include "gltf_draco_corner.h"

#include "arena.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint32_t* Nones(maniArena* arena, uint32_t count)
{
    uint32_t* p = maniArenaTake(arena, count, sizeof(uint32_t));
    if (p != NULL && count > 0)
    {
        memset(p, 0xFF, (size_t)count * sizeof(uint32_t));
    }
    return p;
}

bool maniDracoTableInit(maniArena* arena, uint32_t faces, uint32_t vertices, maniDracoTable* out)
{
    *out = (maniDracoTable){0};
    out->corners = faces * 3u;
    out->opposite = Nones(arena, out->corners);
    out->vertex = Nones(arena, out->corners);
    out->leftMost = Nones(arena, vertices);
    return out->opposite != NULL && out->vertex != NULL && out->leftMost != NULL;
}

bool maniDracoSeamsInit(maniArena* arena, const maniDracoTable* mesh, maniDracoTable* out)
{
    *out = (maniDracoTable){0};
    out->corners = mesh->corners;
    out->opposite = mesh->opposite;
    out->seam = maniArenaTake(arena, mesh->corners, 1);
    out->vertexSeam = maniArenaTake(arena, mesh->vertices, 1);
    return out->seam != NULL && out->vertexSeam != NULL;
}

// Marks the vertices at the ends of the edge opposite c.
static bool MarkEnds(maniDracoTable* attribute, const maniDracoTable* mesh, uint32_t c)
{
    uint32_t a = maniDracoVertex(mesh, maniDracoNext(c));
    uint32_t b = maniDracoVertex(mesh, maniDracoPrevious(c));
    if (a >= mesh->vertices || b >= mesh->vertices)
    {
        return false;
    }
    attribute->vertexSeam[a] = 1;
    attribute->vertexSeam[b] = 1;
    return true;
}

bool maniDracoAddSeam(maniDracoTable* attribute, const maniDracoTable* mesh, uint32_t c)
{
    if (c >= mesh->corners || !MarkEnds(attribute, mesh, c))
    {
        return false;
    }
    attribute->seam[c] = 1;
    uint32_t o = maniDracoOpposite(mesh, c);
    if (o == MANI_DRACO_NONE)
    {
        return true;
    }
    attribute->seam[o] = 1;
    return MarkEnds(attribute, mesh, o);
}

// A seam vertex's first corner: the last one swinging left (over the
// attribute's opposites) before none; false when the walk returns to c
// or runs longer than the corners.
static bool FirstCorner(const maniDracoTable* attribute, uint32_t c, uint32_t* first)
{
    *first = c;
    uint32_t act = maniDracoSwingLeft(attribute, c);
    for (uint32_t n = 0; act != MANI_DRACO_NONE; ++n)
    {
        *first = act;
        act = maniDracoSwingLeft(attribute, act);
        if (act == c || n > attribute->corners)
        {
            return false;
        }
    }
    return true;
}

bool maniDracoRecompute(maniArena* arena, maniDracoTable* attribute, const maniDracoTable* mesh)
{
    uint32_t corners = mesh->corners;
    attribute->vertex = Nones(arena, corners);
    attribute->leftMost = Nones(arena, corners + 1u);
    uint8_t* seen = maniArenaTake(arena, corners, 1);
    if (attribute->vertex == NULL || attribute->leftMost == NULL || seen == NULL)
    {
        return false;
    }
    uint32_t count = 0;
    attribute->vertices = corners + 1u;
    for (uint32_t v = 0; v < mesh->vertices; ++v)
    {
        uint32_t c = maniDracoLeftMost(mesh, v);
        uint32_t first = c;
        if (c == MANI_DRACO_NONE)
        {
            continue;
        }
        if (c >= corners || (attribute->vertexSeam[v] != 0 && !FirstCorner(attribute, c, &first)) ||
            seen[first] != 0 || count > corners)
        {
            return false;
        }
        uint32_t id = count++;
        seen[first] = 1;
        attribute->vertex[first] = id;
        attribute->leftMost[id] = first;
        for (uint32_t act = maniDracoSwingRight(mesh, first);
             act != MANI_DRACO_NONE && act != first; act = maniDracoSwingRight(mesh, act))
        {
            if (seen[act] != 0 || count > corners)
            {
                return false;
            }
            seen[act] = 1;
            if (attribute->seam[maniDracoNext(act)] != 0)
            {
                id = count++;
                attribute->leftMost[id] = act;
            }
            attribute->vertex[act] = id;
        }
    }
    attribute->vertices = count;
    return true;
}
