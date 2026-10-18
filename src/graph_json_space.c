// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_json_space.h"

#include "arena.h"
#include "graph_json_read.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// A blend space's position: {"vector": name}, or {"x": number, "y":
// number} with y 0 when left out.
static bool Position(maniGraphReader* r, const maniGraphScope* scope, uint32_t value, bool plane,
                     maniGraphPosition* p)
{
    static const char* const kKeys[] = {"vector", "x", "y"};
    uint32_t f[3] = {MANI_JSON_NONE, MANI_JSON_NONE, MANI_JSON_NONE};
    *p = (maniGraphPosition){
        MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.0f}, {MANI_NO_PARAMETER, 0.0f}};
    if (!maniGraphJsonMembers(r, value, kKeys, plane ? 3u : 2u, 0u, f))
    {
        return false;
    }
    if (f[0] != MANI_JSON_NONE)
    {
        uint32_t extra = f[1] != MANI_JSON_NONE ? f[1] : f[2];
        return extra == MANI_JSON_NONE ? maniGraphJsonRefer(r, &scope->parameters, f[0], &p->vector)
                                       : maniGraphJsonFault(r, extra - 1, maniGraphFormMember);
    }
    if (f[1] == MANI_JSON_NONE)
    {
        return maniGraphJsonFault(r, value, maniGraphFormMissing);
    }
    return maniGraphJsonScalar(r, scope, f[1], &p->x) &&
           (f[2] == MANI_JSON_NONE || maniGraphJsonScalar(r, scope, f[2], &p->y));
}

// A blend space's points: count numbers, or count [x, y] pairs.
static bool Points(maniGraphReader* r, uint32_t value, uint32_t count, bool pairs,
                   const float** out)
{
    uint32_t n = 0;
    if (!maniGraphJsonArray(r, value, &n))
    {
        return false;
    }
    if (n != count)
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    float* points = maniArenaTake(&r->arena, pairs ? 2 * (size_t)count : count, sizeof(float));
    if (points == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        bool ok = pairs ? maniGraphJsonFloats(r, e, points + 2 * k, 2)
                        : maniGraphJsonFloat(r, e, &points[k]);
        if (!ok)
        {
            return false;
        }
    }
    *out = points;
    return true;
}

// Authored triangles: a list (not empty) of three inputs each.
static bool Triangles(maniGraphReader* r, uint32_t value, maniGraphTrianglesNodeSource* s)
{
    uint32_t count = 0;
    if (!maniGraphJsonArray(r, value, &count))
    {
        return false;
    }
    uint32_t* corners =
        count > 0 ? maniArenaTake(&r->arena, 3 * (size_t)count, sizeof(uint32_t)) : NULL;
    if (count == 0)
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    if (corners == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t t = 0; t < count; ++t)
    {
        e = maniGraphJsonNext(r->json, e, t == 0);
        uint32_t three = 0;
        if (!maniGraphJsonArray(r, e, &three))
        {
            return false;
        }
        if (three != 3)
        {
            return maniGraphJsonFault(r, e, maniGraphFormValue);
        }
        uint32_t c = e;
        for (uint32_t k = 0; k < 3; ++k)
        {
            c = maniGraphJsonNext(r->json, c, k == 0);
            if (!maniGraphJsonIndex(r, c, &corners[3 * t + k]))
            {
                return false;
            }
        }
    }
    s->triangles = corners;
    s->triangleCount = count;
    return true;
}

bool maniGraphJsonSpace(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                        maniGraphNodeSource* n)
{
    if (n->kind == maniGraphNodeBlendSpaceTriangles)
    {
        maniGraphTrianglesNodeSource* s = &n->triangulated;
        s->triangles = NULL;
        s->triangleCount = 0;
        return Position(r, scope, members[2], true, &s->position) &&
               Points(r, members[3], n->inputCount, true, &s->points) &&
               (members[4] == MANI_JSON_NONE || Triangles(r, members[4], s));
    }
    if (n->kind == maniGraphNodeBlendSpace1D)
    {
        maniGraphBlendSpace1DNodeSource* s = &n->blendSpace1D;
        s->period = 0.0f;
        return Position(r, scope, members[2], false, &s->position) &&
               Points(r, members[3], n->inputCount, false, &s->points) &&
               (members[4] == MANI_JSON_NONE || maniGraphJsonFloat(r, members[4], &s->period));
    }
    maniGraphBandsNodeSource* s = &n->bands;
    bool polar = false;
    bool ok = Position(r, scope, members[2], true, &s->position) &&
              Points(r, members[3], n->inputCount, true, &s->points) &&
              (members[4] == MANI_JSON_NONE || maniGraphJsonBool(r, members[4], &polar));
    s->polar = polar ? 1u : 0u;
    return ok;
}
