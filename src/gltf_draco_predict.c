// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Google's mesh prediction scheme decoders (d429de8), in
// the integer arithmetic its builds compute: 32-bit sums wrap, 64-bit
// products wrap where Google's signed ones would overflow, divisions
// truncate. Walks around a vertex are bounded by the table's corners;
// where Google's decoder would index with a vertex or value of none, or
// past a table, the stream is invalid.

#include "gltf_draco_predict.h"

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"
#include "gltf_draco_traverse.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NONE           MANI_DRACO_NONE
#define MAX_COMPONENTS 16u

static int32_t Signed(uint32_t u)
{
    return u <= INT32_MAX ? (int32_t)u : -(int32_t)(~u) - 1;
}

static int64_t Signed64(uint64_t u)
{
    return u <= INT64_MAX ? (int64_t)u : -(int64_t)(~u) - 1;
}

// |x| as Google's std::abs gives it (the most negative value unchanged).
static int64_t Abs64(int64_t x)
{
    return x < 0 ? Signed64(0u - (uint64_t)x) : x;
}

static int64_t Max64(int64_t a, int64_t b)
{
    return a > b ? a : b;
}

// A table vertex's value; false for a vertex of none.
static bool VertexValue(const maniDracoPredictor* p, uint32_t corner, uint32_t* value)
{
    uint32_t v = maniDracoVertex(p->table, corner);
    if (v >= p->order->vertexCount)
    {
        return false;
    }
    *value = p->order->vertexValue[v];
    return true;
}

// Google's ComputeParallelogramPrediction for value id at corner c: 1
// predicted, 0 not, -1 a vertex of none.
static int Parallelogram(const maniDracoPredictor* p, uint32_t id, uint32_t c, const uint32_t* data,
                         uint32_t* pred)
{
    uint32_t o = maniDracoOpposite(p->table, c);
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t d = 0;
    if (o == NONE)
    {
        return 0;
    }
    if (!VertexValue(p, o, &a) || !VertexValue(p, maniDracoNext(o), &b) ||
        !VertexValue(p, maniDracoPrevious(o), &d))
    {
        return -1;
    }
    if (a >= id || b >= id || d >= id)
    {
        return 0;
    }
    uint32_t n = p->components;
    for (uint32_t k = 0; k < n; ++k)
    {
        pred[k] = data[(size_t)b * n + k] + data[(size_t)d * n + k] - data[(size_t)a * n + k];
    }
    return 1;
}

// The transform on value id from pred, or from the previous value.
static void Revert(const maniDracoPredictor* p, uint32_t* data, uint32_t id, const uint32_t* pred)
{
    uint32_t n = p->components;
    uint32_t corr[MAX_COMPONENTS];
    memcpy(corr, data + (size_t)id * n, n * sizeof(uint32_t));
    p->transform(p->context, pred != NULL ? pred : data + (size_t)(id - 1u) * n, corr,
                 data + (size_t)id * n);
}

static bool Single(const maniDracoPredictor* p, uint32_t* data)
{
    uint32_t pred[MAX_COMPONENTS];
    for (uint32_t id = 1; id < p->order->count; ++id)
    {
        int r = Parallelogram(p, id, p->order->valueCorner[id], data, pred);
        if (r < 0)
        {
            return false;
        }
        Revert(p, data, id, r > 0 ? pred : NULL);
    }
    return true;
}

// Swinging right around a value's vertex from its corner: every
// parallelogram's prediction, averaged.
static bool Multi(const maniDracoPredictor* p, uint32_t* data)
{
    uint32_t n = p->components;
    uint32_t pred[MAX_COMPONENTS];
    uint32_t one[MAX_COMPONENTS];
    for (uint32_t id = 1; id < p->order->count; ++id)
    {
        uint32_t start = p->order->valueCorner[id];
        uint32_t count = 0;
        memset(pred, 0, sizeof pred);
        uint32_t c = start;
        for (uint32_t steps = 0; c != NONE; ++steps)
        {
            int r = steps > p->table->corners ? -1 : Parallelogram(p, id, c, data, one);
            if (r < 0)
            {
                return false;
            }
            for (uint32_t k = 0; r > 0 && k < n; ++k)
            {
                pred[k] += one[k];
            }
            count += (uint32_t)r;
            c = maniDracoSwingRight(p->table, c);
            c = c == start ? NONE : c;
        }
        for (uint32_t k = 0; k < n && count > 0; ++k)
        {
            pred[k] = (uint32_t)(Signed(pred[k]) / (int32_t)count);
        }
        Revert(p, data, id, count > 0 ? pred : NULL);
    }
    return true;
}

// Constrained: up to four parallelograms, left then right of the value's
// corner.
static bool Around(const maniDracoPredictor* p, uint32_t id, const uint32_t* data,
                   uint32_t preds[][MAX_COMPONENTS], uint32_t* count)
{
    uint32_t start = p->order->valueCorner[id];
    uint32_t c = start;
    bool left = true;
    *count = 0;
    for (uint32_t steps = 0; c != NONE; ++steps)
    {
        int r = steps > p->table->corners ? -1 : Parallelogram(p, id, c, data, preds[*count]);
        if (r < 0)
        {
            return false;
        }
        *count += (uint32_t)r;
        if (*count == maniDracoCreaseContexts)
        {
            break;
        }
        c = left ? maniDracoSwingLeft(p->table, c) : maniDracoSwingRight(p->table, c);
        if (c == start)
        {
            break;
        }
        if (c == NONE && left)
        {
            left = false;
            c = maniDracoSwingRight(p->table, start);
        }
    }
    return true;
}

static bool Constrained(const maniDracoPredictor* p, uint32_t* data)
{
    uint32_t n = p->components;
    uint32_t preds[maniDracoCreaseContexts][MAX_COMPONENTS];
    uint32_t pred[MAX_COMPONENTS];
    uint32_t at[maniDracoCreaseContexts] = {0};
    for (uint32_t id = 1; id < p->order->count; ++id)
    {
        uint32_t count = 0;
        uint32_t used = 0;
        if (!Around(p, id, data, preds, &count))
        {
            return false;
        }
        memset(pred, 0, sizeof pred);
        for (uint32_t i = 0; i < count; ++i)
        {
            uint32_t context = count - 1u;
            if (at[context] >= p->creaseCount[context])
            {
                return false;
            }
            bool crease = p->crease[context][at[context]++] != 0;
            for (uint32_t k = 0; !crease && k < n; ++k)
            {
                pred[k] += preds[i][k];
            }
            used += crease ? 0u : 1u;
        }
        for (uint32_t k = 0; k < n && used > 0; ++k)
        {
            pred[k] = (uint32_t)(Signed(pred[k]) / (int32_t)used);
        }
        Revert(p, data, id, used > 0 ? pred : NULL);
    }
    return true;
}

// A value's position: the parent's portable value at the value's point.
static bool Position(const maniDracoPredictor* p, uint32_t id, int64_t out[3])
{
    if (id >= p->order->count)
    {
        return false;
    }
    uint32_t point = p->order->points[id];
    if (p->positionOfPoint != NULL && point >= p->pointCount)
    {
        return false;
    }
    uint32_t value = p->positionOfPoint != NULL ? p->positionOfPoint[point] : point;
    if (value >= p->positionCount)
    {
        return false;
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        out[k] = Signed(p->positions[(size_t)value * 3u + k]);
    }
    return true;
}

// a / b in 64 bits, the most negative value over -1 wrapping.
static int64_t Div64(int64_t a, int64_t b)
{
    return b == -1 ? Signed64(0u - (uint64_t)a) : a / b;
}

// Google's IntSqrt: floor(sqrt(x)) by Newton's method from 2^(log4 x).
static uint64_t IntSqrt(uint64_t x)
{
    if (x == 0)
    {
        return 0;
    }
    uint64_t root = 1;
    for (uint64_t n = x; n >= 2; n /= 4)
    {
        root *= 2;
    }
    do
    {
        root = (root + x / root) / 2;
    } while (root * root > x);
    return root;
}

// The coordinate from the positions: the tip's projection on the
// opposite edge, mapped into the edge's texture coordinates and turned
// by the orientation. 1 predicted, 0 a degenerate edge, -1 a failure.
static int TexFromPositions(maniDracoPredictor* p, const uint32_t* data, uint32_t id, uint32_t next,
                            uint32_t prev, uint32_t pred[2])
{
    int64_t nu[2] = {Signed(data[2u * next]), Signed(data[2u * next + 1u])};
    int64_t pu[2] = {Signed(data[2u * prev]), Signed(data[2u * prev + 1u])};
    if (pu[0] == nu[0] && pu[1] == nu[1])
    {
        pred[0] = data[2u * prev];
        pred[1] = data[2u * prev + 1u];
        return 1;
    }
    int64_t tip[3];
    int64_t np[3];
    int64_t pp[3];
    if (!Position(p, id, tip) || !Position(p, next, np) || !Position(p, prev, pp))
    {
        return -1;
    }
    int64_t pn[3];
    uint64_t norm = 0;
    uint64_t dotBits = 0;
    for (uint32_t k = 0; k < 3; ++k)
    {
        pn[k] = pp[k] - np[k];
        norm += (uint64_t)pn[k] * (uint64_t)pn[k];
        dotBits += (uint64_t)pn[k] * (uint64_t)(tip[k] - np[k]);
    }
    if (norm == 0)
    {
        return 0;
    }
    int64_t dot = Signed64(dotBits);
    int64_t puv[2] = {pu[0] - nu[0], pu[1] - nu[1]};
    int64_t pmax = Max64(Max64(Abs64(pn[0]), Abs64(pn[1])), Abs64(pn[2]));
    if ((uint64_t)Max64(Abs64(nu[0]), Abs64(nu[1])) > (uint64_t)INT64_MAX / norm ||
        Abs64(dot) > INT64_MAX / Max64(Abs64(puv[0]), Abs64(puv[1])) ||
        Abs64(dot) > INT64_MAX / pmax || p->orientationCount == 0)
    {
        return -1;
    }
    uint64_t cx = 0;
    for (uint32_t k = 0; k < 3; ++k)
    {
        int64_t along = Div64(Signed64((uint64_t)dot * (uint64_t)pn[k]), Signed64(norm));
        uint64_t d = (uint64_t)tip[k] - ((uint64_t)np[k] + (uint64_t)along);
        cx += d * d;
    }
    uint64_t root = IntSqrt(cx * norm);
    uint64_t turn[2] = {(uint64_t)puv[1] * root, (uint64_t)(0 - puv[0]) * root};
    bool orientation = p->orientation[--p->orientationCount] != 0;
    for (uint32_t k = 0; k < 2; ++k)
    {
        uint64_t x = (uint64_t)nu[k] * norm + (uint64_t)puv[k] * (uint64_t)dot;
        uint64_t s = orientation ? x + turn[k] : x - turn[k];
        pred[k] = (uint32_t)(uint64_t)Div64(Signed64(s), Signed64(norm));
    }
    return 1;
}

// Google's portable texture coordinate predictor at a value's corner;
// without both neighbours' values, a neighbour's or the previous value.
static bool TexPredict(maniDracoPredictor* p, const uint32_t* data, uint32_t id, uint32_t pred[2])
{
    uint32_t corner = p->order->valueCorner[id];
    uint32_t next = 0;
    uint32_t prev = 0;
    if (!VertexValue(p, maniDracoNext(corner), &next) ||
        !VertexValue(p, maniDracoPrevious(corner), &prev))
    {
        return false;
    }
    int r = prev < id && next < id ? TexFromPositions(p, data, id, next, prev, pred) : 0;
    if (r != 0)
    {
        return r > 0;
    }
    // Google's decoder also tests the previous neighbour, but the next
    // one's test always overrides it.
    uint32_t from = next < id ? next : (id > 0 ? id - 1u : NONE);
    pred[0] = from == NONE ? 0u : data[2u * from];
    pred[1] = from == NONE ? 0u : data[2u * from + 1u];
    return true;
}

static bool TexCoords(maniDracoPredictor* p, uint32_t* data)
{
    uint32_t pred[2];
    if (p->components != 2)
    {
        return false;
    }
    for (uint32_t id = 0; id < p->order->count; ++id)
    {
        if (!TexPredict(p, data, id, pred))
        {
            return false;
        }
        Revert(p, data, id, pred);
    }
    return true;
}

static bool CornerPosition(const maniDracoPredictor* p, uint32_t corner, int64_t out[3])
{
    uint32_t id = 0;
    return VertexValue(p, corner, &id) && Position(p, id, out);
}

// Google's area-weighted normal: the cross products of the faces around
// the corner's vertex summed, scaled to a sum of at most 2^29.
static bool AreaNormal(const maniDracoPredictor* p, uint32_t corner, int32_t out[3])
{
    const maniDracoTable* t = p->table;
    int64_t center[3];
    uint64_t normal[3] = {0};
    if (!CornerPosition(p, corner, center))
    {
        return false;
    }
    bool left = true;
    for (uint32_t c = corner, steps = 0; c != NONE; ++steps)
    {
        int64_t a[3];
        int64_t b[3];
        if (steps > t->corners || !CornerPosition(p, maniDracoNext(c), a) ||
            !CornerPosition(p, maniDracoPrevious(c), b))
        {
            return false;
        }
        uint64_t u[3];
        uint64_t v[3];
        for (uint32_t k = 0; k < 3; ++k)
        {
            u[k] = (uint64_t)(a[k] - center[k]);
            v[k] = (uint64_t)(b[k] - center[k]);
        }
        normal[0] += u[1] * v[2] - u[2] * v[1];
        normal[1] += u[2] * v[0] - u[0] * v[2];
        normal[2] += u[0] * v[1] - u[1] * v[0];
        uint32_t next = left ? maniDracoSwingLeft(t, c) : maniDracoSwingRight(t, c);
        if (left && next == NONE)
        {
            left = false;
            next = maniDracoSwingRight(t, corner);
        }
        c = left && next == corner ? NONE : next;
    }
    // AbsSum saturates at the largest 64-bit value.
    uint64_t sum = 0;
    for (uint32_t k = 0; k < 3; ++k)
    {
        // The most negative value's magnitude is 2^63, past the largest.
        uint64_t m = (uint64_t)Abs64(Signed64(normal[k]));
        sum = sum > (uint64_t)INT64_MAX - m || m > (uint64_t)INT64_MAX ? (uint64_t)INT64_MAX
                                                                       : sum + m;
    }
    int64_t quotient = sum > (1u << 29) ? (int64_t)(sum / (1u << 29)) : 1;
    for (uint32_t k = 0; k < 3; ++k)
    {
        out[k] = (int32_t)Signed((uint32_t)(uint64_t)(Signed64(normal[k]) / quotient));
    }
    return true;
}

// Google's octahedron tool box: a vector scaled to |x|+|y|+|z| = center,
// then its octahedral coordinates, canonicalized on the square's edges.
static void Canonicalize(int32_t v[3], int32_t center)
{
    int64_t sum = Abs64(v[0]) + Abs64(v[1]) + Abs64(v[2]);
    if (sum == 0)
    {
        v[0] = center;
        return;
    }
    v[0] = (int32_t)((int64_t)v[0] * center / sum);
    v[1] = (int32_t)((int64_t)v[1] * center / sum);
    int32_t rest = center - (int32_t)Abs64(v[0]) - (int32_t)Abs64(v[1]);
    v[2] = v[2] >= 0 ? rest : -rest;
}

static void Octahedral(const int32_t v[3], int32_t max, int32_t center, uint32_t out[2])
{
    int32_t s = v[1] + center;
    int32_t t = v[2] + center;
    if (v[0] < 0)
    {
        s = v[1] < 0 ? (int32_t)Abs64(v[2]) : max - (int32_t)Abs64(v[2]);
        t = v[2] < 0 ? (int32_t)Abs64(v[1]) : max - (int32_t)Abs64(v[1]);
    }
    if ((s == 0 && t == 0) || (s == 0 && t == max) || (s == max && t == 0))
    {
        s = max;
        t = max;
    }
    else if (s == 0 && t > center)
    {
        t = center - (t - center);
    }
    else if (s == max && t < center)
    {
        t = center + (center - t);
    }
    else if (t == max && s < center)
    {
        s = center + (center - s);
    }
    else if (t == 0 && s > center)
    {
        s = center - (s - center);
    }
    out[0] = (uint32_t)s;
    out[1] = (uint32_t)t;
}

static bool Normals(maniDracoPredictor* p, uint32_t* data)
{
    // The tool box's largest value is one under the transform's.
    int32_t max = p->octahedronMax - 1;
    int32_t center = max / 2;
    if (p->components != 2)
    {
        return false;
    }
    for (uint32_t id = 0; id < p->order->count; ++id)
    {
        int32_t v[3];
        uint32_t pred[2];
        if (!AreaNormal(p, p->order->valueCorner[id], v))
        {
            return false;
        }
        Canonicalize(v, center);
        if (maniDracoRabsRead(&p->flips))
        {
            for (uint32_t k = 0; k < 3; ++k)
            {
                v[k] = -v[k];
            }
        }
        Octahedral(v, max, center, pred);
        Revert(p, data, id, pred);
    }
    return true;
}

// Constrained: per context a LEB128 count of at most the corners, then
// as many rABS crease flags.
static maniDracoStatus Creases(maniDracoReader* r, maniArena* arena, maniDracoPredictor* p)
{
    for (uint32_t i = 0; i < maniDracoCreaseContexts; ++i)
    {
        uint32_t n = 0;
        maniDracoRabs bits;
        if (!maniDracoLeb32(r, &n) || n > p->table->corners)
        {
            return maniDracoInvalid;
        }
        p->creaseCount[i] = n;
        p->crease[i] = maniArenaTake(arena, n, 1);
        if (p->crease[i] == NULL)
        {
            return maniDracoCapacity;
        }
        if (n > 0 && !maniDracoRabsStart(r, &bits))
        {
            return maniDracoInvalid;
        }
        for (uint32_t j = 0; j < n; ++j)
        {
            p->crease[i][j] = maniDracoRabsRead(&bits) ? 1u : 0u;
        }
    }
    return maniDracoDone;
}

// Texture coordinates: a 32-bit count of at most the values, then rABS
// bits, each a zero turning the orientation over.
static maniDracoStatus Orientations(maniDracoReader* r, maniArena* arena, maniDracoPredictor* p,
                                    uint32_t values)
{
    uint32_t n = 0;
    maniDracoRabs bits;
    if (!maniDracoU32(r, &n) || n > INT32_MAX || n > values)
    {
        return maniDracoInvalid;
    }
    p->orientation = maniArenaTake(arena, n, 1);
    if (p->orientation == NULL)
    {
        return maniDracoCapacity;
    }
    if (!maniDracoRabsStart(r, &bits))
    {
        return maniDracoInvalid;
    }
    bool last = true;
    for (uint32_t i = 0; i < n; ++i)
    {
        last = maniDracoRabsRead(&bits) ? last : !last;
        p->orientation[i] = last ? 1u : 0u;
    }
    p->orientationCount = n;
    return maniDracoDone;
}

maniDracoStatus maniDracoPredictData(maniDracoReader* r, maniArena* arena, maniDracoPredictor* p,
                                     uint32_t values)
{
    if (p->scheme == maniDracoConstrained)
    {
        return Creases(r, arena, p);
    }
    return p->scheme == maniDracoTexCoords ? Orientations(r, arena, p, values) : maniDracoDone;
}

bool maniDracoPredictFlips(maniDracoReader* r, maniDracoPredictor* p)
{
    return p->scheme != maniDracoGeometricNormal || maniDracoRabsStart(r, &p->flips);
}

bool maniDracoPredictRevert(maniDracoPredictor* p, uint32_t* data, uint32_t entries)
{
    static const uint32_t kZeros[MAX_COMPONENTS] = {0};
    if (entries == 0)
    {
        return true;
    }
    if (p->order->count > entries || p->components == 0 || p->components > MAX_COMPONENTS)
    {
        return false;
    }
    switch (p->scheme)
    {
    case maniDracoTexCoords:
        return TexCoords(p, data);
    case maniDracoGeometricNormal:
        return Normals(p, data);
    default:
        break;
    }
    Revert(p, data, 0, kZeros);
    switch (p->scheme)
    {
    case maniDracoParallelogram:
        return Single(p, data);
    case maniDracoMultiParallelogram:
        return Multi(p, data);
    case maniDracoConstrained:
        return Constrained(p, data);
    default:
        return false;
    }
}
