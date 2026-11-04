// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The morph job: streams copied, their attributes' weighted deltas added
// in target order, directions renormalized.

#include "bytes.h"
#include "fpenv.h"
#include "invariant.h"
#include "morph_file.h"
#include "morph_view.h"

#include "maul-anim/base.h"
#include "maul-anim/morph.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// The components a stream of the semantic may have: positions and
// normals 3, tangents 4, texture coordinates 2, colors 3 or 4.
static bool Components(maniMorphSemantic semantic, uint32_t components)
{
    switch (semantic)
    {
    case maniMorphPosition:
    case maniMorphNormal:
        return components == 3;
    case maniMorphTangent:
        return components == 4;
    case maniMorphTexcoord:
        return components == 2;
    case maniMorphColor:
        return components == 3 || components == 4;
    default:
        return false;
    }
}

// A stream the job takes: pointers, a known semantic and set, its
// components, strides wide enough.
static bool StreamValid(const maniMorphStream* s)
{
    size_t width = 4 * (size_t)s->components;
    return s->in != NULL && s->out != NULL && Components(s->semantic, s->components) &&
           ((uint32_t)s->semantic > maniMorphTangent || s->set == 0) && s->inStride >= width &&
           s->outStride >= width;
}

static bool JobValid(const maniMorphJob* job)
{
    if (job == NULL || !maniMorphTargetsValid(job->targets) || job->weights == NULL ||
        (job->streams == NULL && job->streamCount > 0) || job->first > job->targets->vertexCount ||
        job->count > job->targets->vertexCount - job->first)
    {
        return false;
    }
    for (uint32_t t = 0; t < job->targets->targetCount; ++t)
    {
        if (!Finite(job->weights[t]))
        {
            return false;
        }
    }
    for (uint32_t i = 0; i < job->streamCount; ++i)
    {
        if (!StreamValid(&job->streams[i]))
        {
            return false;
        }
    }
    return true;
}

// The table's entry for a stream's semantic and set, or
// MANI_MAX_MORPH_ATTRIBUTES for none.
static uint32_t Slot(const maniMorphTargets* m, const maniMorphStream* s)
{
    for (uint32_t a = 0; a < m->attributeCount; ++a)
    {
        uint32_t set = 0;
        if (maniMorphAttribute(m, a, &set) == s->semantic && set == s->set)
        {
            return a;
        }
    }
    return MANI_MAX_MORPH_ATTRIBUTES;
}

// Where a target's deltas for one table entry begin, in floats from the
// deltas' start: past the entries below it that it holds. UINT64_MAX for
// a start in bytes changed since the open past what 64 bits hold.
static uint64_t SlotDeltas(const maniMorphTargets* m, const uint8_t* e, uint32_t slot)
{
    uint32_t mask = maniLoadU32(e + 4);
    uint64_t count = maniLoadU32(e + 8);
    uint64_t at = maniLoadU64(e + 24);
    MANI_ASSERT(slot < m->attributeCount);
    for (uint32_t a = 0; a < slot; ++a)
    {
        uint32_t semantic = maniMorphAttribute(m, a, NULL);
        uint64_t floats = ((mask >> a) & 1u) != 0 ? count * maniMorphWidth(semantic) : 0;
        at = at <= UINT64_MAX - floats ? at + floats : UINT64_MAX;
    }
    return at;
}

// The first of a sparse target's indices at or past vertex.
static uint32_t LowerBound(const uint8_t* indices, uint32_t count, uint32_t vertex)
{
    uint32_t low = 0;
    uint32_t high = count;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (maniLoadU32(indices + 4 * (size_t)mid) < vertex)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    return low;
}

// One stream's view of one target, read once so the stores through the
// output (bytes, which may alias anything) reload nothing: where vertex
// 0 of the range is written, the output stride, the deltas' stride, the
// floats a vertex takes and the weight.
typedef struct Apply
{
    uint8_t* out;
    size_t outStride;
    size_t stride;
    uint32_t floats;
    float weight;
} Apply;

static void AddFloat(uint8_t* out, const uint8_t* delta, float weight)
{
    float x = LoadF32(out) + LoadF32(delta) * weight;
    memcpy(out, &x, sizeof x);
}

static inline void AddDelta(Apply a, uint32_t i, const uint8_t* delta)
{
    uint8_t* out = a.out + (size_t)i * a.outStride;
    if (a.floats == 3)
    {
        // Written out: positions and normals, unrolled at every level.
        AddFloat(out, delta, a.weight);
        AddFloat(out + 4, delta + 4, a.weight);
        AddFloat(out + 8, delta + 8, a.weight);
        return;
    }
    for (uint32_t k = 0; k < a.floats; ++k)
    {
        AddFloat(out + 4 * k, delta + 4 * k, a.weight);
    }
}

// A sparse target's deltas from its index `from` to `stop` onto the range
// starting at first. Indices in bytes changed since the open may be out
// of order: one outside the range is held at its last vertex, within the
// stream.
static inline void AddSparse(Apply a, const uint8_t* deltas, const uint8_t* indices, uint32_t from,
                             uint32_t stop, uint32_t first, uint32_t last)
{
    for (uint32_t i = from; i < stop; ++i)
    {
        uint32_t at = maniLoadU32(indices + 4 * (size_t)i) - first;
        AddDelta(a, at < last ? at : last, deltas + a.stride * i);
    }
}

// A target's dense or sparse deltas over the range [first, end).
static MANI_NOINLINE void AddVertices(Apply a, const uint8_t* deltas, const uint8_t* indices,
                                      uint32_t count, uint32_t first, uint32_t end)
{
    if (indices == NULL)
    {
        for (uint32_t v = first; v < end; ++v)
        {
            AddDelta(a, v - first, deltas + a.stride * v);
        }
        return;
    }
    uint32_t from = LowerBound(indices, count, first);
    uint32_t stop = LowerBound(indices, count, end);
    uint32_t last = end - first - 1;
    // Positions and normals apart, so their loop knows its width: the
    // clamp would otherwise keep the compilers from taking the test out.
    if (a.floats == 3)
    {
        AddSparse(a, deltas, indices, from, stop, first, last);
        return;
    }
    AddSparse(a, deltas, indices, from, stop, first, last);
}

// One target's deltas onto one stream over the job's range.
static void AddTarget(const maniMorphJob* job, uint32_t t, const maniMorphStream* s, uint32_t slot)
{
    const maniMorphTargets* m = job->targets;
    const uint8_t* e = maniMorphTargetEntry(m, t);
    if (((maniLoadU32(e + 4) >> slot) & 1u) == 0)
    {
        return;
    }
    uint32_t width = maniMorphWidth(s->semantic);
    // JobValid took only streams of 2 to 4 components; a table entry's
    // width is that of its semantic.
    MANI_ASSERT(width > 0 && width <= 4 && s->components <= 4);
    Apply a = {(uint8_t*)s->out, s->outStride, 4 * (size_t)width,
               s->components < width ? s->components : width, job->weights[t]};
    bool dense = maniLoadU32(e + 12) == MANI_MORPH_DENSE;
    uint64_t index = maniLoadU64(e + 16);
    uint32_t count = maniLoadU32(e + 8);
    uint32_t end = job->first + job->count;
    MANI_ASSERT(end <= m->vertexCount);
    // In bytes changed since the open a target may claim deltas or indices
    // past their sections, or a dense count other than the vertices: such
    // a target adds nothing.
    uint64_t at = SlotDeltas(m, e, slot);
    uint64_t deltaBytes = maniMorphSectionSize(m, maniMorphTagDeltas);
    uint64_t indexBytes = maniMorphSectionSize(m, maniMorphTagIndices);
    if (at > deltaBytes / 4 || !maniViewFits(4 * at, (uint64_t)count * 4 * width, deltaBytes) ||
        (dense
             ? count != m->vertexCount
             : index > indexBytes / 4 || !maniViewFits(4 * index, 4 * (uint64_t)count, indexBytes)))
    {
        return;
    }
    const uint8_t* deltas = maniMorphSection(m, maniMorphTagDeltas) + 4 * at;
    const uint8_t* indices = dense ? NULL : maniMorphSection(m, maniMorphTagIndices) + 4 * index;
    AddVertices(a, deltas, indices, count, job->first, end);
}

// x, y, z to unit length; zero when the length is 0 or past binary32's
// range.
static void Renormalize(uint8_t* p)
{
    float v[3];
    memcpy(v, p, sizeof v);
    float squared = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    bool usable = squared > 0.0f && squared <= FLT_MAX;
    float inverse = usable ? 1.0f / sqrtf(squared) : 0.0f;
    for (int k = 0; k < 3; ++k)
    {
        v[k] = usable ? v[k] * inverse : 0.0f;
    }
    memcpy(p, v, sizeof v);
}

// A vertex's floats from in to out, read whole before written (as
// memmove, for a vertex that overlaps itself).
static void CopyVertex(uint8_t* out, const uint8_t* in, uint32_t floats)
{
    MANI_ASSERT(floats >= 2 && floats <= 4);
    float v[4];
    memcpy(v, in, 4 * (size_t)floats);
    memcpy(out, v, 4 * (size_t)floats);
}

// One stream over the range: copied, its attribute's targets added in
// order, directions renormalized. The stream's fields are read once: the
// stores go through bytes, which may alias them.
static void ApplyStream(const maniMorphJob* job, const maniMorphStream* s)
{
    uint8_t* out = (uint8_t*)s->out;
    const uint8_t* in = (const uint8_t*)s->in;
    size_t inStride = s->inStride;
    size_t outStride = s->outStride;
    uint32_t floats = s->components;
    uint32_t count = job->count;
    for (uint32_t i = 0; i < count && in != out; ++i)
    {
        if (floats == 3)
        {
            CopyVertex(out + (size_t)i * outStride, in + (size_t)i * inStride, 3);
        }
        else
        {
            CopyVertex(out + (size_t)i * outStride, in + (size_t)i * inStride, floats);
        }
    }
    uint32_t slot = Slot(job->targets, s);
    uint32_t targetCount = job->targets->targetCount;
    for (uint32_t t = 0; slot < MANI_MAX_MORPH_ATTRIBUTES && t < targetCount; ++t)
    {
        if (job->weights[t] != 0.0f)
        {
            AddTarget(job, t, s, slot);
        }
    }
    bool direction = s->semantic == maniMorphNormal || s->semantic == maniMorphTangent;
    for (uint32_t i = 0; i < count && direction; ++i)
    {
        Renormalize(out + (size_t)i * outStride);
    }
}

static MANI_NOINLINE maniResult ApplyMorphTargetsWork(const maniMorphJob* job)
{
    if (!JobValid(job))
    {
        return mani_errorInvalid;
    }
    for (uint32_t k = 0; k < job->streamCount; ++k)
    {
        ApplyStream(job, &job->streams[k]);
    }
    return mani_success;
}

maniResult maniApplyMorphTargets(const maniMorphJob* job)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = ApplyMorphTargetsWork(job);
    maniFloatEnvLeave(env);
    return result;
}
