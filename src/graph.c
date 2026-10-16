// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph file's view (mani-0023): every section checked once as hostile
// input, then read in place.

#include "maul-anim/graph.h"

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_nodes.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint32_t maniGraphDataWords(uint32_t kind, uint32_t inputCount)
{
    switch (kind)
    {
    case maniGraphNodeClip:
        return MANI_GRAPH_CLIP_WORDS;
    case maniGraphNodeBlend:
        return 1 + 2 * inputCount;
    case maniGraphNodeMask:
        return MANI_GRAPH_MASK_WORDS;
    case maniGraphNodeAdditive:
        return inputCount > 0 ? 2 * (inputCount - 1) : 0;
    case maniGraphNodeSelect:
        return MANI_GRAPH_SELECT_WORDS;
    case maniGraphNodeTimeScale:
        return MANI_GRAPH_TIME_SCALE_WORDS;
    case maniGraphNodeSeek:
        return MANI_GRAPH_SEEK_WORDS;
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceBands:
        return MANI_GRAPH_SPACE_WORDS + 2 * inputCount;
    case maniGraphNodeBlendSpaceTriangles:
        return inputCount >= 3 ? MANI_GRAPH_SPACE_WORDS + 2 * inputCount +
                                     3 * MANI_GRAPH_MAX_TRIANGLES(inputCount)
                               : 0;
    default:
        return 0;
    }
}

uint32_t maniGraphInputRange(uint32_t kind, uint32_t* minOut)
{
    switch (kind)
    {
    case maniGraphNodeClip:
        *minOut = 0;
        return 0;
    case maniGraphNodeMask:
        *minOut = 2;
        return 2;
    case maniGraphNodeAdditive:
        *minOut = 2;
        return MANI_GRAPH_MAX_INPUTS;
    case maniGraphNodeBlend:
    case maniGraphNodeSelect:
        *minOut = 1;
        return MANI_GRAPH_MAX_INPUTS;
    case maniGraphNodeTimeScale:
    case maniGraphNodeSeek:
        *minOut = 1;
        return 1;
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceBands:
        *minOut = 1;
        return MANI_GRAPH_MAX_SPACE_POINTS;
    case maniGraphNodeBlendSpaceTriangles:
        *minOut = 3;
        return MANI_GRAPH_MAX_SPACE_POINTS;
    case maniGraphNodeMachine:
        *minOut = 1;
        return MANI_GRAPH_MAX_INPUTS;
    default:
        *minOut = 1;
        return 0;
    }
}

uint64_t maniGraphComputeIdentity(const uint8_t* const* sections, const uint64_t* sizes)
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_GRAPH_SECTIONS; ++i)
    {
        hash = maniHashBytes(hash, sections[i], sizes[i]);
    }
    return hash;
}

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static const maniFileResult kOk = {mani_success, 0, 0};

typedef struct Sections
{
    const uint8_t* data[MANI_GRAPH_SECTIONS];
    uint64_t sizes[MANI_GRAPH_SECTIONS];
    uint64_t at[MANI_GRAPH_SECTIONS];
    // Where each one's size is in the table; the section count for an
    // absent one.
    uint64_t entry[MANI_GRAPH_SECTIONS];
} Sections;

static const uint8_t kEmpty = 0;

static maniFileResult LoadSections(const maniContainer* c, Sections* s)
{
    for (uint32_t i = 0; i < MANI_GRAPH_SECTIONS; ++i)
    {
        s->data[i] = &kEmpty;
        s->sizes[i] = 0;
        s->at[i] = 0;
        s->entry[i] = 12;
    }
    for (uint32_t i = 0; i < c->sectionCount; ++i)
    {
        uint64_t entryAt = MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        const uint8_t* entry = c->bytes + entryAt;
        uint32_t tag = maniLoadU32(entry);
        if (tag == 0 || tag > MANI_GRAPH_SECTIONS)
        {
            return Fail(entryAt);
        }
        uint64_t size = maniLoadU64(entry + 16);
        if (size == 0)
        {
            return Fail(entryAt + 16);
        }
        uint64_t offset = maniLoadU64(entry + 8);
        s->data[tag - 1] = c->bytes + offset;
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = offset;
        s->entry[tag - 1] = entryAt + 16;
    }
    return kOk;
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// The header's counts, each within its limit; the offset of the first
// wrong one.
static maniFileResult CheckHeader(const Sections* s, maniGraph* g)
{
    if (s->sizes[0] != MANI_GRAPH_HEADER_SIZE)
    {
        return Fail(s->entry[0]);
    }
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    uint32_t counts[10];
    for (uint32_t k = 0; k < 10; ++k)
    {
        counts[k] = maniLoadU32(h + 8 + 4 * k);
    }
    // Bones, parameters, nodes, inputs, data words, clips, sync groups,
    // masks, slots, and the zero word.
    const uint32_t lows[10] = {1, 0, 1, 0, 0, 0, 0, 0, 1, 0};
    const uint32_t highs[10] = {MANI_MAX_BONES,
                                MANI_GRAPH_MAX_PARAMETERS,
                                MANI_GRAPH_MAX_NODES,
                                MANI_GRAPH_MAX_NODES * MANI_GRAPH_MAX_INPUTS,
                                UINT32_MAX,
                                MANI_GRAPH_MAX_CLIPS,
                                MANI_GRAPH_MAX_SYNC_GROUPS,
                                MANI_GRAPH_MAX_NODES,
                                MANI_GRAPH_MAX_NODES,
                                0};
    for (uint32_t k = 0; k < 10; ++k)
    {
        if (counts[k] < lows[k] || counts[k] > highs[k])
        {
            return Fail(at + 8 + 4 * k);
        }
    }
    *g = (maniGraph){.skeletonIdentity = maniLoadU64(h),
                     .skeletonBoneCount = counts[0],
                     .parameterCount = counts[1],
                     .nodeCount = counts[2],
                     .inputCount = counts[3],
                     .dataWords = counts[4],
                     .clipCount = counts[5],
                     .syncGroupCount = counts[6],
                     .maskCount = counts[7],
                     .slotCount = counts[8]};
    return kOk;
}

// Every section of the size the header's counts give.
static maniFileResult CheckSizes(const Sections* s, maniGraph* g)
{
    const uint64_t sizes[MANI_GRAPH_SECTIONS - 1] = {
        (uint64_t)g->parameterCount * MANI_GRAPH_PARAMETER_SIZE,
        (uint64_t)g->nodeCount * MANI_GRAPH_NODE_SIZE,
        4 * (uint64_t)g->inputCount,
        4 * (uint64_t)g->dataWords,
        (uint64_t)g->clipCount * MANI_GRAPH_CLIP_SIZE,
        (uint64_t)g->syncGroupCount * MANI_GRAPH_SYNC_GROUP_SIZE,
        4 * (uint64_t)g->maskCount * g->skeletonBoneCount,
    };
    for (uint32_t k = 0; k < MANI_GRAPH_SECTIONS - 2; ++k)
    {
        if (s->sizes[k + 1] != sizes[k])
        {
            return Fail(s->entry[k + 1]);
        }
    }
    g->parameters = s->data[1];
    g->nodes = s->data[2];
    g->inputs = s->data[3];
    g->data = s->data[4];
    g->clips = s->data[5];
    g->syncGroups = s->data[6];
    g->maskWeights = s->data[7];
    g->names = s->data[8];
    g->namesSize = s->sizes[8];
    return kOk;
}

// A name's offset and length (at entry) inside the names, UTF-8, not
// empty, with the hash at hashAt.
static bool NameValid(const maniGraph* g, const uint8_t* entry, const uint8_t* hashAt)
{
    uint64_t offset = maniLoadU32(entry);
    uint64_t length = maniLoadU32(entry + 4);
    if (length == 0 || offset > g->namesSize || length > g->namesSize - offset)
    {
        return false;
    }
    const uint8_t* name = g->names + offset;
    return maniValidUtf8(name, (size_t)length) &&
           maniHashBytes(MANI_HASH_INIT, name, length) == maniLoadU64(hashAt);
}

static bool Finite(const uint8_t* p)
{
    return (maniLoadU32(p) & 0x7F800000u) != 0x7F800000u;
}

// A parameter's default and bounds in form, its unused bytes zero.
static bool ValueValid(uint32_t type, bool bounded, const uint8_t* v)
{
    uint32_t used = type == maniGraphParameterVec2   ? 2u
                    : type == maniGraphParameterVec3 ? 3u
                    : type == maniGraphParameterQuat ? 4u
                                                     : 1u;
    for (uint32_t k = used; k < 4; ++k)
    {
        bool zero = maniLoadU32(v + 4 * k) == 0;
        if (!zero)
        {
            return false;
        }
    }
    bool zeroBounds = maniLoadU32(v + 16) == 0 && maniLoadU32(v + 20) == 0;
    if (type == maniGraphParameterBool || type == maniGraphParameterTrigger)
    {
        return maniLoadU32(v) <= 1u && zeroBounds;
    }
    if (type == maniGraphParameterInt)
    {
        int32_t d = (int32_t)maniLoadU32(v);
        int32_t low = (int32_t)maniLoadU32(v + 16);
        int32_t high = (int32_t)maniLoadU32(v + 20);
        return bounded ? low <= high && d >= low && d <= high : zeroBounds;
    }
    double length = 0.0;
    for (uint32_t k = 0; k < used; ++k)
    {
        float f = LoadF32(v + 4 * k);
        length += (double)f * (double)f;
        if (!Finite(v + 4 * k))
        {
            return false;
        }
    }
    if (type == maniGraphParameterQuat && fabs(length - 1.0) > 1e-5)
    {
        return false;
    }
    if (type != maniGraphParameterFloat || !bounded)
    {
        return zeroBounds;
    }
    float d = LoadF32(v);
    float low = LoadF32(v + 16);
    float high = LoadF32(v + 20);
    return Finite(v + 16) && Finite(v + 20) && low <= high && d >= low && d <= high;
}

static maniFileResult CheckParameters(const maniGraph* g, uint64_t at)
{
    for (uint32_t i = 0; i < g->parameterCount; ++i)
    {
        const uint8_t* e = g->parameters + (size_t)i * MANI_GRAPH_PARAMETER_SIZE;
        uint64_t here = at + (uint64_t)i * MANI_GRAPH_PARAMETER_SIZE;
        uint32_t type = maniLoadU32(e);
        uint32_t flags = maniLoadU32(e + 4);
        bool number = type == maniGraphParameterInt || type == maniGraphParameterFloat;
        if (type > maniGraphParameterTrigger)
        {
            return Fail(here);
        }
        if (flags > (number ? MANI_GRAPH_BOUNDED : 0u))
        {
            return Fail(here + 4);
        }
        if (!NameValid(g, e + 8, e + 16))
        {
            return Fail(here + 8);
        }
        if (!ValueValid(type, flags != 0, e + 24))
        {
            return Fail(here + 24);
        }
    }
    return kOk;
}

static maniFileResult CheckClips(const maniGraph* g, uint64_t at)
{
    for (uint32_t i = 0; i < g->clipCount; ++i)
    {
        const uint8_t* e = g->clips + (size_t)i * MANI_GRAPH_CLIP_SIZE;
        uint64_t here = at + (uint64_t)i * MANI_GRAPH_CLIP_SIZE;
        if (!NameValid(g, e + 16, e + 8))
        {
            return Fail(here + 16);
        }
        if (maniLoadU32(e + 24) > MANI_GRAPH_DIFFERENCE || maniLoadU32(e + 28) != 0)
        {
            return Fail(here + 24);
        }
    }
    return kOk;
}

static maniFileResult CheckSyncGroups(const maniGraph* g, uint64_t at)
{
    for (uint32_t i = 0; i < g->syncGroupCount; ++i)
    {
        const uint8_t* e = g->syncGroups + (size_t)i * MANI_GRAPH_SYNC_GROUP_SIZE;
        uint64_t here = at + (uint64_t)i * MANI_GRAPH_SYNC_GROUP_SIZE;
        if (!NameValid(g, e + 8, e))
        {
            return Fail(here + 8);
        }
        if (maniLoadU32(e + 16) > 1u || maniLoadU32(e + 20) != 0)
        {
            return Fail(here + 16);
        }
    }
    return kOk;
}

static maniFileResult CheckMaskWeights(const maniGraph* g, uint64_t at)
{
    uint64_t n = (uint64_t)g->maskCount * g->skeletonBoneCount;
    for (uint64_t k = 0; k < n; ++k)
    {
        float w = LoadF32(g->maskWeights + 4 * k);
        if (!(w >= 0.0f && w <= 1.0f))
        {
            return Fail(at + 4 * k);
        }
    }
    return kOk;
}

maniFileResult maniOpenGraph(const void* bytes, size_t size, maniGraph* graphOut)
{
    if (graphOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    Sections s;
    maniGraph g;
    r = LoadSections(&c, &s);
    r = r.result == mani_success ? CheckHeader(&s, &g) : r;
    r = r.result == mani_success ? CheckSizes(&s, &g) : r;
    r = r.result == mani_success ? CheckParameters(&g, s.at[1]) : r;
    r = r.result == mani_success ? CheckClips(&g, s.at[5]) : r;
    r = r.result == mani_success ? CheckSyncGroups(&g, s.at[6]) : r;
    r = r.result == mani_success ? CheckMaskWeights(&g, s.at[7]) : r;
    r = r.result == mani_success
            ? maniCheckGraphNodes(&g, s.at[2], s.at[3], s.at[4], &g.machineWords)
            : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniGraphComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    g.identity = c.identity;
    *graphOut = g;
    return kOk;
}

uint64_t maniGraphIdentity(const maniGraph* graph)
{
    return graph != NULL ? graph->identity : 0;
}

uint64_t maniGraphSkeletonIdentity(const maniGraph* graph)
{
    return graph != NULL ? graph->skeletonIdentity : 0;
}

uint32_t maniGraphNodeCount(const maniGraph* graph)
{
    return graph != NULL ? graph->nodeCount : 0;
}

uint32_t maniGraphParameterCount(const maniGraph* graph)
{
    return graph != NULL ? graph->parameterCount : 0;
}

static const char* EntryName(const maniGraph* graph, const uint8_t* entry, uint32_t* lengthOut)
{
    if (lengthOut != NULL)
    {
        *lengthOut = maniLoadU32(entry + 4);
    }
    return (const char*)(graph->names + maniLoadU32(entry));
}

uint32_t maniGraphFindParameter(const maniGraph* graph, const char* name, size_t length)
{
    if (graph == NULL || (name == NULL && length > 0))
    {
        return MANI_NO_PARAMETER;
    }
    uint64_t hash = maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length);
    for (uint32_t i = 0; i < graph->parameterCount; ++i)
    {
        const uint8_t* e = graph->parameters + (size_t)i * MANI_GRAPH_PARAMETER_SIZE;
        uint32_t found = 0;
        const char* text = EntryName(graph, e + 8, &found);
        if (maniLoadU64(e + 16) == hash && found == length &&
            (length == 0 || memcmp(text, name, length) == 0))
        {
            return i;
        }
    }
    return MANI_NO_PARAMETER;
}

maniResult maniGraphParameterTypeOf(const maniGraph* graph, uint32_t parameter,
                                    maniGraphParameterType* typeOut)
{
    if (graph == NULL || typeOut == NULL || parameter >= graph->parameterCount)
    {
        return mani_errorInvalid;
    }
    *typeOut = maniLoadU32(graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE);
    return mani_success;
}

const char* maniGraphParameterName(const maniGraph* graph, uint32_t parameter, uint32_t* lengthOut)
{
    if (graph == NULL || parameter >= graph->parameterCount)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return EntryName(graph, graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE + 8,
                     lengthOut);
}

uint32_t maniGraphClipCount(const maniGraph* graph)
{
    return graph != NULL ? graph->clipCount : 0;
}

uint64_t maniGraphClipIdentity(const maniGraph* graph, uint32_t clip)
{
    if (graph == NULL || clip >= graph->clipCount)
    {
        return 0;
    }
    return maniLoadU64(graph->clips + (size_t)clip * MANI_GRAPH_CLIP_SIZE);
}

const char* maniGraphClipName(const maniGraph* graph, uint32_t clip, uint32_t* lengthOut)
{
    if (graph == NULL || clip >= graph->clipCount)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return EntryName(graph, graph->clips + (size_t)clip * MANI_GRAPH_CLIP_SIZE + 16, lengthOut);
}
