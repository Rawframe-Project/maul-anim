// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Node names (mani-0009), fixed per document. Repeats are found through a
// stable sort of the names' hashes: within a run of equal hashes, a name
// equal to the run's first is a repeat, and only a true hash collision
// falls back to comparing with each earlier name of the run.

#include "gltf_names.h"

#include "allocator.h"
#include "container.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"
#include "sort.h"

#include <string.h>

// "#" and a node index: at most 11 bytes.
#define MANI_SUFFIX_MAX 11u

static size_t WriteSuffix(char* out, uint32_t index)
{
    char digits[10];
    size_t n = 0;
    do
    {
        digits[n++] = (char)('0' + index % 10);
        index /= 10;
    } while (index != 0);
    out[0] = '#';
    for (size_t i = 0; i < n; ++i)
    {
        out[1 + i] = digits[n - 1 - i];
    }
    return n + 1;
}

typedef struct Work
{
    // Decoded original names, and the bone b's span in it.
    char* original;
    uint32_t* spans;
    uint64_t* keys;
    uint32_t* sorted;
    uint32_t* scratch;
    uint8_t* repeated;
} Work;

static bool SameName(const Work* w, uint32_t a, uint32_t b)
{
    uint32_t lengthA = w->spans[a + 1] - w->spans[a];
    uint32_t lengthB = w->spans[b + 1] - w->spans[b];
    return lengthA == lengthB &&
           memcmp(w->original + w->spans[a], w->original + w->spans[b], lengthA) == 0;
}

// Marks each named bone whose name an earlier bone (in the given order)
// has; unnamed bones have keys of their own and are renamed anyway.
static void FindRepeats(Work* w, uint32_t count)
{
    maniSortByKey(w->keys, count, w->sorted, w->scratch);
    uint32_t run = 0;
    for (uint32_t k = 1; k <= count; ++k)
    {
        if (k < count && w->keys[w->sorted[k]] == w->keys[w->sorted[run]])
        {
            continue;
        }
        // sorted[run] to sorted[k - 1]: one hash, in bone order.
        for (uint32_t i = run + 1; i < k; ++i)
        {
            uint32_t b = w->sorted[i];
            bool repeat = SameName(w, b, w->sorted[run]);
            for (uint32_t e = run + 1; !repeat && e < i; ++e)
            {
                repeat = SameName(w, b, w->sorted[e]);
            }
            w->repeated[b] = repeat ? 1 : 0;
        }
        run = k;
    }
}

static maniImportResult Decode(const maniGltf* gltf, const uint32_t* nodes, uint32_t count, Work* w)
{
    uint32_t at = 0;
    for (uint32_t b = 0; b < count; ++b)
    {
        uint32_t name = MANI_JSON_NONE;
        maniImportResult r = maniGltfMember(&gltf->g, gltf->nodeValues[nodes[b]], "name",
                                            maniJsonString, false, nodes[b], &name);
        if (r.result != mani_success)
        {
            return r;
        }
        w->spans[b] = at;
        w->repeated[b] = 0;
        if (name != MANI_JSON_NONE)
        {
            size_t raw = gltf->json.values[name].length;
            at +=
                (uint32_t)maniJsonStringDecode(&gltf->json, name, (uint8_t*)w->original + at, raw);
        }
        uint32_t length = at - w->spans[b];
        // An empty name never matches: its key is its bone's own.
        w->keys[b] = length > 0 ? maniHashBytes(MANI_HASH_INIT,
                                                (const uint8_t*)w->original + w->spans[b], length)
                                : ((uint64_t)1 << 63) | b;
    }
    w->spans[count] = at;
    return maniImportSuccess();
}

static void Compose(const uint32_t* nodes, uint32_t count, const Work* w, maniGltfNames* out)
{
    uint32_t at = 0;
    for (uint32_t b = 0; b < count; ++b)
    {
        uint32_t length = w->spans[b + 1] - w->spans[b];
        out->starts[b] = at;
        memcpy(out->text + at, w->original + w->spans[b], length);
        at += length;
        if (length == 0 || w->repeated[b] != 0)
        {
            at += (uint32_t)WriteSuffix(out->text + at, nodes[b]);
        }
    }
    out->starts[count] = at;
}

// Names the bones whose nodes are given, in that order.
static maniImportResult NameBones(const maniGltf* gltf, const uint32_t* nodes, uint32_t count,
                                  maniGltfNames* namesOut)
{
    *namesOut = (maniGltfNames){0};
    const maniAllocator* a = &gltf->allocator;
    // Decoded names are no longer than their JSON text.
    size_t raw = 0;
    for (uint32_t b = 0; b < count; ++b)
    {
        uint32_t name = maniJsonFind(&gltf->json, gltf->nodeValues[nodes[b]], "name", 4);
        raw += name != MANI_JSON_NONE ? gltf->json.values[name].length : 0;
    }
    size_t n = (size_t)count + 1;
    Work w = {
        .original = maniAllocate(a, raw + 1, 1, 8),
        .spans = maniAllocate(a, n, sizeof(uint32_t), 8),
        .keys = maniAllocate(a, n, sizeof(uint64_t), 8),
        .sorted = maniAllocate(a, n, sizeof(uint32_t), 8),
        .scratch = maniAllocate(a, n, sizeof(uint32_t), 8),
        .repeated = maniAllocate(a, n, 1, 8),
    };
    namesOut->textSize = raw + (size_t)count * MANI_SUFFIX_MAX + 1;
    namesOut->text = maniAllocate(a, namesOut->textSize, 1, 8);
    namesOut->starts = maniAllocate(a, n, sizeof(uint32_t), 8);
    namesOut->count = count;
    maniImportResult r = (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    if (w.original != NULL && w.spans != NULL && w.keys != NULL && w.sorted != NULL &&
        w.scratch != NULL && w.repeated != NULL && namesOut->text != NULL &&
        namesOut->starts != NULL)
    {
        r = Decode(gltf, nodes, count, &w);
        if (r.result == mani_success)
        {
            FindRepeats(&w, count);
            Compose(nodes, count, &w, namesOut);
        }
    }
    maniRelease(a, w.original, raw + 1, 1, 8);
    maniRelease(a, w.spans, n, sizeof(uint32_t), 8);
    maniRelease(a, w.keys, n, sizeof(uint64_t), 8);
    maniRelease(a, w.sorted, n, sizeof(uint32_t), 8);
    maniRelease(a, w.scratch, n, sizeof(uint32_t), 8);
    maniRelease(a, w.repeated, n, 1, 8);
    return r;
}

maniImportResult maniGltfReadNames(maniGltf* gltf)
{
    uint32_t* nodes =
        maniAllocate(&gltf->allocator, (size_t)gltf->nodeCount + 1, sizeof(uint32_t), 8);
    if (nodes == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    for (uint32_t v = 0; v < gltf->nodeCount; ++v)
    {
        nodes[v] = v;
    }
    maniImportResult r = NameBones(gltf, nodes, gltf->nodeCount, &gltf->names);
    maniRelease(&gltf->allocator, nodes, (size_t)gltf->nodeCount + 1, sizeof(uint32_t), 8);
    return r;
}

void maniGltfReleaseNames(maniGltf* gltf)
{
    maniGltfNames* names = &gltf->names;
    maniRelease(&gltf->allocator, names->text, names->textSize, 1, 8);
    maniRelease(&gltf->allocator, names->starts, (size_t)names->count + 1, sizeof(uint32_t), 8);
    *names = (maniGltfNames){0};
}

const char* maniGltfNodeName(const maniGltf* gltf, uint32_t node, uint32_t* lengthOut)
{
    const maniGltfNames* names = &gltf->names;
    *lengthOut = names->starts[node + 1] - names->starts[node];
    return names->text + names->starts[node];
}
