// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bone names, fixed per document. Repeats are
// found through a stable sort of the names' hashes: within a run of equal
// hashes, a name equal to the run's first is a repeat, and only a true
// hash collision falls back to comparing with each earlier name of the
// run.

#include "names.h"

#include "allocator.h"
#include "container.h"
#include "sort.h"

#include <stdbool.h>
#include <string.h>

// "#" and an index: at most 11 bytes.
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
    const char* original;
    const uint32_t* spans;
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

// Marks each named source whose name an earlier source has; unnamed
// sources have keys of their own and are renamed anyway.
static void FindRepeats(Work* w, uint32_t count)
{
    for (uint32_t b = 0; b < count; ++b)
    {
        uint32_t length = w->spans[b + 1] - w->spans[b];
        w->repeated[b] = 0;
        // An empty name never matches: its key is its source's own.
        w->keys[b] = length > 0 ? maniHashBytes(MANI_HASH_INIT,
                                                (const uint8_t*)w->original + w->spans[b], length)
                                : ((uint64_t)1 << 63) | b;
    }
    maniSortByKey(w->keys, count, w->sorted, w->scratch);
    uint32_t run = 0;
    for (uint32_t k = 1; k <= count; ++k)
    {
        if (k < count && w->keys[w->sorted[k]] == w->keys[w->sorted[run]])
        {
            continue;
        }
        // sorted[run] to sorted[k - 1]: one hash, in source order.
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

static void Compose(const uint32_t* indices, uint32_t count, const Work* w, maniBoneNames* out)
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
            at += (uint32_t)WriteSuffix(out->text + at, indices[b]);
            out->renamed[out->renamedCount++] = b;
        }
    }
    out->starts[count] = at;
}

maniImportResult maniNameBones(const maniAllocator* allocator, const char* original,
                               const uint32_t* spans, const uint32_t* indices, uint32_t count,
                               maniBoneNames* namesOut)
{
    *namesOut = (maniBoneNames){0};
    size_t n = (size_t)count + 1;
    Work w = {
        .original = original,
        .spans = spans,
        .keys = maniAllocate(allocator, n, sizeof(uint64_t), 8),
        .sorted = maniAllocate(allocator, n, sizeof(uint32_t), 8),
        .scratch = maniAllocate(allocator, n, sizeof(uint32_t), 8),
        .repeated = maniAllocate(allocator, n, 1, 8),
    };
    namesOut->textSize = (size_t)spans[count] + (size_t)count * MANI_SUFFIX_MAX + 1;
    namesOut->text = maniAllocate(allocator, namesOut->textSize, 1, 8);
    namesOut->starts = maniAllocate(allocator, n, sizeof(uint32_t), 8);
    namesOut->renamed = maniAllocate(allocator, n, sizeof(uint32_t), 8);
    namesOut->count = count;
    maniImportResult r = (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    if (w.keys != NULL && w.sorted != NULL && w.scratch != NULL && w.repeated != NULL &&
        namesOut->text != NULL && namesOut->starts != NULL && namesOut->renamed != NULL)
    {
        FindRepeats(&w, count);
        Compose(indices, count, &w, namesOut);
        r = (maniImportResult){mani_success, maniImportReasonNone, 0, 0};
    }
    maniRelease(allocator, w.keys, n, sizeof(uint64_t), 8);
    maniRelease(allocator, w.sorted, n, sizeof(uint32_t), 8);
    maniRelease(allocator, w.scratch, n, sizeof(uint32_t), 8);
    maniRelease(allocator, w.repeated, n, 1, 8);
    return r;
}

void maniReleaseBoneNames(const maniAllocator* allocator, maniBoneNames* names)
{
    maniRelease(allocator, names->text, names->textSize, 1, 8);
    maniRelease(allocator, names->starts, (size_t)names->count + 1, sizeof(uint32_t), 8);
    maniRelease(allocator, names->renamed, (size_t)names->count + 1, sizeof(uint32_t), 8);
    *names = (maniBoneNames){0};
}

const char* maniBoneName(const maniBoneNames* names, uint32_t b, uint32_t* lengthOut)
{
    *lengthOut = names->starts[b + 1] - names->starts[b];
    return names->text + names->starts[b];
}
