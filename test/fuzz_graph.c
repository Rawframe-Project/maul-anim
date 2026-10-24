// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a graph file, and on success reading every
// parameter and clip slot; then the same bytes with the payload's hash
// made right, and then the identity too, so that mutations reach the
// checks behind them. Run as fuzz_graph <corpus dir>; the seed comes
// from test_graph --seed <file>.

#include "container.h"
#include "graph_file.h"

#include "maul-anim/graph.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniGraph graph;
    if (maniOpenGraph(bytes, size, &graph).result != 0)
    {
        return;
    }
    for (uint32_t p = 0; p < maniGraphParameterCount(&graph); ++p)
    {
        uint32_t length = 0;
        const char* name = maniGraphParameterName(&graph, p, &length);
        maniGraphParameterType type = 0;
        if (name == NULL || maniGraphParameterTypeOf(&graph, p, &type) != 0 ||
            type > maniGraphParameterPose || maniGraphFindParameter(&graph, name, length) > p)
        {
            abort();
        }
    }
    for (uint32_t c = 0; c < maniGraphClipCount(&graph); ++c)
    {
        uint32_t length = 0;
        if (maniGraphClipName(&graph, c, &length) == NULL || length == 0)
        {
            abort();
        }
    }
    if (graph.nodeCount == 0 || graph.slotCount == 0)
    {
        abort();
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_GRAPH_SECTIONS];
    uint64_t sizes[MANI_GRAPH_SECTIONS];
    for (uint32_t t = 1; t <= MANI_GRAPH_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniGraphComputeIdentity(sections, sizes);
    memcpy(bytes + 40, &identity, sizeof identity);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    // The file is opened from a 16-byte aligned copy, as a host would;
    // aligned by hand, as aligned_alloc is not everywhere.
    uint8_t* block = malloc(size + 16);
    if (block == NULL)
    {
        return 0;
    }
    uint8_t* copy = block + (16 - (uintptr_t)block % 16) % 16;
    if (size > 0)
    {
        memcpy(copy, data, size);
    }
    Exercise(copy, size);
    uint64_t fileSize = 0;
    if (size >= 64)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= 64 && fileSize <= size && fileSize - 64 < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + 64, (int32_t)(fileSize - 64));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
        FixIdentity(copy, size);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
