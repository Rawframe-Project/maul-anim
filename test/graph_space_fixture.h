// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The blend space suites' helpers (mani-0023): weights read from an
// instance against those wanted, a position set from parameters x and
// y, a source built and its refusal read, and a built file with one data
// word changed, opened and refused at that word. Included by one suite
// each.

#ifndef MAUL_ANIM_TEST_GRAPH_SPACE_FIXTURE_H
#define MAUL_ANIM_TEST_GRAPH_SPACE_FIXTURE_H

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_fixture.h"

#include <stdio.h>

static inline bool Weights(maniGraphInstance* instance, const maniGraph* g, const float* want,
                           uint32_t count)
{
    bool ok = true;
    for (uint32_t k = 0; k < count; ++k)
    {
        ok = ok && fabs((double)State(instance, g, k)->weight - (double)want[k]) <= 1e-6;
    }
    if (!ok)
    {
        for (uint32_t k = 0; k < count; ++k)
        {
            printf("  weight %u: %.9g, want %.9g\n", k, (double)State(instance, g, k)->weight,
                   (double)want[k]);
        }
    }
    return ok;
}

static inline bool AtPoint(maniGraphInstance* instance, const maniGraph* g, float x, float y,
                           const float* want, uint32_t count)
{
    Set(instance, 0, maniGraphParameterFloat, Float(x));
    Set(instance, 1, maniGraphParameterFloat, Float(y));
    Update(instance, 0.0f);
    return Weights(instance, g, want, count);
}

static inline maniBuildResult BuildOnly(const maniGraphNodeSource* nodes, uint32_t count,
                                        const maniGraphParameterSource* parameters,
                                        uint32_t parameterCount)
{
    maniGraphSource source = {&s_skeleton, parameters, kSlots, NULL, nodes, parameterCount,
                              CLIPS,       0,          count,  0,    0,     NULL,
                              0,           0,          NULL,   NULL};
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, &source);
    maniDestroyBuilder(builder);
    return r;
}

static inline bool Refused(maniBuildResult r, maniBuildReason reason, uint32_t other)
{
    return r.result == mani_errorInvalid && r.reason == reason && r.item == 0 && r.other == other;
}

static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;

static inline void Rehash(uint8_t* file)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, s_size - 64));
    maniContainer c;
    CHECK(maniOpenContainer(file, s_size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
              mani_success,
          "container");
    const uint8_t* sections[MANI_GRAPH_SECTIONS];
    uint64_t sizes[MANI_GRAPH_SECTIONS];
    for (uint32_t t = 1; t <= MANI_GRAPH_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = file;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    maniStoreU64(file + 40, maniGraphComputeIdentity(sections, sizes));
}

// The file of the graph last made, its node p's data word k set to value,
// opened; the data word's file offset at k in atOut.
static inline maniFileResult Crafted(const maniGraph* g, uint32_t p, uint32_t k, uint32_t value,
                                     uint64_t* atOut)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    // Make built s_file; its size is in the container's header.
    s_size = (size_t)maniLoadU64(s_file + 24);
    CHECK(maniOpenContainer(s_file, s_size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
                  mani_success &&
              maniContainerSection(&c, maniGraphTagData, &data, &size, &at),
          "the data");
    uint32_t first = maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + at + 4 * (uint64_t)(first + k), value);
    Rehash(s_crafted);
    *atOut = at + 4 * (uint64_t)first;
    maniGraph crafted;
    return maniOpenGraph(s_crafted, s_size, &crafted);
}

// Refused at data word want.
static inline bool RefusedAtWordOf(const maniGraph* g, uint32_t p, uint32_t k, uint32_t value,
                                   uint32_t want)
{
    uint64_t at = 0;
    maniFileResult r = Crafted(g, p, k, value, &at);
    uint64_t offset = at + 4 * (uint64_t)want;
    if (r.result != mani_errorFormat || r.offset != offset)
    {
        printf("  refused at %llu, expected %llu\n", (unsigned long long)r.offset,
               (unsigned long long)offset);
    }
    return r.result == mani_errorFormat && r.offset == offset;
}

// Refused at the word changed.
static inline bool RefusedAtWord(const maniGraph* g, uint32_t p, uint32_t k, uint32_t value)
{
    return RefusedAtWordOf(g, p, k, value, k);
}

// Node p's data word k in the graph last made.
static inline uint32_t DataWord(const maniGraph* g, uint32_t p, uint32_t k)
{
    uint32_t first = maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    return maniLoadU32(g->data + 4 * (size_t)(first + k));
}

static inline uint32_t Bits(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

#endif // MAUL_ANIM_TEST_GRAPH_SPACE_FIXTURE_H
