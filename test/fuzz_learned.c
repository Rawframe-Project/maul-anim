// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a learned model (mani-0025), and on success reading its
// records and evaluating every network once (a projection of zeros, a
// step and a decompression of its state); then the same bytes with the
// payload's hash made right, and then the identity too, so that
// mutations reach the checks behind them. Run as fuzz_learned <corpus
// dir>; the seed comes from test_learned --seed <file>.

#include "container.h"
#include "learned_file.h"

#include "maul-anim/learned.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

// Evaluations of models past this many weights are skipped, so a run
// stays fast; opening is still checked in full.
#define MANI_FUZZ_WEIGHTS 65536u

static void Records(const maniLearnedModel* m)
{
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        maniMatchChannel ch;
        if (maniLearnedModelChannel(m, c, &ch) != 0 ||
            ch.firstFeature + ch.featureCount > m->featureCount)
        {
            abort();
        }
    }
    for (uint32_t p = 0; p < m->projectorCount; ++p)
    {
        if (maniLearnedModelProjector(m, p, NULL, NULL) != 0)
        {
            abort();
        }
    }
    for (uint32_t k = 0; k < m->floatTrackCount + m->eventCount; ++k)
    {
        uint32_t length = 0;
        const char* name = k < m->floatTrackCount
                               ? maniLearnedModelFloatTrackName(m, k, &length)
                               : maniLearnedModelEventName(m, k - m->floatTrackCount, &length);
        if (name == NULL || length == 0)
        {
            abort();
        }
    }
}

static void Evaluate(const maniLearnedModel* m)
{
    uint64_t weights = 0;
    for (uint32_t l = 0; l < m->layerCount; ++l)
    {
        uint64_t in = 0;
        uint64_t out = 0;
        memcpy(&in, m->layers + (size_t)l * MANI_LEARNED_LAYER_SIZE, 4);
        memcpy(&out, m->layers + (size_t)l * MANI_LEARNED_LAYER_SIZE + 4, 4);
        weights += in * out;
    }
    if (weights > MANI_FUZZ_WEIGHTS)
    {
        return;
    }
    size_t scratchSize = maniLearnedScratchSize(m);
    size_t stateSize = sizeof(float) * ((size_t)m->featureCount + m->latentCount);
    float* scratch = malloc(scratchSize);
    float* state = calloc(1, stateSize);
    float* query = calloc(m->featureCount, sizeof(float));
    if (scratch != NULL && state != NULL && query != NULL)
    {
        maniLearnedOutput out = {NULL, NULL, NULL, {0, 0, 0}, {0, 0, 0}};
        // A model's values may overflow to infinities: refusals then are
        // answers, not faults.
        (void)maniLearnedProject(m, 0, query, state, scratch, scratchSize);
        (void)maniLearnedStep(m, state, scratch, scratchSize);
        (void)maniLearnedDecompress(m, state, &out, scratch, scratchSize);
    }
    free(scratch);
    free(state);
    free(query);
}

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniLearnedModel m;
    if (maniOpenLearnedModel(bytes, size, &m).result != 0)
    {
        return;
    }
    Records(&m);
    Evaluate(&m);
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindLearnedModel, MANI_LEARNED_FORMAT_VERSION, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_LEARNED_SECTIONS];
    uint64_t sizes[MANI_LEARNED_SECTIONS];
    for (uint32_t t = 1; t <= MANI_LEARNED_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniLearnedComputeIdentity(sections, sizes);
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
