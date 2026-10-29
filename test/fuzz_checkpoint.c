// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Restoring a learned model trainer's checkpoint from any
// bytes into the trainer of checkpoint_host.h; then the same bytes with
// the payload's hash made right, and then the identity the trainer's
// signature too, so that mutations reach the checks behind them. A
// checkpoint restored saves back to the same sections, trains on and
// builds a model. Run as fuzz_checkpoint <corpus dir>; the seed comes
// from test_learned_train --seed <file>.

#include "bytes.h"
#include "checkpoint_host.h"
#include "container.h"
#include "learned_train.h"

#include "maul-anim/build.h"
#include "maul-anim/learnedtrain.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static CheckpointHost s_host;
static maniLearnedTrainer* s_trainer;

// Every section of a and b alike: their tags, sizes and bytes.
static bool SameSections(const uint8_t* a, size_t aSize, const uint8_t* b, size_t bSize)
{
    maniContainer ca;
    maniContainer cb;
    if (maniOpenContainer(a, aSize, maniKindLearnedCheckpoint, MANI_CHECKPOINT_FORMAT_VERSION, &ca)
                .result != mani_success ||
        maniOpenContainer(b, bSize, maniKindLearnedCheckpoint, MANI_CHECKPOINT_FORMAT_VERSION, &cb)
                .result != mani_success ||
        ca.sectionCount != cb.sectionCount || ca.identity != cb.identity)
    {
        return false;
    }
    for (uint32_t i = 0; i < ca.sectionCount; ++i)
    {
        uint32_t tag = maniLoadU32(a + MANI_CONTAINER_HEADER_SIZE + 24 * (size_t)i);
        const uint8_t* x = NULL;
        const uint8_t* y = NULL;
        uint64_t xSize = 0;
        uint64_t ySize = 0;
        uint64_t at = 0;
        if (!maniContainerSection(&ca, tag, &x, &xSize, &at) ||
            !maniContainerSection(&cb, tag, &y, &ySize, &at) || xSize != ySize ||
            memcmp(x, y, xSize) != 0)
        {
            return false;
        }
    }
    return true;
}

static void Exercise(const uint8_t* bytes, size_t size)
{
    if (maniLearnedTrainer_RestoreCheckpoint(s_trainer, bytes, size).result != mani_success)
    {
        return;
    }
    // Every value of an accepted checkpoint is checked: its sections save
    // back whole (the gaps between them are the container's).
    size_t saved = CheckpointSize(s_trainer);
    uint8_t* block = malloc(saved + 16);
    if (block == NULL)
    {
        return;
    }
    uint8_t* again = block + (16 - (uintptr_t)block % 16) % 16;
    if (maniLearnedTrainer_SaveCheckpoint(s_trainer, again, saved, NULL) != mani_success ||
        !SameSections(bytes, size, again, saved))
    {
        abort();
    }
    free(block);
    maniLearnedTrainerReport report;
    if (maniLearnedTrainer_Train(s_trainer, 2, &report) != mani_success)
    {
        abort();
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) == mani_success)
    {
        (void)maniBuildLearnedModelFromTrainer(builder, s_trainer, maniLearnedWeightsFloat32);
        maniDestroyBuilder(builder);
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_trainer == NULL)
    {
        s_trainer = MakeCheckpointHost(&s_host);
        if (s_trainer == NULL)
        {
            abort();
        }
    }
    // Read from a 16-byte aligned copy, aligned by hand.
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
    if (size >= MANI_CONTAINER_HEADER_SIZE)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= MANI_CONTAINER_HEADER_SIZE && fileSize <= size &&
        fileSize - MANI_CONTAINER_HEADER_SIZE < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + MANI_CONTAINER_HEADER_SIZE,
                                   (int32_t)(fileSize - MANI_CONTAINER_HEADER_SIZE));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
        memcpy(copy + 40, &s_trainer->signature, sizeof s_trainer->signature);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
