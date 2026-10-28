// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model trainer's checkpoint: a container of kind
// 10 whose identity is the trainer's signature. Sections:
//
//    1  progress: iterations (u64), the generator (4 u64), stage (u32),
//       whether it has begun (u32), the last loss (f32); the iterations,
//       generator and loss 0 while the stage has not begun
//    2  every frame's latent values, once the first stage has ended
//    3  the stage in training's parameters, 4 its first moments, 5 its
//       second moments, 6 their largest, while it trains
//   16 + n  network n's parameters and normalization (inputs' offsets
//       and scales, outputs' offsets and scales), once its stage ended
//
// The signature hashes the database's, skeleton's and clips'
// identities, the options, the Projectors' filters and the names, in a
// fixed little-endian layout.

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "learned_file.h"
#include "learned_train.h"
#include "train_decompress.h"
#include "train_net.h"
#include "train_random.h"
#include "train_solo.h"

#include "maul-anim/base.h"
#include "maul-anim/learnedtrain.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum
{
    kTagProgress = 1,
    kTagLatents = 2,
    kTagParameters = 3,
    kTagLargest = 6,
    kTagNetworks = 16,
};

static uint64_t HashU64(uint64_t hash, uint64_t value)
{
    uint8_t bytes[8];
    maniStoreU64(bytes, value);
    return maniHashBytes(hash, bytes, 8);
}

static uint64_t HashNames(uint64_t hash, const maniLearnedNameSource* names, uint32_t count)
{
    hash = HashU64(hash, count);
    for (uint32_t k = 0; k < count; ++k)
    {
        hash = HashU64(hash, names[k].length);
        hash = maniHashBytes(hash, (const uint8_t*)names[k].name, names[k].length);
    }
    return hash;
}

static uint32_t Bits(float x)
{
    uint32_t bits = 0;
    memcpy(&bits, &x, sizeof bits);
    return bits;
}

uint64_t maniLearnedTrainerSignature(const maniLearnedTrainerSource* s)
{
    const maniLearnedTrainerOptions* o = &s->options;
    const uint64_t values[14] = {maniMotionDatabaseIdentity(s->database),
                                 maniSkeletonIdentity(s->skeleton),
                                 o->latentCount,
                                 o->hiddenWidth,
                                 o->compressorLayers,
                                 o->decompressorLayers,
                                 o->stepperLayers,
                                 o->projectorLayers,
                                 o->batchSize,
                                 o->stepperWindow,
                                 o->iterations,
                                 o->seed,
                                 Bits(o->learningRate),
                                 Bits(o->eventSeconds)};
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t k = 0; k < 14; ++k)
    {
        hash = HashU64(hash, values[k]);
    }
    hash = HashU64(hash, s->clipCount);
    for (uint32_t k = 0; k < s->clipCount; ++k)
    {
        hash = HashU64(hash, maniMotionDatabaseClipIdentity(s->database, k));
    }
    hash = HashU64(hash, s->projectorCount);
    for (uint32_t p = 0; p < s->projectorCount; ++p)
    {
        hash =
            HashU64(hash, ((uint64_t)s->projectors[p].required << 32) | s->projectors[p].excluded);
    }
    hash = HashNames(hash, s->floatTracks, s->floatTrackCount);
    return HashNames(hash, s->events, s->eventCount);
}

// Network n's floats kept: parameters, then normalization.
static uint64_t KeptCount(const maniTrainNet* n)
{
    return n->parameterCount + 2 * (uint64_t)n->sizes[0] + 2 * (uint64_t)n->sizes[n->layerCount];
}

static uint32_t Sections(const maniLearnedTrainer* t, uint8_t* progress,
                         maniSectionSource sections[MANI_CONTAINER_MAX_SECTIONS])
{
    maniStoreU64(progress, t->iteration);
    // A stage not begun has no generator or loss yet: 0.
    for (uint32_t k = 0; k < 4; ++k)
    {
        maniStoreU64(progress + 8 + 8 * k, t->begun ? t->random.s[k] : 0u);
    }
    maniStoreU32(progress + 40, t->stage);
    maniStoreU32(progress + 44, t->begun ? 1u : 0u);
    maniStoreU32(progress + 48, t->begun ? Bits(t->loss) : 0u);
    uint32_t count = 0;
    sections[count++] = (maniSectionSource){kTagProgress, progress, MANI_CHECKPOINT_PROGRESS_SIZE};
    if (t->latents != NULL)
    {
        uint64_t latents = (uint64_t)t->data.frameCount * t->source.options.latentCount;
        sections[count++] = (maniSectionSource){kTagLatents, t->latents, 4 * latents};
    }
    float* arrays[4];
    uint64_t training = t->begun ? maniLearnedTrainerArrays(t, arrays) : 0;
    for (uint32_t k = 0; t->begun && k < 4; ++k)
    {
        sections[count++] = (maniSectionSource){kTagParameters + k, arrays[k], 4 * training};
    }
    for (uint32_t n = 0; n < t->stage; ++n)
    {
        const maniTrainKept* k = &t->networks[n];
        sections[count++] =
            (maniSectionSource){kTagNetworks + n, k->parameters, 4 * KeptCount(&k->net)};
    }
    return count;
}

size_t maniLearnedCheckpointSize(const maniLearnedTrainer* trainer)
{
    if (trainer == NULL)
    {
        return 0;
    }
    uint8_t progress[MANI_CHECKPOINT_PROGRESS_SIZE] = {0};
    maniSectionSource sections[MANI_CONTAINER_MAX_SECTIONS];
    uint32_t count = Sections(trainer, progress, sections);
    uint64_t size = 0;
    return maniContainerSize(sections, count, &size) && size <= SIZE_MAX ? (size_t)size : 0;
}

static MANI_NOINLINE maniResult SaveLearnedCheckpointWork(const maniLearnedTrainer* trainer,
                                                          void* bytes, size_t capacity)
{
    if (trainer == NULL || bytes == NULL)
    {
        return mani_errorInvalid;
    }
    uint8_t progress[MANI_CHECKPOINT_PROGRESS_SIZE] = {0};
    maniSectionSource sections[MANI_CONTAINER_MAX_SECTIONS];
    uint32_t count = Sections(trainer, progress, sections);
    uint64_t size = 0;
    if (!maniContainerSize(sections, count, &size) || size > capacity)
    {
        return mani_errorCapacity;
    }
    maniWriteContainer(bytes, maniKindLearnedCheckpoint, MANI_CHECKPOINT_FORMAT_VERSION,
                       trainer->signature, sections, count);
    return mani_success;
}

maniResult maniSaveLearnedCheckpoint(const maniLearnedTrainer* trainer, void* bytes,
                                     size_t capacity)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = SaveLearnedCheckpointWork(trainer, bytes, capacity);
    maniFloatEnvLeave(env);
    return result;
}
