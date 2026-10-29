// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model trainer's checkpoint restored: the
// container, its identity, every section's size against the shapes,
// every value finite, the moments and every input scale as training
// leaves them; then the trainer's progress replaced. The layout is
// learned_checkpoint.c's.

#include "allocator.h"
#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "learned_file.h"
#include "learned_train.h"
#include "train_net.h"
#include "train_random.h"

#include "maul-anim/base.h"
#include "maul-anim/learnedtrain.h"

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
    kTagNetworks = 16,
};

static const maniFileResult kOk = {mani_success, 0, 0};

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float x = 0.0f;
    memcpy(&x, &bits, sizeof x);
    return x;
}

// What a checkpoint holds, read and checked.
typedef struct Progress
{
    uint32_t stage;
    bool begun;
    uint64_t iteration;
    float loss;
    maniTrainRandom random;
    const uint8_t* latents;
    const uint8_t* training[4];
    const uint8_t* networks[MANI_LEARNED_PROJECTORS + MANI_MAX_LEARNED_PROJECTORS];
} Progress;

// A checkpoint is state, read only by a release of the ABI version that
// wrote it (MANI_ABI_VERSION): the whole version before 1.0, the major and
// minor version from 1.0 on. The container holds the writer's version.
static bool SameAbi(const uint8_t* b)
{
    bool release =
        maniLoadU16(b + 16) == MANI_VERSION_MAJOR && maniLoadU16(b + 18) == MANI_VERSION_MINOR;
#if MANI_VERSION_MAJOR == 0
    release = release && maniLoadU16(b + 20) == MANI_VERSION_PATCH;
#endif
    return release;
}

static maniFileResult ReadProgress(const maniLearnedTrainer* t, const maniContainer* c, Progress* p)
{
    const uint8_t* bytes = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    if (!maniContainerSection(c, kTagProgress, &bytes, &size, &at) ||
        size != MANI_CHECKPOINT_PROGRESS_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE);
    }
    p->iteration = maniLoadU64(bytes);
    for (uint32_t k = 0; k < 4; ++k)
    {
        p->random.s[k] = maniLoadU64(bytes + 8 + 8 * k);
    }
    p->stage = maniLoadU32(bytes + 40);
    uint32_t begun = maniLoadU32(bytes + 44);
    p->loss = LoadF32(bytes + 48);
    p->begun = begun == 1;
    bool working = p->begun && p->stage < t->stageCount;
    // A stage not begun: no loss or generator yet; one begun: a generator
    // not all 0 (xoshiro's one state that never leaves itself, on which
    // drawing below a bound would never end).
    bool fresh = (p->random.s[0] | p->random.s[1] | p->random.s[2] | p->random.s[3]) == 0;
    // The stage and whether it has begun first: the other fields' checks
    // depend on them.
    const bool valid[5] = {
        p->stage <= t->stageCount, begun <= 1 && (working || !p->begun),
        p->iteration <= t->source.options.iterations && (p->begun || p->iteration == 0),
        p->begun != fresh,
        fabsf(p->loss) < HUGE_VALF && (p->begun || maniLoadU32(bytes + 48) == 0)};
    const uint32_t offsets[5] = {40, 44, 0, 8, 48};
    for (uint32_t k = 0; k < 5; ++k)
    {
        if (!valid[k])
        {
            return Fail(at + offsets[k]);
        }
    }
    return kOk;
}

// A section of count floats, each finite; at least 0 when positive; the
// floats from first to first + nonZero not 0.
static maniFileResult Floats(const maniContainer* c, uint32_t tag, uint64_t count, bool positive,
                             uint64_t first, uint64_t nonZero, const uint8_t** out)
{
    uint64_t size = 0;
    uint64_t at = 0;
    if (!maniContainerSection(c, tag, out, &size, &at) || size != 4 * count)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE);
    }
    for (uint64_t k = 0; k < count; ++k)
    {
        float x = LoadF32(*out + 4 * k);
        bool zero = k >= first && k - first < nonZero && x == 0.0f;
        if (!(fabsf(x) < HUGE_VALF) || (positive && x < 0.0f) || zero)
        {
            return Fail(at + 4 * k);
        }
    }
    return kOk;
}

// Kept network n's shape: the Decompressor, the Stepper or a Projector.
static maniTrainNet KeptShape(const maniLearnedTrainer* t, uint32_t n)
{
    maniTrainNet net;
    (void)maniLearnedTrainerShape(t, n < MANI_LEARNED_PROJECTORS ? n + 1 : 3, &net);
    return net;
}

static uint64_t TrainingCount(const maniLearnedTrainer* t, uint32_t stage)
{
    if (stage != MANI_LEARNED_DECOMPRESSOR)
    {
        return KeptShape(t, stage).parameterCount;
    }
    maniTrainNet compressor;
    (void)maniLearnedTrainerShape(t, 0, &compressor);
    return compressor.parameterCount + KeptShape(t, 0).parameterCount;
}

static maniFileResult ReadSections(const maniLearnedTrainer* t, const maniContainer* c, Progress* p)
{
    uint32_t sections = 1 + (p->stage > 0 ? 1u : 0u) + (p->begun ? 4u : 0u) + p->stage;
    if (c->sectionCount != sections)
    {
        return Fail(12);
    }
    uint64_t frames = (uint64_t)t->data.frameCount * t->source.options.latentCount;
    maniFileResult r =
        p->stage > 0 ? Floats(c, kTagLatents, frames, false, 0, 0, &p->latents) : kOk;
    uint64_t count = p->begun ? TrainingCount(t, p->stage) : 0;
    for (uint32_t k = 0; r.result == mani_success && p->begun && k < 4; ++k)
    {
        r = Floats(c, kTagParameters + k, count, k >= 2, 0, 0, &p->training[k]);
    }
    for (uint32_t n = 0; r.result == mani_success && n < p->stage; ++n)
    {
        maniTrainNet net = KeptShape(t, n);
        uint64_t in = net.sizes[0];
        uint64_t kept = net.parameterCount + 2 * in + 2 * (uint64_t)net.sizes[net.layerCount];
        r = Floats(c, kTagNetworks + n, kept, false, net.parameterCount + in, in, &p->networks[n]);
    }
    return r;
}

// The checked progress made the trainer's; false when memory runs out.
static bool Apply(maniLearnedTrainer* t, const Progress* p)
{
    size_t frames = (size_t)t->data.frameCount * t->source.options.latentCount;
    if (p->stage > 0)
    {
        t->latents = maniAllocate(&t->allocator, frames + 1, sizeof(float), 8);
        if (t->latents == NULL)
        {
            return false;
        }
        memcpy(t->latents, p->latents, sizeof(float) * frames);
    }
    for (uint32_t n = 0; n < p->stage; ++n)
    {
        maniTrainNet net = KeptShape(t, n);
        if (!maniLearnedTrainerKeep(t, n, &net))
        {
            return false;
        }
        size_t kept = (size_t)net.parameterCount + 2 * (size_t)net.sizes[0] +
                      2 * (size_t)net.sizes[net.layerCount];
        memcpy(t->networks[n].parameters, p->networks[n], sizeof(float) * kept);
    }
    t->stage = p->stage;
    if (p->begun)
    {
        if (maniLearnedTrainerBegin(t) != mani_success)
        {
            return false;
        }
        float* arrays[4];
        uint64_t count = maniLearnedTrainerArrays(t, arrays);
        for (uint32_t k = 0; k < 4; ++k)
        {
            memcpy(arrays[k], p->training[k], sizeof(float) * (size_t)count);
        }
        t->random = p->random;
        t->iteration = p->iteration;
        t->loss = p->loss;
    }
    return true;
}

static MANI_NOINLINE maniFileResult RestoreLearnedCheckpointWork(const void* bytes, size_t size,
                                                                 void* out)
{
    maniLearnedTrainer* trainer = out;
    if (trainer != NULL && maniHostCalledBack(&trainer->host))
    {
        return (maniFileResult){mani_errorBusy, 0, 0};
    }
    if (trainer == NULL || bytes == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindLearnedCheckpoint,
                                         MANI_CHECKPOINT_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    if (!SameAbi(bytes))
    {
        return (maniFileResult){mani_errorVersion, 0, 16};
    }
    if (c.identity != trainer->signature)
    {
        return Fail(40);
    }
    Progress p;
    memset(&p, 0, sizeof p);
    r = ReadProgress(trainer, &c, &p);
    r = r.result == mani_success ? ReadSections(trainer, &c, &p) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    maniLearnedTrainerDrop(trainer);
    maniLearnedTrainerForget(trainer);
    trainer->stage = 0;
    trainer->iteration = 0;
    trainer->loss = 0.0f;
    if (!Apply(trainer, &p))
    {
        maniLearnedTrainerDrop(trainer);
        maniLearnedTrainerForget(trainer);
        trainer->stage = 0;
        return (maniFileResult){mani_errorCapacity, 0, 0};
    }
    return kOk;
}

maniFileResult maniLearnedTrainer_RestoreCheckpoint(maniLearnedTrainer* trainer, const void* bytes,
                                                    size_t size)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result =
        maniContainerOpenTagged(bytes, size, trainer, RestoreLearnedCheckpointWork);
    maniFloatEnvLeave(env);
    return result;
}
