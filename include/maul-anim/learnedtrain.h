// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Training learned motion matching models in the library,
// from a motion database, its clips and skeleton: Holden's networks,
// losses and optimizer, the same bits on every platform, with any thread
// count and across checkpoints.

#ifndef MAUL_ANIM_LEARNEDTRAIN_H
#define MAUL_ANIM_LEARNEDTRAIN_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A trainer: the networks of one model in training. Training,
    // restoring and destroying refuse a call back in from the trainer's
    // allocator or task hooks with mani_errorState and count one misuse.
    typedef struct maniLearnedTrainer maniLearnedTrainer;

    typedef struct maniLearnedTrainerDef
    {
        uint32_t cookie;
        // MANI_ABI_VERSION, set by maniDefaultLearnedTrainerDef.
        uint32_t version;
        // The allocator the trainer uses; zeroed for the C library's.
        maniAllocator allocator;
        // The host's tasks a batch's chunks run on; zeroed for the
        // calling thread alone. The result is the same either way.
        maniTaskHooks tasks;
    } maniLearnedTrainerDef;

// The cookie maniDefaultLearnedTrainerDef sets and
// maniCreateLearnedTrainer checks.
#define MANI_LEARNED_TRAINER_DEF_COOKIE 0x4D4C5444u

// Named limits: samples a batch, frames a Stepper's window.
#define MANI_LEARNED_MAX_BATCH  65536u
#define MANI_LEARNED_MAX_WINDOW 65536u

    /// Returns a trainer def with every setting at its default.
    ///
    /// @return The def, with a valid cookie.
    /// @par Thread safety
    /// Safe from any thread.
    static inline maniLearnedTrainerDef maniDefaultLearnedTrainerDef(void)
    {
#ifdef __cplusplus
        maniLearnedTrainerDef def = {};
#else
    maniLearnedTrainerDef def = {0};
#endif
        def.cookie = MANI_LEARNED_TRAINER_DEF_COOKIE;
        def.version = MANI_ABI_VERSION;
        return def;
    }

    // Training's options (Holden's by default): the latent count, the
    // hidden layers' width, each network's hidden layer count, samples a
    // batch (a multiple of 4), the Stepper's window in frames, iterations
    // each trained stage runs, the seed, the learning rate (decayed by
    // 0.99 every 1,000 iterations), and the seconds an event's state
    // stays 1 after it.
    typedef struct maniLearnedTrainerOptions
    {
        uint32_t latentCount;
        uint32_t hiddenWidth;
        uint32_t compressorLayers;
        uint32_t decompressorLayers;
        uint32_t stepperLayers;
        uint32_t projectorLayers;
        uint32_t batchSize;
        uint32_t stepperWindow;
        uint64_t iterations;
        uint64_t seed;
        float learningRate;
        float eventSeconds;
    } maniLearnedTrainerOptions;

    /// Returns Holden's options.
    ///
    /// @return The options.
    /// @par Thread safety
    /// Safe from any thread.
    static inline maniLearnedTrainerOptions maniDefaultLearnedTrainerOptions(void)
    {
        maniLearnedTrainerOptions o = {32, 512, 3, 1, 2, 4, 32, 20, 500000, 1234, 0.001f, 0.1f};
        return o;
    }

    // A Projector's tag filter: its targets are frames of entries with all
    // of required and none of excluded.
    typedef struct maniLearnedTagFilter
    {
        uint32_t required;
        uint32_t excluded;
    } maniLearnedTagFilter;

    typedef struct maniLearnedTrainerSource
    {
        const maniSkeleton* skeleton;
        const maniMotionDatabase* database;
        // The database's clips, in its order.
        const maniClip* const* clips;
        const maniLearnedTagFilter* projectors;
        const maniLearnedNameSource* floatTracks;
        const maniLearnedNameSource* events;
        uint32_t clipCount;
        uint32_t projectorCount;
        uint32_t floatTrackCount;
        uint32_t eventCount;
        maniLearnedTrainerOptions options;
    } maniLearnedTrainerSource;

    // A trainer's progress: the stage in training (0 the Compressor and
    // Decompressor, 1 the Stepper, 2 + p Projector p; the stage count
    // once every stage is done), the stage count, the iterations the
    // stage has run, and its last batch's loss (0 before its first).
    typedef struct maniLearnedTrainerReport
    {
        uint32_t stage;
        uint32_t stageCount;
        uint64_t iteration;
        float loss;
        uint32_t reserved;
    } maniLearnedTrainerReport;

    /// Makes a trainer: checks the def and source, and samples the
    /// training data from the database's clips at its frames. The
    /// source's objects and arrays must outlive the trainer.
    ///
    /// @param def        The def.
    /// @param source     What to train on, and how.
    /// @param trainerOut Receives the trainer, or NULL on failure.
    /// @return `mani_success`; `mani_errorVersion` for a def built
    /// against another major or minor version; `mani_errorInvalid` for a
    /// NULL argument, a def without its cookie, an allocator or task
    /// hooks with one function, a list missing, or with the reason and
    /// item for a refused source: `maniBuildReasonOption` (item the
    /// option's place in the options: a count, width or layer count past
    /// its limit, a batch not a multiple of 4 or past
    /// MANI_LEARNED_MAX_BATCH, a window below 2, past
    /// MANI_LEARNED_MAX_WINDOW or longer than every entry, no iterations,
    /// a rate or seconds not finite or negative; item 1 for networks past
    /// MANI_LEARNED_MAX_WEIGHTS), `maniBuildReasonDatabase` (a database
    /// of another skeleton), `maniBuildReasonClip` (item the clip whose
    /// identity is not the database's; the clip count and the database's
    /// as item and other when they differ), `maniBuildReasonTiming` (a
    /// database rate past MANI_LEARNED_MAX_RATE), `maniBuildReasonCount`
    /// (no Projector, or a list past its limit), `maniBuildReasonTags`
    /// (item the Projector), `maniBuildReasonFloatTrack` or
    /// `maniBuildReasonEvent` combined with `maniBuildReasonName` or
    /// `maniBuildReasonDuplicateName`; `mani_errorCapacity` when memory
    /// runs out.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniBuildResult maniCreateLearnedTrainer(
        const maniLearnedTrainerDef* def, const maniLearnedTrainerSource* source,
        maniLearnedTrainer** trainerOut);

    /// Destroys a trainer. Called back in from the trainer's allocator or
    /// task hooks it destroys nothing and counts one misuse.
    ///
    /// @param trainer  The trainer, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the trainer is used by one thread at a time.
    MANI_API void maniDestroyLearnedTrainer(maniLearnedTrainer* trainer);

    /// The misuses counted on a trainer: calls back in from its allocator
    /// or task hooks, which are refused with mani_errorState.
    ///
    /// @param trainer  A trainer, or NULL (0).
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniLearnedTrainerMisuses(const maniLearnedTrainer* trainer);

    /// Trains on: up to iterations iterations, stage after stage. A stage
    /// whose iterations are all done ends (its network kept, the rest of
    /// its memory released) and the next begins when it first runs; the
    /// first stage's end gives every frame's latent values.
    ///
    /// @param trainer    The trainer.
    /// @param iterations Iterations to run at most; 0 only reports.
    /// @param reportOut  Receives the progress.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorCapacity` when memory runs out as a stage begins or
    /// ends (the trainer keeps its progress, and a later call goes on).
    /// @par Thread safety
    /// Safe from any thread; the trainer is used by one thread at a time.
    /// Its chunks run on the def's task hooks.
    MANI_NODISCARD MANI_API maniResult maniTrainLearned(maniLearnedTrainer* trainer,
                                                        uint64_t iterations,
                                                        maniLearnedTrainerReport* reportOut);

    /// Returns the size of a checkpoint of the trainer as it is now.
    ///
    /// @param trainer  The trainer.
    /// @return The size in bytes; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread; the trainer is used by one thread at a time.
    MANI_API size_t maniLearnedCheckpointSize(const maniLearnedTrainer* trainer);

    /// Writes a checkpoint of the trainer: its progress, generator,
    /// networks and AdamW moments, under the signature of its source and
    /// options.
    ///
    /// @param trainer  The trainer.
    /// @param bytes    Receives the checkpoint.
    /// @param capacity The size of bytes.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorCapacity` when bytes holds less than
    /// maniLearnedCheckpointSize.
    /// @par Thread safety
    /// Safe from any thread; the trainer is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSaveLearnedCheckpoint(const maniLearnedTrainer* trainer,
                                                                 void* bytes, size_t capacity);

    /// Restores a checkpoint into a trainer of the same source and
    /// options, replacing its progress; training on gives the same bits
    /// as training through.
    ///
    /// @param trainer  The trainer.
    /// @param bytes    The checkpoint, 16-byte aligned.
    /// @param size     Its size.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong (the identity's, 40, for a checkpoint of another
    /// source or options); `mani_errorVersion` for a newer format;
    /// `mani_errorCapacity` when memory runs out (the trainer then starts
    /// over).
    /// @par Thread safety
    /// Safe from any thread; the trainer is used by one thread at a time.
    MANI_NODISCARD MANI_API maniFileResult maniRestoreLearnedCheckpoint(maniLearnedTrainer* trainer,
                                                                        const void* bytes,
                                                                        size_t size);

    /// Builds the model a trainer has trained, every stage done, through
    /// maniBuildLearnedModel: its step rate the database's sample rate.
    ///
    /// @param builder      The builder.
    /// @param trainer      The trainer.
    /// @param weightFormat How the file keeps the weights.
    /// @return As maniBuildLearnedModel; `mani_errorInvalid` for a NULL
    /// argument or a trainer with a stage to run.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    /// The trainer is not trained meanwhile.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildLearnedModelFromTrainer(
        maniBuilder* builder, const maniLearnedTrainer* trainer, maniLearnedWeights weightFormat);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_LEARNEDTRAIN_H
