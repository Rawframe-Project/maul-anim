// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion databases: a walk (its hips' travel root motion,
// looping), a run (its hips moving in the pose) and a turn (the root
// walking and turning a quarter in a second, a float track rising),
// sampled at 10 frames a second: each frame's features, denormalized,
// against the bone's model position, its velocity (a looping entry's a
// frame either side, a clamped one's nearer at its ends), the root's
// travel and facing at times ahead and behind (wrapped in the looping
// entry, clamped in the others) and the float track; each channel's
// normalized features of mean 0 and mean deviation its weight; frames
// mapped to entries and times; the channels and entries read back; the
// builder's refusals and its leaf sizes; opening's checks of every
// section, the tree's order and boxes recomputed, and of the version. Run as test_match --seed
// <file> it writes the database, a seed of fuzz_match.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "graph_fixture.h"
#include "match_file.h"
#include "source_padding.h"
#include "vecmath.h"

#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static _Alignas(16) uint8_t s_turnFile[8192];
static maniClip s_turn;

// The root walks along x and turns a quarter about y in a second (both
// root motion), and a float track "speed" rises from 0 to 1.
static void OpenTurn(void)
{
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kWalk[6] = {0, 1, 0, 1, 1, 0};
    static const float kTurn[8] = {0, 0, 0, 1, 0, 0.70710678f, 0, 0.70710678f};
    static const float kSpeed[2] = {0.0f, 1.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const maniRootMotionSource kRoot = {0, maniRootAxisX, maniRootRotationTwist, 1, 0, 0, 0};
    const maniBoneTrackSource track = {0,
                                       {kTimes, kWalk, 2, maniInterpolationLinear},
                                       {kTimes, kTurn, 2, maniInterpolationLinear},
                                       kNone};
    const maniFloatTrackSource speed = {"speed", 5, {kTimes, kSpeed, 2, maniInterpolationLinear}};
    maniClipSource source = {.skeleton = &s_skeleton,
                             .boneTracks = &track,
                             .boneTrackCount = 1,
                             .floatTracks = &speed,
                             .floatTrackCount = 1,
                             .loop = maniLoopClamp,
                             .sampleRate = 10.0f,
                             .rootMotion = &kRoot};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_turnFile, sizeof s_turnFile, &size) == mani_success &&
              maniOpenClip(s_turnFile, size, &s_turn).result == mani_success,
          "the turn");
    maniDestroyBuilder(builder);
}

static const float kNow[1] = {0.0f};
static const float kAround[2] = {0.5f, -0.2f};
static const float kAhead[1] = {0.5f};

// Clips: the walk, the run, the turn.
static const maniClip* s_clipList[3];

static maniMatchChannelSource Channel(maniMatchChannelKind kind, uint32_t bone, const float* times,
                                      uint32_t count, float weight)
{
    return (maniMatchChannelSource){kind, bone, 0, count, times, NULL, 0, weight};
}

static maniMatchChannelSource s_channels[4];
static maniMatchEntrySource s_entries[3];

// The walk looping over its second, the run and the turn clamped to
// theirs; the hips' position and velocity now, the root's travel half a
// second ahead and a fifth behind, its facing half a second ahead
// (weight 2).
static maniMotionDatabaseSource Source(void)
{
    s_clipList[0] = &s_clips[0];
    s_clipList[1] = &s_clips[1];
    s_clipList[2] = &s_turn;
    s_channels[0] = Channel(maniMatchBonePosition, 0, kNow, 1, 1.0f);
    s_channels[1] = Channel(maniMatchBoneVelocity, 0, kNow, 1, 1.0f);
    s_channels[2] = Channel(maniMatchTrajectoryPosition, 0, kAround, 2, 1.0f);
    s_channels[3] = Channel(maniMatchTrajectoryFacing, 0, kAhead, 1, 2.0f);
    s_entries[0] = (maniMatchEntrySource){0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u};
    s_entries[1] = (maniMatchEntrySource){1, 0, 0.0f, 1.0f, 0.5f, 2u};
    s_entries[2] = (maniMatchEntrySource){2, MANI_MATCH_NO_JUMP, 0.0f, 1.0f, 0.0f, 4u};
    return (maniMotionDatabaseSource){
        &s_skeleton, s_clipList, s_channels,     s_entries,      3,    4,
        3,           10.0f,      maniMatchAxisY, maniMatchAxisZ, NULL, 0};
}

static _Alignas(16) uint8_t s_database[1 << 16];
static size_t s_size;

static maniBuildResult Build(const maniMotionDatabaseSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildMotionDatabase(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_database, sizeof s_database, &s_size) == mani_success,
              "copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static float s_offsets[MANI_MAX_MATCH_FEATURES];
static float s_scales[MANI_MAX_MATCH_FEATURES];

// Frame f's feature j, denormalized.
static double Raw(const maniMotionDatabase* d, uint32_t f, uint32_t j)
{
    float features[MANI_MAX_MATCH_FEATURES];
    CHECK(maniMotionDatabaseFeatures(d, f, features, d->featureCount, NULL) == mani_success,
          "features");
    return (double)features[j] * (double)s_scales[j] + (double)s_offsets[j];
}

static bool Near4(double a, double b)
{
    return fabs(a - b) <= 1e-4;
}

static _Alignas(32) uint8_t s_poseMemory[4096];

// The root's move in the turn from t over step, clamped to its second.
static maniRootMove TurnMove(float t, float step)
{
    float to = t + step < 0.0f ? 0.0f : (t + step > 1.0f ? 1.0f : t + step);
    maniRootMove move = maniRootMoveIdentity();
    CHECK(maniClipRootMotion(&s_turn, t, to - t, &move) == mani_success, "a move");
    return move;
}

// The hips' model position in clip c at time t.
static maniVec3 Hips(const maniClip* clip, float t, maniPose* pose)
{
    CHECK(maniSampleClip(clip, &s_skeleton, t, pose) == mani_success &&
              maniComputeModel(pose, &s_skeleton) == mani_success,
          "sampled");
    return maniPoseModelTransform(pose, 0).translation;
}

// The walk's frames: the hips held, moving 1 a second with the root;
// the root half a second ahead and a fifth behind, through its loop.
static bool WalkFrame(const maniMotionDatabase* d, uint32_t f, maniPose* pose)
{
    maniVec3 h = Hips(&s_clips[0], (float)f / 10.0f, pose);
    return Near4(Raw(d, f, 0), (double)h.x) && Near4(Raw(d, f, 1), (double)h.y) &&
           Near4(Raw(d, f, 3), 1.0) && Near4(Raw(d, f, 4), 0.0) && Near4(Raw(d, f, 6), 0.5) &&
           Near4(Raw(d, f, 7), 0.0) && Near4(Raw(d, f, 8), -0.2) && Near4(Raw(d, f, 10), 0.0) &&
           Near4(Raw(d, f, 11), 1.0);
}

// The run's frames: the hips moving 3 a second in the pose (clamped
// differences at its ends), the root still.
static bool RunFrame(const maniMotionDatabase* d, uint32_t f, maniPose* pose)
{
    float t = (float)(f - 10) / 10.0f;
    maniVec3 h = Hips(&s_clips[1], t, pose);
    return Near4(Raw(d, f, 0), (double)h.x) && Near4(Raw(d, f, 3), 3.0) &&
           Near4(Raw(d, f, 6), 0.0) && Near4(Raw(d, f, 8), 0.0) && Near4(Raw(d, f, 11), 1.0);
}

// The turn's frames: the root's travel and facing its clip's moves.
static bool TurnFrame(const maniMotionDatabase* d, uint32_t f)
{
    float t = (float)(f - 21) / 10.0f;
    maniRootMove ahead = TurnMove(t, 0.5f);
    maniRootMove behind = TurnMove(t, -0.2f);
    maniVec3 facing = maniQuatRotate(ahead.rotation, (maniVec3){0.0f, 0.0f, 1.0f});
    return Near4(Raw(d, f, 6), (double)ahead.translation.x) &&
           Near4(Raw(d, f, 7), (double)ahead.translation.z) &&
           Near4(Raw(d, f, 8), (double)behind.translation.x) &&
           Near4(Raw(d, f, 9), (double)behind.translation.z) &&
           Near4(Raw(d, f, 10), (double)facing.x) && Near4(Raw(d, f, 11), (double)facing.z);
}

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniMotionDatabase motionDatabase;
    return maniOpenMotionDatabase(bytes, size, &motionDatabase);
}

static void TestFeatures(void)
{
    maniMotionDatabaseSource source = Source();
    maniMotionDatabase d;
    CHECK(Build(&source).result == mani_success &&
              maniOpenMotionDatabase(s_database, s_size, &d).result == mani_success,
          "built and opened");
    CHECK(TestOtherVersionsRefused(s_database, s_size, OpenFile), "another format version refused");
    CHECK(d.frameCount == 32 && d.featureCount == 12 && d.channelCount == 4 && d.timeCount == 5 &&
              d.entryCount == 3 && d.clipCount == 3 && d.skeletonBoneCount == 3,
          "the counts: 10 looping frames, 11 and 11 clamped");
    CHECK(maniMotionDatabaseNormalization(&d, s_offsets, s_scales, d.featureCount, NULL) ==
              mani_success,
          "the scales");
    uint32_t measured[2] = {0, 0};
    uint32_t refused[2] = {0, 0};
    float row[12];
    float offsets[12];
    memset(row, 0x7F, sizeof row);
    memset(offsets, 0x7F, sizeof offsets);
    float kept = row[0];
    CHECK(maniMotionDatabaseFeatures(&d, 0, NULL, 0, &measured[0]) == mani_errorCapacity &&
              measured[0] == 12 &&
              maniMotionDatabaseFeatures(&d, 0, row, 11, &refused[0]) == mani_errorCapacity &&
              refused[0] == 12 && memcmp(&row[0], &kept, sizeof kept) == 0 &&
              maniMotionDatabaseFeatures(&d, 0, NULL, 1, NULL) == mani_errorInvalid,
          "the features measured, a short array refused (nothing written)");
    CHECK(maniMotionDatabaseNormalization(&d, NULL, NULL, 0, &measured[1]) == mani_errorCapacity &&
              measured[1] == 12 &&
              maniMotionDatabaseNormalization(&d, offsets, NULL, 11, &refused[1]) ==
                  mani_errorCapacity &&
              refused[1] == 12 && memcmp(&offsets[0], &kept, sizeof kept) == 0 &&
              maniMotionDatabaseNormalization(&d, NULL, NULL, 1, NULL) == mani_errorInvalid &&
              maniMotionDatabaseNormalization(&d, NULL, s_scales, 12, NULL) == mani_success,
          "the normalization measured, a short array refused (nothing written)");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "a pose");
    bool walk = true;
    bool run = true;
    bool turn = true;
    for (uint32_t f = 0; f < 10; ++f)
    {
        walk = walk && WalkFrame(&d, f, pose);
    }
    for (uint32_t f = 10; f < 21; ++f)
    {
        run = run && RunFrame(&d, f, pose);
    }
    for (uint32_t f = 21; f < 32; ++f)
    {
        turn = turn && TurnFrame(&d, f);
    }
    CHECK(walk, "the walk's frames");
    CHECK(run, "the run's frames");
    CHECK(turn, "the turn's frames");
}

// Each channel's normalized features: mean 0 per feature, their
// deviations' mean the channel's weight.
static void TestNormalization(void)
{
    maniMotionDatabaseSource source = Source();
    maniMotionDatabase d;
    CHECK(Build(&source).result == mani_success &&
              maniOpenMotionDatabase(s_database, s_size, &d).result == mani_success,
          "built and opened");
    static const uint32_t kFirst[5] = {0, 3, 6, 10, 12};
    static const double kWeight[4] = {1.0, 1.0, 1.0, 2.0};
    bool ok = true;
    for (uint32_t c = 0; c < 4; ++c)
    {
        double spread = 0.0;
        for (uint32_t j = kFirst[c]; j < kFirst[c + 1]; ++j)
        {
            double sum = 0.0;
            double squares = 0.0;
            for (uint32_t f = 0; f < d.frameCount; ++f)
            {
                float v[MANI_MAX_MATCH_FEATURES];
                CHECK(maniMotionDatabaseFeatures(&d, f, v, d.featureCount, NULL) == mani_success,
                      "features");
                sum += (double)v[j];
                squares += (double)v[j] * (double)v[j];
            }
            double mean = sum / d.frameCount;
            ok = ok && fabs(mean) < 1e-5;
            spread += sqrt(squares / d.frameCount - mean * mean);
        }
        ok = ok && fabs(spread / (kFirst[c + 1] - kFirst[c]) - kWeight[c]) < 1e-4;
    }
    CHECK(ok, "each channel normalized to its weight");
}

static void TestReadBack(void)
{
    maniMotionDatabaseSource source = Source();
    maniMotionDatabase d;
    CHECK(Build(&source).result == mani_success &&
              maniOpenMotionDatabase(s_database, s_size, &d).result == mani_success,
          "built and opened");
    maniMatchChannel ch;
    CHECK(maniMotionDatabaseChannel(&d, 2, &ch) == mani_success &&
              ch.kind == maniMatchTrajectoryPosition && ch.timeCount == 2 && ch.times[0] == 0.5f &&
              ch.times[1] == -0.2f && ch.firstFeature == 6 && ch.featureCount == 4 &&
              ch.weight == 1.0f,
          "a channel");
    CHECK(maniMotionDatabaseChannel(&d, 4, &ch) == mani_errorInvalid &&
              maniMotionDatabaseChannelName(&d, 0, NULL) == NULL,
          "past the channels; no name");
    maniMatchEntry e;
    CHECK(maniMotionDatabaseEntry(&d, 1, &e) == mani_success && e.clip == 1 && e.flags == 0 &&
              e.costBias == 0.5f && e.tags == 2u && e.firstFrame == 10 && e.frameCount == 11,
          "an entry");
    uint32_t entry = 0;
    float time = 0.0f;
    CHECK(maniMotionDatabaseFrame(&d, 9, &entry, &time) == mani_success && entry == 0 &&
              time == 0.9f && maniMotionDatabaseFrame(&d, 10, &entry, &time) == mani_success &&
              entry == 1 && time == 0.0f &&
              maniMotionDatabaseFrame(&d, 31, &entry, &time) == mani_success && entry == 2 &&
              time == 1.0f && maniMotionDatabaseFrame(&d, 32, &entry, &time) == mani_errorInvalid,
          "frames to entries and times");
    CHECK(maniMotionDatabaseClipIdentity(&d, 2) == maniClipIdentity(&s_turn) &&
              maniMotionDatabaseClipIdentity(&d, 3) == 0 &&
              maniMotionDatabaseSkeletonIdentity(&d) == maniSkeletonIdentity(&s_skeleton) &&
              maniMotionDatabaseIdentity(&d) != 0,
          "identities");
}

// A float track's value, and its name read back.
static void TestFloatTrack(void)
{
    s_clipList[0] = &s_turn;
    maniMatchChannelSource channel = {maniMatchFloatTrack, 0, 0, 1, kNow, "speed", 5, 1.0f};
    maniMatchEntrySource entry = {0, 0, 0.0f, 1.0f, 0.0f, 0u};
    maniMotionDatabaseSource source = {&s_skeleton, s_clipList,     &channel,       &entry, 1, 1, 1,
                                       10.0f,       maniMatchAxisY, maniMatchAxisZ, NULL,   0};
    maniMotionDatabase d;
    CHECK(Build(&source).result == mani_success &&
              maniOpenMotionDatabase(s_database, s_size, &d).result == mani_success &&
              maniMotionDatabaseNormalization(&d, s_offsets, s_scales, d.featureCount, NULL) ==
                  mani_success,
          "built and opened");
    bool ok = true;
    for (uint32_t f = 0; f < 11; ++f)
    {
        ok = ok && Near4(Raw(&d, f, 0), (double)f / 10.0);
    }
    size_t length = 0;
    const char* name = maniMotionDatabaseChannelName(&d, 0, &length);
    CHECK(ok && name != NULL && length == 5 && memcmp(name, "speed", 5) == 0,
          "the float track rising, its name");
}

static bool RefusedWith(const maniMotionDatabaseSource* s, maniResult result,
                        maniBuildReason reason, uint32_t item, uint32_t other)
{
    maniBuildResult r = Build(s);
    if (!(r.result == result && r.reason == reason && r.item == item && r.other == other))
    {
        printf("  refused: result %d reason %u item %u other %u\n", (int)r.result, r.reason, r.item,
               r.other);
        return false;
    }
    return true;
}

static bool Refused(const maniMotionDatabaseSource* s, maniBuildReason reason, uint32_t item,
                    uint32_t other)
{
    return RefusedWith(s, mani_errorInvalid, reason, item, other);
}

// Refused for passing a fixed bound.
static bool PastLimit(const maniMotionDatabaseSource* s, maniBuildReason reason, uint32_t item,
                      uint32_t other)
{
    return RefusedWith(s, mani_errorLimit, reason, item, other);
}

static void TestSourceRefusals(void)
{
    maniMotionDatabaseSource s = Source();
    s.sampleRate = 0.0f;
    CHECK(Refused(&s, maniBuildReasonTiming, 0, 0), "no rate");
    s = Source();
    s.sampleRate = 1001.0f;
    CHECK(PastLimit(&s, maniBuildReasonTiming, 0, 0), "too fast a rate");
    s = Source();
    s.forward = maniMatchAxisNegativeY;
    CHECK(Refused(&s, maniBuildReasonOption, 0, 0), "forward along up");
    s = Source();
    s.up = 6;
    CHECK(Refused(&s, maniBuildReasonOption, 0, 0), "an unknown up");
    s = Source();
    s.channelCount = 0;
    CHECK(Refused(&s, maniBuildReasonCount, 0, 0), "no channels");
    s = Source();
    s.entryCount = 0;
    CHECK(Refused(&s, maniBuildReasonCount, 0, 0), "no entries");
    s = Source();
    s_clipList[1] = &s_clips[2];
    CHECK(Refused(&s, maniBuildReasonJoint, 1, 0), "an additive clip");
}

static void TestChannelRefusals(void)
{
    static const float kFar[1] = {61.0f};
    static const float kNan[1] = {NAN};
    static const float kMany[17] = {0};
    const maniMatchChannelSource kBad[13] = {
        {6, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchBonePosition, 3, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchBoneFacing, 0, 6, 1, kNow, NULL, 0, 1.0f},
        {maniMatchBonePosition, 0, 0, 0, kNow, NULL, 0, 1.0f},
        {maniMatchBonePosition, 0, 0, 17, kMany, NULL, 0, 1.0f},
        {maniMatchBonePosition, 0, 0, 1, kFar, NULL, 0, 1.0f},
        {maniMatchBonePosition, 0, 0, 1, kNan, NULL, 0, 1.0f},
        {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 0.0f},
        {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, HUGE_VALF},
        {maniMatchFloatTrack, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchFloatTrack, 0, 0, 1, kNow, "\xff", 1, 1.0f},
        {maniMatchFloatTrack, 0, 0, 1, kNow, "a", (size_t)MANI_MAX_NAME_BYTES + 1, 1.0f},
        {maniMatchFloatTrack, 0, 0, 1, kNow, "a", SIZE_MAX, 1.0f}};
    for (int k = 0; k < 13; ++k)
    {
        maniMotionDatabaseSource s = Source();
        s_channels[3] = kBad[k];
        // 17 times, a time of 61 seconds and track names past
        // MANI_MAX_NAME_BYTES pass a bound, SIZE_MAX bytes too: the names'
        // total leaves it to the channel.
        CHECK(k == 4 || k == 5 || k >= 11 ? PastLimit(&s, maniBuildReasonChannel, 3, 0)
                                          : Refused(&s, maniBuildReasonChannel, 3, 0),
              "a channel refused");
    }
#if SIZE_MAX > UINT32_MAX
    {
        // Past 32 bits a track name's length would pass the names' total
        // without wrapping it: still left to its channel.
        maniMotionDatabaseSource s = Source();
        s_channels[3] = (maniMatchChannelSource){maniMatchFloatTrack, 0,   0, 1, kNow, "a",
                                                 (size_t)1 << 40,     1.0f};
        CHECK(PastLimit(&s, maniBuildReasonChannel, 3, 0), "a track name of 2^40 bytes");
    }
#endif
    maniMotionDatabaseSource s = Source();
    s_channels[3] = (maniMatchChannelSource){maniMatchFloatTrack, 0, 0, 1, kNow, "speed", 5, 1.0f};
    CHECK(Refused(&s, maniBuildReasonMissingTrack, 3, 0), "the walk lacks the track");
    s = Source();
    s.entryCount = 2;
    CHECK(Refused(&s, maniBuildReasonSpread, 3, 0), "no turning: the facing never varies");
    static maniMatchChannelSource wide[64];
    for (uint32_t c = 0; c < 64; ++c)
    {
        wide[c] = (maniMatchChannelSource){maniMatchBonePosition, 0, 0, 16, kMany, NULL, 0, 1.0f};
    }
    s = Source();
    s.channels = wide;
    s.channelCount = 64;
    CHECK(PastLimit(&s, maniBuildReasonCount, 0, 0), "past the features");
    s.channelCount = 65;
    CHECK(PastLimit(&s, maniBuildReasonCount, 0, 0), "past the channels");
}

// Channels may share one track's name, so names a few times 2^26 bytes
// long reach the 32-bit totals: the clip names its two tracks by 2^26 + 1
// and 2^26 - 1 bytes of one buffer, and 64 channels share one of them.
static void TestNameTotals(void)
{
    const size_t longer = ((size_t)1 << 26) + 1;
    char* name = malloc(longer);
    memset(name, 'a', longer);
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kValues[2] = {0.0f, 1.0f};
    static const float kWalk[6] = {0, 1, 0, 1, 1, 0};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    const maniBoneTrackSource track = {
        0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone};
    const maniFloatTrackSource tracks[2] = {
        {name, longer, {kTimes, kValues, 2, maniInterpolationLinear}},
        {name, longer - 2, {kTimes, kValues, 2, maniInterpolationLinear}}};
    const maniClipSource clipSource = {.skeleton = &s_skeleton,
                                       .boneTracks = &track,
                                       .boneTrackCount = 1,
                                       .floatTracks = tracks,
                                       .floatTrackCount = 2,
                                       .loop = maniLoopClamp,
                                       .sampleRate = 10.0f};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, &clipSource).result == mani_success &&
              maniCopyBuilt(builder, NULL, 0, &size) == mani_errorCapacity,
          "the long names' clip");
    uint8_t* raw = malloc(size + 16);
    uint8_t* file = raw + (16 - (uintptr_t)raw % 16) % 16;
    maniClip clip;
    CHECK(maniCopyBuilt(builder, file, size, &size) == mani_success &&
              maniOpenClip(file, size, &clip).result == mani_success,
          "the long names' clip opens");
    maniDestroyBuilder(builder);
    const maniClip* clips[1] = {&clip};
    maniMatchChannelSource channels[MANI_MAX_MATCH_CHANNELS];
    for (uint32_t c = 0; c < MANI_MAX_MATCH_CHANNELS; ++c)
    {
        channels[c] =
            (maniMatchChannelSource){maniMatchFloatTrack, 0, 0, 1, kNow, name, longer, 1.0f};
    }
    const maniMatchEntrySource entry = {0, 0, 0.0f, 1.0f, 0.0f, 0u};
    maniMotionDatabaseSource source = {
        &s_skeleton, clips, channels,       &entry,         1,    MANI_MAX_MATCH_CHANNELS,
        1,           10.0f, maniMatchAxisY, maniMatchAxisZ, NULL, 0};
    // 64 names of 2^26 + 1 bytes pass the 32 bits the file's offsets have.
    CHECK(Refused(&source, maniBuildReasonCount, 0, 0), "track names past 4 GiB in all");
#if SIZE_MAX == UINT32_MAX
    // 64 of 2^26 - 1 bytes fit the offsets, but the sections with them pass
    // what one allocation holds where size_t has 32 bits.
    for (uint32_t c = 0; c < MANI_MAX_MATCH_CHANNELS; ++c)
    {
        channels[c].trackLength = longer - 2;
    }
    builder = NewBuilder();
    CHECK(maniBuildMotionDatabase(builder, &source).result == mani_errorCapacity,
          "sections past an allocation's room");
    maniDestroyBuilder(builder);
#endif
    free(raw);
    free(name);
}

static void TestEntryRefusals(void)
{
    const maniMatchEntrySource kBad[7] = {
        {3, 0, 0.0f, 1.0f, 0.0f, 0u},  {0, 8, 0.0f, 1.0f, 0.0f, 0u},
        {0, 0, -0.1f, 1.0f, 0.0f, 0u}, {0, 0, 0.0f, 1.5f, 0.0f, 0u},
        {0, 0, 0.6f, 0.5f, 0.0f, 0u},  {0, MANI_MATCH_LOOP, 0.5f, 0.5f, 0.0f, 0u},
        {0, 0, 0.0f, 1.0f, NAN, 0u}};
    for (int k = 0; k < 7; ++k)
    {
        maniMotionDatabaseSource s = Source();
        s_entries[1] = kBad[k];
        CHECK(Refused(&s, maniBuildReasonEntry, 1, 0), "an entry refused");
    }
    maniMotionDatabaseSource s = Source();
    s_entries[1].flags = MANI_MATCH_MIRRORED;
    CHECK(Refused(&s, maniBuildReasonMirrorDatabase, 1, 0), "a mirrored entry without a map");
    s = Source();
    s_entries[1] = (maniMatchEntrySource){1, 0, 0.5f, 0.5f, 0.0f, 0u};
    CHECK(Build(&s).result == mani_success, "a clamped entry of one frame");
    // 4 / 30 in binary32 is a hair above it in binary64: the frame at
    // 4 / 30 rounds to the end itself, so a loop of it holds 4.
    uint32_t looped = 0;
    uint32_t clamped = 0;
    CHECK(maniMatchEntryFrames(0.0f, 4.0f / 30.0f, 30.0f, true, &looped) && looped == 4 &&
              maniMatchEntryFrames(0.0f, 4.0f / 30.0f, 30.0f, false, &clamped) && clamped == 5,
          "frames judged by their binary32 times");
    // Past 2^24 seconds a frame's step no longer moves its time: one
    // frame more at most, at once.
    uint32_t far = 0;
    CHECK(maniMatchEntryFrames(1.0e9f, 1.0e9f, 30.0f, false, &far) && far == 2, "a count at once");
    // A span of MANI_MAX_MATCH_FRAMES frames or more has no count.
    uint32_t none = 7;
    CHECK(!maniMatchEntryFrames(0.0f, (float)MANI_MAX_MATCH_FRAMES / 30.0f, 30.0f, true, &none) &&
              none == 7,
          "a span too long for a count");
}

static void TestMisuse(void)
{
    maniBuilder* builder = NewBuilder();
    maniMotionDatabaseSource s = Source();
    s.skeleton = NULL;
    maniBuildResult a = maniBuildMotionDatabase(builder, NULL);
    maniBuildResult b = maniBuildMotionDatabase(builder, &s);
    s = Source();
    s_clipList[2] = NULL;
    maniBuildResult c = maniBuildMotionDatabase(builder, &s);
    s = Source();
    s.entries = NULL;
    maniBuildResult e = maniBuildMotionDatabase(builder, &s);
    CHECK(a.result == mani_errorInvalid && b.result == mani_errorInvalid &&
              c.result == mani_errorInvalid && e.result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == 4 &&
              maniBuildMotionDatabase(NULL, &s).result == mani_errorInvalid,
          "misuses counted");
    maniDestroyBuilder(builder);
}

static _Alignas(16) uint8_t s_crafted[1 << 16];

// The built file with the word at offset changed, its hash and identity
// made right, opened.
static maniFileResult Crafted(uint64_t offset, uint32_t value)
{
    memcpy(s_crafted, s_database, s_size);
    maniStoreU32(s_crafted + offset, value);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniContainer c;
    CHECK(
        maniOpenContainer(s_crafted, s_size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION, &c)
                .result == mani_success,
        "a container");
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_MATCH_SECTIONS];
    uint64_t sizes[MANI_MATCH_SECTIONS];
    for (uint32_t t = 1; t <= MANI_MATCH_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    maniStoreU64(s_crafted + 40, maniMatchComputeIdentity(sections, sizes));
    maniMotionDatabase d;
    return maniOpenMotionDatabase(s_crafted, s_size, &d);
}

static uint64_t Section(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(
        maniOpenContainer(s_database, s_size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION, &c)
                    .result == mani_success &&
            maniContainerSection(&c, tag, &data, &size, &at),
        "a section");
    return at;
}

static bool RefusedAt(uint64_t offset, uint32_t value, uint64_t want)
{
    maniFileResult r = Crafted(offset, value);
    if (r.result != mani_errorFormat || r.offset != want)
    {
        printf("  refused at %llu (%d), expected %llu\n", (unsigned long long)r.offset,
               (int)r.result, (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want &&
           r.tag == TestTagAt(s_crafted, s_size, want);
}

static uint32_t BitsOf(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

static void TestOpenRefusals(void)
{
    maniMotionDatabaseSource source = Source();
    CHECK(Build(&source).result == mani_success, "built");
    uint64_t header = Section(maniMatchTagHeader);
    uint64_t channels = Section(maniMatchTagChannels);
    uint64_t times = Section(maniMatchTagTimes);
    uint64_t entries = Section(maniMatchTagEntries);
    uint64_t normal = Section(maniMatchTagNormalization);
    uint64_t features = Section(maniMatchTagFeatures);
    uint64_t boxes = Section(maniMatchTagBoxes);
    uint64_t nodes = Section(maniMatchTagNodes);
    uint64_t positions = Section(maniMatchTagPositions);
    uint64_t places = Section(maniMatchTagPlaces);
    // 32 frames in leaves of 16: the root (its second half node 2) and
    // two leaves; 12 features. The frames at positions 0 and 1.
    uint32_t first = maniLoadU32(s_database + positions);
    uint32_t second = maniLoadU32(s_database + positions + 8);
    uint32_t entry = maniLoadU32(s_database + positions + 4);
    CHECK(maniLoadU32(s_database + header + 52) == 3 && maniLoadU32(s_database + nodes) == 2,
          "a root and two leaves");
    const uint64_t kCases[][3] = {{header + 8, 0, header + 8},
                                  {header + 36, 0, header + 36},
                                  {header + 44, maniMatchAxisNegativeY, header + 44},
                                  {channels + 40 + 0, 6, channels + 40},
                                  {channels + 40 + 4, 3, channels + 44},
                                  {channels + 40 + 8, 1, channels + 48},
                                  {channels + 40 + 16, 0, channels + 56},
                                  {channels + 40 + 28, BitsOf(0.0f), channels + 68},
                                  {channels + 40 + 32, 1, channels + 72},
                                  {times + 4, BitsOf(61.0f), times + 4},
                                  {entries + 32 + 0, 3, entries + 32},
                                  {entries + 32 + 4, 4, entries + 36},
                                  {entries + 32 + 12, BitsOf(1.5f), entries + 60},
                                  {entries + 32 + 16, BitsOf(NAN), entries + 48},
                                  {entries + 32 + 24, 9, entries + 56},
                                  {normal + 4 * 12, BitsOf(0.0f), normal + 4 * 12},
                                  {normal + 4, BitsOf(HUGE_VALF), normal + 4},
                                  {features + 8, BitsOf(NAN), features + 8},
                                  {header + 48, 12, header + 48},
                                  {header + 52, 4, header + 52},
                                  {positions, 32, positions},
                                  {positions, second, places + 4 * (uint64_t)second},
                                  {places + 4 * (uint64_t)first, 5, places + 4 * (uint64_t)first},
                                  {positions + 4, (entry + 1) % 3, positions + 4},
                                  {nodes, 1, nodes},
                                  {nodes + 12, 2, nodes + 12},
                                  {nodes + 12 + 4, 31, nodes + 12 + 4},
                                  {nodes + 12 + 8, BitsOf(7.0f), nodes + 12 + 8},
                                  {nodes + 4, 7, nodes + 4},
                                  {nodes + 8, BitsOf(7.0f), nodes + 8},
                                  {boxes + 4 * 24, BitsOf(-9.0f), boxes + 4 * 24},
                                  {boxes + 4 * 36 + 12, BitsOf(9.0f), boxes + 4 * 36 + 12},
                                  {boxes + 4 * 2, BitsOf(-9.0f), boxes + 4 * 2},
                                  {boxes + 4 * 12 + 4, BitsOf(9.0f), boxes + 4 * 12 + 4}};
    for (size_t k = 0; k < sizeof kCases / sizeof kCases[0]; ++k)
    {
        CHECK(RefusedAt(kCases[k][0], (uint32_t)kCases[k][1], kCases[k][2]),
              "a database refused at its byte");
    }
    // An entry too long for a count, its stored count UINT32_MAX: refused
    // at the count, whatever the count.
    memcpy(s_crafted, s_database, s_size);
    maniStoreU32(s_crafted + entries + 32 + 12, BitsOf(1.0e9f));
    maniStoreU32(s_crafted + entries + 32 + 28, UINT32_MAX);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniMotionDatabase overlong;
    maniFileResult refused = maniOpenMotionDatabase(s_crafted, s_size, &overlong);
    CHECK(refused.result == mani_errorFormat && refused.offset == entries + 32 + 28,
          "an entry too long for a count refused at its count");
    if (refused.offset != entries + 32 + 28)
    {
        printf("  refused at %llu, expected %llu\n", (unsigned long long)refused.offset,
               (unsigned long long)(entries + 32 + 28));
    }
    memcpy(s_crafted, s_database, s_size);
    s_crafted[40] ^= 1u;
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniMotionDatabase d;
    maniFileResult r = maniOpenMotionDatabase(s_crafted, s_size, &d);
    CHECK(r.result == mani_errorFormat && r.offset == 40, "an identity not its sections'");
    CHECK(maniOpenMotionDatabase(s_database, s_size, NULL).result == mani_errorInvalid,
          "no view to fill");
}

// Any other format version, the earlier one with the run bounds among
// them, is refused at the version before anything else is read.
static void TestOtherVersions(void)
{
    maniMotionDatabaseSource source = Source();
    CHECK(Build(&source).result == mani_success, "built");
    const uint16_t versions[3] = {1, 2, MANI_MATCH_FORMAT_VERSION + 1};
    for (uint32_t k = 0; k < 3; ++k)
    {
        memcpy(s_crafted, s_database, s_size);
        maniStoreU16(s_crafted + 6, versions[k]);
        maniMotionDatabase d;
        maniFileResult r = maniOpenMotionDatabase(s_crafted, s_size, &d);
        CHECK(r.result == mani_errorVersion && r.offset == 6, "another version refused");
    }
}

// The leaf sizes' query (near frame 20, raw) and the first one's pick.
static float s_query[12];
static maniMatchResult s_pick;

// Each leaf size builds a database the opener takes, with its leaf size
// and node count, and the same features and searches; any other is
// refused as an option.
static void TestLeafSizes(void)
{
    maniBuilder* builder = NewBuilder();
    maniMotionDatabaseSource source = Source();
    static const uint32_t kLeaves[4] = {0, 8, 16, 32};
    static const uint32_t kNodes[4] = {3, 7, 3, 1};
    float features[32][12];
    bool same = true;
    for (uint32_t k = 0; k < 4; ++k)
    {
        source.leafFrames = kLeaves[k];
        size_t size = 0;
        maniMotionDatabase d;
        CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_database, sizeof s_database, &size) == mani_success &&
                  maniOpenMotionDatabase(s_database, size, &d).result == mani_success,
              "a leaf size builds");
        CHECK(d.leafFrames == (kLeaves[k] != 0 ? kLeaves[k] : 16) && d.nodeCount == kNodes[k],
              "its leaf size and nodes");
        for (uint32_t f = 0; f < 32; ++f)
        {
            float row[12];
            CHECK(maniMotionDatabaseFeatures(&d, f, row, d.featureCount, NULL) == mani_success,
                  "a frame's features");
            same = same && (k == 0 ? (memcpy(features[f], row, sizeof row), true)
                                   : memcmp(features[f], row, sizeof row) == 0);
        }
        if (k == 0)
        {
            float offsets[12];
            float scales[12];
            CHECK(maniMotionDatabaseNormalization(&d, offsets, scales, d.featureCount, NULL) ==
                      mani_success,
                  "the normalization");
            for (uint32_t j = 0; j < 12; ++j)
            {
                s_query[j] = (features[20][j] + 0.05f) * scales[j] + offsets[j];
            }
        }
        maniMatchQuery q = {.features = s_query, .current = 4, .jumpThreshold = 0.2f};
        maniMatchResult r;
        CHECK(maniSearchMotionDatabase(&d, &q, &r) == mani_success, "a search");
        same = same && (k == 0 ? (s_pick = r, true)
                               : r.frame == s_pick.frame &&
                                     memcmp(&r.cost, &s_pick.cost, sizeof r.cost) == 0);
    }
    CHECK(same, "the same features and pick at every leaf size");
    const uint32_t wrong[3] = {4, 24, 64};
    for (uint32_t k = 0; k < 3; ++k)
    {
        source.leafFrames = wrong[k];
        maniBuildResult r = maniBuildMotionDatabase(builder, &source);
        CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonOption && r.item == 1,
              "another leaf size refused");
    }
    CHECK(maniBuilderMisuses(builder) == 0, "no misuse counted");
    maniDestroyBuilder(builder);
}

static bool WriteFile(const char* path, const void* data, size_t size)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return false;
    }
    size_t written = fwrite(data, 1, size, out);
    return fclose(out) == 0 && written == size;
}

// The same database whatever bytes pad the source, its channels and
// entries.
static void TestPaddingIgnored(void)
{
    uint8_t* built[2];
    size_t sizes[2];
    for (int k = 0; k < 2; ++k)
    {
        maniMotionDatabaseSource source = Source();
        maniMatchChannelSource channels[4];
        maniMatchEntrySource entries[3];
        memcpy(channels, s_channels, sizeof channels);
        memcpy(entries, s_entries, sizeof entries);
        TEST_PAD(channels, 4, kFieldsMatchChannelSource, kTestPadPatterns[k]);
        TEST_PAD(entries, 3, kFieldsMatchEntrySource, kTestPadPatterns[k]);
        source.channels = channels;
        source.entries = entries;
        TEST_PAD(&source, 1, kFieldsMotionDatabaseSource, kTestPadPatterns[k]);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success, "builds padded");
        built[k] = TestCopyBuilt(builder, &sizes[k]);
        maniDestroyBuilder(builder);
    }
    CHECK(TestSameBuilt(built, sizes), "the same bytes under either padding");
}

int main(int argc, char** argv)
{
    Open();
    OpenTurn();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        maniMotionDatabaseSource source = Source();
        bool ok = Build(&source).result == mani_success && WriteFile(argv[2], s_database, s_size);
        return ok && s_failures == 0 ? 0 : 1;
    }
    TestFeatures();
    TestNormalization();
    TestReadBack();
    TestFloatTrack();
    TestSourceRefusals();
    TestChannelRefusals();
    TestNameTotals();
    TestEntryRefusals();
    TestMisuse();
    TestOpenRefusals();
    TestOtherVersions();
    TestLeafSizes();
    TestPaddingIgnored();
    return s_failures == 0 ? 0 : 1;
}
