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
#include "negative_kinds.h"
#include "source_padding.h"
#include "vecmath.h"

#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/trajectory.h"

#include <math.h>
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
              maniBuilder_CopyBuilt(builder, s_turnFile, sizeof s_turnFile, &size) ==
                  mani_success &&
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
    s_entries[0] = (maniMatchEntrySource){0, maniMatchEntryLoop, 0.0f, 1.0f, 0.0f, 1u};
    s_entries[1] = (maniMatchEntrySource){1, 0, 0.0f, 1.0f, 0.5f, 2u};
    s_entries[2] = (maniMatchEntrySource){2, maniMatchEntryNoJump, 0.0f, 1.0f, 0.0f, 4u};
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
        CHECK(maniBuilder_CopyBuilt(builder, s_database, sizeof s_database, &s_size) ==
                  mani_success,
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
    CHECK(maniMotionDatabase_GetFeatures(d, f, features, d->featureCount, NULL) == mani_success,
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
    maniRootMove move = maniMakeRootMoveIdentity();
    CHECK(maniClip_GetRootMotion(&s_turn, t, to - t, &move) == mani_success, "a move");
    return move;
}

// The hips' model position in clip c at time t.
static maniVec3 Hips(const maniClip* clip, float t, maniPose* pose)
{
    CHECK(maniClip_Sample(clip, &s_skeleton, t, pose) == mani_success &&
              maniPose_ComputeModel(pose, &s_skeleton) == mani_success,
          "sampled");
    return maniPose_GetModelTransform(pose, 0).translation;
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
    CHECK(TestPlacementRefused(s_database, s_size, OpenFile),
          "sections placed as written, or refused");
    CHECK(TestSectionAbsent(s_database, maniMatchTagMirror) &&
              maniMotionDatabase_GetMirrorMapIdentity(&d) == 0,
          "no mirrored entry: the mirror section absent, read as none");
    // Tags 1 to 14 (file-formats.md).
    CHECK(TestLaterTagRefused(s_database, s_size, OpenFile, 14), "a later writer's tag refused");
    CHECK(d.frameCount == 32 && d.featureCount == 12 && d.channelCount == 4 && d.timeCount == 5 &&
              d.entryCount == 3 && d.clipCount == 3 && d.skeletonBoneCount == 3,
          "the counts: 10 looping frames, 11 and 11 clamped");
    CHECK(maniMotionDatabase_GetNormalization(&d, s_offsets, s_scales, d.featureCount, NULL) ==
              mani_success,
          "the scales");
    uint32_t measured[2] = {0, 0};
    uint32_t refused[2] = {0, 0};
    float row[12];
    float offsets[12];
    memset(row, 0x7F, sizeof row);
    memset(offsets, 0x7F, sizeof offsets);
    float kept = row[0];
    CHECK(maniMotionDatabase_GetFeatures(&d, 0, NULL, 0, &measured[0]) == mani_errorCapacity &&
              measured[0] == 12 &&
              maniMotionDatabase_GetFeatures(&d, 0, row, 11, &refused[0]) == mani_errorCapacity &&
              refused[0] == 12 && memcmp(&row[0], &kept, sizeof kept) == 0 &&
              maniMotionDatabase_GetFeatures(&d, 0, NULL, 1, NULL) == mani_errorInvalid,
          "the features measured, a short array refused (nothing written)");
    CHECK(maniMotionDatabase_GetNormalization(&d, NULL, NULL, 0, &measured[1]) ==
                  mani_errorCapacity &&
              measured[1] == 12 &&
              maniMotionDatabase_GetNormalization(&d, offsets, NULL, 11, &refused[1]) ==
                  mani_errorCapacity &&
              refused[1] == 12 && memcmp(&offsets[0], &kept, sizeof kept) == 0 &&
              maniMotionDatabase_GetNormalization(&d, NULL, NULL, 1, NULL) == mani_errorInvalid &&
              maniMotionDatabase_GetNormalization(&d, NULL, s_scales, 12, NULL) == mani_success,
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
                CHECK(maniMotionDatabase_GetFeatures(&d, f, v, d.featureCount, NULL) ==
                          mani_success,
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
    CHECK(maniMotionDatabase_GetChannel(&d, 2, &ch) == mani_success &&
              ch.kind == maniMatchTrajectoryPosition && ch.timeCount == 2 && ch.times[0] == 0.5f &&
              ch.times[1] == -0.2f && ch.firstFeature == 6 && ch.featureCount == 4 &&
              ch.weight == 1.0f,
          "a channel");
    size_t noName = 7;
    CHECK(maniMotionDatabase_GetChannel(&d, 4, &ch) == mani_errorInvalid &&
              maniMotionDatabase_GetChannelName(&d, 0, NULL) == NULL &&
              maniMotionDatabase_GetChannelName(&d, 0, &noName) == NULL && noName == 0,
          "past the channels; no name, its length cleared");
    noName = 7;
    CHECK(maniMotionDatabase_GetChannelName(&d, 4, &noName) == NULL && noName == 0,
          "past the channels the length cleared");
    maniMatchEntry e;
    CHECK(maniMotionDatabase_GetEntry(&d, 1, &e) == mani_success && e.clip == 1 && e.flags == 0 &&
              e.costBias == 0.5f && e.tags == 2u && e.firstFrame == 10 && e.frameCount == 11,
          "an entry");
    uint32_t entry = 0;
    float time = 0.0f;
    CHECK(maniMotionDatabase_GetFrame(&d, 9, &entry, &time) == mani_success && entry == 0 &&
              time == 0.9f && maniMotionDatabase_GetFrame(&d, 10, &entry, &time) == mani_success &&
              entry == 1 && time == 0.0f &&
              maniMotionDatabase_GetFrame(&d, 31, &entry, &time) == mani_success && entry == 2 &&
              time == 1.0f &&
              maniMotionDatabase_GetFrame(&d, 32, &entry, &time) == mani_errorInvalid,
          "frames to entries and times");
    CHECK(maniMotionDatabase_GetClipIdentity(&d, 2) == maniClip_GetIdentity(&s_turn) &&
              maniMotionDatabase_GetClipIdentity(&d, 3) == 0 &&
              maniMotionDatabase_GetSkeletonIdentity(&d) == maniSkeleton_GetIdentity(&s_skeleton) &&
              maniMotionDatabase_GetIdentity(&d) != 0,
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
              maniMotionDatabase_GetNormalization(&d, s_offsets, s_scales, d.featureCount, NULL) ==
                  mani_success,
          "built and opened");
    bool ok = true;
    for (uint32_t f = 0; f < 11; ++f)
    {
        ok = ok && Near4(Raw(&d, f, 0), (double)f / 10.0);
    }
    size_t length = 0;
    const char* name = maniMotionDatabase_GetChannelName(&d, 0, &length);
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
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        s = Source();
        s.up = kNegativeKinds[k];
        CHECK(Refused(&s, maniBuildReasonOption, 0, 0), "a negative up");
        s = Source();
        s.forward = kNegativeKinds[k];
        CHECK(Refused(&s, maniBuildReasonOption, 0, 0), "a negative forward");
    }
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
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        maniMotionDatabaseSource s = Source();
        s_channels[3] = (maniMatchChannelSource){kNegativeKinds[k], 0, 0, 1, kNow, NULL, 0, 1.0f};
        CHECK(Refused(&s, maniBuildReasonChannel, 3, 0), "a channel of a negative kind refused");
        s = Source();
        s_channels[3] = (maniMatchChannelSource){
            maniMatchBoneFacing, 0, kNegativeKinds[k], 1, kNow, NULL, 0, 1.0f};
        CHECK(Refused(&s, maniBuildReasonChannel, 3, 0), "a facing of a negative axis refused");
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
              maniBuilder_CopyBuilt(builder, NULL, 0, &size) == mani_errorCapacity,
          "the long names' clip");
    uint8_t* raw = malloc(size + 16);
    uint8_t* file = raw + (16 - (uintptr_t)raw % 16) % 16;
    maniClip clip;
    CHECK(maniBuilder_CopyBuilt(builder, file, size, &size) == mani_success &&
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
        {0, 0, 0.6f, 0.5f, 0.0f, 0u},  {0, maniMatchEntryLoop, 0.5f, 0.5f, 0.0f, 0u},
        {0, 0, 0.0f, 1.0f, NAN, 0u}};
    for (int k = 0; k < 7; ++k)
    {
        maniMotionDatabaseSource s = Source();
        s_entries[1] = kBad[k];
        CHECK(Refused(&s, maniBuildReasonEntry, 1, 0), "an entry refused");
    }
    maniMotionDatabaseSource s = Source();
    s_entries[1].flags = maniMatchEntryMirrored;
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
              maniBuilder_GetMisuse(builder) == 4 &&
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
    CHECK(maniOpenContainer(s_crafted, s_size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION,
                            MANI_MATCH_SECTIONS, &c)
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
    CHECK(maniOpenContainer(s_database, s_size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION,
                            MANI_MATCH_SECTIONS, &c)
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
    CHECK(maniLoadU32(s_database + header + 48) == 3 && maniLoadU32(s_database + nodes) == 2,
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
                                  {header + 48, 4, header + 48},
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
    // The one before this, whose header held the leaf size, among them.
    const uint16_t versions[4] = {1, 2, MANI_MATCH_FORMAT_VERSION - 1,
                                  MANI_MATCH_FORMAT_VERSION + 1};
    for (uint32_t k = 0; k < 4; ++k)
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
                  maniBuilder_CopyBuilt(builder, s_database, sizeof s_database, &size) ==
                      mani_success &&
                  maniOpenMotionDatabase(s_database, size, &d).result == mani_success,
              "a leaf size builds");
        CHECK(d.leafFrames == (kLeaves[k] != 0 ? kLeaves[k] : 16) && d.nodeCount == kNodes[k],
              "its leaf size and nodes");
        for (uint32_t f = 0; f < 32; ++f)
        {
            float row[12];
            CHECK(maniMotionDatabase_GetFeatures(&d, f, row, d.featureCount, NULL) == mani_success,
                  "a frame's features");
            same = same && (k == 0 ? (memcpy(features[f], row, sizeof row), true)
                                   : memcmp(features[f], row, sizeof row) == 0);
        }
        if (k == 0)
        {
            float offsets[12];
            float scales[12];
            CHECK(maniMotionDatabase_GetNormalization(&d, offsets, scales, d.featureCount, NULL) ==
                      mani_success,
                  "the normalization");
            for (uint32_t j = 0; j < 12; ++j)
            {
                s_query[j] = (features[20][j] + 0.05f) * scales[j] + offsets[j];
            }
        }
        maniMatchQuery q = {.features = s_query, .current = 4, .jumpThreshold = 0.2f};
        maniMatchResult r;
        CHECK(maniMotionDatabase_Search(&d, &q, &r) == mani_success, "a search");
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
    CHECK(maniBuilder_GetMisuse(builder) == 0, "no misuse counted");
    maniDestroyBuilder(builder);
}

// The leaf size is a section of its own, written only off the default
// (16): absent, the default; present, one of the other sizes.
static void TestLeafSection(void)
{
    maniMotionDatabaseSource source = Source();
    maniMotionDatabase d;
    static const uint32_t kDefaults[2] = {0, MANI_MATCH_DEFAULT_LEAF};
    for (uint32_t k = 0; k < 2; ++k)
    {
        source.leafFrames = kDefaults[k];
        CHECK(Build(&source).result == mani_success &&
                  TestSectionAbsent(s_database, maniMatchTagLeaf) &&
                  maniOpenMotionDatabase(s_database, s_size, &d).result == mani_success &&
                  d.leafFrames == MANI_MATCH_DEFAULT_LEAF,
              "the default leaf size: its section absent, read as 16");
    }
    source.leafFrames = 8;
    CHECK(Build(&source).result == mani_success, "leaves of 8");
    uint64_t leaf = Section(maniMatchTagLeaf);
    CHECK(maniLoadU32(s_database + leaf) == 8 &&
              maniOpenMotionDatabase(s_database, s_size, &d).result == mani_success &&
              d.leafFrames == 8,
          "leaves of 8: their section");
    CHECK(RefusedAt(leaf, MANI_MATCH_DEFAULT_LEAF, leaf) && RefusedAt(leaf, 12, leaf) &&
              RefusedAt(leaf, 0, leaf),
          "a leaf size at the default, written, or not one of the sizes");
    // The section 4 bytes longer, the file ending with it: refused at its
    // size in the table's last entry.
    uint64_t last = MANI_CONTAINER_HEADER_SIZE + 24 * (uint64_t)(maniLoadU32(s_database + 12) - 1);
    size_t longer = s_size + 4;
    memcpy(s_crafted, s_database, s_size);
    memset(s_crafted + s_size, 0, 4);
    maniStoreU64(s_crafted + last + 16, 8);
    maniStoreU64(s_crafted + 24, longer);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               longer - MANI_CONTAINER_HEADER_SIZE));
    maniFileResult r = maniOpenMotionDatabase(s_crafted, longer, &d);
    CHECK(maniLoadU32(s_database + last) == maniMatchTagLeaf && r.result == mani_errorFormat &&
              r.offset == last + 16,
          "a leaf size's section of the wrong size");
}

// A heap block of exactly the database just built, opened from it.
static uint8_t* OpenCopy(maniMotionDatabase* out, uint8_t** blockOut)
{
    uint8_t* bytes = TestExactCopy(s_size, blockOut);
    CHECK(bytes != NULL, "a block of the file's length");
    memcpy(bytes, s_database, s_size);
    CHECK(maniOpenMotionDatabase(bytes, s_size, out).result == mani_success, "opens the copy");
    return bytes;
}

// Whether every call refuses a view: the documented none from each
// getter, mani_errorInvalid from the others. The mirror map's identity is
// read from a copy that names one, so that a view taken as given would
// show it.
static bool ViewRefused(const maniMotionDatabase* d)
{
    maniMotionDatabase named;
    memset(&named, 0, sizeof named);
    if (d != NULL)
    {
        named = *d;
        named.mirrorMapIdentity |= 1u;
    }
    maniMatchChannel ch;
    maniMatchEntry e;
    uint32_t entry = 0;
    float time = 0.0f;
    static float features[MANI_MAX_MATCH_FEATURES];
    static float query[MANI_MAX_MATCH_FEATURES];
    size_t length = 1;
    maniMatchQuery q = {.features = query, .current = MANI_MATCH_NO_FRAME};
    maniMatchResult r;
    maniTrajectory t;
    memset(&t, 0, sizeof t);
    t.rotation.w = 1.0f;
    return maniMotionDatabase_GetIdentity(d) == 0 &&
           maniMotionDatabase_GetMirrorMapIdentity(d != NULL ? &named : NULL) == 0 &&
           maniMotionDatabase_GetSkeletonIdentity(d) == 0 &&
           maniMotionDatabase_GetClipIdentity(d, 0) == 0 &&
           maniMotionDatabase_GetChannel(d, 0, &ch) == mani_errorInvalid &&
           maniMotionDatabase_GetChannelName(d, 0, &length) == NULL && length == 0 &&
           maniMotionDatabase_GetEntry(d, 0, &e) == mani_errorInvalid &&
           maniMotionDatabase_GetFrame(d, 0, &entry, &time) == mani_errorInvalid &&
           maniMotionDatabase_GetFeatures(d, 0, features, MANI_MAX_MATCH_FEATURES, NULL) ==
               mani_errorInvalid &&
           maniMotionDatabase_GetNormalization(d, features, NULL, MANI_MAX_MATCH_FEATURES, NULL) ==
               mani_errorInvalid &&
           maniMotionDatabase_Search(d, &q, &r) == mani_errorInvalid &&
           maniMotionDatabase_ComputeTrajectoryFeatures(d, &t, &t, features, NULL) ==
               mani_errorInvalid;
}

// The view is state a caller holds, checked by every call against the
// bytes and size the open was given: counts edited past the file, frame,
// feature, entry and node counts out of range, a rate out of range, a
// leaf size the builder does not write, sections placed past the file,
// one view's counts and sections over a database of fewer channels, a
// zeroed view and another kind's cookie are each refused.
static void TestViewChecked(void)
{
    maniMotionDatabaseSource source = Source();
    CHECK(Build(&source).result == mani_success, "a database builds");
    maniMotionDatabase big;
    uint8_t* bigBlock = NULL;
    (void)OpenCopy(&big, &bigBlock);
    source.channelCount = 2;
    CHECK(Build(&source).result == mani_success, "a database of two channels builds");
    maniMotionDatabase small;
    uint8_t* smallBlock = NULL;
    (void)OpenCopy(&small, &smallBlock);
    const char* counts[8] = {"channels", "times",  "clips", "entries",
                             "features", "frames", "nodes", "name bytes"};
    for (uint32_t k = 0; k < 8; ++k)
    {
        maniMotionDatabase edited = big;
        uint32_t* count[7] = {&edited.channelCount, &edited.timeCount,    &edited.clipCount,
                              &edited.entryCount,   &edited.featureCount, &edited.frameCount,
                              &edited.nodeCount};
        if (k < 7)
        {
            *count[k] = k == 4 ? 1000 : 60000;
        }
        else
        {
            edited.namesSize = 60000;
        }
        CHECK(ViewRefused(&edited), counts[k]);
    }
    const char* sections[11] = {
        "channels past the file",  "times past the file",   "clips past the file",
        "entries past the file",   "normalization past it", "features past the file",
        "boxes past the file",     "names past the file",   "nodes past the file",
        "positions past the file", "places past the file"};
    for (uint32_t k = 0; k < 11; ++k)
    {
        maniMotionDatabase edited = big;
        edited.sections[k] = big.size - 2;
        edited.namesSize = k == 7 ? 4 : edited.namesSize;
        CHECK(ViewRefused(&edited), sections[k]);
    }
    maniMotionDatabase edited = big;
    const char* outOfRange[7] = {"no frames",   "frames past the most",
                                 "no features", "features past the most",
                                 "no entries",  "no nodes",
                                 "no base"};
    for (uint32_t k = 0; k < 7; ++k)
    {
        edited = big;
        uint32_t* word[6] = {&edited.frameCount,   &edited.frameCount, &edited.featureCount,
                             &edited.featureCount, &edited.entryCount, &edited.nodeCount};
        const uint32_t value[6] = {0, MANI_MAX_MATCH_FRAMES + 1, 0, MANI_MAX_MATCH_FEATURES + 1, 0,
                                   0};
        if (k < 6)
        {
            *word[k] = value[k];
        }
        else
        {
            edited.base = NULL;
        }
        CHECK(ViewRefused(&edited), outOfRange[k]);
    }
    const float rates[3] = {0.0f, MANI_MAX_MATCH_RATE * 2.0f, NAN};
    const char* rateNames[3] = {"no rate", "a rate past the most", "a rate not a number"};
    for (uint32_t k = 0; k < 3; ++k)
    {
        edited = big;
        edited.sampleRate = rates[k];
        CHECK(ViewRefused(&edited), rateNames[k]);
    }
    const uint32_t leaves[3] = {7, 12, 64};
    for (uint32_t k = 0; k < 3; ++k)
    {
        edited = big;
        edited.leafFrames = leaves[k];
        CHECK(ViewRefused(&edited), "a leaf size the builder does not write");
    }
    maniMotionDatabase mixed = big;
    mixed.base = small.base;
    mixed.size = small.size;
    CHECK(ViewRefused(&mixed), "counts and sections over another view's bytes");
    maniMotionDatabase zeroed;
    memset(&zeroed, 0, sizeof zeroed);
    maniMotionDatabase other = big;
    other.cookie ^= 1u;
    CHECK(ViewRefused(&zeroed) && ViewRefused(&other) && ViewRefused(NULL),
          "zeroed, another cookie, NULL");
    maniMatchChannel ch;
    CHECK(maniMotionDatabase_GetChannel(&big, 2, &ch) == mani_success &&
              maniMotionDatabase_GetChannel(&small, 2, &ch) == mani_errorInvalid,
          "the views as opened");
    free(bigBlock);
    free(smallBlock);
}

// A channel record's word at offset made value, read back.
static maniMatchChannel ChangedChannel(uint8_t* record, uint32_t offset, uint32_t value,
                                       const maniMotionDatabase* d, uint32_t channel)
{
    uint32_t kept = maniLoadU32(record + offset);
    maniStoreU32(record + offset, value);
    maniMatchChannel ch;
    memset(&ch, 0xA5, sizeof ch);
    CHECK(maniMotionDatabase_GetChannel(d, channel, &ch) == mani_success, "a changed channel read");
    maniStoreU32(record + offset, kept);
    return ch;
}

// Bytes changed after the open are read only within the file, and give
// what the header says. Of the trajectory channel (its times the third
// and fourth of five, its features 6 to 9 of 12): a first time past the
// times leaves it none; a time count past them with the features from
// the first is cut at the times' end; a first feature past the features
// leaves it no features and no times; a kind past the last is a float
// track, a bone past the skeleton none and an axis past the six the
// first. Of the run's entry (frames 10 to 20 of 32): a clip past the
// clips is clip 0, a first frame past the frames leaves it none, and a
// frame count past them is cut at their end. A frame's place past the
// frames reads the last position's features, and a frame whose entry
// starts are changed still finds an entry.
static void TestChangedBytes(void)
{
    maniMotionDatabaseSource source = Source();
    CHECK(Build(&source).result == mani_success, "a database builds");
    maniMotionDatabase d;
    uint8_t* block = NULL;
    uint8_t* bytes = OpenCopy(&d, &block);
    uint8_t* channel = bytes + d.sections[0] + 2 * MANI_MATCH_CHANNEL_SIZE;
    maniMatchChannel a = ChangedChannel(channel, 16, 1000, &d, 2);
    CHECK(a.timeCount == 0 && a.firstFeature == 6 && a.featureCount == 4,
          "a first time past the times: none");
    uint32_t keptFirst = maniLoadU32(channel + 20);
    uint32_t keptFeatures = maniLoadU32(channel + 24);
    maniStoreU32(channel + 20, 0);
    maniStoreU32(channel + 24, 1000);
    maniMatchChannel b = ChangedChannel(channel, 12, 1000, &d, 2);
    maniStoreU32(channel + 20, keptFirst);
    maniStoreU32(channel + 24, keptFeatures);
    CHECK(b.timeCount == 3 && b.firstFeature == 0 && b.featureCount == 12 && b.times[0] == 0.5f &&
              b.times[1] == -0.2f && b.times[2] == 0.5f,
          "a time count past the times cut at their end");
    maniMatchChannel c = ChangedChannel(channel, 20, 1000, &d, 2);
    CHECK(c.timeCount == 0 && c.firstFeature == 12 && c.featureCount == 0,
          "a first feature past the features: no features, no times");
    maniMatchChannel k = ChangedChannel(channel, 0, 9, &d, 2);
    maniMatchChannel bone = ChangedChannel(channel, 4, 1000, &d, 2);
    maniMatchChannel axis = ChangedChannel(channel, 8, 9, &d, 2);
    CHECK(k.kind == maniMatchFloatTrack && k.timeCount == 2 && bone.bone == MANI_NO_BONE &&
              axis.axis == maniMatchAxisX,
          "a kind, a bone and an axis past theirs");
    uint8_t* entry = bytes + d.sections[3] + MANI_MATCH_ENTRY_SIZE;
    const uint32_t offsets[3] = {0, 24, 28};
    maniMatchEntry e[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        uint32_t kept = maniLoadU32(entry + offsets[i]);
        maniStoreU32(entry + offsets[i], 1000);
        CHECK(maniMotionDatabase_GetEntry(&d, 1, &e[i]) == mani_success, "a changed entry read");
        maniStoreU32(entry + offsets[i], kept);
    }
    CHECK(e[0].clip == 0 && e[1].firstFrame == 32 && e[1].frameCount == 0 &&
              e[2].firstFrame == 10 && e[2].frameCount == 22,
          "an entry's clip and frames past theirs");
    uint32_t last = maniLoadU32(bytes + d.sections[9] + 31 * MANI_MATCH_POSITION_SIZE);
    float want[12];
    float got[12];
    CHECK(maniMotionDatabase_GetFeatures(&d, last, want, 12, NULL) == mani_success, "features");
    uint8_t* place = bytes + d.sections[10] + 4 * 5;
    uint32_t kept = maniLoadU32(place);
    maniStoreU32(place, 1000);
    CHECK(maniMotionDatabase_GetFeatures(&d, 5, got, 12, NULL) == mani_success &&
              memcmp(got, want, sizeof got) == 0,
          "a place past the frames: the last position's features");
    maniStoreU32(place, kept);
    maniStoreU32(entry + 24, 0);
    uint32_t found = 0;
    float time = 0.0f;
    CHECK(maniMotionDatabase_GetFrame(&d, 31, &found, &time) == mani_success && found < 3,
          "a frame among changed entry starts finds an entry");
    free(block);
}

// Of a database of three trajectory channels of 8 times each (24 times,
// 48 features), in bytes changed after the open: a channel's time count
// past the most a channel holds, with the features from the first, is cut
// at MANI_MAX_MATCH_TIMES.
static void TestChangedTimes(void)
{
    static const float kEight[8] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f};
    maniMotionDatabaseSource source = Source();
    for (uint32_t k = 0; k < 3; ++k)
    {
        s_channels[k] = Channel(maniMatchTrajectoryPosition, 0, kEight, 8, 1.0f);
    }
    source.channelCount = 3;
    CHECK(Build(&source).result == mani_success, "a database of 24 times builds");
    maniMotionDatabase d;
    uint8_t* block = NULL;
    uint8_t* bytes = OpenCopy(&d, &block);
    uint8_t* channel = bytes + d.sections[0];
    maniStoreU32(channel + 24, 1000);
    maniMatchChannel ch = ChangedChannel(channel, 12, 1000, &d, 0);
    CHECK(d.timeCount == 24 && ch.timeCount == MANI_MAX_MATCH_TIMES && ch.featureCount == 48 &&
              ch.times[8] == 0.1f && ch.times[15] == 0.8f,
          "a time count past the most a channel holds cut at it");
    free(block);
}

// A float track's name in bytes changed after the open, starting or
// running past the names, is empty.
static void TestChangedName(void)
{
    s_clipList[0] = &s_turn;
    maniMatchChannelSource channel = {maniMatchFloatTrack, 0, 0, 1, kNow, "speed", 5, 1.0f};
    maniMatchEntrySource entry = {0, 0, 0.0f, 1.0f, 0.0f, 0u};
    maniMotionDatabaseSource source = {&s_skeleton, s_clipList,     &channel,       &entry, 1, 1, 1,
                                       10.0f,       maniMatchAxisY, maniMatchAxisZ, NULL,   0};
    CHECK(Build(&source).result == mani_success, "a database with a name builds");
    maniMotionDatabase d;
    uint8_t* block = NULL;
    uint8_t* bytes = OpenCopy(&d, &block);
    uint8_t* e = bytes + d.sections[0];
    const char* names = (const char*)bytes + d.sections[7];
    size_t length = 7;
    maniStoreU32(e + 32, (uint32_t)d.namesSize);
    maniStoreU32(e + 36, 1);
    CHECK(maniMotionDatabase_GetChannelName(&d, 0, &length) == names && length == 0,
          "a name starting past the names");
    maniStoreU32(e + 32, 1);
    maniStoreU32(e + 36, (uint32_t)d.namesSize);
    length = 7;
    CHECK(maniMotionDatabase_GetChannelName(&d, 0, &length) == names && length == 0,
          "a name running past the names");
    free(block);
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
    TestLeafSection();
    TestPaddingIgnored();
    TestViewChecked();
    TestChangedBytes();
    TestChangedTimes();
    TestChangedName();
    return s_failures == 0 ? 0 : 1;
}
