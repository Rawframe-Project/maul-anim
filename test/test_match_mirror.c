// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mirrored motion database entries on a rig whose left and
// right arms have unmirrored axes: a database of a walk and the walk
// mirrored holds, frame for frame, the features of one built over the
// walk and its mirrored bake (bone positions, velocities and facings,
// the trajectory and its facing, a float track read from its partner);
// the map's identity kept; the builder's and opener's refusals; a graph
// playing the mirrored entry mirrored (its pose, moves and events),
// under a mirror node as it is, and its refusals.

#include "bytes.h"
#include "container.h"
#include "graph_instance.h"
#include "match_file.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum
{
    kRoot,
    kHips,
    kChest,
    kClavL,
    kArmL,
    kClavR,
    kArmR,
    kBones
};

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

static const maniBoneSource kRig[kBones] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"hips", 4, kRoot, {{0, 1, 0}, REST, {1, 1, 1}}},
    {"chest", 5, kHips, {{0, 0.4f, 0}, REST, {1, 1, 1}}},
    {"clavL", 5, kChest, {{0.1f, 0.1f, 0}, {0, 0, 0.25881904f, 0.96592583f}, {1, 1, 1}}},
    {"armL", 4, kClavL, {{0.2f, 0, 0}, REST, {1, 1, 1}}},
    {"clavR", 5, kChest, {{-0.1f, 0.1f, 0}, {0, 0.70710678f, 0, 0.70710678f}, {1, 1, 1}}},
    {"armR", 4, kClavR, {{0, 0, 0.2f}, REST, {1, 1, 1}}},
};

static const maniMirrorRuleSource kRules[2] = {{"L", 1, maniMirrorSuffix, "R", 1},
                                               {"R", 1, maniMirrorSuffix, "L", 1}};
static const maniMirrorNameSource kNames[4] = {
    {"blinkL", 6}, {"blinkR", 6}, {"stepL", 5}, {"stepR", 5}};

static const float kTimes[3] = {0.0f, 0.5f, 1.0f};
static const float kRootMove[9] = {0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.6f, 0.9f, 0.05f, 1.2f};
static const float kYaw[12] = {0.0f, 0.0f,        0.0f, 1.0f,        0.0f, 0.17364818f,
                               0.0f, 0.98480775f, 0.0f, 0.38268343f, 0.0f, 0.92387953f};
static const float kArm[12] = {0.0f, 0.0f,        0.25881904f, 0.96592583f, 0.1f, 0.2f,
                               0.3f, 0.92736185f, 0.0f,        0.5f,        0.0f, 0.8660254f};
static const float kLeft[3] = {0.0f, 1.0f, 0.25f};
static const float kRight[3] = {1.0f, 0.0f, 0.5f};
static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_mapFiles[2][4096];
static _Alignas(16) uint8_t s_clipFiles[2][65536];
static _Alignas(16) uint8_t s_databaseFiles[2][1 << 16];
static _Alignas(16) uint8_t s_crafted[1 << 16];
static _Alignas(16) uint8_t s_graphFile[65536];
static _Alignas(MANI_GRAPH_BINDING_ALIGNMENT) uint8_t s_bindingMemory[8192];
static _Alignas(MANI_GRAPH_INSTANCE_ALIGNMENT) uint8_t s_instanceMemory[65536];
static _Alignas(MANI_GRAPH_SCRATCH_ALIGNMENT) uint8_t s_scratch[131072];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[2][2048];
static size_t s_databaseSizes[2];
static maniSkeleton s_skeleton;
static maniMirrorMap s_map;
static maniMirrorMap s_mapZ;
static maniClip s_walk;
static maniClip s_baked;
static maniMotionDatabase s_mirrored;
static maniMotionDatabase s_bakedDatabase;
static maniPose* s_out;
static maniPose* s_expected;

static maniChannelSource Keys(const float* values)
{
    return (maniChannelSource){kTimes, values, 3, maniInterpolationLinear};
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

static void OpenMap(maniMirrorAxis axis, uint8_t* file, maniMirrorMap* out)
{
    maniMirrorMapSource source = {.skeleton = &s_skeleton,
                                  .rules = kRules,
                                  .names = kNames,
                                  .ruleCount = 2,
                                  .nameCount = 4,
                                  .axis = axis};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, 4096, &size) == mani_success &&
              maniOpenMirrorMap(file, size, out).result == mani_success,
          "a map");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource rig = {kRig, kBones};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &rig).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "the rig");
    OpenMap(maniMirrorAxisX, s_mapFiles[0], &s_map);
    OpenMap(maniMirrorAxisZ, s_mapFiles[1], &s_mapZ);
    const maniBoneTrackSource tracks[2] = {{kRoot, Keys(kRootMove), Keys(kYaw), kNone},
                                           {kArmL, kNone, Keys(kArm), kNone}};
    const maniFloatTrackSource floats[2] = {{"blinkL", 6, Keys(kLeft)},
                                            {"blinkR", 6, Keys(kRight)}};
    const maniEventSource events[2] = {{0.25f, 7, "stepL", 5}, {0.75f, 9, "jump", 4}};
    const maniRootMotionSource root = {kRoot, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
                                       1,     maniRootReferenceRest,         maniRootReferenceRest,
                                       0};
    const maniClipSource walk = {.skeleton = &s_skeleton,
                                 .boneTracks = tracks,
                                 .floatTracks = floats,
                                 .events = events,
                                 .boneTrackCount = 2,
                                 .floatTrackCount = 2,
                                 .eventCount = 2,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 30.0f,
                                 .duration = 1.0f,
                                 .rootMotion = &root};
    CHECK(maniBuildClip(builder, &walk).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_clipFiles[0], sizeof s_clipFiles[0], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[0], size, &s_walk).result == mani_success,
          "the walk");
    const maniMirrorClipSource mirror = {&s_map, &s_skeleton, &s_walk};
    CHECK(maniBuildMirroredClip(builder, &mirror).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_clipFiles[1], sizeof s_clipFiles[1], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[1], size, &s_baked).result == mani_success,
          "the walk's mirrored bake");
    maniDestroyBuilder(builder);
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[0], sizeof s_poseMemory[0], &s_out) ==
                  mani_success &&
              maniInitPose(&s_skeleton, s_poseMemory[1], sizeof s_poseMemory[1], &s_expected) ==
                  mani_success,
          "poses");
}

static const float kNow[1] = {0.0f};
static const float kNear[2] = {0.0f, 0.2f};
static const float kAhead[2] = {0.2f, 0.5f};
static const float kFar[1] = {0.5f};
static const maniMatchChannelSource kChannels[6] = {
    {maniMatchBonePosition, kArmL, maniMatchAxisX, 2, kNear, NULL, 0, 1.0f},
    {maniMatchBoneVelocity, kArmL, maniMatchAxisX, 1, kNow, NULL, 0, 1.0f},
    {maniMatchBoneFacing, kArmL, maniMatchAxisX, 1, kNow, NULL, 0, 1.0f},
    {maniMatchTrajectoryPosition, 0, maniMatchAxisX, 2, kAhead, NULL, 0, 1.0f},
    {maniMatchTrajectoryFacing, 0, maniMatchAxisX, 1, kFar, NULL, 0, 1.0f},
    {maniMatchFloatTrack, 0, maniMatchAxisX, 1, kNow, "blinkL", 6, 1.0f}};

static maniBuildResult BuildDatabase(const maniMotionDatabaseSource* source, int slot,
                                     maniMotionDatabase* out)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildMotionDatabase(builder, source);
    if (r.result == mani_success)
    {
        CHECK(
            maniBuilder_CopyBuilt(builder, s_databaseFiles[slot], sizeof s_databaseFiles[slot],
                                  &s_databaseSizes[slot]) == mani_success &&
                maniOpenMotionDatabase(s_databaseFiles[slot], s_databaseSizes[slot], out).result ==
                    mani_success,
            "the database opens");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniMatchEntrySource s_entries[2];
static const maniClip* s_clips[2];

// The walk (tags 1) and the walk mirrored (tags 2) of one clip through
// the map.
static maniMotionDatabaseSource Mirrored(const maniMirrorMap* map)
{
    s_clips[0] = &s_walk;
    s_entries[0] = (maniMatchEntrySource){0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u};
    s_entries[1] =
        (maniMatchEntrySource){0, MANI_MATCH_LOOP | MANI_MATCH_MIRRORED, 0.0f, 1.0f, 0.0f, 2u};
    return (maniMotionDatabaseSource){&s_skeleton, s_clips, kChannels,      s_entries,      1,   6,
                                      2,           30.0f,   maniMatchAxisY, maniMatchAxisZ, map, 0};
}

static bool Near(float a, float b)
{
    return fabsf(a - b) < 2e-4f;
}

static void TestFeatures(void)
{
    maniMotionDatabaseSource source = Mirrored(&s_map);
    CHECK(BuildDatabase(&source, 0, &s_mirrored).result == mani_success,
          "a database of mirrored entries builds");
    CHECK(maniMotionDatabase_GetMirrorMapIdentity(&s_mirrored) == s_map.identity &&
              maniMotionDatabase_GetMirrorMapIdentity(NULL) == 0,
          "the map's identity kept");
    static const maniClip* baked[2];
    static const maniMatchEntrySource entries[2] = {{0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u},
                                                    {1, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 2u}};
    baked[0] = &s_walk;
    baked[1] = &s_baked;
    const maniMotionDatabaseSource bake = {
        &s_skeleton, baked, kChannels,      entries,        2,    6,
        2,           30.0f, maniMatchAxisY, maniMatchAxisZ, NULL, 0};
    CHECK(BuildDatabase(&bake, 1, &s_bakedDatabase).result == mani_success,
          "a database of the bake builds");
    CHECK(maniMotionDatabase_GetMirrorMapIdentity(&s_bakedDatabase) == 0, "no map without mirrors");
    uint32_t n = s_mirrored.featureCount;
    bool same = s_mirrored.frameCount == 60 && s_bakedDatabase.frameCount == 60 &&
                s_bakedDatabase.featureCount == n;
    bool differs = false;
    for (uint32_t f = 0; same && f < 60; ++f)
    {
        float a[32];
        float b[32];
        float plain[32];
        same = n <= 32 &&
               maniMotionDatabase_GetFeatures(&s_mirrored, f, a, s_mirrored.featureCount, NULL) ==
                   mani_success &&
               maniMotionDatabase_GetFeatures(&s_bakedDatabase, f, b, s_bakedDatabase.featureCount,
                                              NULL) == mani_success &&
               maniMotionDatabase_GetFeatures(&s_mirrored, f % 30, plain, s_mirrored.featureCount,
                                              NULL) == mani_success;
        for (uint32_t k = 0; same && k < n; ++k)
        {
            same = Near(a[k], b[k]);
            differs = differs || (f >= 30 && !Near(a[k], plain[k]));
        }
    }
    CHECK(same, "each mirrored frame the bake's");
    CHECK(differs, "the mirrored frames not the walk's");
    // Clamped halves, the mirrored one starting where the plain one ends:
    // the sampler's last pose is not the mirrored one's.
    static const maniMatchEntrySource halves[2] = {{0, 0, 0.0f, 0.5f, 0.0f, 1u},
                                                   {0, MANI_MATCH_MIRRORED, 0.5f, 1.0f, 0.0f, 2u}};
    static const maniMatchEntrySource bakedHalves[2] = {{0, 0, 0.0f, 0.5f, 0.0f, 1u},
                                                        {1, 0, 0.5f, 1.0f, 0.0f, 2u}};
    maniMotionDatabase a;
    maniMotionDatabase b;
    source.entries = halves;
    maniMotionDatabaseSource bakedSource = bake;
    bakedSource.entries = bakedHalves;
    CHECK(BuildDatabase(&source, 0, &a).result == mani_success &&
              BuildDatabase(&bakedSource, 1, &b).result == mani_success &&
              a.frameCount == b.frameCount && a.featureCount == n,
          "databases of halves");
    same = true;
    for (uint32_t f = 0; same && f < a.frameCount; ++f)
    {
        float x[32];
        float y[32];
        same = maniMotionDatabase_GetFeatures(&a, f, x, a.featureCount, NULL) == mani_success &&
               maniMotionDatabase_GetFeatures(&b, f, y, b.featureCount, NULL) == mani_success;
        for (uint32_t k = 0; same && k < n; ++k)
        {
            same = Near(x[k], y[k]);
        }
    }
    CHECK(same, "a mirrored half after a plain one the bake's");
    source = Mirrored(&s_map);
    CHECK(BuildDatabase(&source, 0, &s_mirrored).result == mani_success, "the database again");
}

static bool Refused(const maniMotionDatabaseSource* source, maniBuildReason reason, uint32_t item)
{
    maniMotionDatabase d;
    maniBuildResult r = BuildDatabase(source, 1, &d);
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item;
}

static void TestRefusals(void)
{
    maniMotionDatabaseSource source = Mirrored(NULL);
    CHECK(Refused(&source, maniBuildReasonMirrorDatabase, 1), "a mirrored entry without a map");
    maniMirrorMap counted = s_map;
    counted.boneCount = kBones - 1;
    source = Mirrored(&counted);
    CHECK(Refused(&source, maniBuildReasonMirrorDatabase, 0), "a map of another bone count");
    counted = s_map;
    counted.skeletonIdentity ^= 1u;
    source = Mirrored(&counted);
    CHECK(Refused(&source, maniBuildReasonMirrorDatabase, 0), "a map of another skeleton");
    // A channel's partner track missing.
    static const maniMatchChannelSource lone[1] = {
        {maniMatchFloatTrack, 0, maniMatchAxisX, 1, kNow, "flag", 4, 1.0f}};
    static const maniMirrorNamePairSource pair[1] = {{"flag", 4, 4, "flap"}};
    static _Alignas(16) uint8_t file[4096];
    maniMirrorMapSource ms = {.skeleton = &s_skeleton, .namePairs = pair, .namePairCount = 1};
    maniMirrorMap flap;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, &ms).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenMirrorMap(file, size, &flap).result == mani_success,
          "a map pairing flag with flap");
    maniDestroyBuilder(builder);
    source = Mirrored(&flap);
    source.channels = lone;
    source.channelCount = 1;
    maniMotionDatabase d;
    maniBuildResult r = BuildDatabase(&source, 1, &d);
    CHECK(r.reason == maniBuildReasonMissingTrack && r.item == 0 && r.other == 0,
          "the walk without a flag track");
}

static uint64_t SectionAt(const uint8_t* file, size_t size, uint32_t tag, uint64_t* length)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t at = 0;
    *length = 0;
    bool found =
        maniOpenContainer(file, size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION, &c)
                .result == mani_success &&
        maniContainerSection(&c, tag, &data, length, &at);
    return found ? at : 0;
}

static uint64_t Opened(size_t size)
{
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               size - MANI_CONTAINER_HEADER_SIZE));
    maniMotionDatabase d;
    maniFileResult r = maniOpenMotionDatabase(s_crafted, size, &d);
    return r.result == mani_errorFormat ? r.offset : UINT64_MAX;
}

static void TestOpen(void)
{
    size_t size = s_databaseSizes[0];
    uint64_t length = 0;
    uint64_t at = SectionAt(s_databaseFiles[0], size, maniMatchTagMirror, &length);
    CHECK(length == 8 && maniLoadU64(s_databaseFiles[0] + at) == s_map.identity,
          "the mirror section");
    memcpy(s_crafted, s_databaseFiles[0], size);
    maniStoreU64(s_crafted + at, 0);
    CHECK(Opened(size) == at, "a mirror map identity of 0");
    uint64_t records = SectionAt(s_databaseFiles[0], size, maniMatchTagEntries, &length);
    memcpy(s_crafted, s_databaseFiles[0], size);
    maniStoreU32(s_crafted + records + 4, 8);
    CHECK(Opened(size) == records + 4, "an entry flag past the flags");
    maniContainer c;
    CHECK(maniOpenContainer(s_databaseFiles[0], size, maniKindMotionDatabase,
                            MANI_MATCH_FORMAT_VERSION, &c)
                  .result == mani_success,
          "the container");
    // The mirror section's entry in the table.
    uint64_t entry = MANI_CONTAINER_HEADER_SIZE;
    while (maniLoadU32(s_databaseFiles[0] + entry) != maniMatchTagMirror &&
           entry < MANI_CONTAINER_HEADER_SIZE + (uint64_t)c.sectionCount * 24)
    {
        entry += 24;
    }
    memcpy(s_crafted, s_databaseFiles[0], size);
    maniStoreU64(s_crafted + entry + 16, 4);
    CHECK(maniLoadU32(s_databaseFiles[0] + entry) == maniMatchTagMirror &&
              Opened(size) == entry + 16,
          "a mirror section of 4 bytes");
    // Written again without the section: the mirrored entry refused.
    maniSectionSource sections[MANI_MATCH_SECTIONS];
    const uint8_t* parts[MANI_MATCH_SECTIONS];
    uint64_t sizes[MANI_MATCH_SECTIONS];
    uint32_t count = 0;
    for (uint32_t t = 1; t <= MANI_MATCH_SECTIONS; ++t)
    {
        const uint8_t* data = s_databaseFiles[0];
        uint64_t where = 0;
        parts[t - 1] = data;
        sizes[t - 1] = 0;
        if (t != maniMatchTagMirror &&
            maniContainerSection(&c, t, &parts[t - 1], &sizes[t - 1], &where) && sizes[t - 1] > 0)
        {
            sections[count++] = (maniSectionSource){t, parts[t - 1], sizes[t - 1]};
        }
    }
    uint64_t fileSize = 0;
    CHECK(maniContainerSize(sections, count, &fileSize) && fileSize <= sizeof s_crafted,
          "room for the file");
    maniWriteContainer(s_crafted, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION,
                       maniMatchComputeIdentity(parts, sizes), sections, count);
    uint64_t entries = SectionAt(s_crafted, (size_t)fileSize, maniMatchTagEntries, &length);
    maniMotionDatabase d;
    maniFileResult r = maniOpenMotionDatabase(s_crafted, (size_t)fileSize, &d);
    CHECK(r.result == mani_errorFormat && r.offset == entries + MANI_MATCH_ENTRY_SIZE + 4,
          "a mirrored entry without a map");
}

// A graph playing the mirrored entry (required tags 2), under mirrors
// mirror nodes (0 or 1), an inertialize node above.
static const uint32_t kFirst[1] = {0};
static const uint32_t kSecond[1] = {1};
static maniGraphNodeSource s_nodes[3];
static maniGraphClipSource s_slots[1];
static maniGraphDatabaseSource s_databaseSlot[1];
// The query: the trajectory's two places and facing, the blink.
static const maniGraphParameterSource kQueryParameters[4] = {
    {"near", 4, maniGraphParameterVec2, {{0}}, {{0}}, {{0}}, 0},
    {"far", 3, maniGraphParameterVec2, {{0}}, {{0}}, {{0}}, 0},
    {"facing", 6, maniGraphParameterVec2, {{0}}, {{0}}, {{0}}, 0},
    {"blink", 5, maniGraphParameterFloat, {{0}}, {{0}}, {{0}}, 0}};
static const uint32_t kQuery[4] = {0, 1, 2, 3};

static maniGraphSource Graph(uint32_t mirrors, const maniMirrorMap* map)
{
    s_slots[0] = (maniGraphClipSource){&s_walk, "walk", 4, NULL};
    s_databaseSlot[0] = (maniGraphDatabaseSource){&s_mirrored, "moves", 5};
    const maniGraphMatchNodeSource m = {.database = 0,
                                        .trigger = MANI_NO_PARAMETER,
                                        .query = kQuery,
                                        .queryCount = 4,
                                        .period = 10.0f,
                                        .requiredTags = {MANI_NO_PARAMETER, 2u},
                                        .excludedTags = {MANI_NO_PARAMETER, 0},
                                        .duration = {MANI_NO_PARAMETER, 0.0f}};
    s_nodes[0] = (maniGraphNodeSource){maniGraphNodeMatch, 0, NULL, .match = m};
    s_nodes[1] = (maniGraphNodeSource){maniGraphNodeMirror, 1, kFirst, .clip = {0}};
    s_nodes[1 + mirrors] = (maniGraphNodeSource){
        maniGraphNodeInertialize, 1, mirrors > 0 ? kSecond : kFirst,
        .inertialize = {
            MANI_NO_PARAMETER, maniGraphInertialization, {MANI_NO_PARAMETER, 0.0f}, 0, 0, 0}};
    return (maniGraphSource){.skeleton = &s_skeleton,
                             .parameters = kQueryParameters,
                             .clips = s_slots,
                             .nodes = s_nodes,
                             .parameterCount = 4,
                             .clipCount = 1,
                             .nodeCount = 2 + mirrors,
                             .output = 1 + mirrors,
                             .databaseCount = 1,
                             .databases = s_databaseSlot,
                             .mirrorMap = map};
}

static maniBuildResult BuildGraph(const maniGraphSource* source, size_t* sizeOut)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniBuilder_CopyBuilt(builder, s_graphFile, sizeof s_graphFile, sizeOut) ==
                  mani_success,
              "the graph copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniGraphAssets Assets(const maniMotionDatabase* const* databases)
{
    static const maniClip* clips[1];
    clips[0] = &s_walk;
    return (maniGraphAssets){.skeleton = &s_skeleton,
                             .clips = clips,
                             .databases = databases,
                             .clipCount = 1,
                             .databaseCount = 1,
                             .mirrorMap = &s_map};
}

static double Gap(maniVec3 a, maniVec3 b)
{
    return fmax(fabs((double)a.x - (double)b.x),
                fmax(fabs((double)a.y - (double)b.y), fabs((double)a.z - (double)b.z)));
}

static bool Same(const maniPose* a, const maniPose* b)
{
    bool same = true;
    for (uint32_t bone = 0; bone < kBones; ++bone)
    {
        maniTransform x = maniPose_GetLocal(a, bone);
        maniTransform y = maniPose_GetLocal(b, bone);
        maniQuat p = x.rotation;
        maniQuat q = y.rotation;
        same = same && Gap(x.translation, y.translation) < 1e-5 &&
               fabsf(p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w) > 1.0f - 1e-6f;
    }
    return same;
}

static uint64_t Hash(const char* name)
{
    return maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, strlen(name));
}

// Ten updates of a tenth of the mirrored entry under mirrors mirror
// nodes: each pose the walk's mirrored, each move reflected, the steps
// renamed; under a mirror node, the walk's as they are.
static void Play(uint32_t mirrors)
{
    size_t size = 0;
    maniGraphSource source = Graph(mirrors, &s_map);
    const maniMotionDatabase* databases[1] = {&s_mirrored};
    maniGraphAssets assets = Assets(databases);
    maniGraph g;
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(BuildGraph(&source, &size).result == mani_success &&
              maniOpenGraph(s_graphFile, size, &g).result == mani_success &&
              maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
                  mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a graph of the database bound");
    bool flipped = mirrors == 0;
    // The first search places the node somewhere in the mirrored entry.
    (void)maniGraphInstance_Update(instance, 0.0f, NULL, 0, NULL, NULL);
    float start = (float)instance->cells[g.parameterCount].node.time;
    bool poses = true;
    bool moves = true;
    uint64_t names[4] = {0};
    uint32_t heard = 0;
    for (int k = 1; k <= 10; ++k)
    {
        maniRootMove move;
        maniRootMove want;
        maniGraphEvent events[4];
        uint32_t count = 0;
        float from = fmodf(start + (float)(k - 1) * 0.1f, 1.0f);
        float t = fmodf(start + (float)k * 0.1f, 1.0f);
        poses =
            poses &&
            maniGraphInstance_Update(instance, 0.1f, events, 4, &count, &move) == mani_success &&
            maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_out) ==
                mani_success &&
            maniClip_Sample(&s_walk, &s_skeleton, t, s_expected) == mani_success &&
            (!flipped || maniMirrorMap_MirrorPose(&s_map, &s_skeleton, s_expected, s_expected) ==
                             mani_success) &&
            Same(s_out, s_expected);
        moves = moves && maniClip_GetRootMotion(&s_walk, from, 0.1f, &want) == mani_success;
        want = flipped ? maniMirrorMap_MirrorRootMotion(&s_map, want) : want;
        moves = moves && Gap(move.translation, want.translation) < 1e-5;
        for (uint32_t i = 0; i < count && heard < 4; ++i)
        {
            names[heard++] = events[i].nameHash;
        }
    }
    CHECK(poses,
          flipped ? "the mirrored entry's poses mirrored" : "under a mirror, the walk's poses");
    CHECK(moves, flipped ? "its moves reflected" : "under a mirror, the walk's moves");
    CHECK(heard == 2 && names[0] == Hash(flipped ? "stepR" : "stepL") && names[1] == Hash("jump"),
          flipped ? "its events renamed" : "under a mirror, the walk's events");
}

static void TestGraph(void)
{
    Play(0);
    Play(1);
    size_t size = 0;
    const maniMirrorMap* maps[2] = {NULL, &s_mapZ};
    for (int i = 0; i < 2; ++i)
    {
        maniGraphSource source = Graph(0, maps[i]);
        maniBuildResult r = BuildGraph(&source, &size);
        CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonMirrorGraph &&
                  r.item == 0 && r.other == 3,
              i == 0 ? "a mirrored database without the graph's map" : "with another map");
    }
    maniGraphSource source = Graph(0, &s_map);
    maniGraph g;
    CHECK(BuildGraph(&source, &size).result == mani_success &&
              maniOpenGraph(s_graphFile, size, &g).result == mani_success,
          "built and opened");
    // A view of the database through another map.
    maniMotionDatabase other = s_mirrored;
    other.mirrorMapIdentity = s_mapZ.identity;
    const maniMotionDatabase* databases[1] = {&other};
    maniGraphAssets assets = Assets(databases);
    maniGraphBinding* binding = NULL;
    uint32_t unbound = 0;
    CHECK(maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, &unbound) ==
                  mani_errorInvalid &&
              unbound == 1,
          "a database through another map unbound");
}

int main(void)
{
    Setup();
    TestFeatures();
    TestRefusals();
    TestOpen();
    TestGraph();
    return s_failures == 0 ? 0 : 1;
}
