// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion matching nodes: the first search's pick, played as
// its clip (pose, root motion across the walk's loop, events with the
// node and clip slot); searches on the period and on the trigger
// (consumed), each jump requesting the node's duration of the
// inertialize node above while the first pick requests nothing; a
// clamped entry's end searched from no continuing frame, the end
// exclusion and the history keeping picks off the frames just left;
// required tags; a seek above clearing the node; the largest deltas
// under a time scale, the state saved and restored after them; a bone channel's query
// taken from the continuing frame, against the search itself; a
// normalized query taken as it is; the continuing frame of a restored
// playhead between binary32 and binary64 frame times; the query
// interpolated to a playhead between frames, clamped and looping; a
// request for the other clip's travel taken at the first search; a bone
// facing made unit again after the blend.

#include "graph_match_fixture.h"
#include "graph_match_step.h"
#include "match_search.h"

#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[4096];
static _Alignas(32) uint8_t s_clipPoseMemory[4096];
static maniPose* s_pose;
static maniPose* s_clipPose;

// The walk under a mirror node: each move the plain node's reflected
// (after the first, the walk to the left).
static void TestMirrored(void)
{
    static _Alignas(16) uint8_t file[4096];
    const maniMirrorMapSource ms = {.skeleton = &s_skeleton, .axis = maniMirrorAxisX};
    maniMirrorMap map;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, &ms).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenMirrorMap(file, size, &map).result == mani_success,
          "a mirror map");
    maniDestroyBuilder(builder);
    MatchParameters();
    static const uint32_t kMatch[1] = {0};
    // The node under an inertialize node, which a jump requests; then a
    // mirror between them.
    maniGraphNodeSource nodes[3] = {MatchNode(0.1f), Above(0), Above(1)};
    nodes[1] = (maniGraphNodeSource){maniGraphNodeMirror, 1, kMatch, .clip = {0}};
    static const uint32_t kMirror[1] = {1};
    nodes[2].inputs = kMirror;
    maniGraphNodeSource plainNodes[2] = {MatchNode(0.1f), Above(0)};
    maniGraphSource source = {.parameters = s_matchParameters,
                              .nodes = plainNodes,
                              .parameterCount = 4,
                              .nodeCount = 2,
                              .output = 1};
    maniGraph g;
    maniGraphInstance* instance = MakeMatch(source, &g);
    maniRootMove plain[6];
    for (int k = 0; k < 6; ++k)
    {
        (void)maniGraphInstance_Update(instance, 0.15f, NULL, 0, NULL, &plain[k]);
    }
    source.nodes = nodes;
    source.nodeCount = 3;
    source.output = 2;
    source.mirrorMap = &map;
    instance = MakeMatch(source, &g);
    bool reflected = true;
    for (int k = 0; k < 6; ++k)
    {
        maniRootMove move;
        maniRootMove want = maniMirrorMap_MirrorRootMotion(&map, plain[k]);
        reflected =
            reflected &&
            maniGraphInstance_Update(instance, 0.15f, NULL, 0, NULL, &move) == mani_success &&
            NearF(move.translation.x, (double)want.translation.x) &&
            NearF(move.translation.z, (double)want.translation.z) &&
            NearF(move.rotation.y, (double)want.rotation.y) &&
            (k == 0 || want.translation.x < 0.0f);
    }
    CHECK(reflected, "each move the plain node's reflected");
}

static void TestFirst(void)
{
    maniGraph g;
    maniGraphInstance* instance = Matching(MatchNode(10.0f), &g);
    Update(instance, 0.0f);
    CHECK(Playing(instance, &g, 0, 0, 0.0), "the walk's first frame: every walk frame ties");
    CHECK(isinf(Pending(instance, &g, 1)), "the first pick requests nothing");
    instance = Matching(MatchNode(10.0f), &g);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.0f);
    CHECK(Playing(instance, &g, 0, 1, 0.0), "standing still: the run's first frame");
    maniGraphValue v = {{0}};
    v.i = 2;
    maniGraphNodeSource tagged = MatchNode(10.0f);
    tagged.match.requiredTags = (maniGraphTags){2, 0};
    instance = Matching(tagged, &g);
    Set(instance, 2, maniGraphParameterInt, v);
    Update(instance, 0.0f);
    CHECK(Playing(instance, &g, 0, 1, 0.0), "walking but asking for the run's tags");
}

// The walk played: its pose, its root motion across its loop, its steps.
static void TestPlay(void)
{
    maniGraph g;
    maniGraphInstance* instance = Matching(MatchNode(10.0f), &g);
    Update(instance, 0.0f);
    maniRootMove move = maniMakeRootMoveIdentity();
    CHECK(maniGraphInstance_Update(instance, 0.25f, NULL, 0, NULL, &move) == mani_success &&
              Playing(instance, &g, 0, 0, 0.25) && NearF(move.translation.x, 0.25),
          "a quarter of the walk, its root a quarter along");
    CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                  mani_success &&
              maniClip_Sample(&s_clips[0], &s_skeleton, 0.25f, s_clipPose) == mani_success &&
              NearF(maniPose_GetLocal(s_pose, 0).translation.x,
                    (double)maniPose_GetLocal(s_clipPose, 0).translation.x),
          "the walk's pose at its playhead");
    // The run moves its hips in the pose (the walk's root motion leaves
    // them still): its pose at the node's playhead.
    maniGraphNodeSource running = MatchNode(10.0f);
    running.match.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, 2};
    maniGraph h;
    maniGraphInstance* run = Matching(running, &h);
    Update(run, 0.0f);
    Update(run, 0.4f);
    CHECK(Playing(run, &h, 0, 1, 0.4) &&
              maniGraphInstance_Evaluate(run, s_scratch, sizeof s_scratch, s_pose) ==
                  mani_success &&
              fabs((double)maniPose_GetLocal(s_pose, 0).translation.x - 1.2) <= 1e-5,
          "the run's pose at its playhead");
    instance = Matching(MatchNode(10.0f), &g);
    Update(instance, 0.0f);
    (void)maniGraphInstance_Update(instance, 0.25f, NULL, 0, NULL, &move);
    maniGraphEvent events[4];
    uint32_t count = 0;
    CHECK(maniGraphInstance_Update(instance, 0.9f, events, 4, &count, &move) == mani_success &&
              Playing(instance, &g, 0, 0, 0.15) && fabs((double)move.translation.x - 0.9) <= 1e-5,
          "across the loop: back at 0.15, the root 0.9 along");
    CHECK(count == 2 && events[0].node == 0 && events[0].clip == 0 && events[0].payload == 8 &&
              events[1].payload == 7 && events[1].direction == 1,
          "the right step, then the left as it wraps");
}

// An update and its evaluation; the hips' x.
static float Step(maniGraphInstance* instance, float delta)
{
    Update(instance, delta);
    CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "evaluated");
    return maniPose_GetLocal(s_pose, 0).translation.x;
}

static void TestPeriod(void)
{
    maniGraph g;
    maniGraphInstance* instance = Matching(MatchNode(0.1f), &g);
    (void)Step(instance, 0.0f);
    Ahead(instance, 0.0f, 0.0f);
    (void)Step(instance, 0.05f);
    CHECK(Playing(instance, &g, 0, 0, 0.05) && isinf(Pending(instance, &g, 1)),
          "no search before the period");
    Update(instance, 0.05f);
    CHECK(Playing(instance, &g, 0, 1, 0.0) && Pending(instance, &g, 1) == 0.25f,
          "the period passed: the run, eased over d");
    const uint32_t* eased = NodeBlock(instance, &g, 1, maniGraphInertiaBlock);
    CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_pose) ==
                  mani_success &&
              (eased[maniGraphInertiaFlags] & maniGraphInertiaActive) != 0 &&
              maniGraphInertiaGet(eased, maniGraphInertiaDuration) == 0.25f,
          "the jump eased out");
    const uint32_t* block = NodeBlock(instance, &g, 0, maniGraphMatchBlock);
    CHECK(block[maniGraphMatchClock] == 0, "the clock started over by the search");
    Update(instance, 0.05f);
    CHECK(Playing(instance, &g, 0, 1, 0.05) && block[maniGraphMatchClock] == 0x3D4CCCCDu,
          "the run plays on, its clock at a twentieth");
    // Walking on: every walk frame ties, so the continuing one wins and
    // nothing is jumped to or requested.
    instance = Matching(MatchNode(0.1f), &g);
    Update(instance, 0.0f);
    Update(instance, 0.15f);
    CHECK(Playing(instance, &g, 0, 0, 0.15) && isinf(Pending(instance, &g, 1)),
          "the continuing frame kept: no jump, no request");
}

static void TestTrigger(void)
{
    maniGraph g;
    maniGraphInstance* instance = Matching(MatchNode(10.0f), &g);
    Update(instance, 0.0f);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.1f);
    CHECK(Playing(instance, &g, 0, 0, 0.1), "no search without the trigger");
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.1f);
    maniGraphValue v;
    CHECK(Playing(instance, &g, 0, 1, 0.0) &&
              maniGraphInstance_GetParameter(instance, 1, &v) == mani_success && v.b == 0,
          "the trigger searches, and is consumed");
}

// The run clamped at its second: its end searched from no continuing
// frame; the end exclusion keeps its last fifth out, and a history keeps
// its start out once picked.
static void TestEnd(void)
{
    maniGraph g;
    maniGraphInstance* instance = Matching(MatchNode(10.0f), &g);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.0f);
    Update(instance, 0.95f);
    CHECK(Playing(instance, &g, 0, 1, 0.95), "the run near its end");
    Update(instance, 0.1f);
    CHECK(Playing(instance, &g, 0, 1, 0.0) && Pending(instance, &g, 1) == 0.25f,
          "at its end the run starts over");
    maniGraphNodeSource kept = MatchNode(10.0f);
    kept.match.historyCount = 1;
    kept.match.historyTime = 0.5f;
    instance = Matching(kept, &g);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.0f);
    Update(instance, 1.0f);
    CHECK(Playing(instance, &g, 0, 1, 0.5), "its first half kept out by the history");
    Update(instance, 0.5f);
    CHECK(Playing(instance, &g, 0, 1, 0.0), "the start again, the last pick its middle");
    const uint32_t* block = NodeBlock(instance, &g, 0, maniGraphMatchBlock);
    CHECK(block[maniGraphMatchPicks] == 1 && block[maniGraphMatchNext] == 0 &&
              block[maniGraphMatchHistory] == RUN_FIRST,
          "one pick kept, over the oldest");
}

// Under a select: not chosen it stays as it is, unweighed; chosen,
// through a time scale of 2, it moves twice as far.
static void TestScaled(void)
{
    MatchParameters();
    static const uint32_t kScaled[1] = {0};
    static const uint32_t kChoices[2] = {2, 1};
    maniGraphNodeSource nodes[5] = {
        MatchNode(10.0f),
        {maniGraphNodeTimeScale, 1, kScaled, .timeScale = {LITERAL(2.0f)}},
        ClipNode(1, 1.0f, maniGraphLoopRepeat),
        {maniGraphNodeSelect, 2, kChoices, .select = {2}},
        Above(3)};
    maniGraph g;
    maniGraphInstance* instance = MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                                              .nodes = nodes,
                                                              .parameterCount = 4,
                                                              .nodeCount = 5,
                                                              .output = 4},
                                            &g);
    // The select's first input, the run, is placed first: the node is 1.
    CHECK(maniLoadU16(g.nodes + MANI_GRAPH_NODE_SIZE) == maniGraphNodeMatch, "the node at 1");
    Update(instance, 0.5f);
    CHECK(EntryOf(instance, &g, 1) == MANI_GRAPH_MATCH_NO_ENTRY &&
              State(instance, &g, 1)->time == 0.0,
          "unweighed: no search, no move");
    maniGraphValue one = {{0}};
    one.i = 1;
    Set(instance, 2, maniGraphParameterInt, one);
    Update(instance, 0.0f);
    Update(instance, 0.1f);
    CHECK(Playing(instance, &g, 1, 0, 0.2), "reached at twice the time");
    // The largest delta a host can pass, twice: the clock is kept only
    // below the period, and the state saves and restores.
    Update(instance, FLT_MAX);
    Update(instance, FLT_MAX);
    CHECK(Restores(instance), "saved and restored after the largest deltas");
}

// A step of more than two loops lists the steps a clip node playing the
// walk lists.
static void TestWraps(void)
{
    maniGraphEvent matched[16];
    maniGraphEvent played[16];
    uint32_t matchedCount = 0;
    uint32_t playedCount = 0;
    maniGraph g;
    maniGraphInstance* instance = Matching(MatchNode(10.0f), &g);
    Update(instance, 0.0f);
    Update(instance, 0.25f);
    CHECK(maniGraphInstance_Update(instance, 2.3f, matched, 16, &matchedCount, NULL) ==
              mani_success,
          "matched");
    maniGraphNodeSource clip[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    instance = Make((maniGraphSource){.nodes = clip, .nodeCount = 1}, &g);
    Update(instance, 0.25f);
    CHECK(maniGraphInstance_Update(instance, 2.3f, played, 16, &playedCount, NULL) == mani_success,
          "played");
    bool same = matchedCount == playedCount && matchedCount >= 4;
    for (uint32_t k = 0; k < matchedCount && same; ++k)
    {
        same = matched[k].payload == played[k].payload && matched[k].time == played[k].time &&
               matched[k].direction == played[k].direction;
    }
    CHECK(same, "the steps of every loop");
}

static void TestSeek(void)
{
    MatchParameters();
    static const uint32_t kSeek[1] = {0};
    maniGraphNodeSource nodes[3] = {
        MatchNode(10.0f), {maniGraphNodeSeek, 1, kSeek, .seek = {1, LITERAL(0.0f)}}, Above(1)};
    nodes[0].match.trigger = MANI_NO_PARAMETER;
    maniGraph g;
    maniGraphInstance* instance = MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                                              .nodes = nodes,
                                                              .parameterCount = 4,
                                                              .nodeCount = 3,
                                                              .output = 2},
                                            &g);
    Update(instance, 0.0f);
    Update(instance, 0.3f);
    Ahead(instance, 0.0f, 0.0f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.3f);
    CHECK(Playing(instance, &g, 0, 1, 0.0) && isinf(Pending(instance, &g, 2)),
          "a seek above clears it: a first pick, no request");
}

// A bone channel's query comes from the continuing frame: the node's
// pick is the search's over that frame's features as they lie.
static _Alignas(16) uint8_t s_boneFile[1 << 15];

static void TestContinuing(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    maniMotionDatabase bones;
    BuildDatabase(channels, 2, s_boneFile, sizeof s_boneFile, &bones);
    const maniGraphDatabaseSource slot[1] = {{&bones, "bones", 5}};
    MatchParameters();
    maniGraphNodeSource nodes[2] = {MatchNode(10.0f), Above(0)};
    maniGraph g;
    maniGraphInstance* instance = MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                                              .nodes = nodes,
                                                              .parameterCount = 4,
                                                              .nodeCount = 2,
                                                              .output = 1,
                                                              .databaseCount = 1,
                                                              .databases = slot},
                                            &g);
    Update(instance, 0.0f);
    Update(instance, 0.32f);
    uint32_t entry = EntryOf(instance, &g, 0);
    double t = State(instance, &g, 0)->time;
    maniMatchEntry e;
    CHECK(maniMotionDatabase_GetEntry(&bones, entry, &e) == mani_success, "an entry");
    uint32_t current = e.firstFrame + (uint32_t)floor((t - (double)e.start) * 10.0 + 1e-9);
    float features[MANI_MAX_MATCH_FEATURES];
    float offsets[MANI_MAX_MATCH_FEATURES];
    float scales[MANI_MAX_MATCH_FEATURES];
    CHECK(maniMotionDatabase_GetFeatures(&bones, current, features, bones.featureCount, NULL) ==
                  mani_success &&
              maniMotionDatabase_GetNormalization(&bones, offsets, scales, bones.featureCount,
                                                  NULL) == mani_success,
          "the continuing frame's features");
    for (uint32_t j = 3; j < 5; ++j)
    {
        features[j] = ((j == 3 ? 0.2f : 0.0f) - offsets[j]) / scales[j];
    }
    maniMatchQuery q = {.features = features, .current = current, .endExclusion = 0.2f};
    maniMatchResult r;
    // A raw query normalized by hand searches as the raw one does.
    const float raw[5] = {0.4f, 1.1f, -0.2f, 0.3f, 0.05f};
    float normalized[5];
    for (uint32_t j = 0; j < 5; ++j)
    {
        normalized[j] = (raw[j] - offsets[j]) / scales[j];
    }
    maniMatchQuery rq = {.features = raw, .current = MANI_MATCH_NO_FRAME};
    maniMatchQuery nq = {.features = normalized, .current = MANI_MATCH_NO_FRAME};
    maniMatchResult rr;
    maniMatchResult nr;
    CHECK(maniMotionDatabase_Search(&bones, &rq, &rr) == mani_success &&
              maniMatchSearch(&bones, &nq, true, &nr) == mani_success && rr.frame == nr.frame &&
              memcmp(&rr.cost, &nr.cost, sizeof rr.cost) == 0,
          "a normalized query taken as it is");
    float time = (float)t;
    CHECK(maniMatchSearch(&bones, &q, true, &r) == mani_success &&
              (r.frame == current ||
               maniMotionDatabase_GetFrame(&bones, r.frame, &entry, &time) == mani_success),
          "the search");
    Ahead(instance, 0.2f, 0.0f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.0f);
    CHECK(Playing(instance, &g, 0, entry, (double)time), "the node's pick is the search's");
}

// The continuing frame of a playhead restored on a frame time whose
// binary64 product with the rate falls short of it (0.7f in the run:
// 6.9999998...): frame 17, so with the continuing frame priced out the
// pick is a neighbor of it, 16 or 18.
static uint8_t s_saved[4096];
static _Alignas(16) uint8_t s_neighborFile[1 << 15];

static void TestContinuingFrame(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    maniMotionDatabase bones;
    BuildDatabase(channels, 2, s_neighborFile, sizeof s_neighborFile, &bones);
    const maniGraphDatabaseSource slot[1] = {{&bones, "bones", 5}};
    MatchParameters();
    maniGraphNodeSource node = MatchNode(10.0f);
    node.match.continuingBias = 1000.0f;
    node.match.endExclusion = 0.0f;
    node.match.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, 2};
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    maniGraph g;
    maniGraphInstance* instance = MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                                              .nodes = nodes,
                                                              .parameterCount = 4,
                                                              .nodeCount = 2,
                                                              .output = 1,
                                                              .databaseCount = 1,
                                                              .databases = slot},
                                            &g);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.0f);
    size_t size = SavedGraphSize(instance);
    CHECK(maniGraphInstance_Save(instance, s_saved, sizeof s_saved, NULL) == mani_success, "saved");
    uint32_t block = MANI_GRAPH_STATE_HEADER_SIZE + 6 * MANI_GRAPH_VALUE_SIZE;
    uint32_t time = MANI_GRAPH_STATE_HEADER_SIZE + 4 * MANI_GRAPH_VALUE_SIZE + 8;
    double at = (double)0.7f;
    uint64_t bits = 0;
    memcpy(&bits, &at, sizeof bits);
    maniStoreU32(s_saved + block + 4 * maniGraphMatchEntry, 1);
    maniStoreU64(s_saved + time, bits);
    CHECK(maniGraphInstance_Restore(instance, s_saved, size).result == mani_success, "restored");
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.0f);
    double t = State(instance, &g, 0)->time;
    CHECK(EntryOf(instance, &g, 0) == 1 &&
              (fabs(t - (double)0.6f) <= 1e-9 || fabs(t - (double)0.8f) <= 1e-9),
          "a neighbor of frame 17");
}

// The playhead between two frames. On the run (its hips' x
// 3 t, frames 0.3 apart) at 0.79 s the continuing frame (0.7) is scored
// as at the playhead, so the node plays on where the frame's own
// features (2.1 against the query's 2.37) would lose to the frame at
// 0.8; with the continuing frame priced out, the query at 0.75 (2.25,
// interpolated toward the next frame) picks the frame at 0.8, where the
// continuing frame's own features lie half way to 0.6. On a looping
// entry (x t, frames to 0.9, its bone channel alone) at 0.93 s the last
// frame's features are interpolated toward the first's, 0.63, and pick
// the frame at 0.6 (the last frame's own, 0.9, would pick 0.8). A
// playhead whose frame guess is settled up a frame takes that frame's
// own features.
static _Alignas(16) uint8_t s_runFile[1 << 15];
static _Alignas(16) uint8_t s_loopFile[1 << 15];

// The node restored on entry at time, then searched; its query the
// parameter ahead, or none.
static double Searched(const maniMotionDatabase* d, uint32_t tags, uint32_t entry, double at,
                       bool ahead, float bias, uint32_t* entryOut)
{
    const maniGraphDatabaseSource slot[1] = {{d, "bones", 5}};
    MatchParameters();
    maniGraphNodeSource node = MatchNode(10.0f);
    node.match.endExclusion = 0.0f;
    node.match.requiredTags = (maniGraphTags){MANI_NO_PARAMETER, tags};
    node.match.queryCount = ahead ? 1u : 0u;
    node.match.continuingBias = bias;
    maniGraphNodeSource nodes[2] = {node, Above(0)};
    maniGraph g;
    maniGraphInstance* instance = MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                                              .nodes = nodes,
                                                              .parameterCount = 4,
                                                              .nodeCount = 2,
                                                              .output = 1,
                                                              .databaseCount = 1,
                                                              .databases = slot},
                                            &g);
    Ahead(instance, 0.0f, 0.0f);
    Update(instance, 0.0f);
    size_t size = SavedGraphSize(instance);
    CHECK(maniGraphInstance_Save(instance, s_saved, sizeof s_saved, NULL) == mani_success, "saved");
    uint32_t block = MANI_GRAPH_STATE_HEADER_SIZE + 6 * MANI_GRAPH_VALUE_SIZE;
    uint32_t time = MANI_GRAPH_STATE_HEADER_SIZE + 4 * MANI_GRAPH_VALUE_SIZE + 8;
    uint64_t bits = 0;
    memcpy(&bits, &at, sizeof bits);
    maniStoreU32(s_saved + block + 4 * maniGraphMatchEntry, entry);
    maniStoreU64(s_saved + time, bits);
    CHECK(maniGraphInstance_Restore(instance, s_saved, size).result == mani_success, "restored");
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.0f);
    *entryOut = EntryOf(instance, &g, 0);
    return State(instance, &g, 0)->time;
}

static void TestInterpolated(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    maniMotionDatabase run;
    BuildDatabase(channels, 2, s_runFile, sizeof s_runFile, &run);
    uint32_t entry = 0;
    double t = Searched(&run, 2, 1, 0.79, true, 0.0f, &entry);
    CHECK(entry == 1 && t == (double)0.79f, "the continuing frame scored at the playhead");
    t = Searched(&run, 2, 1, 0.75, true, 1000.0f, &entry);
    CHECK(entry == 1 && t == (double)0.8f, "the query toward the run's next frame");
    // A playhead on 0.7f, whose product with the rate falls short of 7:
    // the guess settled up to the frame at 0.7f, weight 0, so with a bias
    // between the frame at 0.8's cost from there (0.3 apart) and from half
    // way (0.15) the node plays on.
    float offsets[MANI_MAX_MATCH_FEATURES];
    float scales[MANI_MAX_MATCH_FEATURES];
    CHECK(maniMotionDatabase_GetNormalization(&run, offsets, scales, run.featureCount, NULL) ==
              mani_success,
          "scales");
    double near = 0.15 / (double)scales[0];
    double far = 0.3 / (double)scales[0];
    t = Searched(&run, 2, 1, (double)0.7f, true, (float)(0.5 * (near * near + far * far)), &entry);
    CHECK(entry == 1 && t == (double)0.7f, "settled up to a frame, its own features");
    const maniClip* clips[1] = {&s_clips[3]};
    const maniMatchEntrySource loop = {0, maniMatchEntryLoop, 0.0f, 1.0f, 0.0f, 1u};
    maniMotionDatabaseSource source = {&s_skeleton, clips, channels,       &loop,          1,    1,
                                       1,           10.0f, maniMatchAxisY, maniMatchAxisZ, NULL, 0};
    maniMotionDatabase looped;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_loopFile, sizeof s_loopFile, &size) ==
                  mani_success &&
              maniOpenMotionDatabase(s_loopFile, size, &looped).result == mani_success,
          "a looping database");
    maniDestroyBuilder(builder);
    t = Searched(&looped, 1, 0, 0.93, false, 1000.0f, &entry);
    CHECK(entry == 0 && t == (double)0.6f, "a loop's last frame toward its first");
}

// A request answered: under bone and trajectory channels, the
// walk playing, the parameter ahead set to x; the searches, one each
// 0.1 s, until the node plays the run (0 for none in 20).
static uint32_t SearchesToLeave(float x)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    maniMotionDatabase bones;
    BuildDatabase(channels, 2, s_boneFile, sizeof s_boneFile, &bones);
    const maniGraphDatabaseSource slot[1] = {{&bones, "bones", 5}};
    MatchParameters();
    maniGraphNodeSource nodes[2] = {MatchNode(0.1f), Above(0)};
    maniGraph g;
    maniGraphInstance* instance = MakeMatch((maniGraphSource){.parameters = s_matchParameters,
                                                              .nodes = nodes,
                                                              .parameterCount = 4,
                                                              .nodeCount = 2,
                                                              .output = 1,
                                                              .databaseCount = 1,
                                                              .databases = slot},
                                            &g);
    Update(instance, 0.0f);
    for (uint32_t i = 0; i < 10; ++i)
    {
        Update(instance, 0.1f);
    }
    CHECK(EntryOf(instance, &g, 0) == 0, "the walk playing");
    Ahead(instance, x, 0.0f);
    for (uint32_t k = 1; k <= 20; ++k)
    {
        Update(instance, 0.1f);
        if (EntryOf(instance, &g, 0) == 1)
        {
            return k;
        }
    }
    return 0;
}

// Every request nearer the run's travel (0) than the walk's (0.5) is
// taken at the first search, and one nearer the walk is not, as when the
// continuing frame was scored by its own features: playing on costs no
// bone difference, but the trajectory still decides.
static void TestLeaves(void)
{
    bool taken = true;
    for (int i = 0; i <= 4; ++i)
    {
        taken = taken && SearchesToLeave(0.05f * (float)i) == 1;
    }
    CHECK(taken, "a request nearer the run taken at the first search");
    CHECK(SearchesToLeave(0.3f) == 0, "one nearer the walk kept on the walk");
}

// The node's blend on a bone facing channel over the fixture's turnA and
// turnB (the spine held at two turns 52 degrees apart; the first bone
// and axis whose facing varies): half way between the two frames whose
// facings differ most, the facing unit again where the line alone falls
// short.
static _Alignas(16) uint8_t s_facingFile[1 << 15];

static void TestFacingBlend(void)
{
    static const float kNow[1] = {0.0f};
    const maniClip* clips[2] = {&s_clips[7], &s_clips[8]};
    const maniMatchEntrySource entries[2] = {{0, maniMatchEntryLoop, 0.0f, 1.0f, 0.0f, 1u},
                                             {1, 0, 0.0f, 1.0f, 0.0f, 2u}};
    maniMotionDatabase d;
    bool built = false;
    for (uint32_t k = 0; k < 9 && !built; ++k)
    {
        const maniMatchChannelSource channel = {
            maniMatchBoneFacing, k / 3, k % 3, 1, kNow, NULL, 0, 1.0f};
        maniMotionDatabaseSource source = {
            &s_skeleton, clips, &channel,       entries,        2,    1,
            2,           10.0f, maniMatchAxisY, maniMatchAxisZ, NULL, 0};
        maniBuilder* builder = NewBuilder();
        size_t size = 0;
        built = maniBuildMotionDatabase(builder, &source).result == mani_success &&
                maniBuilder_CopyBuilt(builder, s_facingFile, sizeof s_facingFile, &size) ==
                    mani_success &&
                maniOpenMotionDatabase(s_facingFile, size, &d).result == mani_success;
        maniDestroyBuilder(builder);
    }
    CHECK(built, "a facing channel that turns");
    if (!built)
    {
        return;
    }
    float offsets[MANI_MAX_MATCH_FEATURES];
    float scales[MANI_MAX_MATCH_FEATURES];
    CHECK(maniMotionDatabase_GetNormalization(&d, offsets, scales, d.featureCount, NULL) ==
              mani_success,
          "scales");
    float a[3];
    float b[3];
    float best[2][3];
    double far = -1.0;
    for (uint32_t f = 0; f + 1 < d.frameCount; ++f)
    {
        if (maniMotionDatabase_GetFeatures(&d, f, a, d.featureCount, NULL) != mani_success ||
            maniMotionDatabase_GetFeatures(&d, f + 1, b, d.featureCount, NULL) != mani_success)
        {
            continue;
        }
        double dot = 0.0;
        for (uint32_t i = 0; i < 3; ++i)
        {
            double s = (double)scales[i];
            double o = (double)offsets[i];
            dot += ((double)a[i] * s + o) * ((double)b[i] * s + o);
        }
        if (1.0 - dot > far)
        {
            far = 1.0 - dot;
            memcpy(best[0], a, sizeof a);
            memcpy(best[1], b, sizeof b);
        }
    }
    float line[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        line[i] = best[0][i] + 0.5f * (best[1][i] - best[0][i]);
    }
    maniGraphMatchBlend(&d, best[0], best[1], 0.5f);
    double lineLength = 0.0;
    double length = 0.0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        double s = (double)scales[i];
        double o = (double)offsets[i];
        double l = (double)line[i] * s + o;
        double v = (double)best[0][i] * s + o;
        lineLength += l * l;
        length += v * v;
    }
    CHECK(sqrt(lineLength) < 1.0 - 1e-4 && fabs(sqrt(length) - 1.0) < 1e-5,
          "the facing unit again where the line falls short");
}

int main(void)
{
    Open();
    OpenDatabase();
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_clipPoseMemory, sizeof s_clipPoseMemory, &s_clipPose) ==
                  mani_success,
          "poses");
    TestFirst();
    TestPlay();
    TestPeriod();
    TestTrigger();
    TestEnd();
    TestSeek();
    TestScaled();
    TestWraps();
    TestContinuing();
    TestContinuingFrame();
    TestInterpolated();
    TestLeaves();
    TestFacingBlend();
    TestMirrored();
    return s_failures == 0 ? 0 : 1;
}
