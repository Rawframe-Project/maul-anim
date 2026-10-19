// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion matching nodes (mani-0024): the first search's pick, played as
// its clip (pose, root motion across the walk's loop, events with the
// node and clip slot); searches on the period and on the trigger
// (consumed), each jump requesting the node's duration of the
// inertialize node above while the first pick requests nothing; a
// clamped entry's end searched from no continuing frame, the end
// exclusion and the history keeping picks off the frames just left;
// required tags; a seek above clearing the node; a bone channel's query
// taken from the continuing frame, against the search itself; a
// normalized query taken as it is; the continuing frame of a restored
// playhead between binary32 and binary64 frame times.

#include "graph_match_fixture.h"
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
              maniCopyBuilt(builder, file, sizeof file, &size) == mani_success &&
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
        (void)maniUpdateGraph(instance, 0.15f, NULL, 0, NULL, &plain[k]);
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
        maniRootMove want = maniMirrorRootMotion(&map, plain[k]);
        reflected = reflected &&
                    maniUpdateGraph(instance, 0.15f, NULL, 0, NULL, &move) == mani_success &&
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
    maniRootMove move = maniRootMoveIdentity();
    CHECK(maniUpdateGraph(instance, 0.25f, NULL, 0, NULL, &move) == mani_success &&
              Playing(instance, &g, 0, 0, 0.25) && NearF(move.translation.x, 0.25),
          "a quarter of the walk, its root a quarter along");
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success &&
              maniSampleClip(&s_clips[0], &s_skeleton, 0.25f, s_clipPose) == mani_success &&
              NearF(maniPoseLocal(s_pose, 0).translation.x,
                    (double)maniPoseLocal(s_clipPose, 0).translation.x),
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
              maniEvaluateGraph(run, s_scratch, sizeof s_scratch, s_pose) == mani_success &&
              fabs((double)maniPoseLocal(s_pose, 0).translation.x - 1.2) <= 1e-5,
          "the run's pose at its playhead");
    instance = Matching(MatchNode(10.0f), &g);
    Update(instance, 0.0f);
    (void)maniUpdateGraph(instance, 0.25f, NULL, 0, NULL, &move);
    maniGraphEvent events[4];
    uint32_t count = 0;
    CHECK(maniUpdateGraph(instance, 0.9f, events, 4, &count, &move) == mani_success &&
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
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "evaluated");
    return maniPoseLocal(s_pose, 0).translation.x;
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
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success &&
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
              maniGetGraphParameter(instance, 1, &v) == mani_success && v.b == 0,
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
    CHECK(maniUpdateGraph(instance, 2.3f, matched, 16, &matchedCount, NULL) == mani_success,
          "matched");
    maniGraphNodeSource clip[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    instance = Make((maniGraphSource){.nodes = clip, .nodeCount = 1}, &g);
    Update(instance, 0.25f);
    CHECK(maniUpdateGraph(instance, 2.3f, played, 16, &playedCount, NULL) == mani_success,
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
    const maniGraphDatabaseSource slot[1] = {{&bones, "bones", 5, 0}};
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
    CHECK(maniMotionDatabaseEntry(&bones, entry, &e) == mani_success, "an entry");
    uint32_t current = e.firstFrame + (uint32_t)floor((t - (double)e.start) * 10.0 + 1e-9);
    float features[MANI_MATCH_MAX_FEATURES];
    float offsets[MANI_MATCH_MAX_FEATURES];
    float scales[MANI_MATCH_MAX_FEATURES];
    CHECK(maniMotionDatabaseFeatures(&bones, current, features) == mani_success &&
              maniMotionDatabaseNormalization(&bones, offsets, scales) == mani_success,
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
    CHECK(maniSearchMotionDatabase(&bones, &rq, &rr) == mani_success &&
              maniMatchSearch(&bones, &nq, true, &nr) == mani_success && rr.frame == nr.frame &&
              memcmp(&rr.cost, &nr.cost, sizeof rr.cost) == 0,
          "a normalized query taken as it is");
    float time = (float)t;
    CHECK(maniMatchSearch(&bones, &q, true, &r) == mani_success &&
              (r.frame == current ||
               maniMotionDatabaseFrame(&bones, r.frame, &entry, &time) == mani_success),
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
    const maniGraphDatabaseSource slot[1] = {{&bones, "bones", 5, 0}};
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
    size_t size = maniGraphStateSize(instance->binding);
    CHECK(maniSaveGraphInstance(instance, s_saved, sizeof s_saved) == mani_success, "saved");
    uint32_t block = MANI_GRAPH_STATE_HEADER_SIZE + 6 * MANI_GRAPH_VALUE_SIZE;
    uint32_t time = MANI_GRAPH_STATE_HEADER_SIZE + 4 * MANI_GRAPH_VALUE_SIZE + 8;
    double at = (double)0.7f;
    uint64_t bits = 0;
    memcpy(&bits, &at, sizeof bits);
    maniStoreU32(s_saved + block + 4 * maniGraphMatchEntry, 1);
    maniStoreU64(s_saved + time, bits);
    CHECK(maniRestoreGraphInstance(instance, s_saved, size).result == mani_success, "restored");
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.0f);
    double t = State(instance, &g, 0)->time;
    CHECK(EntryOf(instance, &g, 0) == 1 &&
              (fabs(t - (double)0.6f) <= 1e-9 || fabs(t - (double)0.8f) <= 1e-9),
          "a neighbor of frame 17");
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
    TestMirrored();
    return s_failures == 0 ? 0 : 1;
}
