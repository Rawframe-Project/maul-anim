// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Dead blending (mani-0023): the walk extrapolated from its last output
// at its speed and blended into the run, with the least, the computed
// and the longest half-life; a still turn slerped into the next; scale
// in log space, an axis turned negative holding; a turn's speed and
// distance on the shorter arc; a speed below 1e-8 by its sign; no speed
// over a step of 0; a decay rate of 0 carrying the speed on; saved
// mid-blend and restored, a negative decay rate refused.

#include "graph_dead_blend.h"
#include "graph_inertia_fixture.h"
#include "pose_memory.h"

// Dead blending's capture word k of bone b: the last output 0 to 9, the
// speeds 10 to 18, their decay rates 19 to 27.
static float DeadWord(maniGraphInstance* instance, const maniGraph* g, uint32_t b, uint32_t k)
{
    return maniGraphInertiaGet(Block(instance, g, 3), CaptureAt(maniGraphDeadBlending, b) + k);
}

// The walk extrapolated from x at speed v, decaying at y, t seconds in,
// blended into the run's xIn by the smoothstep of t over 0.5.
static double Extrapolated(double x, double v, double y, double t, double xIn)
{
    double e = x + v * (1.0 - exp(-y * t)) / y;
    double u = t / 0.5;
    double w = u * u * (3.0 - 2.0 * u);
    return e + (xIn - e) * w;
}

// The walk at twice its speed away from the run: the least half-life;
// backward toward it: scale times distance over speed; slowly toward
// it: the most. Each extrapolated and blended into the run, the run
// alone at the half second.
static void TestDead(void)
{
    static const float kSpeeds[3] = {2.0f, -1.0f, -0.1f};
    static const double kLast[3] = {0.6, 0.7, 0.97};
    static const double kHalfLives[3] = {0.1, 0.21, 1.0};
    for (int k = 0; k < 3; ++k)
    {
        maniGraph g;
        maniGraphInstance* instance = Eased(Dead(), 3, kSpeeds[k], 1, &g);
        Steps(instance, 3);
        Pull(instance, 0);
        maniTransform x = Step(instance, 0.1f, 0);
        double y = log(2.0) / kHalfLives[k];
        CHECK(NearF(DeadWord(instance, &g, 0, 0), kLast[k]) &&
                  fabs((double)DeadWord(instance, &g, 0, 10) - (double)kSpeeds[k]) < 1e-4 &&
                  fabs((double)DeadWord(instance, &g, 0, 19) - y) < 1e-3 &&
                  DeadWord(instance, &g, 0, 11) == 0.0f && DeadWord(instance, &g, 0, 13) == 0.0f,
              "captured: the last output, its speed, its half-life");
        double want = Extrapolated(kLast[k], (double)kSpeeds[k], y, 0.1, 0.0);
        CHECK(fabs((double)x.translation.x - want) < 1e-4 && x.translation.y == 1.0f,
              "extrapolated and blended into the run");
        for (int s = 1; s <= 3; ++s)
        {
            x = Step(instance, 0.1f, 0);
            want = Extrapolated(kLast[k], (double)kSpeeds[k], y, 0.1 * (s + 1), 0.3 * s);
            CHECK(fabs((double)x.translation.x - want) < 1e-4, "blending on");
        }
        x = Step(instance, 0.1f, 0);
        CHECK(Run(x.translation.x, 0.4) &&
                  (Block(instance, &g, 3)[0] & maniGraphInertiaActive) == 0,
              "the run alone at the half second");
    }
}

// The spine still in turn A, then turn B: no speed, so turn A slerped
// into turn B by the smoothstep.
static void TestDeadRotation(void)
{
    maniGraph g;
    maniGraphNodeSource node = Dead();
    node.inertialize.trigger = MANI_NO_PARAMETER;
    maniGraphInstance* instance = Eased(node, 7, 1.0f, 8, &g);
    Steps(instance, 2);
    Pull(instance, 0);
    maniTransform x = Step(instance, 0.1f, 1);
    maniQuat a = {0.0f, 0.0f, 1.0f, 0.0f};
    maniQuat b = {0.0f, 0.0f, -0.9f, 0.43588989f};
    maniQuat want = maniQuatSlerp(a, b, 0.104f);
    CHECK(DeadWord(instance, &g, 1, 15) == 0.0f && NearF(x.rotation.z, (double)want.z) &&
              NearF(x.rotation.w, (double)want.w),
          "turn A slerped into turn B");
    for (int k = 0; k < 4; ++k)
    {
        x = Step(instance, 0.1f, 1);
    }
    CHECK(NearF(x.rotation.z, -0.9) && NearF(x.rotation.w, 0.43588989), "turn B alone at the end");
}

static _Alignas(32) uint32_t s_deadBlock[MANI_GRAPH_INERTIA_BLOCK(3, 1)];

// Scale in log space, captured and applied directly: axes growing from
// 1.8 to 2 toward 1 decay at the least half-life; an axis now negative
// (positive before) takes no speed and holds.
static void TestDeadScale(void)
{
    maniGraphInertia node = {3, maniGraphDeadBlending, 0.3f, 0.1f, 1.0f};
    maniGraphResetInertia(s_deadBlock, &node);
    maniGraphTrs last = {{0, 0, 0}, {0, 0, 0, 1}, {2.0f, 2.0f, -1.0f}};
    maniGraphTrs before = {{0, 0, 0}, {0, 0, 0, 1}, {1.8f, 1.8f, 1.5f}};
    maniGraphTrsHold(s_deadBlock, maniGraphInertiaHistory(s_deadBlock, 3, true), 0, last);
    maniGraphTrsHold(s_deadBlock, maniGraphInertiaHistory(s_deadBlock, 3, false), 0, before);
    maniPoseSetLocal(s_pose, 0, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1.0f, 1.0f, 1.0f}});
    maniGraphDeadCapture(s_deadBlock, &node, maniPoseLocals(s_pose), true, 0.1f);
    double v = log(2.0 / 1.8) / 0.1;
    double y = log(2.0) / 0.1;
    float* words = (float*)(s_deadBlock + CaptureAt(maniGraphDeadBlending, 0));
    CHECK(fabs((double)words[16] - v) < 1e-4 && fabs((double)words[17] - v) < 1e-4 &&
              words[18] == 0.0f && fabs((double)words[25] - y) < 1e-3,
          "the scale's speed in log space, none for a negative axis");
    maniGraphDeadApply(s_deadBlock, &node, maniPoseLocals(s_pose), 0.1f, 0.5f);
    maniTransform x = maniPoseLocal(s_pose, 0);
    double e = 2.0 * exp(v * (1.0 - exp(-y * 0.1)) / y);
    CHECK(fabs((double)x.scale.x - (e + (1.0 - e) * 0.104)) < 1e-4 &&
              fabs((double)x.scale.y - (e + (1.0 - e) * 0.104)) < 1e-4 &&
              NearF(x.scale.z, -1.0 + 2.0 * 0.104),
          "extrapolated by its logarithm, blended");
}

static float DeadAt(uint32_t k)
{
    return maniGraphInertiaGet(s_deadBlock, CaptureAt(maniGraphDeadBlending, 0) + k);
}

// Bone 0 kept as last and before (dt seconds apart), the input set,
// captured for dead blending.
static void DeadCaptures(maniGraphTrs last, maniGraphTrs before, maniTransform input, float dt)
{
    maniGraphInertia node = {3, maniGraphDeadBlending, 0.3f, 0.1f, 1.0f};
    maniGraphResetInertia(s_deadBlock, &node);
    maniGraphTrsHold(s_deadBlock, maniGraphInertiaHistory(s_deadBlock, 3, true), 0, last);
    maniGraphTrsHold(s_deadBlock, maniGraphInertiaHistory(s_deadBlock, 3, false), 0, before);
    maniPoseSetLocal(s_pose, 0, input);
    maniGraphDeadCapture(s_deadBlock, &node, maniPoseLocals(s_pose), true, dt);
}

// A turn closing on the input at 1 per second, kept as the negated
// quaternion: its speed and distance on the shorter arc, its half-life
// scale times distance over speed, extrapolated and slerped; a speed
// below 1e-8 toward the input taking the longest half-life.
static void TestDeadTurn(void)
{
    maniQuat q = AboutZ(0.4f);
    maniGraphTrs last = {{0, 0, 0}, {-q.x, -q.y, -q.z, -q.w}, {1, 1, 1}};
    maniGraphTrs before = {{0, 0, 0}, AboutZ(0.5f), {1, 1, 1}};
    DeadCaptures(last, before, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}, 0.1f);
    double y = log(2.0) / 0.12;
    CHECK(fabs((double)DeadAt(15) + 1.0) < 1e-3 && fabs((double)DeadAt(24) - y) < 1e-2,
          "a turn's speed and half-life on the shorter arc");
    maniGraphInertia node = {3, maniGraphDeadBlending, 0.3f, 0.1f, 1.0f};
    maniGraphDeadApply(s_deadBlock, &node, maniPoseLocals(s_pose), 0.1f, 0.5f);
    maniTransform x = maniPoseLocal(s_pose, 0);
    double angle = (0.4 - (1.0 - exp(-y * 0.1)) / y) * (1.0 - 0.104);
    double got = 2.0 * atan2(fabs((double)x.rotation.z), fabs((double)x.rotation.w));
    CHECK(fabs(got - angle) < 1e-4, "extrapolated and slerped");
    maniGraphTrs slow = {{1.0f, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    maniGraphTrs slower = {{1.00000012f, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    DeadCaptures(slow, slower, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}, 1000.0f);
    CHECK(DeadAt(10) < 0.0f && DeadAt(10) > -1e-8f && fabs((double)DeadAt(19) - log(2.0)) < 1e-5,
          "a tiny speed toward the input: the longest half-life");
}

// Two outputs from one instant (a step of 0) give no speed; a decay rate
// of 0, which only a saved state holds, carries the speed on unbroken.
static void TestDeadEdges(void)
{
    maniGraphTrs last = {{1.0f, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    maniGraphTrs before = {{0.9f, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    DeadCaptures(last, before, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}, 0.0f);
    CHECK(DeadAt(10) == 0.0f, "a step of 0: no speed");
    DeadCaptures(last, before, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}, 0.1f);
    uint32_t at = CaptureAt(maniGraphDeadBlending, 0);
    maniGraphInertiaSet(s_deadBlock, at + 19, 0.0f);
    maniGraphInertia node = {3, maniGraphDeadBlending, 0.3f, 0.1f, 1.0f};
    maniGraphDeadApply(s_deadBlock, &node, maniPoseLocals(s_pose), 0.1f, 0.5f);
    CHECK(NearF(maniPoseLocal(s_pose, 0).translation.x, (1.0 + 1.0 * 0.1) * (1.0 - 0.104)),
          "no decay: the speed carried on");
}

static _Alignas(8) uint8_t s_saved[2][8192];
static _Alignas(8) uint8_t s_other[16384];

// Dead blending saved mid-blend and restored: both go on alike; a
// negative decay rate refused at its word.
static void TestDeadSaved(void)
{
    maniGraph g;
    maniGraphInstance* instance = Eased(Dead(), 3, -1.0f, 1, &g);
    Steps(instance, 2);
    Pull(instance, 0);
    Steps(instance, 1);
    size_t size = maniGraphStateSize(instance->binding);
    maniGraphInstance* other = NULL;
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniRestoreGraphInstance(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    maniTransform a = Step(instance, 0.1f, 0);
    maniTransform b = Step(other, 0.1f, 0);
    CHECK(a.translation.x == b.translation.x && a.translation.x != 0.0f, "blended alike");
    size_t block = size - 4 * (size_t)MANI_GRAPH_INERTIA_BLOCK(3, maniGraphDeadBlending);
    uint32_t capture = CaptureAt(maniGraphDeadBlending, 2);
    const uint32_t kBad[3][2] = {
        {capture + 4, 0x7F800000u}, {capture + 18, 0x7FC00000u}, {capture + 27, 0xBF800000u}};
    for (int k = 0; k < 3; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kBad[k][0], kBad[k][1]);
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kBad[k][0],
              "a saved block refused at its word");
    }
}

int main(void)
{
    Open();
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success,
          "a pose");
    TestDead();
    TestDeadRotation();
    TestDeadScale();
    TestDeadTurn();
    TestDeadEdges();
    TestDeadSaved();
    return s_failures == 0 ? 0 : 1;
}
