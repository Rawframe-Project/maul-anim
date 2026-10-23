// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The IK differential: random two-bone chains (random
// bone lengths and turns, goals in and out of reach, softening and
// weights, twists) solved by ozz-animation 0.17.0's IKTwoBoneJob and by
// maniSolveTwoBone, the middle and end bones' model positions compared;
// then random bones aimed by its IKAimJob and by maniSolveAim (below). Poles within a sine of 0.1
// of the goal line are left out: there ozz flips and Maul Anim blends to the chain's own plane;
// so are goals within a twentieth of the chain's length of the start (ozz turns the
// start toward them, Maul Anim does not) and weighted corrections of nearly half a turn (which way
// the weight turns is rounding's in both). Hinges lie perpendicular to the chain's bend plane, as a
// joint's do. Prints the worst difference over the chain's length and each solver's own error (the
// end's miss of a reachable goal, the middle's distance from the pole's plane); fails when the two
// differ past 5e-4 (ozz turns to the pole's plane from the cosine alone, off it by about 1e-4) or
// 2e-3 for softened goals (ozz softens with reciprocal estimates of about 12 bits), when "reached"
// differs, or when Maul Anim's own error passes 1e-5.
//
// Then times both: nanoseconds a two-bone solve and an aim take; then
// Maul Anim's CCD and FABRIK chains alone.
//
// usage: ik_differential [cases [timing repetitions]]

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "ozz/animation/runtime/ik_aim_job.h"
#include "ozz/animation/runtime/ik_two_bone_job.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/simd_quaternion.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace
{

alignas(16) uint8_t g_skeletonFile[4096];
alignas(64) uint8_t g_memory[2][4096];
uint64_t g_random = 0x9E3779B97F4A7C15ull;

float Random()
{
    g_random ^= g_random << 13;
    g_random ^= g_random >> 7;
    g_random ^= g_random << 17;
    return static_cast<float>(g_random >> 40) / 16777216.0f;
}

float Spread(float low, float high)
{
    return low + (high - low) * Random();
}

maniVec3 Direction()
{
    for (;;)
    {
        maniVec3 v = {Spread(-1, 1), Spread(-1, 1), Spread(-1, 1)};
        float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (l > 0.1f && l <= 1.0f)
        {
            return {v.x / l, v.y / l, v.z / l};
        }
    }
}

maniQuat Turn(maniVec3 axis, float angle)
{
    float s = std::sin(0.5f * angle);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(0.5f * angle)};
}

maniQuat Mul(maniQuat a, maniQuat b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

ozz::math::Float4x4 ToOzz(const maniAffine& a)
{
    using ozz::math::simd_float4::Load;
    return {{Load(a.m[0], a.m[1], a.m[2], 0.0f), Load(a.m[3], a.m[4], a.m[5], 0.0f),
             Load(a.m[6], a.m[7], a.m[8], 0.0f), Load(a.m[9], a.m[10], a.m[11], 1.0f)}};
}

maniVec3 Position(maniPose* pose, const maniSkeleton* skeleton, uint32_t bone)
{
    if (maniComputeModel(pose, skeleton) != mani_success)
    {
        std::abort();
    }
    maniAffine m = maniPoseModel(pose, bone);
    return {m.m[9], m.m[10], m.m[11]};
}

float Apart(maniVec3 a, maniVec3 b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return std::sqrt(x * x + y * y + z * z);
}

// The turn taking a onto b, both non-zero (the half-vector method).
maniQuat Between(maniVec3 a, maniVec3 b)
{
    float la = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    float lb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    maniVec3 h = {a.x / la + b.x / lb, a.y / la + b.y / lb, a.z / la + b.z / lb};
    float lh = std::sqrt(h.x * h.x + h.y * h.y + h.z * h.z);
    h = {h.x / lh, h.y / lh, h.z / lh};
    maniVec3 u = {a.x / la, a.y / la, a.z / la};
    return {u.y * h.z - u.z * h.y, u.z * h.x - u.x * h.z, u.x * h.y - u.y * h.x,
            u.x * h.x + u.y * h.y + u.z * h.z};
}

maniVec3 Rotate(maniQuat q, maniVec3 v)
{
    maniQuat p = Mul(Mul(q, maniQuat{v.x, v.y, v.z, 0.0f}), maniQuat{-q.x, -q.y, -q.z, q.w});
    return {p.x, p.y, p.z};
}

float Sine(maniVec3 a, maniVec3 b)
{
    maniVec3 c = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z) /
           (std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) *
            std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z));
}

// Random bones aimed by ozz's IKAimJob and by maniSolveAim: the turned
// bones' model rotations compared, and each one's miss of the goal (the
// forward line through the offset point, over the goal's distance).
// Poles and up axes (once the forward is on the goal) within a sine of
// 0.1 of the aim line, goals within 0.2 of the bone and weighted turns of
// nearly half a turn are left out; weighted turns are compared with
// ozz's full turn weighted as its two-bone job weights.
bool AimDifferential(maniPose* ours, maniPose* theirs, const maniSkeleton* skeleton, long cases)
{
    float worst = 0.0f;
    float missOurs = 0.0f;
    float missTheirs = 0.0f;
    float planeOurs = 0.0f;
    float planeTheirs = 0.0f;
    long compared = 0;
    long reachDiffers = 0;
    for (long c = 0; c < cases; ++c)
    {
        maniTransform local = {{Spread(-1, 1), Spread(0, 2), Spread(-1, 1)},
                               Turn(Direction(), Spread(-3, 3)),
                               {1, 1, 1}};
        maniPoseSetLocal(ours, 1, local);
        maniPoseSetLocal(theirs, 1, local);
        if (maniComputeModel(theirs, skeleton) != mani_success)
        {
            return false;
        }
        maniAffine model = maniPoseModel(theirs, 1);
        maniVec3 origin = {model.m[9], model.m[10], model.m[11]};
        maniVec3 forward = Direction();
        maniVec3 up = Direction();
        maniVec3 offset = {0, 0, 0};
        if (Random() < 0.5f)
        {
            maniVec3 d = Direction();
            float l = Spread(0.0f, 0.3f);
            offset = {d.x * l, d.y * l, d.z * l};
        }
        maniVec3 dir = Direction();
        float distance = Spread(0.2f, 3.0f);
        maniVec3 goal = {origin.x + dir.x * distance, origin.y + dir.y * distance,
                         origin.z + dir.z * distance};
        maniVec3 pole = Direction();
        float twist = Random() < 0.5f ? 0.0f : Spread(-1, 1);
        float weight = Random() < 0.5f ? 1.0f : 0.5f;
        // The up axis once the forward is on the goal.
        maniVec3 f = Rotate(local.rotation, forward);
        maniVec3 o = Rotate(local.rotation, offset);
        float ao = f.x * o.x + f.y * o.y + f.z * o.z;
        float ac2 = o.x * o.x + o.y * o.y + o.z * o.z - ao * ao;
        float r2 = distance * distance;
        if (Sine(pole, dir) < 0.1f || ac2 > r2)
        {
            continue;
        }
        float ai = std::sqrt(r2 - ac2) - ao;
        maniVec3 aimed = {o.x + f.x * ai, o.y + f.y * ai, o.z + f.z * ai};
        maniVec3 upAimed = Rotate(Between(aimed, dir), Rotate(local.rotation, up));
        if (Sine(upAimed, dir) < 0.1f)
        {
            continue;
        }
        maniAimSolve solve = {};
        solve.bones[0] = 1;
        solve.shares[0] = 1.0f;
        solve.count = 1;
        solve.goal = goal;
        solve.pole = pole;
        solve.forward = forward;
        solve.offset = offset;
        solve.up = up;
        solve.twist = twist;
        solve.weight = weight;
        bool reachedOurs = false;
        if (maniSolveAim(ours, skeleton, &solve, &reachedOurs) != mani_success)
        {
            return false;
        }
        const ozz::math::Float4x4 joint = ToOzz(model);
        ozz::animation::IKAimJob job;
        job.target = ozz::math::simd_float4::Load(goal.x, goal.y, goal.z, 1.0f);
        job.forward = ozz::math::simd_float4::Load(forward.x, forward.y, forward.z, 0.0f);
        job.offset = ozz::math::simd_float4::Load(offset.x, offset.y, offset.z, 1.0f);
        job.up = ozz::math::simd_float4::Load(up.x, up.y, up.z, 0.0f);
        job.pole_vector = ozz::math::simd_float4::Load(pole.x, pole.y, pole.z, 0.0f);
        job.twist_angle = twist;
        job.weight = 1.0f;
        job.joint = &joint;
        ozz::math::SimdQuaternion correction;
        bool reachedTheirs = false;
        job.joint_correction = &correction;
        job.reached = &reachedTheirs;
        if (!job.Run())
        {
            return false;
        }
        float q[4];
        ozz::math::StorePtrU(correction.xyzw, q);
        if (weight < 1.0f)
        {
            // ozz weights its aim by lerping the turn from the identity
            // without first making w positive (ik_aim_job.cc, unlike its
            // two-bone job), so a turn with w below 0 goes the long way,
            // and normalizes the result by an estimate: the full
            // correction is weighted here as its two-bone job does.
            if (std::fabs(q[3]) < 0.05f)
            {
                continue;
            }
            float sign = q[3] < 0.0f ? -1.0f : 1.0f;
            for (int k = 0; k < 4; ++k)
            {
                q[k] = (k == 3 ? 1.0f - weight : 0.0f) + weight * sign * q[k];
            }
            reachedTheirs = false;
        }
        float ql = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (float& v : q)
        {
            v /= ql;
        }
        maniTransform t = local;
        t.rotation = Mul(t.rotation, maniQuat{q[0], q[1], q[2], q[3]});
        maniPoseSetLocal(theirs, 1, t);
        maniQuat a = maniPoseLocal(ours, 1).rotation;
        maniQuat b = t.rotation;
        float dot = std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
        float angle = 2.0f * std::acos(std::fmin(1.0f, dot));
        worst = std::fmax(worst, angle);
        reachDiffers += reachedOurs != reachedTheirs;
        if (weight >= 1.0f && reachedOurs)
        {
            maniQuat rotations[2] = {a, b};
            float miss[2];
            for (int k = 0; k < 2; ++k)
            {
                maniVec3 ff = Rotate(rotations[k], forward);
                maniVec3 oo = Rotate(rotations[k], offset);
                maniVec3 from = {origin.x + oo.x, origin.y + oo.y, origin.z + oo.z};
                maniVec3 g = {goal.x - from.x, goal.y - from.y, goal.z - from.z};
                float along = g.x * ff.x + g.y * ff.y + g.z * ff.z;
                maniVec3 off = {g.x - ff.x * along, g.y - ff.y * along, g.z - ff.z * along};
                miss[k] = std::sqrt(off.x * off.x + off.y * off.y + off.z * off.z) / distance;
            }
            missOurs = std::fmax(missOurs, miss[0]);
            missTheirs = std::fmax(missTheirs, miss[1]);
            if (twist == 0.0f)
            {
                // The up axis's sine to the plane of the aim line and the
                // pole.
                maniVec3 n = {dir.y * pole.z - dir.z * pole.y, dir.z * pole.x - dir.x * pole.z,
                              dir.x * pole.y - dir.y * pole.x};
                float nl = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
                float off[2];
                for (int k = 0; k < 2; ++k)
                {
                    maniVec3 uu = Rotate(rotations[k], up);
                    off[k] = std::fabs(uu.x * n.x + uu.y * n.y + uu.z * n.z) / nl;
                }
                planeOurs = std::fmax(planeOurs, off[0]);
                planeTheirs = std::fmax(planeTheirs, off[1]);
            }
        }
        ++compared;
    }
    std::printf("aim: %ld cases compared, worst difference %.3g radians, reached differs in %ld\n"
                "  own errors, Maul Anim / ozz: the forward's miss %.3g / %.3g of the goal's "
                "distance, the up axis off the pole's plane %.3g / %.3g\n",
                compared, static_cast<double>(worst), reachDiffers, static_cast<double>(missOurs),
                static_cast<double>(missTheirs), static_cast<double>(planeOurs),
                static_cast<double>(planeTheirs));
    return worst <= 3e-3f && reachDiffers == 0 && missOurs <= 1e-5f && planeOurs <= 1e-5f;
}

double Now()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

// A sink the timed loops write to, so no solve is dropped.
volatile float g_sink = 0.0f;

// Nanoseconds a solve takes, over 256 random chains with goals in reach
// repeated reps times: Maul Anim's from the pose's locals (composing the
// root path itself), ozz's jobs on model matrices computed before the
// clock starts (their caller's model pass not counted). The two-bone
// solve, then an aim at the end bone.
void Timings(maniPose* pose, const maniSkeleton* skeleton, long reps)
{
    constexpr int kCases = 256;
    static maniTwoBoneSolve twoBone[kCases];
    static maniAimSolve aim[kCases];
    static ozz::math::Float4x4 models[kCases][3];
    for (int c = 0; c < kCases; ++c)
    {
        maniTransform t = maniPoseLocal(pose, 2);
        t.rotation = Turn(Direction(), Spread(0.2f, 2.5f));
        maniPoseSetLocal(pose, 2, t);
        if (maniComputeModel(pose, skeleton) != mani_success)
        {
            return;
        }
        for (uint32_t b = 0; b < 3; ++b)
        {
            models[c][b] = ToOzz(maniPoseModel(pose, b + 1));
        }
        maniVec3 start = Position(pose, skeleton, 1);
        maniVec3 d = Direction();
        float reach = Spread(0.2f, 0.9f);
        maniVec3 goal = {start.x + d.x * reach, start.y + d.y * reach, start.z + d.z * reach};
        twoBone[c] = {1, 2, 3, 0, goal, Direction(), {0, 0, 1}, 0.0f, 1.0f, 1.0f};
        aim[c] = {{3},       {1.0f},    1,         0,    goal, Direction(),
                  {1, 0, 0}, {0, 0, 0}, {0, 1, 0}, 0.0f, 0.0f, 1.0f};
    }
    float sink = 0.0f;
    double t0 = Now();
    for (long r = 0; r < reps; ++r)
    {
        for (int c = 0; c < kCases; ++c)
        {
            (void)maniSolveTwoBone(pose, skeleton, &twoBone[c], nullptr);
        }
        sink += maniPoseLocal(pose, 2).rotation.x;
    }
    double oursTwo = Now() - t0;
    t0 = Now();
    for (long r = 0; r < reps; ++r)
    {
        for (int c = 0; c < kCases; ++c)
        {
            ozz::animation::IKTwoBoneJob job;
            const maniTwoBoneSolve& s = twoBone[c];
            job.target = ozz::math::simd_float4::Load(s.goal.x, s.goal.y, s.goal.z, 1.0f);
            job.mid_axis = ozz::math::simd_float4::Load(0.0f, 0.0f, 1.0f, 0.0f);
            job.pole_vector = ozz::math::simd_float4::Load(s.pole.x, s.pole.y, s.pole.z, 0.0f);
            job.start_joint = &models[c][0];
            job.mid_joint = &models[c][1];
            job.end_joint = &models[c][2];
            ozz::math::SimdQuaternion corrections[2];
            job.start_joint_correction = &corrections[0];
            job.mid_joint_correction = &corrections[1];
            (void)job.Run();
            sink += ozz::math::GetX(corrections[1].xyzw);
        }
    }
    double ozzTwo = Now() - t0;
    t0 = Now();
    for (long r = 0; r < reps; ++r)
    {
        for (int c = 0; c < kCases; ++c)
        {
            (void)maniSolveAim(pose, skeleton, &aim[c], nullptr);
        }
        sink += maniPoseLocal(pose, 3).rotation.x;
    }
    double oursAim = Now() - t0;
    t0 = Now();
    for (long r = 0; r < reps; ++r)
    {
        for (int c = 0; c < kCases; ++c)
        {
            ozz::animation::IKAimJob job;
            const maniAimSolve& s = aim[c];
            job.target = ozz::math::simd_float4::Load(s.goal.x, s.goal.y, s.goal.z, 1.0f);
            job.forward = ozz::math::simd_float4::x_axis();
            job.up = ozz::math::simd_float4::y_axis();
            job.pole_vector = ozz::math::simd_float4::Load(s.pole.x, s.pole.y, s.pole.z, 0.0f);
            job.joint = &models[c][2];
            ozz::math::SimdQuaternion correction;
            job.joint_correction = &correction;
            (void)job.Run();
            sink += ozz::math::GetX(correction.xyzw);
        }
    }
    double ozzAim = Now() - t0;
    g_sink = sink;
    double n = static_cast<double>(reps) * kCases;
    std::printf("ns a solve, Maul Anim from locals / ozz on given models: two-bone %.0f / %.0f, "
                "aim %.0f / %.0f\n",
                oursTwo / n * 1e9, ozzTwo / n * 1e9, oursAim / n * 1e9, ozzAim / n * 1e9);
}

// Nanoseconds a CCD and a FABRIK solve take on the chain of the start,
// middle and end, 16 iterations at most with a tolerance of 1e-4, over
// 256 random goals within reach repeated reps times, each solve from the
// rest with the middle bent; ozz has no chain solver to time against.
void ChainTimings(maniPose* pose, const maniSkeleton* skeleton, long reps)
{
    constexpr int kCases = 256;
    static maniVec3 goals[kCases];
    if (maniPoseSetRest(pose, skeleton) != mani_success)
    {
        return;
    }
    maniTransform t = maniPoseLocal(pose, 2);
    t.rotation = Turn(Direction(), 0.8f);
    maniPoseSetLocal(pose, 2, t);
    maniTransform bent[4];
    for (uint32_t b = 0; b < 4; ++b)
    {
        bent[b] = maniPoseLocal(pose, b);
    }
    maniVec3 start = Position(pose, skeleton, 1);
    for (int c = 0; c < kCases; ++c)
    {
        maniVec3 d = Direction();
        float reach = Spread(0.1f, 0.9f);
        goals[c] = {start.x + d.x * reach, start.y + d.y * reach, start.z + d.z * reach};
    }
    double seconds[2] = {0.0, 0.0};
    float sink = 0.0f;
    for (int solver = 0; solver < 2; ++solver)
    {
        double t0 = Now();
        for (long r = 0; r < reps; ++r)
        {
            for (int c = 0; c < kCases; ++c)
            {
                for (uint32_t b = 1; b < 4; ++b)
                {
                    maniPoseSetLocal(pose, b, bent[b]);
                }
                maniChainSolve s = {};
                s.root = 1;
                s.end = 3;
                s.goal = goals[c];
                s.rotation = {0.0f, 0.0f, 0.0f, 1.0f};
                s.iterations = 16;
                s.tolerance = 1e-4f;
                s.weight = 1.0f;
                (void)(solver == 0 ? maniSolveCcd(pose, skeleton, &s, nullptr)
                                   : maniSolveFabrik(pose, skeleton, &s, nullptr));
                sink += maniPoseLocal(pose, 2).rotation.x;
            }
        }
        seconds[solver] = Now() - t0;
    }
    g_sink = sink;
    double n = static_cast<double>(reps) * kCases;
    std::printf("ns a chain solve of 3 bones, 16 iterations at most: CCD %.0f, FABRIK %.0f\n",
                seconds[0] / n * 1e9, seconds[1] / n * 1e9);
}

} // namespace

int main(int argc, char** argv)
{
    long cases = argc > 1 ? std::atol(argv[1]) : 100000;
    long reps = argc > 2 ? std::atol(argv[2]) : 2000;
    const maniBoneSource bones[4] = {{"root", 4, -1, {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}},
                                     {"start", 5, 0, {{0, 1, 0}, {0, 0, 0, 1}, {1, 1, 1}}},
                                     {"middle", 6, 1, {{0, -0.5f, 0}, {0, 0, 0, 1}, {1, 1, 1}}},
                                     {"end", 3, 2, {{0, -0.5f, 0}, {0, 0, 0, 1}, {1, 1, 1}}}};
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = nullptr;
    maniSkeletonSource source = {bones, 4, 0};
    size_t size = 0;
    maniSkeleton skeleton;
    maniPose* ours = nullptr;
    maniPose* theirs = nullptr;
    if (maniCreateBuilder(&def, &builder) != mani_success ||
        maniBuildSkeleton(builder, &source).result != mani_success ||
        maniCopyBuilt(builder, g_skeletonFile, sizeof g_skeletonFile, &size) != mani_success ||
        maniOpenSkeleton(g_skeletonFile, size, &skeleton).result != mani_success ||
        maniInitPose(&skeleton, g_memory[0], sizeof g_memory[0], &ours) != mani_success ||
        maniInitPose(&skeleton, g_memory[1], sizeof g_memory[1], &theirs) != mani_success)
    {
        std::fprintf(stderr, "setup failed\n");
        return 1;
    }
    maniDestroyBuilder(builder);
    // Softened goals apart: ozz softens with reciprocal estimates
    // (RcpEstX, about 12 bits), Maul Anim divides.
    float worst = 0.0f;
    float worstSoftened = 0.0f;
    // Each solver's own error on reachable goals at full weight: the end's
    // miss, and the middle's distance from the pole's plane (no twist),
    // over the chain's length.
    float missOurs = 0.0f;
    float missTheirs = 0.0f;
    float planeOurs = 0.0f;
    float planeTheirs = 0.0f;
    long compared = 0;
    long reachDiffers = 0;
    long halfTurns = 0;
    for (long c = 0; c < cases; ++c)
    {
        // Random lengths and turns for the start and middle, the end
        // bone's offset from the middle along a random direction.
        maniTransform locals[4] = {{{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}},
                                   {{0, 1, 0}, Turn(Direction(), Spread(-3, 3)), {1, 1, 1}},
                                   {{0, 0, 0}, Turn(Direction(), Spread(-3, 3)), {1, 1, 1}},
                                   {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}};
        maniVec3 d = Direction();
        float la = Spread(0.2f, 1.0f);
        locals[2].translation = {d.x * la, d.y * la, d.z * la};
        d = Direction();
        float lb = Spread(0.2f, 1.0f);
        locals[3].translation = {d.x * lb, d.y * lb, d.z * lb};
        for (uint32_t b = 0; b < 4; ++b)
        {
            maniPoseSetLocal(ours, b, locals[b]);
            maniPoseSetLocal(theirs, b, locals[b]);
        }
        maniVec3 start = Position(ours, &skeleton, 1);
        maniVec3 dir = Direction();
        float reach = (la + lb) * Spread(0.0f, 1.3f);
        maniVec3 goal = {start.x + dir.x * reach, start.y + dir.y * reach, start.z + dir.z * reach};
        maniVec3 pole = Direction();
        float along = pole.x * dir.x + pole.y * dir.y + pole.z * dir.z;
        // A goal within a twentieth of the chain's length of the start
        // leaves the start unturned here and turns it toward the goal in
        // ozz: left out.
        if (1.0f - along * along < 0.01f || reach < 0.05f * (la + lb))
        {
            continue;
        }
        // The hinge perpendicular to the chain's bend plane, either way
        // (a chain bent backward against it too), in the middle's frame:
        // a hinge is an axis of the joint; off the plane the two solvers
        // define the plane's turn differently.
        maniVec3 middle = Position(ours, &skeleton, 2);
        maniVec3 end = Position(ours, &skeleton, 3);
        maniVec3 a = {middle.x - start.x, middle.y - start.y, middle.z - start.z};
        maniVec3 b = {end.x - middle.x, end.y - middle.y, end.z - middle.z};
        maniVec3 n = {b.y * a.z - b.z * a.y, b.z * a.x - b.x * a.z, b.x * a.y - b.y * a.x};
        float nl = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (nl < 1e-3f * la * lb)
        {
            continue;
        }
        float side = Random() < 0.5f ? 1.0f : -1.0f;
        maniQuat r = maniPoseModelTransform(ours, 2).rotation;
        maniQuat back = {-r.x, -r.y, -r.z, r.w};
        maniQuat v =
            Mul(Mul(back, maniQuat{n.x / nl * side, n.y / nl * side, n.z / nl * side, 0}), r);
        maniVec3 hinge = {v.x, v.y, v.z};
        float soften = Random() < 0.5f ? 1.0f : 0.8f;
        float weight = Random() < 0.5f ? 1.0f : 0.5f;
        float twist = Random() < 0.5f ? 0.0f : Spread(-1, 1);

        maniTwoBoneSolve solve = {1, 2, 3, 0, goal, pole, hinge, twist, soften, weight};
        bool reachedOurs = false;
        if (maniSolveTwoBone(ours, &skeleton, &solve, &reachedOurs) != mani_success)
        {
            std::fprintf(stderr, "case %ld refused\n", c);
            return 1;
        }

        if (maniComputeModel(theirs, &skeleton) != mani_success)
        {
            return 1;
        }
        const ozz::math::Float4x4 models[3] = {ToOzz(maniPoseModel(theirs, 1)),
                                               ToOzz(maniPoseModel(theirs, 2)),
                                               ToOzz(maniPoseModel(theirs, 3))};
        ozz::animation::IKTwoBoneJob job;
        job.target = ozz::math::simd_float4::Load(goal.x, goal.y, goal.z, 1.0f);
        job.mid_axis = ozz::math::simd_float4::Load(hinge.x, hinge.y, hinge.z, 0.0f);
        job.pole_vector = ozz::math::simd_float4::Load(pole.x, pole.y, pole.z, 0.0f);
        job.twist_angle = twist;
        job.soften = soften;
        job.weight = weight;
        job.start_joint = &models[0];
        job.mid_joint = &models[1];
        job.end_joint = &models[2];
        ozz::math::SimdQuaternion corrections[2];
        bool reachedTheirs = false;
        job.start_joint_correction = &corrections[0];
        job.mid_joint_correction = &corrections[1];
        job.reached = &reachedTheirs;
        if (!job.Run())
        {
            std::fprintf(stderr, "ozz refused case %ld\n", c);
            return 1;
        }
        // A correction of nearly half a turn has w near 0, its sign (which
        // way a weight below 1 turns) a matter of rounding in both: those
        // weighted are left out, judged on ozz's full corrections.
        bool halfTurn = false;
        if (weight < 1.0f)
        {
            ozz::animation::IKTwoBoneJob full = job;
            ozz::math::SimdQuaternion whole[2];
            full.weight = 1.0f;
            full.start_joint_correction = &whole[0];
            full.mid_joint_correction = &whole[1];
            full.reached = nullptr;
            if (!full.Run())
            {
                return 1;
            }
            for (uint32_t k = 0; k < 2; ++k)
            {
                float q[4];
                ozz::math::StorePtrU(whole[k].xyzw, q);
                halfTurn = halfTurn || std::fabs(q[3]) < 0.05f;
            }
        }
        if (halfTurn)
        {
            ++halfTurns;
            continue;
        }
        for (uint32_t k = 0; k < 2; ++k)
        {
            float q[4];
            ozz::math::StorePtrU(corrections[k].xyzw, q);
            maniTransform t = maniPoseLocal(theirs, k + 1);
            t.rotation = Mul(t.rotation, maniQuat{q[0], q[1], q[2], q[3]});
            maniPoseSetLocal(theirs, k + 1, t);
        }
        float scale = la + lb;
        float apart =
            std::fmax(Apart(Position(ours, &skeleton, 2), Position(theirs, &skeleton, 2)),
                      Apart(Position(ours, &skeleton, 3), Position(theirs, &skeleton, 3))) /
            scale;
        bool softened = soften < 1.0f && reach > soften * scale;
        float& into = softened ? worstSoftened : worst;
        into = std::fmax(into, apart);
        if (weight >= 1.0f && soften >= 1.0f && reachedOurs)
        {
            missOurs = std::fmax(missOurs, Apart(Position(ours, &skeleton, 3), goal) / scale);
            missTheirs = std::fmax(missTheirs, Apart(Position(theirs, &skeleton, 3), goal) / scale);
        }
        if (weight >= 1.0f && soften >= 1.0f && reachedOurs && twist == 0.0f)
        {
            maniVec3 u = {goal.x - start.x, goal.y - start.y, goal.z - start.z};
            maniVec3 normal = {u.y * pole.z - u.z * pole.y, u.z * pole.x - u.x * pole.z,
                               u.x * pole.y - u.y * pole.x};
            float length =
                std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z) * scale;
            maniVec3 k[2] = {Position(ours, &skeleton, 2), Position(theirs, &skeleton, 2)};
            float off[2];
            for (int i = 0; i < 2; ++i)
            {
                off[i] = std::fabs((k[i].x - start.x) * normal.x + (k[i].y - start.y) * normal.y +
                                   (k[i].z - start.z) * normal.z) /
                         length;
            }
            planeOurs = std::fmax(planeOurs, off[0]);
            planeTheirs = std::fmax(planeTheirs, off[1]);
        }
        reachDiffers += reachedOurs != reachedTheirs;
        ++compared;
    }
    std::printf("two-bone: %ld cases compared, worst difference %.3g of the chain's length "
                "(%.3g softened), reached differs in %ld; %ld weighted half turns left out\n"
                "  own errors, Maul Anim / ozz: the end's miss %.3g / %.3g, the middle off the "
                "pole's plane %.3g / %.3g\n",
                compared, static_cast<double>(worst), static_cast<double>(worstSoftened),
                reachDiffers, halfTurns, static_cast<double>(missOurs),
                static_cast<double>(missTheirs), static_cast<double>(planeOurs),
                static_cast<double>(planeTheirs));
    bool agree = worst <= 5e-4f && worstSoftened <= 2e-3f && reachDiffers == 0;
    bool aim = AimDifferential(ours, theirs, &skeleton, cases);
    Timings(ours, &skeleton, reps);
    ChainTimings(ours, &skeleton, reps);
    return agree && missOurs <= 1e-5f && planeOurs <= 1e-5f && aim ? 0 : 1;
}
