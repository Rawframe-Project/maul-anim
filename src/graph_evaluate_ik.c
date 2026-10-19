// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// IK and full-body nodes evaluated (mani-0029 to mani-0031): goals and
// settings read from the node's words and the instance's parameters, the
// solve run on a copy of the input, the input given when a crafted file's
// bones or settings are refused.

#include "graph_evaluate_ik.h"

#include "bytes.h"
#include "graph_body.h"
#include "graph_chain.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "ik_chain.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// A vec3 parameter's value, or zero for none.
static maniVec3 Vector(const maniGraphEvaluation* e, uint32_t parameter)
{
    if (parameter == MANI_NO_PARAMETER)
    {
        return (maniVec3){0.0f, 0.0f, 0.0f};
    }
    const float* f = e->instance->cells[parameter].value.f;
    return (maniVec3){f[0], f[1], f[2]};
}

// A quat parameter's value, or no turn for none.
static maniQuat Rotation(const maniGraphEvaluation* e, uint32_t parameter)
{
    if (parameter == MANI_NO_PARAMETER)
    {
        return (maniQuat){0.0f, 0.0f, 0.0f, 1.0f};
    }
    const float* f = e->instance->cells[parameter].value.f;
    return (maniQuat){f[0], f[1], f[2], f[3]};
}

// A binary32 data word.
static float Number(const maniGraphEvaluation* e, uint32_t at)
{
    uint32_t bits = maniGraphEvalWord(e, at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// Three binary32 data words from at.
static maniVec3 Axis(const maniGraphEvaluation* e, uint32_t at)
{
    float f[3];
    for (uint32_t k = 0; k < 3; ++k)
    {
        uint32_t bits = maniGraphEvalWord(e, at + k);
        memcpy(&f[k], &bits, sizeof bits);
    }
    return (maniVec3){f[0], f[1], f[2]};
}

// A scalar clamped from low to high (a parameter's value may lie past).
static float Clamped(const maniGraphEvaluation* e, uint32_t at, float low, float high)
{
    float v = maniGraphEvalScalar(e, at);
    return v < low ? low : (v > high ? high : v);
}

// The input of an IK node copied to out; false with none (out then
// neutral) or a weight of 0 at data word weightAt.
static bool IkInput(const maniGraphEvaluation* e, const uint8_t* record, uint32_t weightAt,
                    maniPose* out)
{
    const maniPose* input = maniGraphEvalInput(e, record, 0);
    if (input == NULL)
    {
        maniGraphEvalNeutral(e, out, false);
        return false;
    }
    maniPoseCopy(out, input);
    return Clamped(e, weightAt, 0.0f, 1.0f) > 0.0f;
}

// A two-bone node: its input with its chain solved; the input when the
// solve refuses (a crafted file's bones not each an ancestor of the
// next).
void maniGraphEvaluateTwoBone(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphTwoBoneWeight, out))
    {
        return;
    }
    maniTwoBoneSolve s = {maniGraphEvalWord(e, d),
                          maniGraphEvalWord(e, d + 1),
                          maniGraphEvalWord(e, d + 2),
                          0,
                          Vector(e, maniGraphEvalWord(e, d + maniGraphTwoBoneGoal)),
                          Vector(e, maniGraphEvalWord(e, d + maniGraphTwoBonePole)),
                          Axis(e, d + maniGraphTwoBoneHinge),
                          maniGraphEvalScalar(e, d + maniGraphTwoBoneTwist),
                          Clamped(e, d + maniGraphTwoBoneSoften, 0.0f, 1.0f),
                          Clamped(e, d + maniGraphTwoBoneWeight, 0.0f, 1.0f)};
    (void)maniSolveTwoBone(out, e->binding->skeleton, &s, NULL);
}

// An aim node: its input with its bones aimed; the input when the solve
// refuses.
void maniGraphEvaluateAim(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphAimWeight, out))
    {
        return;
    }
    maniAimSolve s = {.count = maniGraphEvalWord(e, d),
                      .goal = Vector(e, maniGraphEvalWord(e, d + maniGraphAimGoal)),
                      .pole = Vector(e, maniGraphEvalWord(e, d + maniGraphAimPole)),
                      .forward = Axis(e, d + maniGraphAimForward),
                      .offset = Axis(e, d + maniGraphAimOffset),
                      .up = Axis(e, d + maniGraphAimUp),
                      .twist = maniGraphEvalScalar(e, d + maniGraphAimTwist),
                      .cone = Clamped(e, d + maniGraphAimCone, 0.0f, MANI_AIM_MAX_CONE),
                      .weight = Clamped(e, d + maniGraphAimWeight, 0.0f, 1.0f)};
    for (uint32_t k = 0; k < MANI_AIM_MAX_BONES; ++k)
    {
        uint32_t bits = maniGraphEvalWord(e, d + maniGraphAimShares + k);
        s.bones[k] = maniGraphEvalWord(e, d + maniGraphAimBones + k);
        memcpy(&s.shares[k], &bits, sizeof bits);
    }
    (void)maniSolveAim(out, e->binding->skeleton, &s, NULL);
}

// A chain node: its input with its chain solved; the input when the
// solve refuses (a crafted file's bones not a chain) or its limits are
// not one per turning bone of the bound skeleton's path.
void maniGraphEvaluateChain(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                            bool fabrik)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphChainWeight, out))
    {
        return;
    }
    const maniSkeleton* skeleton = e->binding->skeleton;
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t path =
        maniIkChainPath(skeleton, maniGraphEvalWord(e, d), maniGraphEvalWord(e, d + 1), bones);
    uint32_t count = maniGraphEvalWord(e, d + maniGraphChainLimitCount);
    if (count != 0 && count + 1 != path)
    {
        return;
    }
    maniJointLimit limits[MANI_IK_CHAIN_MAX_BONES] = {0};
    for (uint32_t k = 0; k < count; ++k)
    {
        limits[k] = maniGraphChainLimit(e->graph, d, k);
    }
    uint32_t rotation = maniGraphEvalWord(e, d + maniGraphChainRotation);
    maniChainSolve s = {.root = maniGraphEvalWord(e, d),
                        .end = maniGraphEvalWord(e, d + 1),
                        .limits = count > 0 ? limits : NULL,
                        .goal = Vector(e, maniGraphEvalWord(e, d + maniGraphChainGoal)),
                        .pole = Vector(e, maniGraphEvalWord(e, d + maniGraphChainPole)),
                        .rotation = Rotation(e, rotation),
                        .iterations = maniGraphEvalWord(e, d + maniGraphChainIterations),
                        .tolerance = Number(e, d + maniGraphChainTolerance),
                        .step = Number(e, d + maniGraphChainStep),
                        .rotationWeight =
                            rotation != MANI_NO_PARAMETER
                                ? Clamped(e, d + maniGraphChainRotationWeight, 0.0f, 1.0f)
                                : 0.0f,
                        .weight = Clamped(e, d + maniGraphChainWeight, 0.0f, 1.0f)};
    (void)(fabrik ? maniSolveFabrik(out, skeleton, &s, NULL)
                  : maniSolveCcd(out, skeleton, &s, NULL));
}

// Effector k of the body node whose words begin at d, its goals and
// weights the parameters' values now.
static maniBodyEffector Effector(const maniGraphEvaluation* e, uint32_t d, uint32_t k)
{
    uint32_t at = maniGraphBodyEffectorAt(d, k);
    return (maniBodyEffector){
        .bone = maniGraphEvalWord(e, at + maniGraphEffectorBone),
        .position = Vector(e, maniGraphEvalWord(e, at + maniGraphEffectorPosition)),
        .rotation = Rotation(e, maniGraphEvalWord(e, at + maniGraphEffectorRotation)),
        .positionWeight = Clamped(e, at + maniGraphEffectorPositionWeight, 0.0f, 1.0f),
        .rotationWeight = Clamped(e, at + maniGraphEffectorRotationWeight, 0.0f, 1.0f),
        .strength = Number(e, at + maniGraphEffectorStrength),
        .pullChain = Number(e, at + maniGraphEffectorPull)};
}

// A full-body node: its input with the body solved, its effectors,
// settings and the solve's scratch laid in the instance's room from a
// 16-byte boundary; the input when the solve refuses (a crafted file's
// bones not a part the solve takes, a goal not finite, a room short).
void maniGraphEvaluateBody(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                           bool sdls)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphBodyWeight, out))
    {
        return;
    }
    uint32_t count = maniGraphEvalWord(e, d + maniGraphBodyEffectorCount);
    uint32_t settings = maniGraphEvalWord(e, d + maniGraphBodySettingCount);
    uint8_t* room = (uint8_t*)e->instance + e->instance->roomAt;
    size_t at = (16u - (uintptr_t)room % 16u) % 16u;
    maniBodyEffector* effectors = (void*)(room + at);
    at += maniGraphBodyRounded(count * sizeof(maniBodyEffector));
    maniBodyBone* bones = (void*)(room + at);
    at += maniGraphBodyRounded(settings * sizeof(maniBodyBone));
    // The room holds every body node's arrays (maniGraphBodyRoom); the
    // check keeps a room short of them from being written past.
    size_t have = e->instance->size - e->instance->roomAt;
    if (at > have)
    {
        return;
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        effectors[k] = Effector(e, d, k);
    }
    for (uint32_t k = 0; k < settings; ++k)
    {
        bones[k] = maniGraphBodySetting(e->graph, d, k);
    }
    const maniBodySolve s = {.top = maniGraphEvalWord(e, d + maniGraphBodyTop),
                             .topMode =
                                 (maniBodyTopMode)maniGraphEvalWord(e, d + maniGraphBodyMode),
                             .effectors = effectors,
                             .bones = bones,
                             .effectorCount = count,
                             .boneCount = settings,
                             .iterations = maniGraphEvalWord(e, d + maniGraphBodyIterations),
                             .stretch = maniGraphEvalWord(e, d + maniGraphBodyStretch) != 0,
                             .tolerance = Number(e, d + maniGraphBodyTolerance),
                             .weight = Clamped(e, d + maniGraphBodyWeight, 0.0f, 1.0f)};
    const maniSkeleton* skeleton = e->binding->skeleton;
    (void)(sdls ? maniSolveBodySdls(out, skeleton, &s, room + at, have - at, NULL)
                : maniSolveBodyXpbd(out, skeleton, &s, room + at, have - at, NULL));
}
