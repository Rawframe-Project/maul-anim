// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Full-body IK nodes' words (mani-0031). An effector without a rotation
// keeps its rotation weight the number 0, and a limit its unused words
// 0, so each has one form.

#include "graph_body.h"

#include "bytes.h"
#include "graph_chain.h"
#include "graph_file.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/graph.h"
#include "maul-anim/ik.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static float WordFloat(const maniGraph* g, uint32_t at)
{
    uint32_t bits = Word(g, at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static bool UnitAt(const maniGraph* g, uint32_t at)
{
    float v = WordFloat(g, at);
    return v >= 0.0f && v <= 1.0f;
}

uint32_t maniGraphBodyDataWords(const maniGraph* g, uint32_t data, uint32_t available)
{
    if (available < MANI_GRAPH_BODY_WORDS)
    {
        return 0;
    }
    uint32_t effectors = Word(g, data + maniGraphBodyEffectorCount);
    uint32_t settings = Word(g, data + maniGraphBodySettingCount);
    if (effectors < 1 || effectors > MANI_BODY_IK_MAX_EFFECTORS ||
        settings > MANI_BODY_IK_MAX_BONES)
    {
        return 0;
    }
    uint32_t words = MANI_GRAPH_BODY_WORDS + MANI_GRAPH_EFFECTOR_WORDS * effectors +
                     MANI_GRAPH_SETTING_WORDS * settings;
    return words <= available ? words : 0;
}

uint32_t maniGraphBodyEffectorAt(uint32_t data, uint32_t e)
{
    return data + MANI_GRAPH_BODY_WORDS + MANI_GRAPH_EFFECTOR_WORDS * e;
}

uint32_t maniGraphBodySettingAt(const maniGraph* g, uint32_t data, uint32_t k)
{
    return data + MANI_GRAPH_BODY_WORDS +
           MANI_GRAPH_EFFECTOR_WORDS * Word(g, data + maniGraphBodyEffectorCount) +
           MANI_GRAPH_SETTING_WORDS * k;
}

maniBodyBone maniGraphBodySetting(const maniGraph* g, uint32_t data, uint32_t k)
{
    uint32_t at = maniGraphBodySettingAt(g, data, k);
    uint32_t p = at + maniGraphSettingPreferred;
    return (maniBodyBone){.bone = Word(g, at + maniGraphSettingBone),
                          .limit = maniGraphLimitAt(g, at + maniGraphSettingLimit),
                          .preferred = {WordFloat(g, p), WordFloat(g, p + 1), WordFloat(g, p + 2)},
                          .rotationStiffness = WordFloat(g, at + maniGraphSettingRotationStiffness),
                          .positionStiffness =
                              WordFloat(g, at + maniGraphSettingPositionStiffness)};
}

// The first wrong word of effector e, or UINT32_MAX.
static uint32_t EffectorValid(const maniGraph* g, uint32_t data, uint32_t e)
{
    uint32_t at = maniGraphBodyEffectorAt(data, e);
    if (Word(g, at + maniGraphEffectorBone) >= g->skeletonBoneCount)
    {
        return at + maniGraphEffectorBone;
    }
    if (!maniGraphIkParameterValid(g, at + maniGraphEffectorPosition, maniGraphParameterVec3,
                                   false))
    {
        return at + maniGraphEffectorPosition;
    }
    if (!maniGraphIkParameterValid(g, at + maniGraphEffectorRotation, maniGraphParameterQuat, true))
    {
        return at + maniGraphEffectorRotation;
    }
    if (!maniGraphIkWeightValid(g, at + maniGraphEffectorPositionWeight))
    {
        return at + maniGraphEffectorPositionWeight;
    }
    uint32_t w = at + maniGraphEffectorRotationWeight;
    bool none = Word(g, at + maniGraphEffectorRotation) == MANI_NO_PARAMETER;
    if (!maniGraphIkWeightValid(g, w) ||
        (none && (Word(g, w) != MANI_NO_PARAMETER || Word(g, w + 1) != 0)))
    {
        return w;
    }
    if (!UnitAt(g, at + maniGraphEffectorStrength))
    {
        return at + maniGraphEffectorStrength;
    }
    return UnitAt(g, at + maniGraphEffectorPull) ? UINT32_MAX : at + maniGraphEffectorPull;
}

// The first wrong word of setting k, or UINT32_MAX.
static uint32_t SettingValid(const maniGraph* g, uint32_t data, uint32_t k)
{
    uint32_t at = maniGraphBodySettingAt(g, data, k);
    if (Word(g, at + maniGraphSettingBone) >= g->skeletonBoneCount)
    {
        return at + maniGraphSettingBone;
    }
    uint32_t wrong = maniGraphLimitValid(g, at + maniGraphSettingLimit);
    if (wrong != UINT32_MAX)
    {
        return wrong;
    }
    for (uint32_t c = 0; c < 3; ++c)
    {
        if (!isfinite(WordFloat(g, at + maniGraphSettingPreferred + c)))
        {
            return at + maniGraphSettingPreferred + c;
        }
    }
    if (!UnitAt(g, at + maniGraphSettingRotationStiffness))
    {
        return at + maniGraphSettingRotationStiffness;
    }
    return UnitAt(g, at + maniGraphSettingPositionStiffness)
               ? UINT32_MAX
               : at + maniGraphSettingPositionStiffness;
}

uint32_t maniGraphBodyValid(const maniGraph* g, uint32_t data, bool sdls)
{
    uint32_t stretch = Word(g, data + maniGraphBodyStretch);
    uint32_t iterations = Word(g, data + maniGraphBodyIterations);
    float tolerance = WordFloat(g, data + maniGraphBodyTolerance);
    const bool ok[5] = {Word(g, data + maniGraphBodyTop) < g->skeletonBoneCount,
                        Word(g, data + maniGraphBodyMode) <= maniBodyTopPrePull,
                        stretch == 0 || (stretch == 1 && !sdls),
                        iterations >= 1 && iterations <= MANI_IK_MAX_ITERATIONS,
                        tolerance >= 0.0f && isfinite(tolerance)};
    for (uint32_t k = 0; k < 5; ++k)
    {
        if (!ok[k])
        {
            return data + k;
        }
    }
    if (!maniGraphIkWeightValid(g, data + maniGraphBodyWeight))
    {
        return data + maniGraphBodyWeight;
    }
    for (uint32_t e = 0; e < Word(g, data + maniGraphBodyEffectorCount); ++e)
    {
        uint32_t wrong = EffectorValid(g, data, e);
        if (wrong != UINT32_MAX)
        {
            return wrong;
        }
    }
    for (uint32_t k = 0; k < Word(g, data + maniGraphBodySettingCount); ++k)
    {
        uint32_t wrong = SettingValid(g, data, k);
        if (wrong != UINT32_MAX)
        {
            return wrong;
        }
    }
    return UINT32_MAX;
}

size_t maniGraphBodyRounded(size_t bytes)
{
    return (bytes + 15u) & ~(size_t)15u;
}

size_t maniGraphBodyRoom(const maniGraph* g, uint32_t data, const maniSkeleton* skeleton)
{
    uint32_t count = Word(g, data + maniGraphBodyEffectorCount);
    uint32_t settings = Word(g, data + maniGraphBodySettingCount);
    // The part is the top and the effectors' paths to it: goals,
    // settings and numbers leave its size as it is.
    maniBodyEffector effectors[MANI_BODY_IK_MAX_EFFECTORS];
    for (uint32_t e = 0; e < count; ++e)
    {
        effectors[e] = (maniBodyEffector){
            .bone = Word(g, maniGraphBodyEffectorAt(data, e) + maniGraphEffectorBone),
            .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
            .positionWeight = 1.0f,
            .rotationWeight = 1.0f,
            .strength = 1.0f};
    }
    const maniBodySolve solve = {.top = Word(g, data + maniGraphBodyTop),
                                 .topMode = (maniBodyTopMode)Word(g, data + maniGraphBodyMode),
                                 .effectors = effectors,
                                 .effectorCount = count,
                                 .iterations = 1,
                                 .weight = 1.0f};
    return 15u + maniGraphBodyRounded(count * sizeof(maniBodyEffector)) +
           maniGraphBodyRounded(settings * sizeof(maniBodyBone)) +
           maniBodyIkScratchSize(skeleton, &solve);
}
