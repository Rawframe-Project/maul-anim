// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Chain IK nodes' words. A limit read back is checked as a
// solve checks it (ik_limit.c), so a file holds only limits a solve
// would take; its unused words must be 0, so each limit has one form.

#include "graph_chain.h"

#include "bytes.h"
#include "graph_file.h"
#include "ik_limit.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/ik.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PI_F 3.14159265358979f

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

bool maniGraphIkParameterValid(const maniGraph* g, uint32_t at, uint32_t type, bool optional)
{
    uint32_t p = Word(g, at);
    if (p == MANI_NO_PARAMETER)
    {
        return optional;
    }
    return p < g->parameterCount &&
           maniLoadU32(g->parameters + (size_t)p * MANI_GRAPH_PARAMETER_SIZE) == type;
}

bool maniGraphIkWeightValid(const maniGraph* g, uint32_t at)
{
    if (Word(g, at) != MANI_NO_PARAMETER)
    {
        return maniGraphIkParameterValid(g, at, maniGraphParameterFloat, false) &&
               Word(g, at + 1) == 0;
    }
    float v = WordFloat(g, at + 1);
    return v >= 0.0f && v <= 1.0f;
}

uint32_t maniGraphChainDataWords(const maniGraph* g, uint32_t data, uint32_t available)
{
    if (available < MANI_GRAPH_CHAIN_WORDS)
    {
        return 0;
    }
    uint32_t limits = Word(g, data + maniGraphChainLimitCount);
    uint32_t room = (available - MANI_GRAPH_CHAIN_WORDS) / MANI_GRAPH_LIMIT_WORDS;
    return limits < MANI_IK_CHAIN_MAX_BONES && limits <= room
               ? MANI_GRAPH_CHAIN_WORDS + MANI_GRAPH_LIMIT_WORDS * limits
               : 0;
}

maniJointLimit maniGraphLimitAt(const maniGraph* g, uint32_t at)
{
    return (maniJointLimit){Word(g, at),
                            0,
                            {WordFloat(g, at + 1), WordFloat(g, at + 2), WordFloat(g, at + 3)},
                            WordFloat(g, at + 4),
                            WordFloat(g, at + 5),
                            WordFloat(g, at + 6),
                            WordFloat(g, at + 7),
                            WordFloat(g, at + 8)};
}

maniJointLimit maniGraphChainLimit(const maniGraph* g, uint32_t data, uint32_t k)
{
    return maniGraphLimitAt(g, data + MANI_GRAPH_CHAIN_WORDS + MANI_GRAPH_LIMIT_WORDS * k);
}

uint32_t maniGraphLimitValid(const maniGraph* g, uint32_t at)
{
    maniJointLimit limit = maniGraphLimitAt(g, at);
    if (limit.kind > maniJointLimitHinge)
    {
        return at;
    }
    // The words each kind leaves unused: a cone's axis, low and high; a
    // hinge's cone and twists; every number of no limit.
    static const uint16_t kUnused[3] = {0x1FEu, 0x6Eu, 0x190u};
    for (uint32_t w = 1; w < MANI_GRAPH_LIMIT_WORDS; ++w)
    {
        if ((kUnused[limit.kind] >> w & 1u) != 0 && Word(g, at + w) != 0)
        {
            return at + w;
        }
    }
    return maniIkLimitValid(&limit) ? UINT32_MAX : at + 1;
}

uint32_t maniGraphChainValid(const maniGraph* g, uint32_t data, bool fabrik)
{
    for (uint32_t k = 0; k < 2; ++k)
    {
        if (Word(g, data + k) >= g->skeletonBoneCount)
        {
            return data + k;
        }
    }
    const uint32_t parameters[3] = {maniGraphChainGoal, maniGraphChainPole, maniGraphChainRotation};
    const uint32_t types[3] = {maniGraphParameterVec3, maniGraphParameterVec3,
                               maniGraphParameterQuat};
    for (uint32_t k = 0; k < 3; ++k)
    {
        bool none = k == 1 && !fabrik;
        bool ok = none ? Word(g, data + parameters[k]) == MANI_NO_PARAMETER
                       : maniGraphIkParameterValid(g, data + parameters[k], types[k], k > 0);
        if (!ok)
        {
            return data + parameters[k];
        }
    }
    uint32_t iterations = Word(g, data + maniGraphChainIterations);
    float tolerance = WordFloat(g, data + maniGraphChainTolerance);
    float step = WordFloat(g, data + maniGraphChainStep);
    if (iterations < 1 || iterations > MANI_IK_MAX_ITERATIONS)
    {
        return data + maniGraphChainIterations;
    }
    if (!(tolerance >= 0.0f) || !isfinite(tolerance))
    {
        return data + maniGraphChainTolerance;
    }
    if (!(step >= 0.0f && step <= PI_F) || (fabrik && Word(g, data + maniGraphChainStep) != 0))
    {
        return data + maniGraphChainStep;
    }
    if (!maniGraphIkWeightValid(g, data + maniGraphChainRotationWeight))
    {
        return data + maniGraphChainRotationWeight;
    }
    if (!maniGraphIkWeightValid(g, data + maniGraphChainWeight))
    {
        return data + maniGraphChainWeight;
    }
    uint32_t limits = Word(g, data + maniGraphChainLimitCount);
    for (uint32_t k = 0; k < limits; ++k)
    {
        uint32_t wrong =
            maniGraphLimitValid(g, data + MANI_GRAPH_CHAIN_WORDS + MANI_GRAPH_LIMIT_WORDS * k);
        if (wrong != UINT32_MAX)
        {
            return wrong;
        }
    }
    return UINT32_MAX;
}
