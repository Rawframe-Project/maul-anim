// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node's words name a spring set slot, a vec3 position, a quat
// rotation and a reset trigger (each a parameter of its type or none),
// and its set's state words (at least a state's header, at most
// MANI_GRAPH_SPRING_MAX_WORDS); binding checks the words against the
// set. A saved block keeps a time finite and not negative, and a spring
// state checked as the standalone state's restore checks it.

#include "graph_spring.h"

#include "bytes.h"
#include "graph_file.h"
#include "spring_state.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

// A parameter of the type, or none.
static bool ParameterOf(const maniGraph* g, uint32_t parameter, uint32_t type)
{
    return parameter == MANI_NO_PARAMETER ||
           (parameter < g->parameterCount &&
            maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE) == type);
}

uint32_t maniGraphSpringValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    const uint32_t types[3] = {maniGraphParameterVec3, maniGraphParameterQuat,
                               maniGraphParameterTrigger};
    if (Word(g, data + maniGraphSpringSet) >= g->springSetCount)
    {
        return data + maniGraphSpringSet;
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (!ParameterOf(g, Word(g, data + maniGraphSpringPosition + k), types[k]))
        {
            return data + maniGraphSpringPosition + k;
        }
    }
    if (Word(g, data + maniGraphSpringBlock) != block)
    {
        return data + maniGraphSpringBlock;
    }
    uint32_t words = Word(g, data + maniGraphSpringStateWords);
    if (words < maniSpringJoints || words > MANI_GRAPH_SPRING_MAX_WORDS)
    {
        return data + maniGraphSpringStateWords;
    }
    return ParameterOf(g, Word(g, data + maniGraphSpringForce), maniGraphParameterVec3)
               ? UINT32_MAX
               : data + maniGraphSpringForce;
}

uint64_t maniGraphSpringSetWords(const maniSpringSet* set)
{
    return maniSpringBlockWords(set);
}

uint32_t maniGraphSpringBlockWords(const maniGraph* g, uint32_t data)
{
    return maniGraphSpringState + Word(g, data + maniGraphSpringStateWords);
}

bool maniGraphSpringFits(const maniGraph* g, uint32_t data, const maniSpringSet* set)
{
    return maniSpringBlockWords(set) == Word(g, data + maniGraphSpringStateWords);
}

static double Pending(const uint32_t* block)
{
    uint64_t bits =
        (uint64_t)block[maniGraphSpringPending] | (uint64_t)block[maniGraphSpringPending + 1] << 32;
    double v = 0.0;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static void SetPending(uint32_t* block, double v)
{
    uint64_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    block[maniGraphSpringPending] = (uint32_t)bits;
    block[maniGraphSpringPending + 1] = (uint32_t)(bits >> 32);
}

uint32_t maniGraphSavedSpringValid(const uint8_t* bytes, const maniSpringSet* set)
{
    double pending = 0.0;
    uint64_t bits = maniLoadU64(bytes);
    memcpy(&pending, &bits, sizeof pending);
    if (!(pending >= 0.0 && isfinite(pending)))
    {
        return maniGraphSpringPending;
    }
    uint64_t wrong = maniSpringBlockValid(set, bytes + 4 * maniGraphSpringState);
    return wrong == UINT64_MAX ? UINT32_MAX : maniGraphSpringState + (uint32_t)(wrong / 4);
}

void maniGraphSpringAdd(uint32_t* block, double seconds)
{
    SetPending(block, Pending(block) + seconds);
}

void maniGraphResetSpring(const maniGraph* g, uint32_t data, uint32_t* block)
{
    memset(block, 0, 4 * (size_t)maniGraphSpringBlockWords(g, data));
}

uint64_t maniGraphSpringWorkWords(const maniSpringSet* set)
{
    return maniSpringWorkWords(set);
}

void maniGraphSpringPose(const maniSpringSet* set, const maniSkeleton* skeleton, uint32_t* block,
                         uint32_t* work, const maniTransform* placement, maniVec3 force,
                         maniPose* pose)
{
    if (maniComputeModel(pose, skeleton) != mani_success)
    {
        return;
    }
    double pending = Pending(block);
    SetPending(block, 0.0);
    uint32_t* state = block + maniGraphSpringState;
    maniSpringStepBlock(set, state, work, pose, placement, force, pending);
    maniSpringApplyBlock(set, skeleton, state, pose);
}
