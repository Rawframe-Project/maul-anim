// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A saved block's phase and weight agree: idle at 0, playing at 1, a
// fade anywhere between.

#include "graph_shot.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float FloatOf(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static uint32_t BitsOf(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

uint32_t maniGraphShotWord(const maniGraphShot* s, uint32_t at)
{
    return maniLoadU32(s->graph->data + 4 * (size_t)(s->data + at));
}

float maniGraphShotFloat(const maniGraphShot* s, uint32_t at)
{
    return FloatOf(maniGraphShotWord(s, at));
}

uint32_t maniGraphShotClip(const maniGraphShot* s, uint32_t i)
{
    return maniGraphShotWord(s, MANI_GRAPH_SHOT_WORDS + i);
}

void maniGraphResetShot(uint32_t* block)
{
    uint64_t bits = 0;
    double none = HUGE_VAL;
    memcpy(&bits, &none, sizeof bits);
    block[maniGraphShotPhase] = maniGraphShotIdle;
    block[maniGraphShotElapsed] = BitsOf(0.0f);
    block[maniGraphShotStart] = BitsOf(0.0f);
    block[maniGraphShotWeight] = BitsOf(0.0f);
    block[maniGraphShotRemaining] = (uint32_t)bits;
    block[maniGraphShotRemaining + 1] = (uint32_t)(bits >> 32);
}

static bool Unit(float value)
{
    return value >= 0.0f && value <= 1.0f;
}

uint32_t maniGraphSavedShotValid(const uint8_t* bytes)
{
    uint32_t phase = maniLoadU32(bytes);
    float elapsed = FloatOf(maniLoadU32(bytes + 4 * maniGraphShotElapsed));
    float weight = FloatOf(maniLoadU32(bytes + 4 * maniGraphShotWeight));
    uint64_t bits = maniLoadU64(bytes + 4 * maniGraphShotRemaining);
    double remaining = 0.0;
    memcpy(&remaining, &bits, sizeof remaining);
    if (phase > maniGraphShotOut)
    {
        return maniGraphShotPhase;
    }
    if (!(isfinite(elapsed) && elapsed >= 0.0f))
    {
        return maniGraphShotElapsed;
    }
    if (!Unit(FloatOf(maniLoadU32(bytes + 4 * maniGraphShotStart))))
    {
        return maniGraphShotStart;
    }
    bool settled = phase == maniGraphShotIdle || phase == maniGraphShotPlaying;
    float want = phase == maniGraphShotPlaying ? 1.0f : 0.0f;
    if (!Unit(weight) || (settled && weight != want))
    {
        return maniGraphShotWeight;
    }
    return remaining >= 0.0 ? UINT32_MAX : (uint32_t)maniGraphShotRemaining;
}

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

uint32_t maniGraphShotDataWords(const maniGraph* g, uint32_t data, uint32_t available)
{
    if (available < MANI_GRAPH_SHOT_WORDS)
    {
        return UINT32_MAX;
    }
    uint32_t count = Word(g, data + maniGraphShotClips);
    return count <= available - MANI_GRAPH_SHOT_WORDS ? MANI_GRAPH_SHOT_WORDS + count : UINT32_MAX;
}

static bool Trigger(const maniGraph* g, uint32_t parameter, bool optional)
{
    if (parameter == MANI_NO_PARAMETER)
    {
        return optional;
    }
    return parameter < g->parameterCount &&
           maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE) ==
               maniGraphParameterTrigger;
}

static uint32_t Kind(const maniGraph* g, uint32_t node)
{
    return maniLoadU16(g->nodes + (size_t)node * MANI_GRAPH_NODE_SIZE);
}

static bool Seconds(const maniGraph* g, uint32_t at)
{
    float value = FloatOf(Word(g, at));
    return isfinite(value) && value >= 0.0f;
}

// Its words as the builder writes them: triggers, curve, seconds, mask,
// block, then its clips before it in node order.
uint32_t maniGraphShotValid(const maniGraph* g, uint32_t p, uint32_t data, uint32_t block)
{
    uint32_t mask = Word(g, data + maniGraphShotMask);
    const bool valid[8] = {Trigger(g, Word(g, data), false),
                           Trigger(g, Word(g, data + 1), true),
                           Trigger(g, Word(g, data + 2), true),
                           Word(g, data + 3) <= maniGraphFadeSmoothstep,
                           Seconds(g, data + 4),
                           Seconds(g, data + 5),
                           mask == MANI_GRAPH_NO_MASK || mask < g->maskCount,
                           Word(g, data + 7) == block};
    for (uint32_t k = 0; k < 8; ++k)
    {
        if (!valid[k])
        {
            return data + k;
        }
    }
    uint32_t first = data + MANI_GRAPH_SHOT_WORDS;
    for (uint32_t i = 0; i < Word(g, data + maniGraphShotClips); ++i)
    {
        uint32_t clip = Word(g, first + i);
        if (clip >= p || Kind(g, clip) != maniGraphNodeClip ||
            (i > 0 && clip <= Word(g, first + i - 1)))
        {
            return first + i;
        }
    }
    return UINT32_MAX;
}
