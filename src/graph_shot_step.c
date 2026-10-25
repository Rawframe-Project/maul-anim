// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A fade moves the weight from where it began: in toward 1, out toward 0,
// along the curve of its elapsed fraction; a fade whose time is up
// settles (in to playing at 1, out to idle at 0). A fade begun from a
// weight takes its whole time, so a second fire rises without a pop.

#include "graph_shot_step.h"

#include "graph_file.h"
#include "graph_instance.h"
#include "graph_shot.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float Get(const uint32_t* block, uint32_t at)
{
    float value = 0.0f;
    memcpy(&value, &block[at], sizeof value);
    return value;
}

static void Put(uint32_t* block, uint32_t at, float value)
{
    memcpy(&block[at], &value, sizeof value);
}

static void Fade(const maniGraphShot* s, uint32_t* block)
{
    uint32_t phase = block[maniGraphShotPhase];
    if (phase != maniGraphShotIn && phase != maniGraphShotOut)
    {
        return;
    }
    bool in = phase == maniGraphShotIn;
    float duration = maniGraphShotFloat(s, in ? maniGraphShotFadeIn : maniGraphShotFadeOut);
    float elapsed = Get(block, maniGraphShotElapsed);
    bool done = !(elapsed < duration);
    float c = done ? 1.0f : elapsed / duration;
    if (maniGraphShotWord(s, maniGraphShotCurve) == maniGraphFadeSmoothstep)
    {
        c = c * c * (3.0f - 2.0f * c);
    }
    float start = Get(block, maniGraphShotStart);
    float weight = in ? start + (1.0f - start) * c : start * (1.0f - c);
    Put(block, maniGraphShotWeight, done ? (in ? 1.0f : 0.0f) : weight);
    block[maniGraphShotPhase] =
        done ? (in ? (uint32_t)maniGraphShotPlaying : (uint32_t)maniGraphShotIdle) : phase;
}

static void Begin(const maniGraphShot* s, uint32_t* block, uint32_t phase)
{
    Put(block, maniGraphShotStart, Get(block, maniGraphShotWeight));
    Put(block, maniGraphShotElapsed, 0.0f);
    block[maniGraphShotPhase] = phase;
    Fade(s, block);
}

// A trigger the one-shot names, consumed: whether it was set.
static bool Take(const maniGraphShot* s, maniGraphCell* cells, uint32_t at)
{
    uint32_t parameter = maniGraphShotWord(s, at);
    if (parameter == MANI_NO_PARAMETER)
    {
        return false;
    }
    bool set = cells[parameter].value.b != 0;
    cells[parameter].value.b = 0;
    return set;
}

static double Remaining(const uint32_t* block)
{
    uint64_t bits = (uint64_t)block[maniGraphShotRemaining] +
                    ((uint64_t)block[maniGraphShotRemaining + 1] << 32);
    double value = 0.0;
    memcpy(&value, &bits, sizeof value);
    return value;
}

bool maniGraphStepShot(const maniGraphShot* s, maniGraphCell* cells, uint32_t* block,
                       double seconds)
{
    uint32_t phase = block[maniGraphShotPhase];
    if (phase == maniGraphShotIn || phase == maniGraphShotOut)
    {
        double elapsed = (double)Get(block, maniGraphShotElapsed) + seconds;
        Put(block, maniGraphShotElapsed, maniGraphSeconds(elapsed));
        Fade(s, block);
    }
    bool abort = Take(s, cells, maniGraphShotAbort);
    bool fire = Take(s, cells, maniGraphShotFire);
    bool out = Take(s, cells, maniGraphShotFadeOutTrigger);
    phase = block[maniGraphShotPhase];
    bool active = phase == maniGraphShotIn || phase == maniGraphShotPlaying;
    if (abort)
    {
        maniGraphResetShot(block);
        return false;
    }
    if (fire)
    {
        // The clip time left stays until the update records the restarted
        // shot's: fire returns before it is read.
        Begin(s, block, maniGraphShotIn);
        return true;
    }
    float fadeOut = maniGraphShotFloat(s, maniGraphShotFadeOut);
    if (active && (out || Remaining(block) <= (double)fadeOut))
    {
        Begin(s, block, maniGraphShotOut);
    }
    return false;
}
