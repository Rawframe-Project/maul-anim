// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An FBX clip build's sample times (mani-0016).

#include "fbx_clip_state.h"

#include <math.h>
#include <stdbool.h>

// x rounded to the nearest whole number, for x not negative.
static double Round(double x)
{
    double y = x + 0.5;
    return y - fmod(y, 1.0);
}

double maniFbxGridTime(const maniFbx* fbx, const maniFbxAnimation* animation, double exactRate,
                       uint32_t count, uint32_t k)
{
    double tps = (double)fbx->settings.ticksPerSecond;
    bool last = k + 1 == count && k > 0;
    double tick = last && animation->stop > animation->start
                      ? (double)animation->stop
                      : (double)animation->start + Round((double)k * tps / exactRate);
    return tick / tps;
}

double maniFbxClipStackTime(const maniFbxClipState* c, uint32_t k)
{
    return maniFbxGridTime(c->fbx, &c->animation, c->exactRate, c->sampleCount, k);
}

double maniFbxDefaultRate(const maniFbx* fbx)
{
    const maniFbxGlobalSettings* s = &fbx->settings;
    double scene = s->rateDenominator != 0 ? (double)s->rateNumerator / (double)s->rateDenominator
                                           : s->customRate;
    return scene > 0.0 && scene <= 120.0 ? scene : 30.0;
}
