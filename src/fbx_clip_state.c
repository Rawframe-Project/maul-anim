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

double maniFbxClipStackTime(const maniFbxClipState* c, uint32_t k)
{
    double tps = (double)c->fbx->settings.ticksPerSecond;
    bool last = k + 1 == c->sampleCount && k > 0;
    double tick = last && c->animation.stop > c->animation.start
                      ? (double)c->animation.stop
                      : (double)c->animation.start + Round((double)k * tps / c->exactRate);
    return tick / tps;
}
