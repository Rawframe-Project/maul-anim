// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph parameter's value (graph_values.h): as an instance keeps it,
// and in the bytes of saved state.

#include "graph_values.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/graph.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

maniGraphValue maniGraphLoadValue(const uint8_t* p)
{
    maniGraphValue v;
    uint32_t words[4];
    for (uint32_t k = 0; k < 4; ++k)
    {
        words[k] = maniLoadU32(p + 4 * k);
    }
    memcpy(&v, words, sizeof v);
    return v;
}

void maniGraphStoreValue(uint8_t* p, maniGraphValue v)
{
    uint32_t words[4];
    memcpy(words, &v, sizeof words);
    for (uint32_t k = 0; k < 4; ++k)
    {
        maniStoreU32(p + 4 * k, words[k]);
    }
}

static float Clamp(float v, const uint8_t* entry)
{
    float low = 0.0f;
    float high = 0.0f;
    memcpy(&low, entry + 40, sizeof low);
    memcpy(&high, entry + 44, sizeof high);
    return v < low ? low : (v > high ? high : v);
}

// The float and vector values as kept: finite, a quaternion normalized,
// a bounded float clamped; false for one out of form.
static bool CanonicalFloats(maniGraphParameterType type, const uint8_t* entry, bool bounded,
                            maniGraphValue* v)
{
    uint32_t n = type == maniGraphParameterFloat  ? 1u
                 : type == maniGraphParameterVec2 ? 2u
                 : type == maniGraphParameterVec3 ? 3u
                                                  : 4u;
    maniGraphValue out = {{0}};
    double length = 0.0;
    for (uint32_t k = 0; k < n; ++k)
    {
        if (!isfinite(v->f[k]))
        {
            return false;
        }
        out.f[k] = v->f[k];
        length += (double)v->f[k] * (double)v->f[k];
    }
    if (type == maniGraphParameterQuat)
    {
        if (!(length > 0.0) || !isfinite(length))
        {
            return false;
        }
        double scale = 1.0 / sqrt(length);
        for (uint32_t k = 0; k < 4; ++k)
        {
            out.f[k] = (float)((double)out.f[k] * scale);
        }
    }
    out.f[0] = type == maniGraphParameterFloat && bounded ? Clamp(out.f[0], entry) : out.f[0];
    *v = out;
    return true;
}

// The value as the parameter keeps it, or false for one out of form.
bool maniGraphCanonicalValue(maniGraphParameterType type, const uint8_t* entry, maniGraphValue* v)
{
    bool bounded = (maniLoadU32(entry + 4) & MANI_GRAPH_BOUNDED) != 0;
    maniGraphValue out = {{0}};
    if (type == maniGraphParameterPose)
    {
        *v = out;
        return true;
    }
    if (type == maniGraphParameterBool || type == maniGraphParameterTrigger)
    {
        out.b = v->b;
        *v = out;
        return out.b <= 1u;
    }
    if (type == maniGraphParameterInt)
    {
        int32_t low = (int32_t)maniLoadU32(entry + 40);
        int32_t high = (int32_t)maniLoadU32(entry + 44);
        out.i = bounded ? (v->i < low ? low : (v->i > high ? high : v->i)) : v->i;
        *v = out;
        return true;
    }
    return CanonicalFloats(type, entry, bounded, v);
}
