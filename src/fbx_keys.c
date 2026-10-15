// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX 6.x keys, as ufbx reads them (research 26). A Key list runs time,
// value and a mode letter per key, then the mode's own values: `L`
// linear, `C` constant (from KeyVer 4004 a letter, `n` for next), `U`
// cubic with a slope mode and weight modes whose counts depend on the
// KeyVer. Each key's time and value are read ahead with the previous
// key, so auto slopes see both neighbours. Handles are as for 7.x
// curves: dx a weight times the gap (binary32), dy dx times the slope.

#include "fbx_keys.h"

#include "allocator.h"
#include "fbx_curve.h"
#include "fbx_document.h"
#include "fbx_value.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

// The letter a value holds, 0 outside ASCII, as ufbx converts.
static char Letter(double v)
{
    return v >= 0.0 && v <= 127.0 ? (char)(int)v : 0;
}

typedef struct Parser
{
    const double* data;
    uint32_t at;
    uint32_t end;
    int64_t version;
    double ticksPerSecond;
    // Carried from key to key.
    float slopeLeft;
    float weightLeft;
    double previousTime;
    double nextTicks;
    double nextValue;
} Parser;

// What a key's modes give its handles.
typedef struct Modes
{
    float slopeRight;
    float weightRight;
    float nextSlopeLeft;
    float nextWeightLeft;
    bool autoSlope;
} Modes;

static bool Has(const Parser* p, uint32_t n)
{
    return p->end - p->at >= n;
}

// One weight mode; false for one unknown or values missing.
static bool WeightMode(Parser* p, Modes* m)
{
    if (!Has(p, 1))
    {
        return false;
    }
    char mode = Letter(p->data[p->at++]);
    uint32_t need = mode == 'a' ? 2 : (mode == 'l' || mode == 'r' ? 1 : 0);
    if (!(mode == 'n' || mode == 'c' || need > 0) || !Has(p, need))
    {
        return false;
    }
    if (mode == 'a' || mode == 'r')
    {
        m->weightRight = maniFbxNarrow(p->data[p->at]);
    }
    if (mode == 'a' || mode == 'l')
    {
        m->nextWeightLeft = maniFbxNarrow(p->data[p->at + need - 1]);
    }
    p->at += need;
    return true;
}

// A slope mode's values to skip and weight modes to read after it, by
// KeyVer; false for a mode ufbx does not know. Modes s and b hold two
// slopes (read by the caller); the others are auto.
static bool SlopeCounts(char mode, int64_t version, uint32_t* skip, uint32_t* weights)
{
    bool old = version <= 4004;
    switch (mode)
    {
    case 's':
    case 'b':
        *skip = 2;
        *weights = version == 4003 ? 0 : 1;
        return true;
    case 'a':
        *skip = 0;
        *weights = old ? 0 : 1;
        return true;
    case 'p':
    case 'q':
        *skip = 2;
        *weights = old ? 1 : 2;
        return true;
    case 't':
        *skip = 3;
        *weights = 0;
        return true;
    case 'd':
        *skip = 1;
        *weights = 1;
        return true;
    default:
        return false;
    }
}

// A cubic key's slope mode and its weight modes.
static bool Cubic(Parser* p, Modes* m)
{
    if (!Has(p, 1))
    {
        return false;
    }
    char mode = Letter(p->data[p->at++]);
    uint32_t skip = 0;
    uint32_t weights = 0;
    if (!SlopeCounts(mode, p->version, &skip, &weights) || !Has(p, skip))
    {
        return false;
    }
    if (mode == 's' || mode == 'b')
    {
        m->slopeRight = maniFbxNarrow(p->data[p->at]);
        m->nextSlopeLeft = maniFbxNarrow(p->data[p->at + 1]);
    }
    else
    {
        m->autoSlope = true;
    }
    p->at += skip;
    for (; weights > 0; --weights)
    {
        if (!WeightMode(p, m))
        {
            return false;
        }
    }
    return true;
}

// A key's mode and its values; false for what ufbx fails on.
static bool Mode(Parser* p, maniFbxKey* key, Modes* m)
{
    char mode = Letter(p->data[p->at - 1]);
    if (mode == 'U')
    {
        key->interpolation = maniFbxCubic;
        return Cubic(p, m);
    }
    if (mode == 'L')
    {
        key->interpolation = maniFbxLinear;
        return true;
    }
    if (mode != 'C')
    {
        return false;
    }
    key->interpolation = maniFbxConstantPrevious;
    if (p->version >= 4004)
    {
        if (!Has(p, 1))
        {
            return false;
        }
        key->interpolation =
            Letter(p->data[p->at++]) == 'n' ? maniFbxConstantNext : maniFbxConstantPrevious;
    }
    return true;
}

// A time in ticks as seconds; false unless whole and in range.
static bool Ticks(double ticks, int64_t* out)
{
    if (!(ticks >= -0x1p63 && ticks < 0x1p63) || ticks != (double)(int64_t)ticks)
    {
        return false;
    }
    *out = (int64_t)ticks;
    return true;
}

// The handles from the slopes and weights, against the neighbours.
static void Handles(Parser* p, maniFbxKey* key, const Modes* m, double nextTime)
{
    if (key->time > p->previousTime)
    {
        double delta = key->time - p->previousTime;
        float dx = maniFbxNarrow((double)p->weightLeft * delta);
        key->leftDx = (double)dx;
        key->leftDy = (double)maniFbxNarrow((double)dx * (double)p->slopeLeft);
    }
    if (nextTime > key->time)
    {
        double delta = nextTime - key->time;
        float dx = maniFbxNarrow((double)m->weightRight * delta);
        key->rightDx = (double)dx;
        key->rightDy = (double)maniFbxNarrow((double)dx * (double)m->slopeRight);
    }
    p->slopeLeft = m->nextSlopeLeft;
    p->weightLeft = m->nextWeightLeft;
    p->previousTime = key->time;
}

// Key i (keys[i - 1] read before it); false for what ufbx fails on.
static bool Key(Parser* p, maniFbxKey* keys, uint32_t i, uint32_t count)
{
    maniFbxKey* key = &keys[i];
    memset(key, 0, sizeof *key);
    if (!Has(p, 3))
    {
        return false;
    }
    key->time = p->nextTicks / p->ticksPerSecond;
    key->value = p->nextValue;
    p->at += 3;
    Modes m = {0.0f, MANI_FBX_THIRD, 0.0f, MANI_FBX_THIRD, false};
    if (!Mode(p, key, &m))
    {
        return false;
    }
    if (i + 1 < count)
    {
        if (!Has(p, 2))
        {
            return false;
        }
        p->nextTicks = p->data[p->at];
        p->nextValue = p->data[p->at + 1];
    }
    double nextTime = p->nextTicks / p->ticksPerSecond;
    if (m.autoSlope)
    {
        float slope =
            i > 0 ? maniFbxAutoSlope(p->previousTime, key->time, nextTime, keys[i - 1].value,
                                     key->value, p->nextValue, p->weightLeft, m.weightRight, 0.0f,
                                     maniFbxKeyClampProgressive | maniFbxKeyTimeIndependent)
                  : 0.0f;
        p->slopeLeft = slope;
        m.slopeRight = slope;
    }
    if (key->interpolation == maniFbxLinear)
    {
        float slope = nextTime > key->time
                          ? maniFbxNarrow((p->nextValue - key->value) / (nextTime - key->time))
                          : 0.0f;
        m.slopeRight = slope;
        m.nextSlopeLeft = slope;
    }
    Handles(p, key, &m, nextTime);
    return true;
}

// The keys of a parsed list, their end ticks and order checked.
static bool Parse(Parser* p, maniFbxCurve* out)
{
    if (out->count > 0 && !Has(p, 2))
    {
        return false;
    }
    p->nextTicks = out->count > 0 ? p->data[0] : 0.0;
    p->nextValue = out->count > 0 ? p->data[1] : 0.0;
    int64_t previous = INT64_MIN;
    for (uint32_t i = 0; i < out->count; ++i)
    {
        int64_t ticks = 0;
        if (!Ticks(p->nextTicks, &ticks) || ticks < previous || !Key(p, out->keys, i, out->count))
        {
            return false;
        }
        out->firstTick = i == 0 ? ticks : out->firstTick;
        out->lastTick = ticks;
        previous = ticks;
    }
    return p->at == p->end;
}

// KeyVer, its version's default when absent or not positive.
static int64_t KeyVersion(const maniFbx* fbx, uint32_t node)
{
    uint32_t child = maniFbxFindChild(fbx, node, "KeyVer", 6);
    int64_t version = 0;
    if (child != MANI_FBX_NONE)
    {
        (void)maniFbxInteger(fbx, maniFbxNodeProperty(fbx, child, 0), INT32_MIN, INT32_MAX,
                             &version);
    }
    if (version > 0)
    {
        return version;
    }
    return fbx->version < 5000 ? 4003 : (fbx->version < 6000 ? 4004 : 4005);
}

maniImportResult maniFbxReadKeys(const maniFbx* fbx, uint32_t node, uint32_t item,
                                 maniFbxCurve* out)
{
    *out = (maniFbxCurve){NULL, 0, 0, {0, -1}, {0, -1}, (double)fbx->settings.ticksPerSecond, 0, 0};
    uint32_t list = maniFbxFindChild(fbx, node, "Key", 3);
    if (list == MANI_FBX_NONE)
    {
        return maniFbxSuccess();
    }
    out->pre = maniFbxReadExtrapolation(fbx, node, "Pre-Extrapolation");
    out->post = maniFbxReadExtrapolation(fbx, node, "Post-Extrapolation");
    maniImportResult refused =
        maniFbxFail(maniImportReasonProperty, fbx->nodes[node].nameOffset, item);
    uint32_t countNode = maniFbxFindChild(fbx, node, "KeyCount", 8);
    uint32_t values = 0;
    int64_t count = 0;
    (void)maniFbxReadValues(fbx, list, NULL, &values);
    // Every key takes at least three values.
    if (countNode == MANI_FBX_NONE ||
        !maniFbxInteger(fbx, maniFbxNodeProperty(fbx, countNode, 0), 0, values / 3, &count))
    {
        return refused;
    }
    double* data = maniAllocate(&fbx->allocator, (size_t)values + 1, sizeof(double), 8);
    out->keys = maniAllocate(&fbx->allocator, (size_t)count + 1, sizeof(maniFbxKey), 8);
    maniImportResult r = data != NULL && out->keys != NULL ? maniFbxSuccess() : Capacity();
    r = r.result == mani_success ? maniFbxReadValues(fbx, list, data, &values) : r;
    r.item = r.result == mani_errorFormat ? item : r.item;
    out->count = (uint32_t)count;
    Parser p = {data, 0,   values, KeyVersion(fbx, node), out->ticksPerSecond, 0.0f, MANI_FBX_THIRD,
                0.0,  0.0, 0.0};
    if (r.result == mani_success && !Parse(&p, out))
    {
        r = refused;
    }
    maniRelease(&fbx->allocator, data, (size_t)values + 1, sizeof(double), 8);
    if (r.result != mani_success)
    {
        maniRelease(&fbx->allocator, out->keys, (size_t)count + 1, sizeof(maniFbxKey), 8);
        out->keys = NULL;
        out->count = 0;
    }
    return r;
}
