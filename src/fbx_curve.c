// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX animation curves (mani-0016), as ufbx reads and evaluates them
// (research 24), its float roundings kept so the differential compares
// like with like. Flags and their four attribute words are run-length
// encoded over the keys. Tangents become Bézier handles: dx is a weight
// (1/3 unless weighted) times the time to the neighbour, dy the slope
// times dx. Between keys the cubic's parameter is found by Newton's
// method; outside them each end's extrapolation applies, counted in
// whole ticks from the end keys' own.

#include "fbx_curve.h"

#include "allocator.h"
#include "fbx_document.h"
#include "fbx_value.h"

#include <float.h>
#include <math.h>
#include <string.h>

// KeyAttrFlags bits.
#define KEY_CONSTANT          0x2u
#define KEY_CUBIC             0x8u
#define KEY_CONSTANT_NEXT     0x100u
#define KEY_TCB               0x200u
#define KEY_USER              0x400u
#define KEY_CLAMP             0x1000u
#define KEY_TIME_INDEPENDENT  0x2000u
#define KEY_CLAMP_PROGRESSIVE 0x4000u
#define KEY_WEIGHTED_RIGHT    0x1000000u
#define KEY_WEIGHTED_LEFT     0x2000000u

// FBX's unweighted handle length, as binary32 the way ufbx keeps it.
#define THIRD 0.333333f

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static float FloatOf(uint32_t bits)
{
    float f = 0.0f;
    memcpy(&f, &bits, sizeof f);
    return f;
}

// A double rounded to binary32 as ufbx keeps tangents, infinite past
// its range (where C leaves the conversion undefined).
static float Narrow(double x)
{
    return x > (double)FLT_MAX ? INFINITY : x < -(double)FLT_MAX ? -INFINITY : (float)x;
}

// floor and rint are not among the exactly rounded functions the library
// may call; fmod is.
static double Floor(double x)
{
    double r = x - fmod(x, 1.0);
    return r > x ? r - 1.0 : r;
}

// The curve's arrays, read whole.
typedef struct Arrays
{
    int64_t* ticks;
    double* values;
    int64_t* flags;
    uint32_t* attributes;
    int64_t* runs;
    uint32_t keyCount;
    uint32_t runCount;
    uint32_t attributeCount;
} Arrays;

static void* Block(const maniFbx* fbx, size_t count, size_t size)
{
    return maniAllocate(&fbx->allocator, count + 1, size, 8);
}

static void FreeBlock(const maniFbx* fbx, void* block, size_t count, size_t size)
{
    maniRelease(&fbx->allocator, block, count + 1, size, 8);
}

// A named array child's first property and element count; refused
// (Property) when there is none.
static maniImportResult Find(const maniFbx* fbx, uint32_t node, const char* name, uint32_t object,
                             uint32_t* property, uint32_t* count)
{
    uint32_t child = maniFbxFindChild(fbx, node, name, strlen(name));
    *property = child != MANI_FBX_NONE ? maniFbxNodeProperty(fbx, child, 0) : MANI_FBX_NONE;
    *count = maniFbxArrayCount(fbx, *property);
    bool array = *property != MANI_FBX_NONE && fbx->properties[*property].type != 0 &&
                 strchr("bcifldA", fbx->properties[*property].type) != NULL;
    if (!array)
    {
        uint64_t at = child != MANI_FBX_NONE ? fbx->nodes[child].nameOffset
                                             : fbx->nodes[fbx->objects[object].node].nameOffset;
        return maniFbxFail(maniImportReasonProperty, at, object);
    }
    return maniFbxSuccess();
}

static void ReleaseArrays(const maniFbx* fbx, Arrays* a)
{
    FreeBlock(fbx, a->ticks, a->keyCount, sizeof(int64_t));
    FreeBlock(fbx, a->values, a->keyCount, sizeof(double));
    FreeBlock(fbx, a->flags, a->runCount, sizeof(int64_t));
    FreeBlock(fbx, a->attributes, a->attributeCount, sizeof(uint32_t));
    FreeBlock(fbx, a->runs, a->runCount, sizeof(int64_t));
}

static maniImportResult WithItem(maniImportResult r, uint32_t object)
{
    r.item = r.result == mani_errorFormat ? object : r.item;
    return r;
}

static maniImportResult ReadArrays(const maniFbx* fbx, uint32_t object, Arrays* a)
{
    uint32_t node = fbx->objects[object].node;
    uint32_t p[5];
    uint32_t n[5];
    static const char* const kNames[5] = {"KeyTime", "KeyValueFloat", "KeyAttrFlags",
                                          "KeyAttrDataFloat", "KeyAttrRefCount"};
    for (uint32_t k = 0; k < 5; ++k)
    {
        maniImportResult r = Find(fbx, node, kNames[k], object, &p[k], &n[k]);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    uint64_t at = fbx->nodes[node].nameOffset;
    if (n[0] != n[1] || n[2] != n[4] || (uint64_t)n[3] != (uint64_t)n[4] * 4u)
    {
        return maniFbxFail(maniImportReasonProperty, at, object);
    }
    a->keyCount = n[0];
    a->runCount = n[4];
    a->attributeCount = n[3];
    a->ticks = Block(fbx, n[0], sizeof(int64_t));
    a->values = Block(fbx, n[0], sizeof(double));
    a->flags = Block(fbx, n[2], sizeof(int64_t));
    a->attributes = Block(fbx, n[3], sizeof(uint32_t));
    a->runs = Block(fbx, n[4], sizeof(int64_t));
    if (a->ticks == NULL || a->values == NULL || a->flags == NULL || a->attributes == NULL ||
        a->runs == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniFbxReadIntegers(fbx, p[0], a->ticks);
    r = r.result == mani_success ? maniFbxReadDoubles(fbx, p[1], a->values) : r;
    r = r.result == mani_success ? maniFbxReadIntegers(fbx, p[2], a->flags) : r;
    r = r.result == mani_success ? maniFbxReadFloatBits(fbx, p[3], a->attributes) : r;
    r = r.result == mani_success ? maniFbxReadIntegers(fbx, p[4], a->runs) : r;
    return WithItem(r, object);
}

// Times in order and runs covering every key.
static maniImportResult Check(const maniFbx* fbx, uint32_t object, const Arrays* a)
{
    uint64_t at = fbx->nodes[fbx->objects[object].node].nameOffset;
    uint64_t covered = 0;
    for (uint32_t k = 0; k < a->runCount && covered < a->keyCount; ++k)
    {
        if (a->runs[k] <= 0)
        {
            return maniFbxFail(maniImportReasonProperty, at, object);
        }
        covered += (uint64_t)a->runs[k];
    }
    if (covered < a->keyCount)
    {
        return maniFbxFail(maniImportReasonProperty, at, object);
    }
    for (uint32_t i = 1; i < a->keyCount; ++i)
    {
        if (a->ticks[i] < a->ticks[i - 1])
        {
            return maniFbxFail(maniImportReasonProperty, at, object);
        }
    }
    return maniFbxSuccess();
}

// The time-weighted slope pulled toward the left or right one by the
// auto bias (|bias| / 100), pushed by a bias beyond 500.
static double Biased(double slope, double left, double right, float bias)
{
    double weight = fabs((double)bias) / 100.0;
    if (!(weight > 0.0001))
    {
        return slope;
    }
    double target = bias > 0.0f ? right : left;
    double toward = target - slope;
    slope = slope * (1.0 - weight) + target * weight;
    double beyond = weight - 5.0;
    if (beyond > 0.0)
    {
        double sign = fabs(toward) > 0.00001 ? toward : (double)bias;
        slope += beyond * beyond * (sign > 0.0 ? 1.0 : -1.0) * 40.0;
    }
    return slope;
}

// The slope limited so neither handle overshoots its neighbour's value.
static double Progressive(double slope, double rangeLeft, double rangeRight, double riseLeft,
                          double riseRight)
{
    double sign = slope >= 0.0 ? 1.0 : -1.0;
    double magnitude = sign * slope;
    double maxLeft = rangeLeft > 0.0 ? sign * riseLeft / rangeLeft : 0.0;
    double maxRight = rangeRight > 0.0 ? sign * riseRight / rangeRight : 0.0;
    maxLeft = maxLeft > 0.0 ? maxLeft : 0.0;
    maxRight = maxRight > 0.0 ? maxRight : 0.0;
    magnitude = magnitude > maxLeft ? maxLeft : magnitude;
    magnitude = magnitude > maxRight ? maxRight : magnitude;
    return sign * magnitude;
}

// Auto tangents, as ufbx (and Maya) solve them; the clamp threshold is
// ufbx's default, 0.
static float AutoSlope(double prevTime, double time, double nextTime, double prevValue,
                       double value, double nextValue, float weightLeft, float weightRight,
                       float bias, uint32_t flags)
{
    if ((flags & KEY_CLAMP) != 0 && (prevValue == value || nextValue == value))
    {
        return 0.0f;
    }
    double slope = (nextValue - prevValue) / (nextTime - prevTime);
    if ((flags & KEY_TIME_INDEPENDENT) == 0)
    {
        double left = (value - prevValue) / (time - prevTime);
        double right = (nextValue - value) / (nextTime - time);
        double delta = (time - prevTime) / (nextTime - prevTime);
        slope = slope * 0.5 + (left * (1.0 - delta) + right * delta) * 0.5;
        slope = Biased(slope, left, right, bias);
    }
    if ((flags & KEY_CLAMP_PROGRESSIVE) != 0)
    {
        slope = Progressive(slope, (double)weightLeft * (time - prevTime),
                            (double)weightRight * (nextTime - time), value - prevValue,
                            nextValue - value);
    }
    return Narrow(slope);
}

// An end key's auto tangent: the one-sided slope, pushed by a bias
// beyond 500.
static float EndSlope(double fromTime, double toTime, double fromValue, double toValue, float bias,
                      uint32_t flags)
{
    if ((flags & KEY_CLAMP_PROGRESSIVE) != 0 || ((flags & KEY_CLAMP) != 0 && toValue == fromValue))
    {
        return 0.0f;
    }
    double slope = (toValue - fromValue) / (toTime - fromTime);
    double beyond = fabs((double)bias) / 100.0 - 5.0;
    if ((flags & KEY_TIME_INDEPENDENT) == 0 && beyond > 0.0)
    {
        slope += beyond * beyond * (bias > 0.0f ? 1.0 : -1.0) * 40.0;
    }
    return Narrow(slope);
}

// The decoder's state across keys: the left slope and weight the
// previous key gave this one, and the previous key's time.
typedef struct Decoder
{
    const Arrays* a;
    double tps;
    float slopeLeft;
    float weightLeft;
    double previousTime;
} Decoder;

static double TimeOf(const Decoder* d, uint32_t i)
{
    return (double)d->a->ticks[i] / d->tps;
}

// Key i's slopes (left and right) from its auto tangents.
static void AutoTangents(const Decoder* d, uint32_t i, double time, double next, float weightRight,
                         uint32_t flags, float* left, float* right)
{
    const double* v = d->a->values;
    bool before = i > 0 && time > d->previousTime;
    bool after = i + 1 < d->a->keyCount && next > time;
    if (before && after)
    {
        if (fabsf(*left + *right) <= 0.0001f)
        {
            *left = *right = AutoSlope(d->previousTime, time, next, v[i - 1], v[i], v[i + 1],
                                       d->weightLeft, weightRight, *right, flags);
            return;
        }
        float l = AutoSlope(d->previousTime, time, next, v[i - 1], v[i], v[i + 1], d->weightLeft,
                            weightRight, -*left, flags);
        *right = AutoSlope(d->previousTime, time, next, v[i - 1], v[i], v[i + 1], d->weightLeft,
                           weightRight, *right, flags);
        *left = l;
    }
    else if (before)
    {
        *left = *right = EndSlope(d->previousTime, time, v[i - 1], v[i], -*left, flags);
    }
    else if (after)
    {
        *left = *right = EndSlope(time, next, v[i], v[i + 1], *right, flags);
    }
    else
    {
        *left = *right = 0.0f;
    }
}

// Kochanek-Bartels slopes from tension, continuity and bias.
static void TcbTangents(const Decoder* d, uint32_t i, double time, double next,
                        const uint32_t* words, float* left, float* right)
{
    const double* v = d->a->values;
    double slopeLeft = 0.0;
    double slopeRight = 0.0;
    bool edge = false;
    if (i > 0 && time > d->previousTime)
    {
        slopeLeft = (v[i] - v[i - 1]) / (time - d->previousTime);
    }
    else
    {
        edge = true;
    }
    if (i + 1 < d->a->keyCount && next > time)
    {
        slopeRight = (v[i + 1] - v[i]) / (next - time);
    }
    else
    {
        edge = true;
    }
    double tension = (double)FloatOf(words[0]);
    double continuity = (double)FloatOf(words[1]);
    double bias = (double)FloatOf(words[2]);
    double factor = edge ? 1.0 : 0.5;
    double d00 = factor * (1.0 - tension) * (1.0 + bias) * (1.0 - continuity);
    double d01 = factor * (1.0 - tension) * (1.0 - bias) * (1.0 + continuity);
    double d10 = factor * (1.0 - tension) * (1.0 + bias) * (1.0 + continuity);
    double d11 = factor * (1.0 - tension) * (1.0 - bias) * (1.0 - continuity);
    *left = Narrow(d00 * slopeLeft + d01 * slopeRight);
    *right = Narrow(d10 * slopeLeft + d11 * slopeRight);
}

// One key's interpolation and handles, from its flags and words.
static void DecodeKey(Decoder* d, uint32_t i, uint32_t flags, const uint32_t* words,
                      maniFbxKey* key)
{
    const double* v = d->a->values;
    double time = TimeOf(d, i);
    double next = i + 1 < d->a->keyCount ? TimeOf(d, i + 1) : time;
    float slopeRight = FloatOf(words[0]);
    float weightRight = THIRD;
    float nextSlopeLeft = FloatOf(words[1]);
    float nextWeightLeft = THIRD;
    if ((flags & KEY_WEIGHTED_RIGHT) != 0)
    {
        weightRight = (float)(words[2] & 0xffffu) * 0.0001f;
    }
    if ((flags & KEY_WEIGHTED_LEFT) != 0)
    {
        nextWeightLeft = (float)(words[2] >> 16) * 0.0001f;
    }
    key->time = time;
    key->value = v[i];
    if ((flags & KEY_CONSTANT) != 0)
    {
        key->interpolation =
            (flags & KEY_CONSTANT_NEXT) != 0 ? maniFbxConstantNext : maniFbxConstantPrevious;
        weightRight = nextWeightLeft = THIRD;
        slopeRight = nextSlopeLeft = 0.0f;
    }
    else if ((flags & KEY_CUBIC) != 0)
    {
        key->interpolation = maniFbxCubic;
        if ((flags & KEY_TCB) != 0)
        {
            TcbTangents(d, i, time, next, words, &d->slopeLeft, &slopeRight);
            nextSlopeLeft = 0.0f;
            nextWeightLeft = THIRD;
        }
        else if ((flags & KEY_USER) == 0)
        {
            AutoTangents(d, i, time, next, weightRight, flags, &d->slopeLeft, &slopeRight);
        }
    }
    else
    {
        // Linear, or a kind FBX does not name: the straight line.
        key->interpolation = maniFbxLinear;
        weightRight = nextWeightLeft = THIRD;
        double slope = next > time ? (v[i + 1] - v[i]) / (next - time) : 0.0;
        slopeRight = nextSlopeLeft = Narrow(slope);
    }
    key->leftDx = time > d->previousTime
                      ? (double)Narrow((double)d->weightLeft * (time - d->previousTime))
                      : 0.0;
    key->leftDy = (double)Narrow(key->leftDx * (double)d->slopeLeft);
    key->rightDx = next > time ? (double)Narrow((double)weightRight * (next - time)) : 0.0;
    key->rightDy = (double)Narrow(key->rightDx * (double)slopeRight);
    d->slopeLeft = nextSlopeLeft;
    d->weightLeft = nextWeightLeft;
    d->previousTime = time;
}

static maniFbxExtrapolation ReadExtrapolation(const maniFbx* fbx, uint32_t node, const char* name)
{
    maniFbxExtrapolation e = {maniFbxExtrapolateConstant, -1};
    uint32_t child = maniFbxFindChild(fbx, node, name, strlen(name));
    uint32_t type =
        child != MANI_FBX_NONE ? maniFbxFindChild(fbx, child, "Type", 4) : MANI_FBX_NONE;
    int64_t mode = 0;
    if (type == MANI_FBX_NONE ||
        !maniFbxInteger(fbx, maniFbxNodeProperty(fbx, type, 0), INT32_MIN, INT32_MAX, &mode))
    {
        return e;
    }
    e.mode = mode == 'A'   ? maniFbxExtrapolateRelative
             : mode == 'K' ? maniFbxExtrapolateSlope
             : mode == 'M' ? maniFbxExtrapolateMirror
             : mode == 'R' ? maniFbxExtrapolateRepeat
                           : maniFbxExtrapolateConstant;
    uint32_t repetition = maniFbxFindChild(fbx, child, "Repetition", 10);
    int64_t count = -1;
    if (repetition != MANI_FBX_NONE &&
        maniFbxInteger(fbx, maniFbxNodeProperty(fbx, repetition, 0), INT32_MIN, INT32_MAX, &count))
    {
        e.repeat = count < 0 ? -1 : (int32_t)count;
    }
    return e;
}

maniImportResult maniFbxReadCurve(const maniFbx* fbx, uint32_t object, maniFbxCurve* out)
{
    *out = (maniFbxCurve){NULL, 0, 0, {0, -1}, {0, -1}, (double)fbx->settings.ticksPerSecond};
    uint32_t node = fbx->objects[object].node;
    out->pre = ReadExtrapolation(fbx, node, "Pre-Extrapolation");
    out->post = ReadExtrapolation(fbx, node, "Post-Extrapolation");
    Arrays a = {0};
    maniImportResult r = ReadArrays(fbx, object, &a);
    r = r.result == mani_success ? Check(fbx, object, &a) : r;
    if (r.result == mani_success)
    {
        out->keys = Block(fbx, a.keyCount, sizeof(maniFbxKey));
        r = out->keys != NULL ? r : Capacity();
    }
    if (r.result == mani_success)
    {
        out->count = a.keyCount;
        Decoder d = {&a, out->ticksPerSecond, 0.0f, THIRD, 0.0};
        uint32_t run = 0;
        int64_t left = a.runCount > 0 ? a.runs[0] : 0;
        for (uint32_t i = 0; i < a.keyCount; ++i)
        {
            DecodeKey(&d, i, (uint32_t)a.flags[run], a.attributes + 4 * (size_t)run, &out->keys[i]);
            if (--left == 0 && run + 1 < a.runCount)
            {
                left = a.runs[++run];
            }
        }
    }
    ReleaseArrays(fbx, &a);
    return r;
}

void maniFbxReleaseCurve(const maniFbx* fbx, maniFbxCurve* curve)
{
    FreeBlock(fbx, curve->keys, curve->count, sizeof(maniFbxKey));
    curve->keys = NULL;
    curve->count = 0;
}

// Newton's method on the Bézier's time component, as ufbx runs it.
static double BezierParameter(double p1, double p2, double x0)
{
    double a = p1 * 3.0 - p2 * 3.0 + 1.0;
    double b = p2 * 3.0 - p1 * 3.0 - p1 * 3.0;
    double c = p1 * 3.0;
    double t = x0;
    double x1 = 0.0;
    for (uint32_t i = 0; i < 11; ++i)
    {
        double t2 = t * t;
        x1 = a * t2 * t + b * t2 + c * t - x0;
        t -= x1 / (3.0 * a * t2 + 2.0 * b * t + c);
        // Three steps always, then pairs until within 4 ulp of 1.
        if (i >= 2 && i % 2 == 0 && fabs(x1) <= 8.881784197001252e-16)
        {
            break;
        }
    }
    return t;
}

// Between keys, or exactly on one, with no extrapolation.
static double Inside(const maniFbxCurve* c, double time)
{
    const maniFbxKey* k = c->keys;
    uint32_t low = 0;
    uint32_t high = c->count;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (k[mid].time <= time)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    if (low == 0 || low == c->count)
    {
        return low == 0 ? k[0].value : k[c->count - 1].value;
    }
    const maniFbxKey* prev = &k[low - 1];
    const maniFbxKey* next = &k[low];
    if (prev->time == time)
    {
        return prev->value;
    }
    double rcp = 1.0 / (next->time - prev->time);
    double t = (time - prev->time) * rcp;
    switch (prev->interpolation)
    {
    case maniFbxConstantPrevious:
        return prev->value;
    case maniFbxConstantNext:
        return next->value;
    case maniFbxLinear:
        return prev->value * (1.0 - t) + next->value * t;
    default:
        break;
    }
    t = BezierParameter(prev->rightDx * rcp, 1.0 - next->leftDx * rcp, t);
    double u = 1.0 - t;
    double y1 = prev->value + prev->rightDy;
    double y2 = next->value - next->leftDy;
    return u * u * u * prev->value + 3.0 * (u * u * t * y1 + u * t * t * y2) +
           t * t * t * next->value;
}

// Before the first key or after the last: constant, slope, or a repeat
// in whole ticks (plain, relative or mirrored).
static double Outside(const maniFbxCurve* c, double time)
{
    bool pre = time < c->keys[0].time;
    const maniFbxKey* key = pre ? &c->keys[0] : &c->keys[c->count - 1];
    maniFbxExtrapolation e = pre ? c->pre : c->post;
    if (e.mode == maniFbxExtrapolateSlope)
    {
        double dx = pre ? key->rightDx : key->leftDx;
        double dy = pre ? key->rightDy : key->leftDy;
        return dx != 0.0 ? key->value + dy * ((time - key->time) / dx) : key->value;
    }
    if (e.mode == maniFbxExtrapolateConstant || e.repeat == 0)
    {
        return key->value;
    }
    double scale = c->ticksPerSecond;
    double first = Floor(c->keys[0].time * scale + 0.5);
    double last = Floor(c->keys[c->count - 1].time * scale + 0.5);
    double ticks = time * scale;
    double delta = pre ? first - ticks : ticks - last;
    double duration = last - first;
    if (!(duration >= 1.0))
    {
        return key->value;
    }
    double n = Floor(delta / duration);
    double into = delta - n * duration;
    if (e.repeat > 0 && n >= (double)e.repeat)
    {
        n = (double)(e.repeat - 1);
        into = duration;
    }
    if (e.mode == maniFbxExtrapolateMirror && n * 0.5 - Floor(n * 0.5) <= 0.25)
    {
        into = duration - into;
    }
    into = pre ? duration - into : into;
    double value = Inside(c, (first + into) / scale);
    if (e.mode == maniFbxExtrapolateRelative)
    {
        double change = c->keys[c->count - 1].value - c->keys[0].value;
        value += (pre ? -change : change) * (n + 1.0);
    }
    return value;
}

double maniFbxEvaluateCurve(const maniFbxCurve* curve, double time, double fallback)
{
    if (curve->count <= 1)
    {
        return curve->count == 1 ? curve->keys[0].value : fallback;
    }
    if (time < curve->keys[0].time || time > curve->keys[curve->count - 1].time)
    {
        return Outside(curve, time);
    }
    return Inside(curve, time);
}
