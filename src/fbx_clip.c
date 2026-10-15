// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX clips (mani-0016): a stack sampled for an opened skeleton. Each
// bone of the skeleton named as a model (or as a model's helper, its
// name with "#unscaled") takes its local at every sample the way a
// skeleton built from the file formed its rest: its own transform under
// its parent's bone (normal inheritance) or its parent's helper, else
// its world relative to its parent bone's, decomposed (refused, Matrix,
// where that shears); the skeleton's root its own transform, or folded
// with the placement; a helper the inverse of its model's inherited
// scale. Samples lie on the clip builder's grid, evaluated at whole
// ticks from the stack's start, and reach the builder as linear keys at
// its own sample times. A channel that never leaves the skeleton's rest
// is left out. Other numbers a bone's model animates become float tracks
// named bone/property (with /component for vectors).

#include "arena.h"
#include "basis.h"
#include "clip_build.h"
#include "clip_file.h"
#include "decompose.h"
#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_options.h"
#include "fbx_sample.h"
#include "fbx_transform.h"
#include "fbx_value.h"
#include "fbx_world.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdckdint.h>
#include <stddef.h>
#include <string.h>

static const char kHelperSuffix[] = "#unscaled";
#define MANI_HELPER_SUFFIX (sizeof kHelperSuffix - 1)

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

uint32_t maniFbxClipCount(const maniFbx* fbx)
{
    return fbx != NULL ? fbx->stackCount : 0;
}

const char* maniFbxClipName(const maniFbx* fbx, uint32_t clip, uint32_t* lengthOut)
{
    bool inRange = fbx != NULL && clip < fbx->stackCount;
    if (lengthOut != NULL)
    {
        *lengthOut =
            inRange ? (uint32_t)(fbx->stackNameOffsets[clip + 1] - fbx->stackNameOffsets[clip]) : 0;
    }
    return inRange ? fbx->stackNames + fbx->stackNameOffsets[clip] : NULL;
}

// Where a bone's local comes from.
enum
{
    // No model: no track.
    kNone,
    // The skeleton's root.
    kRoot,
    // Its model's helper: the inverse inherited scale.
    kHelper,
    // Under its model's parent's bone: its own transform when normal,
    // else relative.
    kUnderParent,
    // Under its model's parent's helper: its own transform unless normal.
    kUnderHelper,
    // Anywhere else: relative to its parent bone's world.
    kRelative,
    // Under a bone with no model: its own transform.
    kOwn,
};

typedef struct Clip
{
    const maniFbx* fbx;
    const maniSkeleton* skeleton;
    const maniFbxOptions* options;
    maniBasis basis;
    maniArena arena;
    maniFbxAnimation animation;
    maniFbxSampler sampler;
    uint32_t boneCount;
    // Per bone: its model (MANI_FBX_NONE for none) and its rule.
    uint32_t* modelOf;
    uint8_t* rule;
    // Per model: needed for the worlds.
    uint8_t* needed;
    maniFbxWorld* worlds;
    float rate;
    float duration;
    uint32_t sampleCount;
    // The builder's sample times, and the poses: sampleCount by boneCount.
    float* times;
    maniTransform* poses;
    maniBoneTrackSource* tracks;
    uint32_t trackCount;
    maniFloatTrackSource* floats;
    uint32_t floatCount;
    // Per float track, its model, for refusals.
    uint32_t* floatModel;
} Clip;

static uint64_t ModelOffset(const maniFbx* fbx, uint32_t model)
{
    return fbx->nodes[fbx->objects[fbx->models[model]].node].nameOffset;
}

// Each model's bone, and its helper's, by name in the skeleton.
static maniImportResult Bind(Clip* c)
{
    const maniFbx* fbx = c->fbx;
    uint8_t* helper = maniArenaTake(&c->arena, c->boneCount, 1);
    char* name = NULL;
    if (helper == NULL)
    {
        return Capacity();
    }
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        uint32_t length = 0;
        const char* own = maniFbxModelName(fbx, m, &length);
        uint32_t bone = maniSkeletonFindBone(c->skeleton, own, length);
        if (bone != MANI_NO_BONE)
        {
            c->modelOf[bone] = m;
        }
        name = maniArenaTake(&c->arena, (size_t)length + MANI_HELPER_SUFFIX, 1);
        if (name == NULL)
        {
            return Capacity();
        }
        memcpy(name, own, length);
        memcpy(name + length, kHelperSuffix, MANI_HELPER_SUFFIX);
        bone = maniSkeletonFindBone(c->skeleton, name, (size_t)length + MANI_HELPER_SUFFIX);
        if (bone != MANI_NO_BONE)
        {
            c->modelOf[bone] = m;
            helper[bone] = 1;
        }
    }
    for (uint32_t b = 0; b < c->boneCount; ++b)
    {
        c->rule[b] = helper[b] != 0 ? kHelper : c->rule[b];
    }
    return maniFbxSuccess();
}

// Each bone's rule from where the skeleton puts it.
static void Rules(Clip* c)
{
    const maniFbx* fbx = c->fbx;
    for (uint32_t b = 0; b < c->boneCount; ++b)
    {
        uint32_t m = c->modelOf[b];
        if (m == MANI_FBX_NONE || c->rule[b] == kHelper)
        {
            continue;
        }
        uint32_t pb = maniSkeletonParent(c->skeleton, b);
        uint32_t p = fbx->modelParents[m];
        uint32_t pm = pb != MANI_NO_BONE ? c->modelOf[pb] : MANI_FBX_NONE;
        bool parentHelper = pb != MANI_NO_BONE && c->rule[pb] == kHelper;
        c->rule[b] = pb == MANI_NO_BONE        ? kRoot
                     : pm == MANI_FBX_NONE     ? kOwn
                     : pm == p && parentHelper ? kUnderHelper
                     : pm == p                 ? kUnderParent
                                               : kRelative;
    }
}

// The models the bones read, and every model above them.
static void Need(Clip* c)
{
    const maniFbx* fbx = c->fbx;
    for (uint32_t b = 0; b < c->boneCount; ++b)
    {
        for (uint32_t v = c->modelOf[b]; v != MANI_FBX_NONE && c->needed[v] == 0;
             v = fbx->modelParents[v])
        {
            c->needed[v] = 1;
        }
    }
}

// The frame rate the scene states, or 0.
static double SceneRate(const maniFbx* fbx)
{
    const maniFbxGlobalSettings* s = &fbx->settings;
    return s->rateDenominator != 0 ? (double)s->rateNumerator / (double)s->rateDenominator
                                   : s->customRate;
}

// x rounded to the nearest whole number, for x not negative.
static double Round(double x)
{
    double y = x + 0.5;
    return y - fmod(y, 1.0);
}

static maniImportResult Timing(Clip* c, float sampleRate)
{
    double scene = SceneRate(c->fbx);
    c->rate = sampleRate != 0.0f              ? sampleRate
              : scene > 0.0 && scene <= 120.0 ? (float)scene
                                              : 30.0f;
    double tps = (double)c->fbx->settings.ticksPerSecond;
    double span = (double)c->animation.stop - (double)c->animation.start;
    c->duration = span > 0.0 ? (float)(span / tps) : 0.0f;
    if (!maniClipSampleCountFor(c->duration, c->rate, &c->sampleCount))
    {
        return maniFbxFail(maniImportReasonBuild, 0, maniBuildReasonTiming);
    }
    size_t poses = 0;
    if (ckd_mul(&poses, (size_t)c->sampleCount, (size_t)c->boneCount))
    {
        return Capacity();
    }
    c->times = maniArenaTake(&c->arena, c->sampleCount, sizeof(float));
    c->poses = maniArenaTake(&c->arena, poses, sizeof(maniTransform));
    return c->times != NULL && c->poses != NULL ? maniFbxSuccess() : Capacity();
}

// Sample k's time in the stack, in seconds: whole ticks from its start,
// the last at its stop.
static double StackTime(const Clip* c, uint32_t k)
{
    double tps = (double)c->fbx->settings.ticksPerSecond;
    double t = maniClipSampleTime(k, c->sampleCount, c->rate, c->duration);
    bool last = k + 1 == c->sampleCount && k > 0;
    double tick = last && c->animation.stop > c->animation.start
                      ? (double)c->animation.stop
                      : (double)c->animation.start + Round(t * tps);
    return tick / tps;
}

static maniFbxMatrix Identity(void)
{
    maniFbxMatrix m = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    return m;
}

// m's world relative to a frame, decomposed; false where it shears or
// the frame has no inverse.
static bool Relative(const maniFbxMatrix* frame, const maniFbxMatrix* world, maniTransform* out)
{
    maniFbxMatrix inverse;
    if (!maniFbxMatrixInverse(frame, &inverse))
    {
        return false;
    }
    maniFbxMatrix local = maniFbxMatrixMul(&inverse, world);
    maniAffine affine = maniFbxMatrixAffine(&local);
    return maniDecomposeExact(&affine, out);
}

// The root's local: its own transform, or folded with its placement as
// the skeleton's rest was.
static bool Root(const Clip* c, uint32_t m, maniTransform* out)
{
    const maniFbxWorld* w = &c->worlds[m];
    if (c->options->foldPlacement == 0)
    {
        *out = maniFbxTrsTransform(&w->transform);
        return true;
    }
    uint32_t p = c->fbx->modelParents[m];
    maniFbxMatrix placement = p == MANI_FBX_NONE                 ? Identity()
                              : w->inherit == maniFbxInheritRSrs ? c->worlds[p].world
                                                                 : c->worlds[p].unscaled;
    maniFbxMatrix own = maniFbxTrsMatrix(&w->transform);
    maniFbxMatrix folded = maniFbxMatrixMul(&placement, &own);
    maniAffine affine = maniFbxMatrixAffine(&folded);
    return maniDecomposeExact(&affine, out);
}

static bool Helper(const Clip* c, uint32_t m, maniTransform* out)
{
    const double* s = c->worlds[m].inheritScale;
    *out = (maniTransform){
        {0, 0, 0}, {0, 0, 0, 1}, {(float)(1.0 / s[0]), (float)(1.0 / s[1]), (float)(1.0 / s[2])}};
    return true;
}

// A bone's frame: its model's world, or unscaled world for a helper.
static const maniFbxMatrix* Frame(const Clip* c, uint32_t bone)
{
    const maniFbxWorld* w = &c->worlds[c->modelOf[bone]];
    return c->rule[bone] == kHelper ? &w->unscaled : &w->world;
}

// One bone's local at the sample the worlds hold; false where it does
// not decompose.
static bool Local(const Clip* c, uint32_t b, maniTransform* out)
{
    uint32_t m = c->modelOf[b];
    const maniFbxWorld* w = &c->worlds[m];
    bool normal = w->inherit == maniFbxInheritRSrs;
    switch (c->rule[b])
    {
    case kRoot:
        return Root(c, m, out);
    case kHelper:
        return Helper(c, m, out);
    case kOwn:
        *out = maniFbxTrsTransform(&w->transform);
        return true;
    default:
        break;
    }
    bool own = (c->rule[b] == kUnderParent && normal) || (c->rule[b] == kUnderHelper && !normal);
    if (own)
    {
        *out = maniFbxTrsTransform(&w->transform);
        return true;
    }
    return Relative(Frame(c, maniSkeletonParent(c->skeleton, b)), &w->world, out);
}

// A transform's ten floats: translation, rotation, scale.
static_assert(sizeof(maniTransform) == 10 * sizeof(float), "ten floats");

static void Floats(const maniTransform* t, float out[10])
{
    memcpy(out, t, sizeof *t);
}

static bool Finite(const maniTransform* t)
{
    float v[10];
    Floats(t, v);
    for (uint32_t i = 0; i < 10; ++i)
    {
        if (!isfinite(v[i]))
        {
            return false;
        }
    }
    return true;
}

static maniImportResult Sample(Clip* c)
{
    for (uint32_t k = 0; k < c->sampleCount; ++k)
    {
        c->times[k] = (float)maniClipSampleTime(k, c->sampleCount, c->rate, c->duration);
        maniImportResult r = maniFbxSampleWorlds(&c->sampler, StackTime(c, k), c->worlds);
        if (r.result != mani_success)
        {
            return r;
        }
        maniTransform* pose = c->poses + (size_t)k * c->boneCount;
        for (uint32_t b = 0; b < c->boneCount; ++b)
        {
            if (c->rule[b] == kNone)
            {
                continue;
            }
            if (!Local(c, b, &pose[b]) || !Finite(&pose[b]))
            {
                uint32_t m = c->modelOf[b];
                return maniFbxFail(maniImportReasonMatrix, ModelOffset(c->fbx, m), m);
            }
            pose[b] = maniBasisTransform(&c->basis, pose[b]);
        }
    }
    return maniFbxSuccess();
}

// Which of a bone's channels leave its rest: bits for translation,
// rotation (either sign) and scale.
static uint32_t Moving(const Clip* c, uint32_t b)
{
    float rest[10];
    maniTransform r = maniSkeletonRest(c->skeleton, b);
    Floats(&r, rest);
    uint32_t bits = 0;
    for (uint32_t k = 0; k < c->sampleCount; ++k)
    {
        float t[10];
        Floats(&c->poses[(size_t)k * c->boneCount + b], t);
        bool sameRotation = true;
        bool oppositeRotation = true;
        for (uint32_t i = 3; i < 7; ++i)
        {
            sameRotation = sameRotation && t[i] == rest[i];
            oppositeRotation = oppositeRotation && t[i] == -rest[i];
        }
        bits |= t[0] != rest[0] || t[1] != rest[1] || t[2] != rest[2] ? 1u : 0u;
        bits |= !sameRotation && !oppositeRotation ? 2u : 0u;
        bits |= t[7] != rest[7] || t[8] != rest[8] || t[9] != rest[9] ? 4u : 0u;
    }
    return bits;
}

// A channel of one bone over the samples: width floats of each pose from
// the field at offset.
static maniImportResult Channel(Clip* c, uint32_t b, size_t offset, uint32_t width,
                                maniChannelSource* out)
{
    float* values = maniArenaTake(&c->arena, (size_t)c->sampleCount * width, sizeof(float));
    if (values == NULL)
    {
        return Capacity();
    }
    for (uint32_t k = 0; k < c->sampleCount; ++k)
    {
        const char* pose = (const char*)&c->poses[(size_t)k * c->boneCount + b];
        memcpy(values + (size_t)k * width, pose + offset, width * sizeof(float));
    }
    *out = (maniChannelSource){c->times, values, c->sampleCount, maniInterpolationLinear};
    return maniFbxSuccess();
}

static maniImportResult BoneTracks(Clip* c)
{
    c->tracks = maniArenaTake(&c->arena, c->boneCount, sizeof(maniBoneTrackSource));
    if (c->tracks == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniFbxSuccess();
    for (uint32_t b = 0; r.result == mani_success && b < c->boneCount; ++b)
    {
        uint32_t bits = c->rule[b] != kNone ? Moving(c, b) : 0;
        if (bits == 0)
        {
            continue;
        }
        maniBoneTrackSource* t = &c->tracks[c->trackCount++];
        t->bone = b;
        if ((bits & 1u) != 0)
        {
            r = Channel(c, b, offsetof(maniTransform, translation), 3, &t->translation);
        }
        if (r.result == mani_success && (bits & 2u) != 0)
        {
            r = Channel(c, b, offsetof(maniTransform, rotation), 4, &t->rotation);
        }
        if (r.result == mani_success && (bits & 4u) != 0)
        {
            r = Channel(c, b, offsetof(maniTransform, scale), 3, &t->scale);
        }
    }
    return r;
}

// Whether a property is one of the local transform's inputs.
static bool InChain(const maniFbx* fbx, uint32_t name)
{
    for (uint32_t k = 0; k < maniFbxLocalVectors; ++k)
    {
        const char* n = maniFbxLocalVectorNames[k];
        if (maniFbxStringEquals(fbx, name, n, strlen(n)))
        {
            return true;
        }
    }
    for (uint32_t k = 0; k < maniFbxLocalIntegers; ++k)
    {
        const char* n = maniFbxLocalIntegerNames[k];
        if (maniFbxStringEquals(fbx, name, n, strlen(n)))
        {
            return true;
        }
    }
    return false;
}

// A property's static value and its component count: the entry's
// numbers, or 0 and the channel's keyed components when the model has
// no entry.
static uint32_t Static(const maniFbx* fbx, uint32_t object, const char* name, size_t length,
                       const maniFbxChannel* channel, double value[3])
{
    value[0] = value[1] = value[2] = 0.0;
    uint32_t entry = maniFbxFindProperty(fbx, object, name, length);
    if (entry == MANI_FBX_NONE)
    {
        return channel->curves[1].count > 0 || channel->curves[2].count > 0 ? 3 : 1;
    }
    uint32_t n = 0;
    while (n < 3 && maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, n), &value[n]))
    {
        ++n;
    }
    return n > 0 ? n : 1;
}

// The float track's name: the bone's, '/', the property's, and '/'
// with the component for vectors.
static char* TrackName(Clip* c, uint32_t b, const char* property, size_t length, uint32_t width,
                       uint32_t k, uint32_t* lengthOut)
{
    uint32_t boneLength = 0;
    const char* bone = maniSkeletonBoneName(c->skeleton, b, &boneLength);
    size_t n = (size_t)boneLength + 1 + length + (width > 1 ? 2 : 0);
    char* name = maniArenaTake(&c->arena, n, 1);
    if (name != NULL)
    {
        memcpy(name, bone, boneLength);
        name[boneLength] = '/';
        memcpy(name + boneLength + 1, property, length);
        if (width > 1)
        {
            name[n - 2] = '/';
            name[n - 1] = (char)('0' + k);
        }
    }
    *lengthOut = (uint32_t)n;
    return name;
}

// The float tracks of one property of a bone's model.
static maniImportResult PropertyTracks(Clip* c, uint32_t b, const maniFbxChannel* channel)
{
    const maniFbx* fbx = c->fbx;
    uint32_t object = fbx->models[c->modelOf[b]];
    const uint8_t* raw = NULL;
    uint32_t rawLength = 0;
    (void)maniFbxString(fbx, channel->name, &raw, &rawLength);
    char* property = maniArenaTake(&c->arena, (size_t)rawLength + 1, 1);
    if (property == NULL)
    {
        return Capacity();
    }
    size_t length = maniFbxStringDecode(fbx, channel->name, (uint8_t*)property, rawLength);
    double rest[3];
    uint32_t width = Static(fbx, object, property, length, channel, rest);
    float* values = maniArenaTake(&c->arena, (size_t)c->sampleCount * width, sizeof(float));
    if (values == NULL)
    {
        return Capacity();
    }
    for (uint32_t s = 0; s < c->sampleCount; ++s)
    {
        double v[3] = {rest[0], rest[1], rest[2]};
        (void)maniFbxAnimateProperty(&c->animation, fbx, object, property, length, StackTime(c, s),
                                     0, v);
        for (uint32_t k = 0; k < width; ++k)
        {
            values[(size_t)k * c->sampleCount + s] = (float)v[k];
        }
    }
    for (uint32_t k = 0; k < width; ++k)
    {
        maniFloatTrackSource* f = &c->floats[c->floatCount];
        f->name = TrackName(c, b, property, length, width, k, &f->nameLength);
        if (f->name == NULL)
        {
            return Capacity();
        }
        f->keys = (maniChannelSource){c->times, values + (size_t)k * c->sampleCount, c->sampleCount,
                                      maniInterpolationLinear};
        c->floatModel[c->floatCount++] = c->modelOf[b];
    }
    return maniFbxSuccess();
}

// Whether an earlier channel of the run names the same property.
static bool Seen(const Clip* c, uint32_t first, uint32_t i)
{
    const maniFbxChannel* channels = c->animation.channels;
    for (uint32_t j = first; j < i; ++j)
    {
        if (channels[j].nameHash == channels[i].nameHash &&
            maniFbxSameString(c->fbx, channels[j].name, channels[i].name))
        {
            return true;
        }
    }
    return false;
}

static maniImportResult FloatTracks(Clip* c)
{
    // At most three tracks a channel.
    size_t most = 0;
    if (ckd_mul(&most, (size_t)c->animation.channelCount, (size_t)3))
    {
        return Capacity();
    }
    c->floats = maniArenaTake(&c->arena, most, sizeof(maniFloatTrackSource));
    c->floatModel = maniArenaTake(&c->arena, most, sizeof(uint32_t));
    if (c->floats == NULL || c->floatModel == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniFbxSuccess();
    for (uint32_t b = 0; r.result == mani_success && b < c->boneCount; ++b)
    {
        if (c->rule[b] == kNone || c->rule[b] == kHelper)
        {
            continue;
        }
        uint32_t first = 0;
        uint32_t n = maniFbxAnimatedRun(&c->animation, c->fbx->models[c->modelOf[b]], &first);
        for (uint32_t i = first; r.result == mani_success && i < first + n; ++i)
        {
            const maniFbxChannel* channel = &c->animation.channels[i];
            if (!InChain(c->fbx, channel->name) && !Seen(c, first, i))
            {
                r = PropertyTracks(c, b, channel);
            }
        }
    }
    return r;
}

// The channels bound to the models the clip reads.
static uint32_t Bound(const Clip* c)
{
    uint32_t n = 0;
    for (uint32_t m = 0; m < c->fbx->modelCount; ++m)
    {
        uint32_t first = 0;
        n += c->needed[m] != 0 ? maniFbxAnimatedRun(&c->animation, c->fbx->models[m], &first) : 0;
    }
    return n;
}

// The clip builder's refusal as the import's: a repeated float track
// name as Name on its model, the rest as Build.
static maniImportResult FromBuild(const Clip* c, maniBuildResult r)
{
    if (r.result == mani_success || r.result == mani_errorCapacity)
    {
        return (maniImportResult){r.result, maniImportReasonNone, 0, 0, 0};
    }
    bool floats = (r.reason & 0xF00u) == maniBuildReasonFloatTrack;
    if (floats && (r.reason & 0xFFu) == maniBuildReasonTrack && r.item < c->floatCount)
    {
        uint32_t m = c->floatModel[r.item];
        return maniFbxFail(maniImportReasonName, ModelOffset(c->fbx, m), m);
    }
    return maniFbxFail(maniImportReasonBuild, 0, r.reason);
}

static maniImportResult Prepare(Clip* c, uint32_t clip)
{
    const maniFbx* fbx = c->fbx;
    c->boneCount = maniSkeletonBoneCount(c->skeleton);
    c->modelOf = maniArenaTake(&c->arena, c->boneCount, sizeof(uint32_t));
    c->rule = maniArenaTake(&c->arena, c->boneCount, 1);
    c->needed = maniArenaTake(&c->arena, (size_t)fbx->modelCount + 1, 1);
    c->worlds = maniArenaTake(&c->arena, (size_t)fbx->modelCount + 1, sizeof(maniFbxWorld));
    if (c->modelOf == NULL || c->rule == NULL || c->needed == NULL || c->worlds == NULL)
    {
        return Capacity();
    }
    for (uint32_t b = 0; b < c->boneCount; ++b)
    {
        c->modelOf[b] = MANI_FBX_NONE;
    }
    maniImportResult r = Bind(c);
    if (r.result != mani_success)
    {
        return r;
    }
    Rules(c);
    Need(c);
    r = maniFbxReadAnimation(fbx, maniFbxStackObject(fbx, clip), &c->animation);
    // A refusal names the stack's object; the clip is the item a caller
    // knows.
    r.item = r.result == mani_errorFormat ? clip : r.item;
    return r.result == mani_success
               ? maniFbxStartSampler(fbx, &c->animation, c->needed, &c->sampler)
               : r;
}

static maniImportResult Run(maniBuilder* builder, Clip* c, uint32_t clip, float sampleRate,
                            maniLoopPolicy loop, uint32_t* boundOut)
{
    maniImportResult r = Prepare(c, clip);
    r = r.result == mani_success ? Timing(c, sampleRate) : r;
    r = r.result == mani_success ? Sample(c) : r;
    r = r.result == mani_success ? BoneTracks(c) : r;
    r = r.result == mani_success ? FloatTracks(c) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (boundOut != NULL)
    {
        *boundOut = Bound(c);
    }
    maniClipSource source = {.skeleton = c->skeleton,
                             .boneTracks = c->tracks,
                             .floatTracks = c->floats,
                             .boneTrackCount = c->trackCount,
                             .floatTrackCount = c->floatCount,
                             .loop = loop,
                             .sampleRate = c->rate,
                             .duration = c->duration};
    return FromBuild(c, maniBuildClip(builder, &source));
}

maniImportResult maniFbxBuildClip(maniBuilder* builder, const maniFbx* fbx, uint32_t clip,
                                  const maniSkeleton* skeleton, const maniFbxOptions* options,
                                  float sampleRate, maniLoopPolicy loop, uint32_t* boundOut)
{
    bool usable = maniFbxUsable(builder, fbx, options,
                                fbx != NULL && skeleton != NULL && clip < fbx->stackCount &&
                                    isfinite(sampleRate) && sampleRate >= 0.0f);
    // fbx is not NULL when usable; said again for the analyzer.
    if (!usable || fbx == NULL)
    {
        return Invalid();
    }
    if (boundOut != NULL)
    {
        *boundOut = 0;
    }
    Clip c = {.fbx = fbx,
              .skeleton = skeleton,
              .options = options,
              .basis = maniFbxBasis(fbx, options),
              .arena = maniArenaMake(&fbx->allocator)};
    maniImportResult r = Run(builder, &c, clip, sampleRate, loop, boundOut);
    maniFbxReleaseSampler(&c.sampler);
    if (c.animation.layers != NULL || c.animation.channels != NULL)
    {
        maniFbxReleaseAnimation(fbx, &c.animation);
    }
    maniArenaRelease(&c.arena);
    return r;
}
