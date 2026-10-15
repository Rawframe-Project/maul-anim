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
#include "fbx_clip_state.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_options.h"
#include "fbx_sample.h"
#include "fbx_take.h"
#include "fbx_tracks.h"
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

typedef maniFbxClipState Clip;

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

static maniImportResult Timing(Clip* c, float sampleRate)
{
    double scene = SceneRate(c->fbx);
    c->exactRate = sampleRate != 0.0f              ? (double)sampleRate
                   : scene > 0.0 && scene <= 120.0 ? scene
                                                   : 30.0;
    c->rate = (float)c->exactRate;
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
        maniImportResult r =
            maniFbxSampleWorlds(&c->sampler, maniFbxClipStackTime(c, k), c->worlds);
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
// rotation and scale (a rotation of the other sign is a track, harmless).
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
        bits |= t[0] != rest[0] || t[1] != rest[1] || t[2] != rest[2] ? 1u : 0u;
        bits |= t[3] != rest[3] || t[4] != rest[4] || t[5] != rest[5] || t[6] != rest[6] ? 2u : 0u;
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

static maniImportResult FloatTracks(Clip* c)
{
    // At most three tracks a channel, and one a blend shape.
    size_t most = 0;
    size_t shapes = 0;
    (void)maniFbxBlendTracks(c, true, &shapes);
    if (ckd_mul(&most, (size_t)c->animation.channelCount, (size_t)3) ||
        ckd_add(&most, most, shapes))
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
        r = maniFbxBoneFloatTracks(c, b);
    }
    size_t unused = 0;
    return r.result == mani_success ? maniFbxBlendTracks(c, false, &unused) : r;
}

// The channels bound to the models the clip reads, and to the blend
// shape channels it keeps.
static uint32_t Bound(const Clip* c)
{
    uint32_t n = c->blendBound;
    for (uint32_t m = 0; m < c->fbx->modelCount; ++m)
    {
        uint32_t first = 0;
        uint32_t count = maniFbxAnimatedRun(&c->animation, c->fbx->models[m], &first);
        for (uint32_t i = first; c->needed[m] != 0 && i < first + count; ++i)
        {
            n += c->animation.channels[i].attribute ? 0 : 1;
        }
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
    r = maniFbxReadStack(fbx, clip, &c->animation);
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
