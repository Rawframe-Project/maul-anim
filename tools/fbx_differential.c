// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ufbx differential (mani-0018): reads each FBX file with ufbx and
// with maul-anim and compares what maul-anim builds, through its public
// interface only: whether each opens, the frame rate, every skeleton's
// rests, every skin's joints and inverse binds, and every clip (built on
// the first skeleton) at five of its sample times, bone poses and blend
// shape weights, against ufbx's scene evaluated at the same stack time.
// Bones compare by their world matrices (the skeleton's placement times
// the model matrix), as maul-anim bakes inherit modes into its rests.
// Values agree within 1e-4 of their size (at least 1).
//
// A mismatch whose file and aspect a deviations file lists (lines of a
// file name part, or * for any, and an aspect; # comments) is reported
// as such and not counted. With --expect, nothing is compared: ufbx's values for the
// same checks are written as a C header for test_fbx_fixtures.c.
//
// Built only with MAUL_ANIM_DIFFERENTIAL, against ufbx at its pinned
// commit; never part of the library.
//
// usage: fbx_differential [--deviations FILE] FILE...
//        fbx_differential --expect FILE...

#include "ufbx.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    kMaxDeviations = 256,
    kTimes = 5,
};

typedef struct Deviation
{
    char file[128];
    char aspect[32];
} Deviation;

// A built file, kept aligned for opening.
typedef struct Blob
{
    void* raw;
    void* data;
    size_t size;
} Blob;

typedef struct Context
{
    const char* path;
    uint32_t file;
    bool expect;
    const Deviation* deviations;
    uint32_t deviationCount;
    uint32_t mismatches;
    uint32_t deviated;
    ufbx_scene* scene;
    maniBuilder* builder;
    maniFbx* fbx;
    maniSkeleton skeleton;
    Blob skeletonBlob;
    maniAffine placement;
    bool hasSkeleton;
} Context;

static void Release(Blob* blob)
{
    free(blob->raw);
    *blob = (Blob){NULL, NULL, 0};
}

// Copies the builder's file into an aligned blob.
static bool Copy(maniBuilder* builder, Blob* blob)
{
    Release(blob);
    size_t size = 0;
    (void)maniCopyBuilt(builder, NULL, 0, &size);
    blob->raw = malloc(size + 64);
    if (blob->raw == NULL)
    {
        return false;
    }
    blob->data = (void*)(((uintptr_t)blob->raw + 63) & ~(uintptr_t)63);
    blob->size = size;
    return maniCopyBuilt(builder, blob->data, size, &size) == mani_success;
}

static void* ReadFile(const char* path, size_t* size)
{
    FILE* f = fopen(path, "rb");
    if (f == NULL)
    {
        return NULL;
    }
    (void)fseek(f, 0, SEEK_END);
    long n = ftell(f);
    (void)fseek(f, 0, SEEK_SET);
    void* bytes = n >= 0 ? malloc((size_t)n + 1) : NULL;
    if (bytes != NULL && fread(bytes, 1, (size_t)n, f) != (size_t)n)
    {
        free(bytes);
        bytes = NULL;
    }
    (void)fclose(f);
    *size = n >= 0 ? (size_t)n : 0;
    return bytes;
}

static bool Deviates(const Context* c, const char* aspect)
{
    for (uint32_t k = 0; k < c->deviationCount; ++k)
    {
        const char* file = c->deviations[k].file;
        if ((strcmp(file, "*") == 0 || strstr(c->path, file) != NULL) &&
            strcmp(aspect, c->deviations[k].aspect) == 0)
        {
            return true;
        }
    }
    return false;
}

static void Mismatch(Context* c, const char* aspect, const char* format, ...)
{
    bool listed = Deviates(c, aspect);
    printf("%s: %s %s: ", c->path, listed ? "deviation" : "MISMATCH", aspect);
    va_list args;
    va_start(args, format);
    (void)vprintf(format, args);
    va_end(args);
    printf("\n");
    if (listed)
    {
        ++c->deviated;
    }
    else
    {
        ++c->mismatches;
    }
}

static bool Near(double a, double b)
{
    double size = fabs(b) > 1.0 ? fabs(b) : 1.0;
    return fabs(a - b) <= 1e-4 * size;
}

// a times b, both 3x4 column-major affine matrices.
static void Compose(const double a[12], const double b[12], double out[12])
{
    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 3; ++row)
        {
            double v =
                a[row] * b[col * 3] + a[3 + row] * b[col * 3 + 1] + a[6 + row] * b[col * 3 + 2];
            out[col * 3 + row] = col == 3 ? v + a[9 + row] : v;
        }
    }
}

static void FromAffine(const maniAffine* a, double out[12])
{
    for (int i = 0; i < 12; ++i)
    {
        out[i] = (double)a->m[i];
    }
}

// A bone's world: the placement times its model matrix.
static void OurWorld(const maniAffine* placement, const maniPose* pose, uint32_t bone,
                     double out[12])
{
    double p[12];
    double m[12];
    maniAffine model = maniPoseModel(pose, bone);
    FromAffine(placement, p);
    FromAffine(&model, m);
    Compose(p, m, out);
}

static bool SameMatrix(const double a[12], const double b[12])
{
    bool same = true;
    for (int i = 0; same && i < 12; ++i)
    {
        same = Near(a[i], b[i]);
    }
    return same;
}

// The header line of one expected value set.
static void Emit(const Context* c, const char* kind, uint32_t index, double time, const char* name,
                 size_t nameLength, const double* values, uint32_t count)
{
    printf("    {%u, \"%s\", %u, %.9g, \"%.*s\", %u, {", c->file, kind, index, time,
           (int)nameLength, name, count);
    for (uint32_t i = 0; i < count; ++i)
    {
        printf("%s%.17g", i > 0 ? ", " : "", values[i]);
    }
    printf("}},\n");
}

static void CheckWorld(Context* c, const char* kind, uint32_t index, double time, const char* name,
                       size_t length, const double ours[12], const double theirs[12])
{
    if (c->expect)
    {
        Emit(c, kind, index, time, name, length, theirs, 12);
        return;
    }
    if (!SameMatrix(ours, theirs))
    {
        Mismatch(c, kind,
                 "%u t=%g %.*s: ours %g %g %g / %g %g %g / %g %g %g / %g %g %g; ufbx %g %g %g / "
                 "%g %g %g / %g %g %g / %g %g %g",
                 index, time, (int)length, name, ours[0], ours[1], ours[2], ours[3], ours[4],
                 ours[5], ours[6], ours[7], ours[8], ours[9], ours[10], ours[11], theirs[0],
                 theirs[1], theirs[2], theirs[3], theirs[4], theirs[5], theirs[6], theirs[7],
                 theirs[8], theirs[9], theirs[10], theirs[11]);
    }
}

// Every bone's world in a pose against the nodes of the same name in a
// ufbx scene (a helper bone or a renamed model has none).
static void CompareWorlds(Context* c, const char* kind, uint32_t index, double time,
                          const maniSkeleton* skeleton, const maniAffine* placement, maniPose* pose,
                          const ufbx_scene* scene)
{
    if (maniComputeModel(pose, skeleton) != mani_success)
    {
        Mismatch(c, kind, "%u t=%g: no model matrices", index, time);
        return;
    }
    uint32_t n = maniSkeletonBoneCount(skeleton);
    for (uint32_t b = 0; b < n; ++b)
    {
        uint32_t length = 0;
        const char* name = maniSkeletonBoneName(skeleton, b, &length);
        ufbx_node* node = ufbx_find_node_len(scene, name, length);
        if (node != NULL)
        {
            double ours[12];
            OurWorld(placement, pose, b, ours);
            CheckWorld(c, kind, index, time, name, length, ours, node->node_to_world.v);
        }
    }
}

// A pose of a skeleton in aligned memory of its own (free raw after).
static maniPose* NewPose(const maniSkeleton* skeleton, void** raw)
{
    size_t size = maniPoseSize(skeleton);
    *raw = malloc(size + 64);
    maniPose* pose = NULL;
    void* memory = *raw != NULL ? (void*)(((uintptr_t)*raw + 63) & ~(uintptr_t)63) : NULL;
    if (memory == NULL || maniInitPose(skeleton, memory, size, &pose) != mani_success ||
        maniPoseSetRest(pose, skeleton) != mani_success)
    {
        return NULL;
    }
    return pose;
}

// Both open, or both refuse; false when there is nothing more to compare.
static bool CompareOpen(Context* c, const void* bytes, size_t size)
{
    ufbx_load_opts options = {0};
    ufbx_error error;
    c->scene = ufbx_load_memory(bytes, size, &options, &error);
    maniImportResult r = maniOpenFbx(c->builder, bytes, size, &c->fbx);
    bool ours = r.result == mani_success;
    bool theirs = c->scene != NULL;
    if (c->expect)
    {
        double opened = theirs ? 1.0 : 0.0;
        Emit(c, "open", 0, 0.0, "", 0, &opened, 1);
    }
    else if (ours != theirs)
    {
        Mismatch(c, "open", "ours %s (reason %d at %llu), ufbx %s (%s)", ours ? "opens" : "refuses",
                 (int)r.reason, (unsigned long long)r.offset, theirs ? "opens" : "refuses",
                 theirs ? "" : error.description.data);
    }
    return ours && theirs;
}

static void CompareRate(Context* c)
{
    maniFbxSettings s;
    if (maniFbxReadSettings(c->fbx, &s) != mani_success)
    {
        Mismatch(c, "rate", "settings unread");
        return;
    }
    double ours =
        s.rateDenominator != 0 ? (double)s.rateNumerator / s.rateDenominator : s.customRate;
    double theirs = c->scene->settings.frames_per_second;
    if (c->expect)
    {
        Emit(c, "rate", 0, 0.0, "", 0, &theirs, 1);
    }
    else if (!Near(ours, theirs))
    {
        // No TimeMode: ufbx reads 24, maul-anim FBX's default mode, 30.
        bool unset = s.timeMode == 0;
        Mismatch(c, unset ? "rate-default" : "rate", "ours %g, ufbx %g", ours, theirs);
    }
}

static bool BuildSkeleton(Context* c, uint32_t s, const maniFbxOptions* options, maniSkeleton* out,
                          Blob* blob)
{
    maniImportResult r = maniFbxBuildSkeleton(c->builder, c->fbx, s, options);
    if (r.result != mani_success || !Copy(c->builder, blob) ||
        maniOpenSkeleton(blob->data, blob->size, out).result != mani_success)
    {
        Mismatch(c, "skeleton", "%u not built (result %d, reason %d)", s, (int)r.result,
                 (int)r.reason);
        return false;
    }
    return true;
}

static void CompareRests(Context* c, uint32_t s, const maniSkeleton* skeleton)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    maniAffine placement;
    void* raw = NULL;
    maniPose* pose = NewPose(skeleton, &raw);
    if (pose == NULL ||
        maniFbxSkeletonPlacement(c->fbx, s, &options, &placement).result != mani_success)
    {
        Mismatch(c, "rest", "%u: no pose or placement", s);
    }
    else
    {
        CompareWorlds(c, "rest", s, 0.0, skeleton, &placement, pose, c->scene);
    }
    free(raw);
}

// Every skeleton's rests, its placement reported.
static void CompareSkeletons(Context* c)
{
    uint32_t n = maniFbxSkeletonCount(c->fbx);
    maniFbxOptions options = maniDefaultFbxOptions();
    for (uint32_t s = 0; s < n; ++s)
    {
        maniSkeleton skeleton;
        Blob blob = {NULL, NULL, 0};
        if (BuildSkeleton(c, s, &options, &skeleton, &blob))
        {
            CompareRests(c, s, &skeleton);
        }
        Release(&blob);
    }
}

// The first skeleton for the clips, its placement folded: animation above
// its root is then part of the clips (mani-0016), as in ufbx's worlds.
static void KeepClipSkeleton(Context* c)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    c->hasSkeleton =
        maniFbxSkeletonCount(c->fbx) > 0 &&
        BuildSkeleton(c, 0, &options, &c->skeleton, &c->skeletonBlob) &&
        maniFbxSkeletonPlacement(c->fbx, 0, &options, &c->placement).result == mani_success;
}

// The ufbx skin deformer our skin k stands for: the mesh's deformer of
// the same rank among its skins.
static ufbx_skin_deformer* TheirSkin(const Context* c, uint32_t k)
{
    uint32_t model = maniFbxSkinModel(c->fbx, k);
    uint32_t rank = 0;
    for (uint32_t j = 0; j < k; ++j)
    {
        rank += maniFbxSkinModel(c->fbx, j) == model ? 1 : 0;
    }
    uint32_t length = 0;
    const char* name = model != UINT32_MAX ? maniFbxModelBoneName(c->fbx, model, &length) : NULL;
    ufbx_node* node = name != NULL ? ufbx_find_node_len(c->scene, name, length) : NULL;
    if (node == NULL || node->mesh == NULL || rank >= node->mesh->skin_deformers.count)
    {
        return NULL;
    }
    return node->mesh->skin_deformers.data[rank];
}

// Our joint j's cluster: the j-th that links a bone.
static ufbx_skin_cluster* TheirCluster(ufbx_skin_deformer* skin, uint32_t j)
{
    for (size_t i = 0; i < skin->clusters.count; ++i)
    {
        ufbx_skin_cluster* cluster = skin->clusters.data[i];
        if (cluster->bone_node != NULL && j-- == 0)
        {
            return cluster;
        }
    }
    return NULL;
}

static void CompareBinds(Context* c, uint32_t k, const maniSkin* skin, const maniSkeleton* skeleton)
{
    ufbx_skin_deformer* theirs = TheirSkin(c, k);
    uint32_t n = maniSkinJointCount(skin);
    for (uint32_t j = 0; theirs != NULL && j < n; ++j)
    {
        uint32_t length = 0;
        const char* name = maniSkeletonBoneName(skeleton, maniSkinJoint(skin, j), &length);
        ufbx_skin_cluster* cluster = TheirCluster(theirs, j);
        maniAffine bind = maniSkinInverseBind(skin, j);
        double ours[12];
        double expected[12];
        bool same = cluster != NULL && length == cluster->bone_node->name.length &&
                    memcmp(name, cluster->bone_node->name.data, length) == 0;
        for (int i = 0; i < 12; ++i)
        {
            ours[i] = (double)bind.m[i];
            expected[i] = cluster != NULL ? cluster->geometry_to_bone.v[i] : 0.0;
            same = same && Near(ours[i], expected[i]);
        }
        if (c->expect)
        {
            Emit(c, "bind", k, (double)j, name, length, expected, 12);
        }
        else if (!same)
        {
            Mismatch(c, "bind", "skin %u joint %u %.*s", k, j, (int)length, name);
        }
    }
    if (theirs == NULL)
    {
        Mismatch(c, "bind", "skin %u has no ufbx deformer", k);
    }
}

static void CompareSkins(Context* c)
{
    uint32_t n = maniFbxSkinCount(c->fbx);
    for (uint32_t k = 0; k < n; ++k)
    {
        uint32_t s = maniFbxSkinSkeleton(c->fbx, k);
        maniSkeleton skeleton;
        Blob skeletonBlob = {NULL, NULL, 0};
        Blob skinBlob = {NULL, NULL, 0};
        maniFbxOptions options = maniDefaultFbxOptions();
        maniSkin skin;
        bool built =
            s != UINT32_MAX && BuildSkeleton(c, s, &options, &skeleton, &skeletonBlob) &&
            maniFbxBuildSkin(c->builder, c->fbx, k, &skeleton, &options).result == mani_success &&
            Copy(c->builder, &skinBlob) &&
            maniOpenSkin(skinBlob.data, skinBlob.size, &skin).result == mani_success;
        if (built)
        {
            CompareBinds(c, k, &skin, &skeleton);
        }
        else if (s != UINT32_MAX)
        {
            Mismatch(c, "skin", "%u not built", k);
        }
        Release(&skinBlob);
        Release(&skeletonBlob);
    }
}

// A stack's start: its time range's, else its first key's (as maul-anim
// takes the key span where a stack gives no times).
static double StackStart(const ufbx_anim_stack* stack, double* stop)
{
    *stop = stack->time_end;
    if (stack->time_end > stack->time_begin)
    {
        return stack->time_begin;
    }
    double first = (double)INFINITY;
    double last = -(double)INFINITY;
    for (size_t l = 0; l < stack->layers.count; ++l)
    {
        const ufbx_anim_layer* layer = stack->layers.data[l];
        for (size_t v = 0; v < layer->anim_values.count; ++v)
        {
            for (int k = 0; k < 3; ++k)
            {
                const ufbx_anim_curve* curve = layer->anim_values.data[v]->curves[k];
                if (curve != NULL && curve->keyframes.count > 0)
                {
                    first = fmin(first, curve->keyframes.data[0].time);
                    last = fmax(last, curve->keyframes.data[curve->keyframes.count - 1].time);
                }
            }
        }
    }
    *stop = first <= last ? last : 0.0;
    return first <= last ? first : 0.0;
}

// The value of a float track in an evaluated scene: a shape's effective
// weight ("mesh/shape") or a node's number property ("node/property", a
// vector's component k "node/property/k").
static bool TheirFloat(const ufbx_scene* scene, const char* track, size_t length, double* out)
{
    for (size_t n = 0; n < scene->nodes.count; ++n)
    {
        const ufbx_node* node = scene->nodes.data[n];
        size_t prefix = node->name.length;
        if (prefix + 1 >= length || memcmp(track, node->name.data, prefix) != 0 ||
            track[prefix] != '/')
        {
            continue;
        }
        const char* rest = track + prefix + 1;
        size_t restLength = length - prefix - 1;
        for (size_t d = 0; node->mesh != NULL && d < node->mesh->blend_deformers.count; ++d)
        {
            const ufbx_blend_deformer* deformer = node->mesh->blend_deformers.data[d];
            for (size_t ch = 0; ch < deformer->channels.count; ++ch)
            {
                const ufbx_blend_channel* channel = deformer->channels.data[ch];
                for (size_t k = 0; k < channel->keyframes.count; ++k)
                {
                    const ufbx_blend_shape* shape = channel->keyframes.data[k].shape;
                    if (shape->name.length == restLength &&
                        memcmp(shape->name.data, rest, restLength) == 0)
                    {
                        *out = channel->keyframes.data[k].effective_weight;
                        return true;
                    }
                }
            }
        }
        // A vector property's component k is "property/k".
        size_t nameLength = restLength;
        size_t component = 0;
        if (restLength > 2 && rest[restLength - 2] == '/' && rest[restLength - 1] >= '0' &&
            rest[restLength - 1] <= '3')
        {
            nameLength = restLength - 2;
            component = (size_t)(rest[restLength - 1] - '0');
        }
        const ufbx_prop* prop = ufbx_find_prop_len(&node->props, rest, nameLength);
        if (prop != NULL)
        {
            *out = prop->value_real_arr[component];
            return true;
        }
    }
    return false;
}

// One sample time of clip k: every bone's pose and every float track.
static void CompareTime(Context* c, uint32_t k, const maniClip* clip, maniPose* pose,
                        const ufbx_scene* evaluated, double time)
{
    const maniSkeleton* skeleton = &c->skeleton;
    if (maniSampleClip(clip, skeleton, (float)time, pose) != mani_success)
    {
        Mismatch(c, "sample", "clip %u not sampled at %g", k, time);
        return;
    }
    CompareWorlds(c, "sample", k, time, skeleton, &c->placement, pose, evaluated);
    uint32_t tracks = maniClipFloatTrackCount(clip);
    float values[256];
    bool sampled = tracks <= 256 &&
                   maniSampleClipFloats(clip, (float)time, values, tracks, NULL) == mani_success;
    for (uint32_t t = 0; sampled && t < tracks; ++t)
    {
        uint32_t length = 0;
        const char* name = maniClipFloatTrackName(clip, t, &length);
        double theirs = 0.0;
        bool found = TheirFloat(evaluated, name, length, &theirs);
        if (c->expect && found)
        {
            Emit(c, "float", k, time, name, length, &theirs, 1);
        }
        else if (!c->expect && (!found || !Near((double)values[t], theirs)))
        {
            Mismatch(c, "float", "clip %u t=%g %.*s: ours %g, ufbx %s %g", k, time, (int)length,
                     name, (double)values[t], found ? "" : "none", theirs);
        }
    }
}

// Clip k's five times, ufbx evaluated at the stack's start plus each.
static void CompareClipTimes(Context* c, uint32_t k, const maniClip* clip,
                             const ufbx_anim_stack* stack, double start)
{
    void* raw = NULL;
    maniPose* pose = NewPose(&c->skeleton, &raw);
    if (pose == NULL)
    {
        free(raw);
        return;
    }
    uint32_t n = maniClipSampleCount(clip);
    double rate = (double)maniClipSampleRate(clip);
    double duration = (double)maniClipDuration(clip);
    for (uint32_t i = 0; n > 0 && i < kTimes; ++i)
    {
        uint32_t frame = (uint32_t)((uint64_t)(n - 1) * i / (kTimes - 1));
        double time = fmin(frame / rate, duration);
        ufbx_error error;
        ufbx_scene* evaluated =
            ufbx_evaluate_scene(c->scene, stack->anim, start + time, NULL, &error);
        if (evaluated != NULL)
        {
            CompareTime(c, k, clip, pose, evaluated, time);
            ufbx_free_scene(evaluated);
        }
    }
    free(raw);
}

static void CompareClip(Context* c, uint32_t k)
{
    uint32_t length = 0;
    const char* name = maniFbxClipName(c->fbx, k, &length);
    ufbx_anim_stack* stack = name != NULL ? ufbx_find_anim_stack_len(c->scene, name, length) : NULL;
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    Blob blob = {NULL, NULL, 0};
    maniClip clip;
    maniImportResult r = {mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    if (stack != NULL)
    {
        r = maniFbxBuildClip(c->builder, c->fbx, k, &c->skeleton, &options, 0.0f, maniLoopClamp,
                             NULL);
    }
    bool built = r.result == mani_success && Copy(c->builder, &blob) &&
                 maniOpenClip(blob.data, blob.size, &clip).result == mani_success;
    if (!built)
    {
        Mismatch(c, "clip", "%u (%.*s) not built (result %d, reason %d at %llu, item %u)%s", k,
                 (int)length, name != NULL ? name : "", (int)r.result, (int)r.reason,
                 (unsigned long long)r.offset, r.item, stack == NULL ? ", no ufbx stack" : "");
        Release(&blob);
        return;
    }
    double stop = 0.0;
    double start = StackStart(stack, &stop);
    double theirs = stop - start;
    if (c->expect)
    {
        Emit(c, "duration", k, 0.0, name, length, &theirs, 1);
    }
    else if (!Near((double)maniClipDuration(&clip), theirs))
    {
        Mismatch(c, "duration", "clip %u: ours %g, ufbx %g", k, (double)maniClipDuration(&clip),
                 theirs);
    }
    CompareClipTimes(c, k, &clip, stack, start);
    Release(&blob);
}

static void CompareFile(Context* c)
{
    size_t size = 0;
    void* bytes = ReadFile(c->path, &size);
    maniBuilderDef def = maniDefaultBuilderDef();
    if (bytes == NULL || maniCreateBuilder(&def, &c->builder) != mani_success)
    {
        Mismatch(c, "read", "the file or a builder");
        free(bytes);
        return;
    }
    if (CompareOpen(c, bytes, size))
    {
        CompareRate(c);
        CompareSkeletons(c);
        CompareSkins(c);
        KeepClipSkeleton(c);
        uint32_t n = c->hasSkeleton ? maniFbxClipCount(c->fbx) : 0;
        for (uint32_t k = 0; k < n; ++k)
        {
            CompareClip(c, k);
        }
    }
    Release(&c->skeletonBlob);
    maniCloseFbx(c->fbx);
    ufbx_free_scene(c->scene);
    maniDestroyBuilder(c->builder);
    free(bytes);
}

static uint32_t ReadDeviations(const char* path, Deviation* out)
{
    FILE* f = fopen(path, "r");
    char line[256];
    uint32_t n = 0;
    while (f != NULL && n < kMaxDeviations && fgets(line, sizeof line, f) != NULL)
    {
        char* hash = strchr(line, '#');
        if (hash != NULL)
        {
            *hash = '\0';
        }
        if (sscanf(line, "%127s %31s", out[n].file, out[n].aspect) == 2)
        {
            ++n;
        }
    }
    if (f != NULL)
    {
        (void)fclose(f);
    }
    return n;
}

int main(int argc, char** argv)
{
    static Deviation deviations[kMaxDeviations];
    Context base = {0};
    int first = 1;
    if (argc > 1 && strcmp(argv[1], "--expect") == 0)
    {
        base.expect = true;
        first = 2;
    }
    else if (argc > 2 && strcmp(argv[1], "--deviations") == 0)
    {
        base.deviations = deviations;
        base.deviationCount = ReadDeviations(argv[2], deviations);
        first = 3;
    }
    uint32_t mismatches = 0;
    uint32_t deviated = 0;
    for (int i = first; i < argc; ++i)
    {
        Context c = base;
        c.path = argv[i];
        c.file = (uint32_t)(i - first);
        CompareFile(&c);
        mismatches += c.mismatches;
        deviated += c.deviated;
    }
    if (!base.expect)
    {
        printf("%d files, %u mismatches, %u deviations\n", argc - first, mismatches, deviated);
    }
    return mismatches == 0 ? 0 : 1;
}
