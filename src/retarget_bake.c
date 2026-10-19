// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking a clip through a retargeting map (mani-0032): each sample of the
// source clip carried by the pose transfer into linear keys of the target
// at the same times, the named items copied, the root motion's moves
// scaled and turned into the target bone's parent's frame, then built as
// any clip with its moves given (the bone's channels already in place).

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "clip_build.h"
#include "clip_file.h"
#include "clip_view.h"
#include "retarget_file.h"
#include "root_motion_build.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdckdint.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The floats a target bone's keys take per sample: translation 3,
// rotation 4, scale 3, each channel's samples together.
#define BONE_FLOATS 10u
#define BAKE_BLOCKS 12u
#define BAKE_POSES  5u

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static maniQuat LoadQuat(const uint8_t* p)
{
    return (maniQuat){LoadF32(p), LoadF32(p + 4), LoadF32(p + 8), LoadF32(p + 12)};
}

typedef struct Block
{
    void* p;
    size_t count;
    size_t size;
} Block;

typedef struct Bake
{
    const maniRetargetClipSource* s;
    const maniAllocator* allocator;
    uint32_t samples;
    uint32_t bones;
    uint32_t floats;
    size_t keys;
    // The target bone taking the root motion, MANI_NO_BONE without it.
    uint32_t root;
    // The root motion bone's parent's reference model rotation on each
    // side, and the turn from the source's frame to the target's.
    maniQuat sourceFrame;
    maniQuat targetFrame;
    maniQuat turn;
    float* times;
    float* values;
    float* floatValues;
    float* sampleFloats;
    float* discreteTimes;
    int32_t* discreteValues;
    maniBoneTrackSource* boneTracks;
    maniFloatTrackSource* floatTracks;
    maniDiscreteTrackSource* discreteTracks;
    maniEventSource* events;
    double* moves;
    // The sampled source pose, the source pose made whole, the target's
    // pose, its basis and its rest (the last two for additive clips).
    maniPose* poses[BAKE_POSES];
    void* scratch;
    size_t scratchSize;
    Block blocks[BAKE_BLOCKS];
    uint32_t blockCount;
} Bake;

// count items of size bytes (one more, so none is empty), kept for the
// release; NULL when the allocator fails.
static void* Take(Bake* b, size_t count, size_t size)
{
    void* p = b->blockCount < BAKE_BLOCKS && count < SIZE_MAX
                  ? maniAllocate(b->allocator, count + 1, size, 8)
                  : NULL;
    if (p != NULL)
    {
        b->blocks[b->blockCount++] = (Block){p, count + 1, size};
    }
    return p;
}

static void ReleaseAll(Bake* b)
{
    for (uint32_t i = 0; i < b->blockCount; ++i)
    {
        maniRelease(b->allocator, b->blocks[i].p, b->blocks[i].count, b->blocks[i].size, 8);
    }
    b->blockCount = 0;
}

static size_t Aligned(size_t n)
{
    return (n + MANI_POSE_ALIGNMENT - 1) / MANI_POSE_ALIGNMENT * MANI_POSE_ALIGNMENT;
}

// The poses and the transfer's scratch, in one block.
static bool TakePoses(Bake* b)
{
    const maniRetargetClipSource* s = b->s;
    size_t sourceSize = Aligned(maniPoseSize(s->source));
    size_t targetSize = Aligned(maniPoseSize(s->target));
    b->scratchSize = maniRetargetScratchSize(s->map);
    size_t total = 2 * sourceSize + 3 * targetSize + b->scratchSize + MANI_POSE_ALIGNMENT;
    uint8_t* at = Take(b, total, 1);
    if (at == NULL)
    {
        return false;
    }
    at += (MANI_POSE_ALIGNMENT - (uintptr_t)at % MANI_POSE_ALIGNMENT) % MANI_POSE_ALIGNMENT;
    bool made = true;
    for (uint32_t i = 0; i < BAKE_POSES; ++i)
    {
        const maniSkeleton* skeleton = i < 2 ? s->source : s->target;
        size_t size = i < 2 ? sourceSize : targetSize;
        made = made && maniInitPose(skeleton, at, size, &b->poses[i]) == mani_success;
        at += size;
    }
    b->scratch = at;
    return made;
}

// The lists' memory; false when the allocator fails or a size overflows.
static bool TakeLists(Bake* b)
{
    const maniClip* clip = b->s->clip;
    size_t n = b->samples;
    size_t boneFloats = 0;
    size_t floatValues = 0;
    if (ckd_mul(&boneFloats, n * BONE_FLOATS, (size_t)b->bones) ||
        ckd_mul(&floatValues, n, (size_t)b->floats))
    {
        return false;
    }
    b->times = Take(b, n, sizeof(float));
    b->values = Take(b, boneFloats, sizeof(float));
    b->floatValues = Take(b, floatValues, sizeof(float));
    b->sampleFloats = Take(b, b->floats, sizeof(float));
    b->discreteTimes = Take(b, b->keys, sizeof(float));
    b->discreteValues = Take(b, b->keys, sizeof(int32_t));
    b->boneTracks = Take(b, b->bones, sizeof(maniBoneTrackSource));
    b->floatTracks = Take(b, b->floats, sizeof(maniFloatTrackSource));
    b->discreteTracks = Take(b, clip->discreteTrackCount, sizeof(maniDiscreteTrackSource));
    b->events = Take(b, (size_t)clip->eventCount + clip->markerCount, sizeof(maniEventSource));
    b->moves = b->root != MANI_NO_BONE ? Take(b, n * 7, sizeof(double)) : NULL;
    return b->times != NULL && b->values != NULL && b->floatValues != NULL &&
           b->sampleFloats != NULL && b->discreteTimes != NULL && b->discreteValues != NULL &&
           b->boneTracks != NULL && b->floatTracks != NULL && b->discreteTracks != NULL &&
           b->events != NULL && (b->root == MANI_NO_BONE || b->moves != NULL) && TakePoses(b);
}

// The first target bone that follows source bone a alone.
static uint32_t Follower(const maniRetargetMap* map, uint32_t a)
{
    for (uint32_t t = 0; t < map->targetBoneCount; ++t)
    {
        const uint8_t* e = map->plan + (size_t)t * MANI_RETARGET_PLAN_SIZE;
        if (maniLoadU32(e + maniRetargetPlanKind) == maniRetargetPlanOne &&
            maniLoadU32(e + maniRetargetPlanA) == a)
        {
            return t;
        }
    }
    return MANI_NO_BONE;
}

// The root motion's target bone and frames; a refusal for a bone past
// the target's or none following the source's.
static maniBuildResult PlaceRoot(Bake* b)
{
    const maniRetargetClipSource* s = b->s;
    maniRootMotionSource from;
    b->root = MANI_NO_BONE;
    if (!maniClipRootMotionSource(s->clip, &from))
    {
        return Result(mani_success, maniBuildReasonNone, 0, 0);
    }
    b->root = s->rootMotionBone != MANI_NO_BONE ? s->rootMotionBone : Follower(s->map, from.bone);
    if (b->root >= s->map->targetBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRootMotion, s->rootMotionBone, 0);
    }
    uint32_t parentS = maniSkeletonParent(s->source, from.bone);
    uint32_t parentT = maniSkeletonParent(s->target, b->root);
    b->sourceFrame =
        parentS == MANI_NO_BONE
            ? maniQuatIdentity()
            : LoadQuat(s->map->sourceBones + (size_t)parentS * MANI_RETARGET_SOURCE_SIZE);
    b->targetFrame =
        parentT == MANI_NO_BONE
            ? maniQuatIdentity()
            : LoadQuat(s->map->targetBones + (size_t)parentT * MANI_RETARGET_TARGET_SIZE);
    b->turn = maniQuatMul(maniQuatConjugate(b->targetFrame), b->sourceFrame);
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

// A move's translation: to the model through the source's frame, scaled,
// back through the target's.
static maniVec3 MoveTranslation(const Bake* b, maniVec3 t)
{
    maniVec3 model = maniRetargetRootMotion(b->s->map, maniQuatRotate(b->sourceFrame, t));
    return maniQuatRotate(maniQuatConjugate(b->targetFrame), model);
}

// Sample k's move, carried.
static void Move(const Bake* b, uint32_t k)
{
    const uint8_t* p = b->s->clip->rootMotion + MANI_ROOT_MOTION_HEADER_SIZE +
                       MANI_ROOT_MOTION_PERIOD_SIZE + 4 * (size_t)MANI_ROOT_MOTION_MOVE_FLOATS * k;
    maniVec3 t = MoveTranslation(b, maniVec3Make(LoadF32(p), LoadF32(p + 4), LoadF32(p + 8)));
    maniQuat r = maniQuatNormalize(maniQuatMul(
        maniQuatMul(b->turn, maniQuatNormalize(LoadQuat(p + 12))), maniQuatConjugate(b->turn)));
    double* m = b->moves + 7 * (size_t)k;
    const float v[7] = {t.x, t.y, t.z, r.x, r.y, r.z, r.w};
    for (uint32_t i = 0; i < 7; ++i)
    {
        m[i] = (double)v[i];
    }
}

static maniVec3 Unit(uint32_t axis)
{
    return maniVec3Make(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
}

// The axis (0 to 2) a vector of length above 0 lies along, within a
// ten-thousandth; 3 for none.
static uint32_t AxisOf(maniVec3 v)
{
    const float a[3] = {fabsf(v.x), fabsf(v.y), fabsf(v.z)};
    uint32_t j = a[1] > a[0] ? 1u : 0u;
    j = a[2] > a[j] ? 2u : j;
    float off = a[0] + a[1] + a[2] - a[j];
    return off <= 1e-4f * a[j] ? j : 3u;
}

// The target's root motion source: the source's on the target bone, its
// axes carried (a move's image along each, or the axis turned where the
// scales take it to nothing), all axes or the whole rotation where one
// leaves the axes.
static maniRootMotionSource Declared(const Bake* b, maniRootMotionSource s)
{
    uint32_t axes = 0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (((s.translationAxes >> i) & 1u) == 0)
        {
            continue;
        }
        maniVec3 v = MoveTranslation(b, Unit(i));
        v = maniVec3Length(v) < 1e-6f ? maniQuatRotate(b->turn, Unit(i)) : v;
        uint32_t j = AxisOf(v);
        axes |= j < 3 ? 1u << j : 7u;
    }
    s.translationAxes = axes;
    if (s.rotation == maniRootRotationTwist)
    {
        uint32_t j = AxisOf(maniQuatRotate(b->turn, Unit(s.twistAxis)));
        s.rotation = j < 3 ? maniRootRotationTwist : maniRootRotationWhole;
        s.twistAxis = j < 3 ? j : s.twistAxis;
    }
    s.bone = b->root;
    return s;
}

// The target pose of sample k of an additive clip, as a pose: the
// source's difference added to its rest, carried, taken from the basis
// (the source's rest carried) and added to the target's rest.
static maniResult Additive(Bake* b)
{
    const maniRetargetClipSource* s = b->s;
    maniBlendLayer layer = {b->poses[0], 1.0f, NULL};
    maniResult r = maniPoseSetRest(b->poses[1], s->source);
    r = r == mani_success ? maniAddPoses(s->source, b->poses[1], &layer, 1) : r;
    r = r == mani_success ? maniRetargetPose(s->map, s->source, b->poses[1], s->target, b->poses[2],
                                             b->scratch, b->scratchSize)
                          : r;
    r = r == mani_success ? maniPoseDifference(s->target, b->poses[2], b->poses[3], b->poses[2])
                          : r;
    layer.pose = b->poses[2];
    r = r == mani_success ? maniPoseSetRest(b->poses[4], s->target) : r;
    return r == mani_success ? maniAddPoses(s->target, b->poses[4], &layer, 1) : r;
}

// Sample k: the target's locals into the keys, the float tracks' values,
// the move.
static maniResult Sample(Bake* b, uint32_t k)
{
    const maniRetargetClipSource* s = b->s;
    size_t n = b->samples;
    maniSampleClipAt(s->clip, s->source, k, b->poses[0], b->sampleFloats);
    bool additive = s->clip->additive != maniAdditiveNone;
    maniResult r = additive ? Additive(b)
                            : maniRetargetPose(s->map, s->source, b->poses[0], s->target,
                                               b->poses[2], b->scratch, b->scratchSize);
    const maniPose* pose = b->poses[additive ? 4 : 2];
    for (uint32_t bone = 0; bone < b->bones; ++bone)
    {
        maniTransform l = maniPoseLocal(pose, bone);
        float* v = b->values + (size_t)bone * BONE_FLOATS * n;
        const float t[3] = {l.translation.x, l.translation.y, l.translation.z};
        const float q[4] = {l.rotation.x, l.rotation.y, l.rotation.z, l.rotation.w};
        const float c[3] = {l.scale.x, l.scale.y, l.scale.z};
        memcpy(v + 3 * k, t, sizeof t);
        memcpy(v + 3 * n + 4 * k, q, sizeof q);
        memcpy(v + 7 * n + 3 * k, c, sizeof c);
    }
    for (uint32_t i = 0; i < b->floats; ++i)
    {
        b->floatValues[(size_t)i * n + k] = b->sampleFloats[i];
    }
    if (b->root != MANI_NO_BONE)
    {
        Move(b, k);
    }
    return r;
}

// Whether a channel's samples (dimension floats each) are all its rest's.
static bool AtRest(const float* values, uint32_t dimension, size_t n, const float* rest)
{
    for (size_t k = 0; k < n; ++k)
    {
        if (memcmp(values + dimension * k, rest, dimension * sizeof(float)) != 0)
        {
            return false;
        }
    }
    return true;
}

static maniChannelSource Channel(const Bake* b, const float* values, bool kept)
{
    return (maniChannelSource){b->times, values, kept ? b->samples : 0u, maniInterpolationLinear};
}

// The bone tracks: each target bone's channels not at its rest (the root
// motion bone's translation and rotation always); their count.
static uint32_t LayBoneTracks(Bake* b)
{
    size_t n = b->samples;
    uint32_t count = 0;
    for (uint32_t bone = 0; bone < b->bones; ++bone)
    {
        maniTransform rest = maniSkeletonRest(b->s->target, bone);
        const float t[3] = {rest.translation.x, rest.translation.y, rest.translation.z};
        const float q[4] = {rest.rotation.x, rest.rotation.y, rest.rotation.z, rest.rotation.w};
        const float c[3] = {rest.scale.x, rest.scale.y, rest.scale.z};
        const float* v = b->values + (size_t)bone * BONE_FLOATS * n;
        bool root = bone == b->root;
        maniBoneTrackSource track = {bone, 0, Channel(b, v, root || !AtRest(v, 3, n, t)),
                                     Channel(b, v + 3 * n, root || !AtRest(v + 3 * n, 4, n, q)),
                                     Channel(b, v + 7 * n, !AtRest(v + 7 * n, 3, n, c))};
        if (track.translation.keyCount + track.rotation.keyCount + track.scale.keyCount > 0)
        {
            b->boneTracks[count++] = track;
        }
    }
    return count;
}

// The float tracks: names and interpolations from the clip, values from
// the samples.
static void LayFloatTracks(Bake* b)
{
    const maniClip* clip = b->s->clip;
    for (uint32_t i = 0; i < b->floats; ++i)
    {
        uint32_t length = 0;
        const char* name = maniClipFloatTrackName(clip, i, &length);
        uint32_t interpolation =
            maniLoadU32(clip->floatTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 16);
        b->floatTracks[i] = (maniFloatTrackSource){
            name, length, 0,
            (maniChannelSource){b->times, b->floatValues + (size_t)i * b->samples, b->samples,
                                interpolation}};
    }
}

// The discrete tracks' keys, the events and the markers, copied.
static void LayCopied(Bake* b)
{
    const maniClip* clip = b->s->clip;
    size_t at = 0;
    for (uint32_t i = 0; i < clip->discreteTrackCount; ++i)
    {
        const uint8_t* e = clip->discreteTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        uint32_t first = maniLoadU32(e + 16);
        uint32_t count = maniLoadU32(e + 20);
        for (uint32_t j = 0; j < count; ++j)
        {
            const uint8_t* key = clip->discreteKeys + (size_t)(first + j) * MANI_CLIP_KEY_SIZE;
            b->discreteTimes[at + j] = LoadF32(key);
            b->discreteValues[at + j] = (int32_t)maniLoadU32(key + 4);
        }
        uint32_t length = 0;
        const char* name = maniClipDiscreteTrackName(clip, i, &length);
        b->discreteTracks[i] = (maniDiscreteTrackSource){name, length, count, b->discreteTimes + at,
                                                         b->discreteValues + at};
        at += count;
    }
    uint32_t events = clip->eventCount;
    for (uint32_t i = 0; i < events + clip->markerCount; ++i)
    {
        maniClipEvent e =
            i < events ? maniClipEventAt(clip, i) : maniClipMarkerAt(clip, i - events);
        b->events[i] = (maniEventSource){e.time, e.payload, e.name, e.nameLength, 0};
    }
}

// Samples every frame and builds the clip.
static maniBuildResult Build(maniBuilder* builder, Bake* b)
{
    const maniClip* clip = b->s->clip;
    maniResult r = mani_success;
    if (clip->additive != maniAdditiveNone)
    {
        // The basis: the source's rest (the pose as made) carried.
        r = maniRetargetPose(b->s->map, b->s->source, b->poses[1], b->s->target, b->poses[3],
                             b->scratch, b->scratchSize);
    }
    for (uint32_t k = 0; r == mani_success && k < b->samples; ++k)
    {
        b->times[k] = (float)maniClipSampleTime(k, b->samples, clip->sampleRate, clip->duration);
        r = Sample(b, k);
    }
    if (r != mani_success)
    {
        return Result(mani_errorInvalid, maniBuildReasonBasis, 0, 0);
    }
    maniClipSource c = {0};
    c.skeleton = b->s->target;
    c.boneTracks = b->boneTracks;
    c.boneTrackCount = LayBoneTracks(b);
    LayFloatTracks(b);
    LayCopied(b);
    c.floatTracks = b->floatTracks;
    c.floatTrackCount = b->floats;
    c.discreteTracks = b->discreteTracks;
    c.discreteTrackCount = clip->discreteTrackCount;
    c.events = b->events;
    c.eventCount = clip->eventCount;
    c.markers = b->events + clip->eventCount;
    c.markerCount = clip->markerCount;
    c.loop = clip->loop;
    c.sampleRate = clip->sampleRate;
    c.duration = clip->duration;
    c.additive = clip->additive;
    maniRootMotionSource from;
    if (!maniClipRootMotionSource(clip, &from))
    {
        return maniBuildClipMoved(builder, &c, NULL);
    }
    maniRootMotionWork work = {Declared(b, from), b->moves, b->samples};
    return maniBuildClipMoved(builder, &c, &work);
}

static bool Usable(const maniRetargetClipSource* s)
{
    return s != NULL && s->map != NULL && s->source != NULL && s->target != NULL &&
           s->clip != NULL && s->reserved == 0;
}

// The skeletons the map's, the clip the source's.
static maniBuildResult Matches(const maniRetargetClipSource* s)
{
    const maniRetargetMap* m = s->map;
    if (maniSkeletonIdentity(s->source) != m->sourceIdentity ||
        maniSkeletonBoneCount(s->source) != m->sourceBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRetargetClip, 0, 0);
    }
    if (maniSkeletonIdentity(s->target) != m->targetIdentity ||
        maniSkeletonBoneCount(s->target) != m->targetBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRetargetClip, 0, 1);
    }
    if (s->clip->skeletonIdentity != m->sourceIdentity ||
        s->clip->skeletonBoneCount != m->sourceBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRetargetClip, 0, 2);
    }
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

// The discrete keys of every track.
static size_t DiscreteKeys(const maniClip* clip)
{
    size_t keys = 0;
    for (uint32_t i = 0; i < clip->discreteTrackCount; ++i)
    {
        keys += maniLoadU32(clip->discreteTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 20);
    }
    return keys;
}

maniBuildResult maniBuildRetargetedClip(maniBuilder* builder, const maniRetargetClipSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (!Usable(source))
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    Bake b = {0};
    b.s = source;
    b.allocator = &builder->allocator;
    b.samples = source->clip->sampleCount;
    b.bones = source->map->targetBoneCount;
    b.floats = source->clip->floatTrackCount;
    maniBuildResult r = Matches(source);
    r = r.result == mani_success ? PlaceRoot(&b) : r;
    if (r.result == mani_success)
    {
        b.keys = DiscreteKeys(source->clip);
        r = TakeLists(&b) ? Build(builder, &b)
                          : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    ReleaseAll(&b);
    return r;
}
