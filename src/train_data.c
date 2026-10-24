// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's training data; see train_data.h. Two
// passes over the frames: the first finds the bones whose scale changes,
// the second writes every row.

#include "train_data.h"

#include "allocator.h"
#include "match_move.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// A frame's three places (its time, a database frame ahead and behind),
// the seconds between the outer two, and three poses sampled there.
typedef struct Sampler
{
    const maniTrainDataSource* s;
    const maniClip* clip;
    maniPose* poses[3];
    uint8_t* memory[3];
    size_t poseSize;
    float* floats;
    uint32_t floatCapacity;
    uint32_t reserved;
    maniMatchPlace places[3];
    maniMatchEntry entry;
    float time;
    float apart;
} Sampler;

static maniPose* Align(uint8_t* memory)
{
    return (maniPose*)(memory + (MANI_POSE_ALIGNMENT - (uintptr_t)memory % MANI_POSE_ALIGNMENT) %
                                    MANI_POSE_ALIGNMENT);
}

static void ReleaseSampler(const maniAllocator* a, Sampler* p)
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        maniRelease(a, p->memory[k], p->poseSize + MANI_POSE_ALIGNMENT, 1, 8);
    }
    maniRelease(a, p->floats, (size_t)p->floatCapacity + 1, sizeof(float), 4);
}

static bool InitSampler(const maniAllocator* a, const maniTrainDataSource* s, Sampler* p)
{
    *p = (Sampler){.s = s, .poseSize = maniPoseSize(s->skeleton)};
    for (uint32_t c = 0; c < s->database->clipCount; ++c)
    {
        uint32_t count = maniClipFloatTrackCount(s->clips[c]);
        p->floatCapacity = count > p->floatCapacity ? count : p->floatCapacity;
    }
    bool ok = true;
    for (uint32_t k = 0; k < 3; ++k)
    {
        p->memory[k] = maniAllocate(a, p->poseSize + MANI_POSE_ALIGNMENT, 1, 8);
        ok = ok && p->memory[k] != NULL &&
             maniInitPose(s->skeleton, Align(p->memory[k]), p->poseSize, &p->poses[k]) ==
                 mani_success;
    }
    p->floats = maniAllocate(a, (size_t)p->floatCapacity + 1, sizeof(float), 4);
    return ok && p->floats != NULL;
}

// Places frame f and samples its poses: only the first when only is set.
static bool Sample(Sampler* p, uint32_t f, bool only)
{
    const maniMotionDatabase* d = p->s->database;
    uint32_t entry = 0;
    if (maniMotionDatabaseFrame(d, f, &entry, &p->time) != mani_success ||
        maniMotionDatabaseEntry(d, entry, &p->entry) != mani_success)
    {
        return false;
    }
    p->clip = p->s->clips[p->entry.clip];
    bool loop = (p->entry.flags & MANI_MATCH_LOOP) != 0;
    maniMatchSpan span = {p->entry.start, p->entry.end, loop};
    double h = 1.0 / (double)d->sampleRate;
    p->places[0] = maniMatchLocate(p->clip, span, p->time, 0.0);
    p->places[1] = maniMatchLocate(p->clip, span, p->time, h);
    p->places[2] = maniMatchLocate(p->clip, span, p->time, -h);
    p->apart =
        loop ? (float)(2.0 * h) : (float)((double)p->places[1].time - (double)p->places[2].time);
    for (uint32_t k = 0; k < (only ? 1u : 3u); ++k)
    {
        if (maniSampleClip(p->clip, p->s->skeleton, p->places[k].time, p->poses[k]) !=
                mani_success ||
            maniComputeModel(p->poses[k], p->s->skeleton) != mani_success)
        {
            return false;
        }
    }
    return true;
}

static void Put3(float* out, maniVec3 v)
{
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}

static void PutColumns(float* out, maniQuat q)
{
    Put3(out, maniQuatRotate(q, maniVec3Make(1.0f, 0.0f, 0.0f)));
    Put3(out + 3, maniQuatRotate(q, maniVec3Make(0.0f, 1.0f, 0.0f)));
}

// The difference of b and a over seconds (0 when no time is between).
static maniVec3 Rate(maniVec3 a, maniVec3 b, float seconds)
{
    float k = seconds > 0.0f ? 1.0f / seconds : 0.0f;
    return maniVec3Scale(maniVec3Sub(b, a), k);
}

// Bone b's character space at a pose: its model translation from bone
// 0's and its model columns, turned by bone 0's rotation undone.
static void Character(const maniPose* pose, uint32_t b, float* position, float* columns)
{
    maniQuat undo = maniQuatConjugate(maniPoseLocal(pose, 0).rotation);
    maniAffine root = maniPoseModel(pose, 0);
    maniAffine m = maniPoseModel(pose, b);
    maniVec3 offset = maniVec3Make(m.m[9] - root.m[9], m.m[10] - root.m[10], m.m[11] - root.m[11]);
    Put3(position, maniQuatRotate(undo, offset));
    Put3(columns, maniQuatRotate(undo, maniVec3Make(m.m[0], m.m[1], m.m[2])));
    Put3(columns + 3, maniQuatRotate(undo, maniVec3Make(m.m[3], m.m[4], m.m[5])));
}

static bool SameName(const char* a, uint32_t aLength, const maniLearnedNameSource* b)
{
    return aLength == b->length && memcmp(a, b->name, aLength) == 0;
}

// The float tracks' values and the events' states at the frame.
static bool Extras(Sampler* p, float* out)
{
    const maniTrainDataSource* s = p->s;
    uint32_t count = 0;
    if (maniSampleClipFloats(p->clip, p->time, p->floats, p->floatCapacity, &count) != mani_success)
    {
        return false;
    }
    for (uint32_t k = 0; k < s->floatTrackCount; ++k)
    {
        uint32_t track =
            maniClipFindFloatTrack(p->clip, s->floatTracks[k].name, s->floatTracks[k].length);
        out[k] = track < count ? p->floats[track] : 0.0f;
    }
    bool loop = (p->entry.flags & MANI_MATCH_LOOP) != 0;
    float duration = maniClipDuration(p->clip);
    for (uint32_t k = 0; k < s->eventCount; ++k)
    {
        float state = 0.0f;
        for (uint32_t e = 0; e < maniClipEventCount(p->clip); ++e)
        {
            maniClipEvent event = maniClipEventAt(p->clip, e);
            double since = (double)p->time - (double)event.time;
            since += loop && since < 0.0 ? (double)duration : 0.0;
            bool near = since >= 0.0 && since < (double)s->eventSeconds;
            state = near && SameName(event.name, event.nameLength, &s->events[k]) ? 1.0f : state;
        }
        out[s->floatTrackCount + k] = state;
    }
    return true;
}

// The first pass: the bones whose scale differs from frame 0's.
static bool FindScaled(Sampler* p, maniTrainData* d)
{
    uint32_t bones = d->boneCount;
    for (uint32_t f = 0; f < d->frameCount; ++f)
    {
        if (!Sample(p, f, true))
        {
            return false;
        }
        for (uint32_t b = 0; b < bones; ++b)
        {
            maniVec3 scale = maniPoseLocal(p->poses[0], b).scale;
            if (f == 0)
            {
                Put3(d->scales + 3 * (size_t)b, scale);
                continue;
            }
            const float* first = d->scales + 3 * (size_t)b;
            bool same = scale.x == first[0] && scale.y == first[1] && scale.z == first[2];
            // A bone's scale changing is marked by its place's sign.
            d->scaledBones[b] = same ? d->scaledBones[b] : 1u;
        }
    }
    uint32_t count = 0;
    for (uint32_t b = 0; b < bones; ++b)
    {
        if (d->scaledBones[b] != 0)
        {
            d->scaledBones[count++] = b;
        }
    }
    d->scaledBoneCount = count;
    return true;
}

// One bone's local and character values in the input groups.
static void BoneInputs(const Sampler* p, const maniTrainData* d, uint32_t b, float* in)
{
    const uint32_t* g = d->groupStarts;
    maniTransform now = maniPoseLocal(p->poses[0], b);
    maniTransform ahead = maniPoseLocal(p->poses[1], b);
    maniTransform behind = maniPoseLocal(p->poses[2], b);
    Put3(in + g[maniTrainGroupPosition] + 3 * b, now.translation);
    PutColumns(in + g[maniTrainGroupColumns] + 6 * b, now.rotation);
    Put3(in + g[maniTrainGroupVelocity] + 3 * b,
         Rate(behind.translation, ahead.translation, p->apart));
    maniVec3 turn =
        maniQuatToRotationVector(maniQuatMul(ahead.rotation, maniQuatConjugate(behind.rotation)));
    Put3(in + g[maniTrainGroupSpin] + 3 * b, Rate(maniVec3Make(0, 0, 0), turn, p->apart));
    float position[3][3];
    float columns[3][6];
    for (uint32_t k = 0; k < 3; ++k)
    {
        Character(p->poses[k], b, position[k], columns[k]);
    }
    memcpy(in + g[maniTrainGroupCharacterPosition] + 3 * b, position[0], sizeof position[0]);
    memcpy(in + g[maniTrainGroupCharacterColumns] + 6 * b, columns[0], sizeof columns[0]);
    float k = p->apart > 0.0f ? 1.0f / p->apart : 0.0f;
    for (uint32_t i = 0; i < 3; ++i)
    {
        in[g[maniTrainGroupCharacterVelocity] + 3 * b + i] = (position[1][i] - position[2][i]) * k;
    }
    for (uint32_t i = 0; i < 6; ++i)
    {
        in[g[maniTrainGroupCharacterColumnVelocity] + 6 * b + i] =
            (columns[1][i] - columns[2][i]) * k;
    }
}

// The second pass's row of frame f.
static bool Row(Sampler* p, const maniTrainData* d, uint32_t f)
{
    if (!Sample(p, f, false) ||
        maniMotionDatabaseFeatures(p->s->database, f, d->features + (size_t)f * d->featureCount) !=
            mani_success)
    {
        return false;
    }
    float* out = d->outputs + (size_t)f * d->outputCount;
    float* in = d->inputs + (size_t)f * d->inputCount;
    const uint32_t* g = d->groupStarts;
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        maniTransform t = maniPoseLocal(p->poses[0], b);
        Put3(out + 9 * (size_t)b, t.translation);
        PutColumns(out + 9 * (size_t)b + 3, t.rotation);
        BoneInputs(p, d, b, in);
    }
    float* rest = out + 9 * (size_t)d->boneCount;
    for (uint32_t k = 0; k < d->scaledBoneCount; ++k)
    {
        Put3(rest + 3 * k, maniPoseLocal(p->poses[0], d->scaledBones[k]).scale);
    }
    rest += 3 * (size_t)d->scaledBoneCount;
    maniRootMove ahead = p->places[1].move;
    maniRootMove behind = p->places[2].move;
    Put3(rest, Rate(behind.translation, ahead.translation, p->apart));
    Put3(rest + 3, Rate(maniQuatToRotationVector(behind.rotation),
                        maniQuatToRotationVector(ahead.rotation), p->apart));
    if (!Extras(p, rest + 6))
    {
        return false;
    }
    size_t tail = 3 * (size_t)d->scaledBoneCount + 6 + d->floatTrackCount + d->eventCount;
    memcpy(in + g[maniTrainGroupScale], out + 9 * (size_t)d->boneCount, sizeof(float) * tail);
    return true;
}

static void Layout(maniTrainData* d)
{
    uint32_t b = d->boneCount;
    const uint32_t widths[maniTrainGroupCount] = {3 * b, 6 * b, 3 * b,
                                                  3 * b, 3 * b, 6 * b,
                                                  3 * b, 6 * b, 3 * d->scaledBoneCount,
                                                  3,     3,     d->floatTrackCount + d->eventCount};
    d->groupStarts[0] = 0;
    for (uint32_t k = 0; k < maniTrainGroupCount; ++k)
    {
        d->groupStarts[k + 1] = d->groupStarts[k] + widths[k];
    }
    d->inputCount = d->groupStarts[maniTrainGroupCount];
    d->outputCount = 9 * b + 3 * d->scaledBoneCount + 6 + d->floatTrackCount + d->eventCount;
}

static void Pairs(const maniMotionDatabase* database, maniTrainData* d)
{
    for (uint32_t e = 0; e < database->entryCount; ++e)
    {
        maniMatchEntry entry;
        if (maniMotionDatabaseEntry(database, e, &entry) != mani_success)
        {
            continue;
        }
        for (uint32_t k = 0; k + 1 < entry.frameCount; ++k)
        {
            d->pairs[d->pairCount++] = entry.firstFrame + k;
        }
    }
}

void maniTrainDataRelease(const maniAllocator* allocator, maniTrainData* data)
{
    size_t frames = data->frameCount;
    maniRelease(allocator, data->features, frames * data->featureCount + 1, sizeof(float), 8);
    maniRelease(allocator, data->outputs, frames * data->outputCount + 1, sizeof(float), 8);
    maniRelease(allocator, data->inputs, frames * data->inputCount + 1, sizeof(float), 8);
    maniRelease(allocator, data->scaledBones, (size_t)data->boneCount + 1, sizeof(uint32_t), 4);
    maniRelease(allocator, data->scales, 3 * (size_t)data->boneCount, sizeof(float), 4);
    maniRelease(allocator, data->pairs, frames + 1, sizeof(uint32_t), 4);
    maniRelease(allocator, data->parents, (size_t)data->boneCount + 1, sizeof(uint32_t), 4);
    *data = (maniTrainData){0};
}

bool maniTrainDataBuild(const maniAllocator* allocator, const maniTrainDataSource* source,
                        maniTrainData* dataOut)
{
    const maniMotionDatabase* database = source->database;
    maniTrainData d = {.frameCount = database->frameCount,
                       .featureCount = database->featureCount,
                       .boneCount = source->skeleton->boneCount,
                       .floatTrackCount = source->floatTrackCount,
                       .eventCount = source->eventCount,
                       .frameSeconds = 1.0f / database->sampleRate};
    size_t frames = d.frameCount;
    d.scaledBones = maniAllocate(allocator, (size_t)d.boneCount + 1, sizeof(uint32_t), 4);
    d.scales = maniAllocate(allocator, 3 * (size_t)d.boneCount, sizeof(float), 4);
    d.pairs = maniAllocate(allocator, frames + 1, sizeof(uint32_t), 4);
    d.parents = maniAllocate(allocator, (size_t)d.boneCount + 1, sizeof(uint32_t), 4);
    for (uint32_t b = 0; d.parents != NULL && b < d.boneCount; ++b)
    {
        d.parents[b] = maniSkeletonParent(source->skeleton, b);
    }
    Sampler p;
    bool ok = InitSampler(allocator, source, &p) && d.scaledBones != NULL && d.scales != NULL &&
              d.pairs != NULL && d.parents != NULL;
    if (ok)
    {
        memset(d.scaledBones, 0, sizeof(uint32_t) * d.boneCount);
        ok = FindScaled(&p, &d);
    }
    Layout(&d);
    d.features = ok ? maniAllocate(allocator, frames * d.featureCount + 1, sizeof(float), 8) : NULL;
    d.outputs = ok ? maniAllocate(allocator, frames * d.outputCount + 1, sizeof(float), 8) : NULL;
    d.inputs = ok ? maniAllocate(allocator, frames * d.inputCount + 1, sizeof(float), 8) : NULL;
    ok = ok && d.features != NULL && d.outputs != NULL && d.inputs != NULL;
    for (uint32_t f = 0; ok && f < d.frameCount; ++f)
    {
        ok = Row(&p, &d, f);
    }
    if (ok)
    {
        Pairs(database, &d);
    }
    ReleaseSampler(allocator, &p);
    if (!ok)
    {
        maniTrainDataRelease(allocator, &d);
        return false;
    }
    *dataOut = d;
    return true;
}

void maniTrainMeasure(const float* rows, uint32_t width, uint32_t frames, uint32_t first,
                      uint32_t count, float* offsets, float* scales, bool group)
{
    double all = 0.0;
    double allSquares = 0.0;
    for (uint32_t j = first; j < first + count; ++j)
    {
        double sum = 0.0;
        double squares = 0.0;
        for (uint32_t f = 0; f < frames; ++f)
        {
            double x = (double)rows[(size_t)f * width + j];
            sum += x;
            squares += x * x;
        }
        double mean = sum / frames;
        double variance = squares / frames - mean * mean;
        offsets[j] = (float)mean;
        scales[j] = (float)sqrt(variance > 0.0 ? variance : 0.0);
        all += sum;
        allSquares += squares;
    }
    if (group && count > 0)
    {
        double n = (double)frames * count;
        double mean = all / n;
        double variance = allSquares / n - mean * mean;
        float deviation = (float)sqrt(variance > 0.0 ? variance : 0.0);
        for (uint32_t j = first; j < first + count; ++j)
        {
            scales[j] = deviation > 0.0f ? deviation : 1.0f;
        }
    }
}
