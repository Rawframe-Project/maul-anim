// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A compressed clip's samples decoded (its sections are checked at open
// by clip_compressed_open.c). Sub-tracks run in kind order: each kind of bone channel
// (rotation, translation, scale) over the tracks that store it, then the
// float tracks; a bone channel is the rest, constant or animated. An
// animated value at a sample is its bits at its segment's rate, through
// the segment range (with several segments) and the clip range. A kind
// is decoded eight sub-tracks at a time on the lanes, each lane with the
// scalar arithmetic in its order, so every backend gives the same bits.

#include "blend_lanes.h"
#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_lanes.h"
#include "clip_view.h"
#include "fpenv.h"
#include "pose_memory.h"
#include "simd.h"

#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define LANES MANI_POSE_GROUP_BONES

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// Where a sample's animated sub-tracks are: its segment's rates and
// ranges, its bits, and how far along them the walk is.
typedef struct Walker
{
    const uint8_t* rates;
    const uint8_t* ranges;
    const uint8_t* stream;
    uint64_t bit;
    bool several;
} Walker;

// The segment holding a sample: segments are about the same length, so
// a division lands on it or next to it.
static const uint8_t* SegmentOf(const maniClip* clip, uint32_t sample)
{
    uint32_t n = clip->segmentCount;
    uint32_t per = clip->sampleCount / n;
    uint32_t s = sample / per < n ? sample / per : n - 1;
    while (s > 0 && maniLoadU32(clip->segments + (size_t)s * MANI_CLIP_SEGMENT_SIZE) > sample)
    {
        --s;
    }
    while (s + 1 < n &&
           maniLoadU32(clip->segments + (size_t)(s + 1) * MANI_CLIP_SEGMENT_SIZE) <= sample)
    {
        ++s;
    }
    return clip->segments + (size_t)s * MANI_CLIP_SEGMENT_SIZE;
}

// A walker at a sample's first bone channel, or (floats) its first float.
static Walker WalkerAt(const maniClip* clip, uint32_t sample, bool floats)
{
    const uint8_t* e = SegmentOf(clip, sample);
    const uint8_t* base = clip->segmentData + maniLoadU32(e + 8);
    uint64_t channelBits = maniLoadU32(e + 12);
    uint64_t sampleBits = channelBits + maniLoadU32(e + 16);
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    Walker w = {base, base + n, base + maniCompressedSegmentHead(clip),
                (uint64_t)(sample - maniLoadU32(e)) * sampleBits, clip->segmentCount > 1};
    if (floats)
    {
        w.rates += clip->animatedChannels;
        w.ranges += w.several ? 6 * (size_t)clip->animatedChannels : 0;
        w.bit += channelBits;
    }
    return w;
}

// Moves a walker past a sub-track of `width` components unread.
static void Pass(Walker* w, uint32_t width)
{
    uint32_t rate = *w->rates++;
    w->ranges += w->several ? 2 * (size_t)width : 0;
    w->bit += (uint64_t)width * maniCodecBits(rate);
}

// The next animated float track's value at a walker, through its clip
// range.
static float DecodeFloat(Walker* w, float minimum, float extent)
{
    uint32_t rate = *w->rates++;
    const uint8_t* segment = w->ranges;
    w->ranges += w->several ? 2 : 0;
    uint32_t bits = maniCodecBits(rate);
    uint32_t q = rate == 0 ? 0 : maniBitRead(w->stream, w->bit, bits);
    w->bit += bits;
    if (rate == MANI_CODEC_RAW_RATE)
    {
        float raw;
        memcpy(&raw, &q, sizeof q);
        return raw;
    }
    float n = rate == 0 ? maniCodecUnpack16(maniLoadU16(segment)) : maniCodecUnpack(q, bits);
    if (w->several && rate != 0)
    {
        n = maniCodecSegmentValue(n, segment[0], segment[1]);
    }
    return maniCodecValue(n, minimum, extent);
}

// How a group's coded values decode at one sample, lane by lane: the
// rate's step (0 for a raw value), all bits set for a raw value and for
// rate 0, and with several segments the segment range's bytes.
typedef struct Scale
{
    float step[LANES];
    float raw[LANES];
    float constant[LANES];
    float low[3][LANES];
    float span[3][LANES];
} Scale;

// A scale's segment ranges on the lanes: each byte over 255 as
// maniCodecRangeByte gives it, or at rate 0 (the value constant in its
// segment) -0 and 1, which leave a value's bits.
typedef struct Ranges
{
    manif8 low[3];
    manif8 span[3];
} Ranges;

static float Mask(bool set)
{
    uint32_t bits = set ? UINT32_MAX : 0u;
    float mask;
    memcpy(&mask, &bits, sizeof mask);
    return mask;
}

// A lane's scale from its sub-track's rate and segment range.
static void ScaleLane(Scale* s, uint32_t lane, uint32_t rate, const uint8_t* segment, bool several)
{
    // maniCodecInverse[0] is 0, the step of a raw value, never used.
    s->step[lane] = maniCodecInverse[rate == 0 ? 16 : (rate < MANI_CODEC_RAW_RATE ? rate : 0)];
    s->raw[lane] = Mask(rate == MANI_CODEC_RAW_RATE);
    s->constant[lane] = Mask(rate == 0);
    for (uint32_t k = 0; several && k < 3; ++k)
    {
        s->low[k][lane] = (float)segment[k];
        s->span[k][lane] = (float)segment[3 + k];
    }
}

static void RangesOf(const Scale* s, Ranges* r)
{
    manif8 byte = maniF8Set1(maniCodecInverse[8]);
    manif8 constant = maniF8Load(s->constant);
    for (uint32_t k = 0; k < 3; ++k)
    {
        r->low[k] =
            maniF8Select(constant, maniF8Set1(-0.0f), maniF8Mul(maniF8Load(s->low[k]), byte));
        r->span[k] =
            maniF8Select(constant, maniF8Set1(1.0f), maniF8Mul(maniF8Load(s->span[k]), byte));
    }
}

// A sub-track's three components at a bit into a lane, as maniBitRead
// reads them: as floats, a raw value's bits as they are, at rate 0 the
// segment's 16-bit values.
static void Read(const uint8_t* stream, uint64_t bit, uint32_t rate, const uint8_t* segment,
                 float (*q)[LANES], uint32_t lane)
{
    if (rate - 1u < 19u)
    {
        // 1 to 19 bits: the three fields and the first one's place in its
        // byte fit one 64-bit load.
        uint64_t word = maniLoadU64(stream + (bit >> 3)) >> (bit & 7u);
        uint64_t mask = ((uint64_t)1 << rate) - 1u;
        q[0][lane] = (float)(uint32_t)(word & mask);
        q[1][lane] = (float)(uint32_t)((word >> rate) & mask);
        q[2][lane] = (float)(uint32_t)((word >> (2 * rate)) & mask);
        return;
    }
    uint32_t bits = maniCodecBits(rate);
    for (uint32_t k = 0; k < 3; ++k)
    {
        uint32_t v = rate == 0 ? maniLoadU16(segment + 2 * k)
                               : maniBitRead(stream, bit + (uint64_t)k * bits, bits);
        float value = (float)v;
        if (rate == MANI_CODEC_RAW_RATE)
        {
            memcpy(&value, &v, sizeof value);
        }
        q[k][lane] = value;
    }
}

// The next animated sub-track at a walker into a lane (out of line: the
// decoder's path, and random access's when its samples' segments differ).
static MANI_NOINLINE void CodeOne(Walker* w, Scale* scale, float (*q)[LANES], uint32_t lane)
{
    uint32_t rate = *w->rates++;
    const uint8_t* segment = w->ranges;
    w->ranges += w->several ? 6 : 0;
    ScaleLane(scale, lane, rate, segment, w->several);
    Read(w->stream, w->bit, rate, segment, q, lane);
    w->bit += 3 * (uint64_t)maniCodecBits(rate);
}

// A group's values at one sample: q times the step, through the segment
// range, times the clip range's extent plus its minimum; a raw value as
// it is (maniCodecUnpack, maniCodecSegmentValue, maniCodecValue).
static void Values(const float (*q)[LANES], const Scale* scale, const Ranges* ranges,
                   const float (*minimum)[LANES], const float (*extent)[LANES], manif8 out[3])
{
    manif8 step = maniF8Load(scale->step);
    manif8 raw = maniF8Load(scale->raw);
    for (uint32_t k = 0; k < 3; ++k)
    {
        manif8 v = maniF8Load(q[k]);
        manif8 n = maniF8Mul(v, step);
        if (ranges != NULL)
        {
            n = maniF8Add(ranges->low[k], maniF8Mul(n, ranges->span[k]));
        }
        manif8 value = maniF8Add(maniF8Mul(n, maniF8Load(extent[k])), maniF8Load(minimum[k]));
        out[k] = maniF8Select(raw, v, value);
    }
}

// maniCodecRebuild on the lanes: sqrt(|((1 - a*a) - b*b) - c*c|), the
// absolute value as fabsf's bits.
static manif8 Rebuild(manif8 a, manif8 b, manif8 c)
{
    uint32_t bits = 0x7FFFFFFFu;
    float magnitude;
    memcpy(&magnitude, &bits, sizeof magnitude);
    manif8 squared = maniF8Sub(maniF8Set1(1.0f), maniF8Mul(a, a));
    squared = maniF8Sub(squared, maniF8Mul(b, b));
    squared = maniF8Sub(squared, maniF8Mul(c, c));
    return maniF8Sqrt(maniF8And(squared, maniF8Set1(magnitude)));
}

// maniCodecRotation on the lanes: the rebuilt component at its place.
static void Place(const manif8 kept[3], manif8 dropped, manif8 q[4])
{
    manif8 a = kept[0];
    manif8 b = kept[1];
    manif8 c = kept[2];
    manif8 r = Rebuild(a, b, c);
    manif8 d0 = maniF8EQ(dropped, maniF8Zero());
    manif8 d1 = maniF8EQ(dropped, maniF8Set1(1.0f));
    manif8 d2 = maniF8EQ(dropped, maniF8Set1(2.0f));
    manif8 d3 = maniF8EQ(dropped, maniF8Set1(3.0f));
    q[0] = maniF8Select(d0, r, a);
    q[1] = maniF8Select(d0, a, maniF8Select(d1, r, b));
    q[2] = maniF8Select(maniF8LT(dropped, maniF8Set1(2.0f)), b, maniF8Select(d2, r, c));
    q[3] = maniF8Select(d3, r, c);
}

// A group's values at one sample, a rotation rebuilt with its dropped
// component (dropped not NULL; four rows) or a vector as it is (three).
// Out of line: random access and the decoder share it.
static MANI_NOINLINE void Decoded(const float (*q)[LANES], const Scale* scale, const Ranges* ranges,
                                  const float (*minimum)[LANES], const float (*extent)[LANES],
                                  const float* dropped, manif8* out)
{
    manif8 kept[3];
    Values(q, scale, ranges, minimum, extent, kept);
    if (dropped != NULL)
    {
        Place(kept, maniF8Load(dropped), out);
        return;
    }
    memcpy(out, kept, sizeof kept);
}

// Up to eight sub-tracks of one kind on the lanes: each lane's bone and
// slot, its entry among the kind's, its stored rotation's dropped
// component, and its values at the two samples: coded bits and their
// scales, or (constants, caches) the values. Lanes past the count keep
// what an earlier group left, finite values never stored.
typedef struct Group
{
    uint32_t count;
    uint32_t bones[LANES];
    float* slots[LANES];
    uint32_t entries[LANES];
    float dropped[LANES];
    float qa[4][LANES];
    float qb[4][LANES];
    Scale sa;
    Scale sb;
    float minimum[3][LANES];
    float extent[3][LANES];
} Group;

// A group's rows (first to first + count of a slot's components) into
// its slots: one store a row when its lanes are a whole group of the
// pose's bones in order.
static void Scatter(const Group* g, const manif8* rows, uint32_t first, uint32_t count)
{
    if (g->count == LANES && g->bones[0] % LANES == 0 && g->bones[LANES - 1] == g->bones[0] + 7)
    {
        for (uint32_t k = 0; k < count; ++k)
        {
            maniF8Store(g->slots[0] + (size_t)(first + k) * LANES, rows[k]);
        }
        return;
    }
    float values[4][LANES];
    for (uint32_t k = 0; k < count; ++k)
    {
        maniF8Store(values[k], rows[k]);
    }
    for (uint32_t i = 0; i < g->count; ++i)
    {
        for (uint32_t k = 0; k < count; ++k)
        {
            g->slots[i][(first + k) * LANES] = values[k][i];
        }
    }
}

// Two rotations a lane interpolated on the shorter arc and normalized as
// the scalar Nlerp does (both keep their dropped component positive, so
// the second is negated when they point apart), into the slots.
static void Rotations(const Group* g, const manif8 qa[4], const manif8 qb[4], float alpha)
{
    manif8 dot = maniF8Mul(qa[0], qb[0]);
    dot = maniF8Add(dot, maniF8Mul(qa[1], qb[1]));
    dot = maniF8Add(dot, maniF8Mul(qa[2], qb[2]));
    dot = maniF8Add(dot, maniF8Mul(qa[3], qb[3]));
    manif8 s = maniF8Select(maniF8LT(dot, maniF8Zero()), maniF8Set1(-1.0f), maniF8Set1(1.0f));
    manif8 t = maniF8Set1(alpha);
    float group[MANI_POSE_COMPONENTS * LANES];
    for (int k = 0; k < 4; ++k)
    {
        manif8 v = maniF8Add(qa[k], maniF8Mul(maniF8Sub(maniF8Mul(s, qb[k]), qa[k]), t));
        maniF8Store(group + (3 + k) * LANES, v);
    }
    maniBlendLanesNormalize(group);
    manif8 rows[4];
    for (int k = 0; k < 4; ++k)
    {
        rows[k] = maniF8Load(group + (3 + k) * LANES);
    }
    Scatter(g, rows, 3, 4);
}

// Two vectors a lane interpolated, into the slots from first.
static void Vectors(const Group* g, const manif8 va[3], const manif8 vb[3], float alpha,
                    uint32_t first)
{
    manif8 t = maniF8Set1(alpha);
    manif8 rows[3];
    for (int k = 0; k < 3; ++k)
    {
        rows[k] = maniF8Add(va[k], maniF8Mul(maniF8Sub(vb[k], va[k]), t));
    }
    Scatter(g, rows, first, 3);
}

// The walk over the kinds' channels: the pose, the bones listed (or every
// one), the two samples' walkers (sharing their segment's rates and
// ranges when both are in it) or caches, how far along the constants and
// clip ranges it is, and the group on the lanes.
typedef struct Sampling
{
    const maniClip* clip;
    const maniBoneList* list;
    float* locals;
    const uint8_t* dropped;
    Walker a;
    Walker b;
    bool shared;
    const float* cacheA;
    const float* cacheB;
    const uint8_t* constant;
    const uint8_t* ranges;
    float alpha;
    Group group;
} Sampling;

static bool Listed(const Sampling* s, uint32_t bone, uint32_t* next)
{
    return s->list == NULL || maniBoneListed(s->list, bone, next);
}

// The dropped component as a float, by its 2 bits in the forms.
static const float kDropped[4] = {3.0f, 2.0f, 1.0f, 0.0f};

// A group's lane for a track's bone; a rotation's dropped component.
static void Lane(Group* g, const Sampling* s, uint32_t kind, uint32_t t, uint32_t bone,
                 uint32_t entry)
{
    uint32_t i = g->count++;
    g->bones[i] = bone;
    g->slots[i] = maniPoseSlot(s->locals, bone);
    g->entries[i] = entry;
    if (kind == maniClipKindRotation)
    {
        g->dropped[i] = kDropped[maniCompressedDroppedField(s->dropped, t)];
    }
}

static void LoadRows(const float (*rows)[LANES], uint32_t count, manif8* out)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        out[k] = maniF8Load(rows[k]);
    }
}

// A group of one kind's constants: what interpolating two samples of one
// value gives, v + (v - v) alpha, which is v + 0 (a -0 made +0); for a
// rotation the dot product with itself is at least 1, so the shorter arc
// keeps its sign, then it is normalized.
static void FinishConstants(Group* g, uint32_t kind)
{
    manif8 v[4];
    LoadRows(g->qa, 3, v);
    manif8 zero = maniF8Zero();
    if (kind == maniClipKindRotation)
    {
        manif8 q[4];
        Place(v, maniF8Load(g->dropped), q);
        float group[MANI_POSE_COMPONENTS * LANES];
        for (int k = 0; k < 4; ++k)
        {
            maniF8Store(group + (3 + k) * LANES, maniF8Add(q[k], zero));
        }
        maniBlendLanesNormalize(group);
        LoadRows((const float (*)[LANES])(group + 3 * LANES), 4, q);
        Scatter(g, q, 3, 4);
    }
    else
    {
        for (int k = 0; k < 3; ++k)
        {
            v[k] = maniF8Add(v[k], zero);
        }
        Scatter(g, v, maniChannelFirst(maniClipKindChannel(kind)), 3);
    }
    g->count = 0;
}

static void Constants(Sampling* s, uint32_t kind)
{
    Group* g = &s->group;
    maniTrackWalk walk = maniWalkTracks(s->clip, kind, 0);
    uint32_t next = 0;
    for (uint32_t i = 0; i < s->clip->constantKinds[kind]; ++i)
    {
        uint32_t t = maniNextTrack(&walk);
        uint32_t bone = maniClipTrackBone(s->clip, t);
        const uint8_t* value = s->constant;
        s->constant += 12;
        if (!Listed(s, bone, &next))
        {
            continue;
        }
        for (uint32_t k = 0; k < 3; ++k)
        {
            g->qa[k][g->count] = LoadF32(value + 4 * k);
        }
        Lane(g, s, kind, t, bone, i);
        if (g->count == LANES)
        {
            FinishConstants(g, kind);
        }
    }
    if (g->count > 0)
    {
        FinishConstants(g, kind);
    }
}

// A group's clip ranges: in place for a whole block of eight (its lanes
// the block's sub-tracks in order), else copied lane by lane.
static void GroupRanges(Group* g, const uint8_t* ranges, uint32_t n)
{
    uint32_t first = g->entries[0];
    bool block = g->count == LANES && first % LANES == 0 && g->entries[LANES - 1] == first + 7;
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (block)
        {
            // A row of the block: eight binary32, little-endian as the host.
            memcpy(g->minimum[k], ranges + maniClipRangeAt(n, 3, first, k, 0),
                   sizeof g->minimum[k]);
            memcpy(g->extent[k], ranges + maniClipRangeAt(n, 3, first, k, 1), sizeof g->extent[k]);
            continue;
        }
        for (uint32_t i = 0; i < g->count; ++i)
        {
            g->minimum[k][i] = LoadF32(ranges + maniClipRangeAt(n, 3, g->entries[i], k, 0));
            g->extent[k][i] = LoadF32(ranges + maniClipRangeAt(n, 3, g->entries[i], k, 1));
        }
    }
}

// A group of one kind's animated sub-tracks: decoded at both samples (or
// as the caches hold them), interpolated, into the slots.
static void FinishAnimated(Sampling* s, uint32_t kind, bool cached)
{
    Group* g = &s->group;
    bool rotation = kind == maniClipKindRotation;
    manif8 qa[4];
    manif8 qb[4];
    if (cached)
    {
        LoadRows(g->qa, rotation ? 4 : 3, qa);
        LoadRows(g->qb, rotation ? 4 : 3, qb);
    }
    else
    {
        GroupRanges(g, s->ranges, s->clip->animatedKinds[kind]);
        Ranges ra;
        Ranges rb;
        bool several = s->a.several;
        if (several)
        {
            RangesOf(&g->sa, &ra);
            if (!s->shared)
            {
                RangesOf(&g->sb, &rb);
            }
        }
        const float* dropped = rotation ? g->dropped : NULL;
        Decoded(g->qa, &g->sa, several ? &ra : NULL, g->minimum, g->extent, dropped, qa);
        Decoded(g->qb, s->shared ? &g->sa : &g->sb, several ? (s->shared ? &ra : &rb) : NULL,
                g->minimum, g->extent, dropped, qb);
    }
    if (rotation)
    {
        Rotations(g, qa, qb, s->alpha);
    }
    else
    {
        Vectors(g, qa, qb, s->alpha, maniChannelFirst(maniClipKindChannel(kind)));
    }
    g->count = 0;
}

// The next animated sub-track at both samples into a lane: a pair in one
// segment shares its rate and ranges, so its scale is coded once.
static void CodePair(Sampling* s, uint32_t lane)
{
    Group* g = &s->group;
    if (!s->shared)
    {
        CodeOne(&s->a, &g->sa, g->qa, lane);
        CodeOne(&s->b, &g->sb, g->qb, lane);
        return;
    }
    Walker* a = &s->a;
    uint32_t rate = *a->rates++;
    const uint8_t* segment = a->ranges;
    a->ranges += a->several ? 6 : 0;
    ScaleLane(&g->sa, lane, rate, segment, a->several);
    Read(a->stream, a->bit, rate, segment, g->qa, lane);
    Read(s->b.stream, s->b.bit, rate, segment, g->qb, lane);
    uint64_t bits = 3 * (uint64_t)maniCodecBits(rate);
    a->bit += bits;
    s->b.bit += bits;
}

// Both samples moved past an animated sub-track unread.
static void PassPair(Sampling* s)
{
    if (!s->shared)
    {
        Pass(&s->a, 3);
        Pass(&s->b, 3);
        return;
    }
    uint64_t bits = 3 * (uint64_t)maniCodecBits(*s->a.rates);
    Pass(&s->a, 3);
    s->b.bit += bits;
}

static void Animated(Sampling* s, uint32_t kind)
{
    Group* g = &s->group;
    uint32_t n = s->clip->animatedKinds[kind];
    uint32_t width = kind == maniClipKindRotation ? 4 : 3;
    bool cached = s->cacheA != NULL;
    maniTrackWalk walk = maniWalkTracks(s->clip, kind, 1);
    uint32_t next = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t t = maniNextTrack(&walk);
        uint32_t bone = maniClipTrackBone(s->clip, t);
        bool listed = Listed(s, bone, &next);
        if (cached)
        {
            for (uint32_t k = 0; listed && k < width; ++k)
            {
                g->qa[k][g->count] = s->cacheA[k];
                g->qb[k][g->count] = s->cacheB[k];
            }
            s->cacheA += width;
            s->cacheB += width;
        }
        else if (listed)
        {
            CodePair(s, g->count);
        }
        else
        {
            PassPair(s);
        }
        if (!listed)
        {
            continue;
        }
        Lane(g, s, kind, t, bone, i);
        if (g->count == LANES)
        {
            FinishAnimated(s, kind, cached);
        }
    }
    if (g->count > 0)
    {
        FinishAnimated(s, kind, cached);
    }
    s->ranges += 24 * (size_t)n;
}

// Every bone's (or listed bone's) slots at its rest, the identity for a
// difference, as maniSlotRest writes them channel by channel; the whole
// groups of a whole pose's rests turned into their rows at once.
static void FillRests(float* locals, const maniSkeleton* skeleton, uint32_t bones, bool identity,
                      const maniBoneList* list)
{
    static const float kIdentity[MANI_POSE_COMPONENTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                          0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    uint32_t bone = 0;
    const uint32_t lanes = MANI_POSE_GROUP_BONES;
    for (; list == NULL && !identity && bone + lanes <= bones; bone += lanes)
    {
        float rests[MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS];
        memcpy(rests, skeleton->rests + MANI_POSE_COMPONENTS * sizeof(float) * (size_t)bone,
               sizeof rests);
        float* group = locals + (size_t)bone * MANI_POSE_COMPONENTS;
        for (uint32_t k = 0; k < MANI_POSE_COMPONENTS; ++k)
        {
            for (uint32_t lane = 0; lane < lanes; ++lane)
            {
                group[k * lanes + lane] = rests[lane * MANI_POSE_COMPONENTS + k];
            }
        }
    }
    uint32_t next = 0;
    for (; bone < bones; ++bone)
    {
        if (list != NULL && !maniBoneListed(list, bone, &next))
        {
            continue;
        }
        const float* rest = kIdentity;
        float values[MANI_POSE_COMPONENTS];
        if (!identity)
        {
            memcpy(values, skeleton->rests + MANI_POSE_COMPONENTS * sizeof(float) * (size_t)bone,
                   sizeof values);
            rest = values;
        }
        maniPoseSlotPut(maniPoseSlot(locals, bone), 0, rest, MANI_POSE_COMPONENTS);
    }
}

void maniSampleCompressedBones(const maniClip* clip, const maniSkeleton* skeleton,
                               const maniClipPair* pair, const maniBoneList* list, maniPose* pose)
{
    // The caches hold both samples, or the walkers read them.
    bool cached = pair->cacheA != NULL && pair->cacheB != NULL;
    Sampling s = {.clip = clip,
                  .list = list,
                  .locals = maniPoseLocals(pose),
                  .dropped = maniCompressedDropped(clip),
                  .cacheA = cached ? pair->cacheA : NULL,
                  .cacheB = cached ? pair->cacheB : NULL,
                  .constant = clip->constants,
                  .ranges = clip->ranges,
                  .alpha = pair->alpha};
    if (!cached)
    {
        s.a = WalkerAt(clip, pair->first, false);
        s.b = WalkerAt(clip, pair->second, false);
        s.shared = s.a.rates == s.b.rates;
    }
    // Every bone (or listed one) at rest, then each kind's stored
    // channels over their rests: an unlisted bone's are passed unread.
    FillRests(s.locals, skeleton, clip->skeletonBoneCount, clip->additive != maniAdditiveNone,
              list);
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        Constants(&s, kind);
        Animated(&s, kind);
    }
    maniPoseMarkStale(pose);
}

#if defined(MANI_CLIP_LANES)
// The lanes past the last bone: the identity, never interpolated.
static void HeldPadding(uint32_t bones, float* groups, float* masks)
{
    static const float kIdentity[MANI_POSE_COMPONENTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                          0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    for (uint32_t bone = bones; bone % MANI_POSE_GROUP_BONES != 0; ++bone)
    {
        maniPoseSlotPut(maniPoseSlot(groups, bone), 0, kIdentity, MANI_POSE_COMPONENTS);
        for (uint32_t c = 0; c < 3; ++c)
        {
            masks[(size_t)(bone / MANI_POSE_GROUP_BONES) * 3 * MANI_POSE_GROUP_BONES +
                  c * MANI_POSE_GROUP_BONES + bone % MANI_POSE_GROUP_BONES] = 0.0f;
        }
    }
}

static float* MaskOf(float* masks, uint32_t bone, uint32_t channel)
{
    return masks + (size_t)(bone / MANI_POSE_GROUP_BONES) * 3 * MANI_POSE_GROUP_BONES +
           channel * MANI_POSE_GROUP_BONES + bone % MANI_POSE_GROUP_BONES;
}

// A held sample (maniDecodeCompressedSample's floats) in a pose's group
// layout for the forward cursor: each channel the clip stores at its
// value (a constant read, a rotation rebuilt), the others at rest as the
// walk writes them, padding lanes the identity; in masks (three rows of
// eight a group: translation, rotation, scale) all bits set where the
// walk interpolates the channel, clear where it writes a rest.
// A stored channel's held value for the groups: the next decoded one
// (animated) or the next constant, a rotation rebuilt.
static void HeldValue(const maniClip* clip, uint32_t channel, uint32_t t, bool animated,
                      const float** cache, const uint8_t** constant, float v[4])
{
    uint32_t width = channel == 1 ? 4 : 3;
    if (animated)
    {
        memcpy(v, *cache, width * sizeof(float));
        *cache += width;
        return;
    }
    float kept[3];
    for (int k = 0; k < 3; ++k)
    {
        kept[k] = LoadF32(*constant + 4 * k);
    }
    *constant += 12;
    if (channel == 1)
    {
        maniCodecRotation(kept, 3u - maniCompressedDroppedField(maniCompressedDropped(clip), t), v);
        return;
    }
    memcpy(v, kept, sizeof kept);
}

void maniExpandCompressedSample(const maniClip* clip, const maniSkeleton* skeleton,
                                const float* cache, float* groups, float* masks)
{
    uint32_t bones = clip->skeletonBoneCount;
    bool identity = clip->additive != maniAdditiveNone;
    for (uint32_t bone = 0; bone < bones; ++bone)
    {
        for (uint32_t c = 0; c < 3; ++c)
        {
            maniSlotRest(maniPoseSlot(groups, bone), skeleton, bone, c, identity);
            *MaskOf(masks, bone, c) = 0.0f;
        }
    }
    const uint8_t* constant = clip->constants;
    uint32_t all = UINT32_MAX;
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        uint32_t channel = maniClipKindChannel(kind);
        for (uint32_t animated = 0; animated < 2; ++animated)
        {
            maniTrackWalk walk = maniWalkTracks(clip, kind, animated);
            uint32_t n = animated != 0 ? clip->animatedKinds[kind] : clip->constantKinds[kind];
            for (uint32_t i = 0; i < n; ++i)
            {
                uint32_t t = maniNextTrack(&walk);
                uint32_t bone = maniClipTrackBone(clip, t);
                float v[4];
                HeldValue(clip, channel, t, animated != 0, &cache, &constant, v);
                maniPoseSlotPut(maniPoseSlot(groups, bone), maniChannelFirst(channel), v,
                                channel == 1 ? 4 : 3);
                memcpy(MaskOf(masks, bone, channel), &all, sizeof all);
            }
        }
    }
    HeldPadding(bones, groups, masks);
}
#endif

void maniSampleCompressedFloats(const maniClip* clip, const maniClipPair* pair, float* values)
{
    size_t channels =
        4 * (size_t)clip->animatedKinds[maniClipKindRotation] +
        3 * (size_t)(clip->animatedChannels - clip->animatedKinds[maniClipKindRotation]);
    bool cached = pair->cacheA != NULL && pair->cacheB != NULL;
    const float* cacheA = cached ? pair->cacheA + channels : NULL;
    const float* cacheB = cached ? pair->cacheB + channels : NULL;
    // The bone channels' constants and ranges come first.
    const uint8_t* constant = clip->constants + 12 * (size_t)clip->constantChannels;
    const uint8_t* ranges = clip->ranges + 24 * (size_t)clip->animatedChannels;
    Walker a = {0};
    Walker b = {0};
    if (!cached)
    {
        a = WalkerAt(clip, pair->first, true);
        b = WalkerAt(clip, pair->second, true);
    }
    const uint8_t* forms = maniCompressedFloatForms(clip);
    uint32_t n = clip->animatedFloats;
    uint32_t j = 0;
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        float va = 0.0f;
        float vb = 0.0f;
        if (forms[i] == maniClipFormConstant)
        {
            va = LoadF32(constant);
            vb = va;
            constant += 4;
        }
        else if (cached)
        {
            va = *cacheA++;
            vb = *cacheB++;
            ++j;
        }
        else
        {
            float minimum = LoadF32(ranges + maniClipRangeAt(n, 1, j, 0, 0));
            float extent = LoadF32(ranges + maniClipRangeAt(n, 1, j, 0, 1));
            va = DecodeFloat(&a, minimum, extent);
            vb = DecodeFloat(&b, minimum, extent);
            ++j;
        }
        bool step =
            maniLoadU32(clip->floatTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 16) != 0;
        float alpha = pair->alpha;
        values[i] = step ? (alpha < 1.0f ? va : vb) : va + (vb - va) * alpha;
    }
}

uint32_t maniCompressedSampleFloats(const maniClip* clip)
{
    uint32_t rotations = clip->animatedKinds[maniClipKindRotation];
    return 4 * rotations + 3 * (clip->animatedChannels - rotations) + clip->animatedFloats;
}

// A group of one kind's animated sub-tracks at a walker decoded into out,
// in order: a rotation rebuilt (four floats), a vector (three). Returns
// out past them.
static float* DecodeGroup(Walker* w, Group* g, const uint8_t* ranges, uint32_t n, bool rotation,
                          float* out)
{
    GroupRanges(g, ranges, n);
    Ranges r;
    if (w->several)
    {
        RangesOf(&g->sa, &r);
    }
    manif8 q[4];
    Decoded(g->qa, &g->sa, w->several ? &r : NULL, g->minimum, g->extent,
            rotation ? g->dropped : NULL, q);
    uint32_t width = rotation ? 4 : 3;
    float rows[4][LANES];
    for (uint32_t k = 0; k < width; ++k)
    {
        maniF8Store(rows[k], q[k]);
    }
    for (uint32_t i = 0; i < g->count; ++i)
    {
        for (uint32_t k = 0; k < width; ++k)
        {
            out[k] = rows[k][i];
        }
        out += width;
    }
    return out;
}

void maniDecodeCompressedSample(const maniClip* clip, uint32_t sample, float* out)
{
    Walker w = WalkerAt(clip, sample, false);
    const uint8_t* ranges = clip->ranges;
    const uint8_t* dropped = maniCompressedDropped(clip);
    Group g;
    memset(&g, 0, sizeof g);
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        uint32_t n = clip->animatedKinds[kind];
        maniTrackWalk walk = maniWalkTracks(clip, kind, 1);
        for (uint32_t first = 0; first < n; first += LANES)
        {
            g.count = 0;
            for (uint32_t i = first; i < n && i < first + LANES; ++i)
            {
                uint32_t t = maniNextTrack(&walk);
                CodeOne(&w, &g.sa, g.qa, g.count);
                g.dropped[g.count] = kDropped[maniCompressedDroppedField(dropped, t)];
                g.entries[g.count++] = i;
            }
            out = DecodeGroup(&w, &g, ranges, n, kind == maniClipKindRotation, out);
        }
        ranges += 24 * (size_t)n;
    }
    uint32_t n = clip->animatedFloats;
    for (uint32_t j = 0; j < n; ++j)
    {
        *out++ = DecodeFloat(&w, LoadF32(ranges + maniClipRangeAt(n, 1, j, 0, 0)),
                             LoadF32(ranges + maniClipRangeAt(n, 1, j, 0, 1)));
    }
}
