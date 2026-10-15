// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Folding and the bit rate search by the shell metric (mani-0006), after
// ACL's v2. Each bone has its own shell (distance and precision) and a
// dominant one, grown leaves first to reach its descendants. A bone
// channel folds to the rest or to its first value when the bone, under
// its unchanged parent, stays within both shells at every sample. Then
// per segment, leaves first, each bone's rate combinations are tried in
// increasing size; one passes when the bone and every descendant (with
// their rates already chosen) stay within both their shells at every
// sample of the segment. The first size that passes ends the search, and
// of its combinations the one with the least room left wins; when none
// passes, the one that misses by least. Errors are measured in binary64
// on exactly what the sampler decodes.

#include "clip_search.h"

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_plan.h"
#include "clip_shell.h"
#include "clip_view.h"

#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdalign.h>
#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

typedef struct Shell
{
    double own;
    double ownPrecision;
    double dominant;
    double dominantPrecision;
    // The farthest its translation reaches from its parent.
    double reach;
} Shell;

// Rate combinations for 1, 2 and 3 sub-tracks, by increasing size.
#define TABLE_SIZES (25u + 625u + 15625u)

typedef struct Search
{
    maniCompressPlan* plan;
    uint32_t bones;
    Shell* shells;
    // A pose, in a block aligned by hand.
    uint8_t* poseBlock;
    size_t poseBlockSize;
    maniPose* pose;
    // Per sample of a segment (or the one sample folding looks at), per
    // bone: the full clip's locals and matrices, the decoded locals and
    // matrices, and matrices relative to the bone searched.
    size_t cells;
    maniTransform* raw;
    maniTransform* decoded;
    maniShellMatrix* rawModels;
    maniShellMatrix* decodedModels;
    maniShellMatrix* relative;
    // Per sub-track of the current segment: range bytes and the 16-bit
    // values of rate 0, six each.
    uint8_t* ranges;
    uint8_t* table;
} Search;

static const uint32_t kTableStart[3] = {0, 25, 25 + 2 * 625};
static const uint32_t kTableCount[3] = {25, 625, 15625};

// The tables: every combination of rates 0 to 24 for 1, 2 and 3
// sub-tracks, stably sorted by their bits (a counting sort).
static void FillTables(uint8_t* table)
{
    for (uint32_t m = 1; m <= 3; ++m)
    {
        uint32_t count[97] = {0};
        uint32_t total = kTableCount[m - 1];
        for (uint32_t e = 0; e < total; ++e)
        {
            uint32_t bits = 0;
            for (uint32_t i = 0, v = e; i < m; ++i, v /= 25)
            {
                bits += maniCodecBits(v % 25);
            }
            ++count[bits];
        }
        uint32_t at = 0;
        for (uint32_t b = 0; b < 97; ++b)
        {
            uint32_t n = count[b];
            count[b] = at;
            at += n;
        }
        for (uint32_t e = 0; e < total; ++e)
        {
            uint8_t rates[3] = {0, 0, 0};
            uint32_t bits = 0;
            for (uint32_t i = 0, v = e; i < m; ++i, v /= 25)
            {
                rates[m - 1 - i] = (uint8_t)(v % 25);
                bits += maniCodecBits(v % 25);
            }
            memcpy(table + kTableStart[m - 1] + (size_t)m * count[bits]++, rates, m);
        }
    }
}

static void Release(Search* s)
{
    const maniAllocator* a = s->plan->allocator;
    maniRelease(a, s->shells, s->bones, sizeof(Shell), alignof(Shell));
    maniRelease(a, s->poseBlock, s->poseBlockSize, 1, 8);
    maniRelease(a, s->raw, 2 * s->cells, sizeof(maniTransform), alignof(maniTransform));
    maniRelease(a, s->rawModels, 3 * s->cells, sizeof(maniShellMatrix), alignof(maniShellMatrix));
    maniRelease(a, s->ranges, 12 * (size_t)s->plan->subCount + 1, 1, 1);
    maniRelease(a, s->table, TABLE_SIZES * 3, 1, 1);
}

static maniResult Make(maniCompressPlan* p, Search* s)
{
    const maniAllocator* a = p->allocator;
    *s = (Search){.plan = p, .bones = maniSkeletonBoneCount(p->skeleton)};
    size_t pose = maniPoseSize(p->skeleton);
    s->poseBlockSize = pose + MANI_POSE_ALIGNMENT;
    s->cells = (size_t)MANI_CLIP_SEGMENT_MAX * s->bones;
    s->shells = maniAllocate(a, s->bones, sizeof(Shell), alignof(Shell));
    s->poseBlock = maniAllocate(a, s->poseBlockSize, 1, 8);
    s->raw = maniAllocate(a, 2 * s->cells, sizeof(maniTransform), alignof(maniTransform));
    s->rawModels = maniAllocate(a, 3 * s->cells, sizeof(maniShellMatrix), alignof(maniShellMatrix));
    s->ranges = maniAllocate(a, 12 * (size_t)p->subCount + 1, 1, 1);
    s->table = maniAllocate(a, TABLE_SIZES * 3, 1, 1);
    if (s->shells == NULL || s->poseBlock == NULL || s->raw == NULL || s->rawModels == NULL ||
        s->ranges == NULL || s->table == NULL)
    {
        return mani_errorCapacity;
    }
    s->decoded = s->raw + s->cells;
    s->decodedModels = s->rawModels + s->cells;
    s->relative = s->rawModels + 2 * s->cells;
    uint8_t* at = s->poseBlock;
    at += (MANI_POSE_ALIGNMENT - (uintptr_t)at % MANI_POSE_ALIGNMENT) % MANI_POSE_ALIGNMENT;
    FillTables(s->table);
    return maniInitPose(p->skeleton, at, pose, &s->pose);
}

// The full clip's sample k into row j: its locals and matrices.
static void LoadRaw(Search* s, uint32_t k, uint32_t j)
{
    const maniCompressPlan* p = s->plan;
    maniSampleClipAt(p->clip, p->skeleton, k, s->pose, NULL);
    maniTransform* raw = s->raw + (size_t)j * s->bones;
    for (uint32_t b = 0; b < s->bones; ++b)
    {
        raw[b] = maniPoseLocal(s->pose, b);
    }
    maniShellModels(p->skeleton, s->pose, s->rawModels + (size_t)j * s->bones);
}

// Each bone's shells: its own from the options, its dominant grown
// leaves first by each child's dominant shell and the farthest the
// child's translation reaches over the clip.
static void Shells(Search* s)
{
    const maniCompressPlan* p = s->plan;
    for (uint32_t b = 0; b < s->bones; ++b)
    {
        double own = (double)maniPlanShell(p, b);
        double precision = (double)maniPlanPrecision(p, b);
        s->shells[b] = (Shell){own, precision, own, precision, 0.0};
    }
    for (uint32_t k = 0; k < p->clip->sampleCount; ++k)
    {
        LoadRaw(s, k, 0);
        for (uint32_t b = 0; b < s->bones; ++b)
        {
            maniVec3 t = s->raw[b].translation;
            double x = (double)t.x;
            double y = (double)t.y;
            double z = (double)t.z;
            double reach = sqrt(x * x + y * y + z * z);
            s->shells[b].reach = reach > s->shells[b].reach ? reach : s->shells[b].reach;
        }
    }
    // Children come after their parents: walking back, a bone's children
    // have all grown it before it grows its parent.
    for (uint32_t b = s->bones; b-- > 0;)
    {
        uint32_t parent = maniSkeletonParent(p->skeleton, b);
        double reach = s->shells[b].dominant + s->shells[b].reach;
        if (parent != MANI_NO_BONE && reach > s->shells[parent].dominant)
        {
            s->shells[parent].dominant = reach;
            s->shells[parent].dominantPrecision = s->shells[b].dominantPrecision;
        }
    }
}

// How far a matrix misses a reference by a bone's shells: the larger of
// the two errors less their precisions (at most 0 within both).
static double Excess(const Shell* shell, const maniShellMatrix* reference, const maniShellMatrix* m)
{
    double own = maniShellError(reference, m, shell->own) - shell->ownPrecision;
    double dominant = maniShellError(reference, m, shell->dominant) - shell->dominantPrecision;
    return own > dominant ? own : dominant;
}

// A bone's matrix from a local, under its parent's matrix in a row.
static maniShellMatrix Under(const Search* s, const maniShellMatrix* row, uint32_t bone,
                             maniTransform local)
{
    maniShellMatrix m = maniShellLocal(local);
    uint32_t parent = maniSkeletonParent(s->plan->skeleton, bone);
    return parent == MANI_NO_BONE ? m : maniShellMul(&row[parent], &m);
}

// A bone's local of row 0 with its sub-tracks before `before` set as
// decided (folded ones to their value, animated ones to themselves).
static maniTransform Folded(const Search* s, uint32_t bone, uint32_t before)
{
    const maniCompressPlan* p = s->plan;
    maniTransform local = s->raw[bone];
    for (uint32_t i = p->boneSubs[bone]; i < before; ++i)
    {
        if (p->subs[i].form != maniClipFormAnimated)
        {
            maniPlanSetChannel(p, &p->subs[i], 0, MANI_CODEC_RAW_RATE, NULL, &local);
        }
    }
    return local;
}

// Folding one channel of every bone track over the clip: for each, whether
// the rest and whether the first value keep the bone within its shells.
static void FoldChannel(Search* s, uint32_t channel, bool* restOk, bool* firstOk)
{
    maniCompressPlan* p = s->plan;
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        restOk[i] = p->subs[i].channel == channel;
        firstOk[i] = restOk[i];
    }
    for (uint32_t k = 0; k < p->clip->sampleCount; ++k)
    {
        LoadRaw(s, k, 0);
        for (uint32_t i = 0; i < p->subCount; ++i)
        {
            maniSubTrack t = p->subs[i];
            if (!restOk[i] && !firstOk[i])
            {
                continue;
            }
            maniTransform base = Folded(s, t.bone, i);
            const Shell* shell = &s->shells[t.bone];
            const maniShellMatrix* reference = &s->rawModels[t.bone];
            maniTransform local = base;
            t.form = maniClipFormDefault;
            maniPlanSetChannel(p, &t, 0, MANI_CODEC_RAW_RATE, NULL, &local);
            maniShellMatrix m = Under(s, s->rawModels, t.bone, local);
            restOk[i] = restOk[i] && Excess(shell, reference, &m) <= 0.0;
            local = base;
            t.form = maniClipFormConstant;
            maniPlanSetChannel(p, &t, 0, MANI_CODEC_RAW_RATE, NULL, &local);
            m = Under(s, s->rawModels, t.bone, local);
            firstOk[i] = firstOk[i] && Excess(shell, reference, &m) <= 0.0;
        }
    }
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        if (p->subs[i].channel == channel)
        {
            p->subs[i].form = restOk[i]    ? maniClipFormDefault
                              : firstOk[i] ? maniClipFormConstant
                                           : maniClipFormAnimated;
        }
    }
}

maniResult maniSearchFold(maniCompressPlan* p)
{
    Search s;
    maniResult r = Make(p, &s);
    size_t flags = 2 * (size_t)p->subCount + 1;
    bool* ok = r == mani_success ? maniAllocate(p->allocator, flags, sizeof(bool), 1) : NULL;
    r = r == mani_success && ok == NULL ? mani_errorCapacity : r;
    if (r == mani_success)
    {
        // Animated until decided, so Folded leaves undecided ones alone.
        for (uint32_t i = 0; i < p->subCount; ++i)
        {
            p->subs[i].form =
                p->subs[i].channel == maniSubFloat ? p->subs[i].form : maniClipFormAnimated;
        }
        Shells(&s);
        for (uint32_t channel = 0; channel < 3; ++channel)
        {
            FoldChannel(&s, channel, ok, ok + p->subCount);
        }
    }
    maniRelease(p->allocator, ok, flags, sizeof(bool), 1);
    Release(&s);
    return r;
}

// The bone being searched in a segment: its animated sub-tracks and the
// end of its subtree.
typedef struct Bone
{
    uint32_t bone;
    uint32_t end;
    uint32_t count;
    uint32_t subs[3];
    uint32_t first;
    uint32_t samples;
} Bone;

static const uint8_t* RangeOf(const Search* s, uint32_t sub, uint32_t rate)
{
    if (rate == 0)
    {
        return s->ranges + 12 * (size_t)sub + 6;
    }
    return s->plan->segmentCount > 1 ? s->ranges + 12 * (size_t)sub : NULL;
}

// Row j's decoded local of the bone with the given rates.
static maniTransform Candidate(const Search* s, const Bone* b, const uint8_t* rates, uint32_t j)
{
    const maniCompressPlan* p = s->plan;
    maniTransform local = s->decoded[(size_t)j * s->bones + b->bone];
    for (uint32_t i = 0; i < b->count; ++i)
    {
        maniPlanSetChannel(p, &p->subs[b->subs[i]], b->first + j, rates[i],
                           RangeOf(s, b->subs[i], rates[i]), &local);
    }
    return local;
}

// How far the bone and its subtree miss their shells at the rates, worst
// over the segment; stops once past the bound.
static double Evaluate(const Search* s, const Bone* b, const uint8_t* rates, double bound)
{
    double worst = -DBL_MAX;
    for (uint32_t j = 0; j < b->samples && worst <= bound; ++j)
    {
        size_t row = (size_t)j * s->bones;
        maniShellMatrix top = Under(s, s->decodedModels + row, b->bone, Candidate(s, b, rates, j));
        for (uint32_t c = b->bone; c < b->end && worst <= bound; ++c)
        {
            maniShellMatrix m = c == b->bone ? top : maniShellMul(&top, &s->relative[row + c]);
            double e = Excess(&s->shells[c], &s->rawModels[row + c], &m);
            worst = e > worst ? e : worst;
        }
    }
    return worst;
}

static bool Allowed(const Search* s, const uint8_t* rates, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (rates[i] == 0 && s->plan->segmentCount == 1)
        {
            return false;
        }
    }
    return true;
}

static uint32_t Bits(const uint8_t* rates, uint32_t count)
{
    uint32_t bits = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        bits += maniCodecBits(rates[i]);
    }
    return bits;
}

// The chosen combination's index in the bone's table.
static uint32_t Choose(const Search* s, const Bone* b)
{
    const uint8_t* table = s->table + kTableStart[b->count - 1];
    uint32_t total = kTableCount[b->count - 1];
    uint32_t best = total - 1;
    double bestExcess = -DBL_MAX;
    uint32_t bestBits = UINT32_MAX;
    for (uint32_t e = 0; e < total; ++e)
    {
        const uint8_t* rates = table + (size_t)e * b->count;
        uint32_t bits = Bits(rates, b->count);
        if (bits > bestBits)
        {
            return best;
        }
        double excess = Allowed(s, rates, b->count) ? Evaluate(s, b, rates, 0.0) : DBL_MAX;
        // Least room left: the passing excess nearest 0.
        if (excess <= 0.0 && excess > bestExcess)
        {
            best = e;
            bestExcess = excess;
            bestBits = bits;
        }
    }
    if (bestBits != UINT32_MAX)
    {
        return best;
    }
    bestExcess = DBL_MAX;
    for (uint32_t e = 0; e < total; ++e)
    {
        const uint8_t* rates = table + (size_t)e * b->count;
        double excess = Allowed(s, rates, b->count) ? Evaluate(s, b, rates, bestExcess) : DBL_MAX;
        if (excess < bestExcess)
        {
            best = e;
            bestExcess = excess;
        }
    }
    return best;
}

// A segment's rows: the full clip's and the decoded (rates at raw),
// locals and matrices, and each sub-track's ranges.
static void LoadSegment(Search* s, uint32_t segment)
{
    const maniCompressPlan* p = s->plan;
    uint32_t first = p->firsts[segment];
    uint32_t samples = p->firsts[segment + 1] - first;
    for (uint32_t j = 0; j < samples; ++j)
    {
        LoadRaw(s, first + j, j);
        size_t row = (size_t)j * s->bones;
        for (uint32_t b = 0; b < s->bones; ++b)
        {
            maniTransform local = s->raw[row + b];
            for (uint32_t i = p->boneSubs[b];
                 i < p->subCount && p->subs[i].bone == b && p->subs[i].channel != maniSubFloat; ++i)
            {
                maniPlanSetChannel(p, &p->subs[i], first + j, MANI_CODEC_RAW_RATE, NULL, &local);
            }
            s->decoded[row + b] = local;
            s->decodedModels[row + b] = Under(s, s->decodedModels + row, b, local);
        }
    }
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        if (p->subs[i].form == maniClipFormAnimated && p->subs[i].channel != maniSubFloat)
        {
            uint8_t* r = s->ranges + 12 * (size_t)i;
            maniPlanSegmentRange(p, &p->subs[i], segment, r, r + 6);
        }
    }
}

// The bone's animated sub-tracks, or a count of 0.
static Bone BoneAt(const Search* s, uint32_t bone, uint32_t segment)
{
    const maniCompressPlan* p = s->plan;
    Bone b = {.bone = bone,
              .end = maniSkeletonSubtreeEnd(p->skeleton, bone),
              .first = p->firsts[segment],
              .samples = p->firsts[segment + 1] - p->firsts[segment]};
    for (uint32_t i = p->boneSubs[bone];
         i < p->subCount && p->subs[i].bone == bone && p->subs[i].channel != maniSubFloat; ++i)
    {
        if (p->subs[i].form == maniClipFormAnimated)
        {
            b.subs[b.count++] = i;
        }
    }
    return b;
}

// The subtree's matrices relative to the bone, from the decoded locals.
static void Relative(Search* s, const Bone* b)
{
    for (uint32_t j = 0; j < b->samples; ++j)
    {
        maniShellMatrix* row = s->relative + (size_t)j * s->bones;
        const maniTransform* decoded = s->decoded + (size_t)j * s->bones;
        for (uint32_t c = b->bone + 1; c < b->end; ++c)
        {
            maniShellMatrix local = maniShellLocal(decoded[c]);
            uint32_t parent = maniSkeletonParent(s->plan->skeleton, c);
            row[c] = parent == b->bone ? local : maniShellMul(&row[parent], &local);
        }
    }
}

// The chosen rates stored, and the bone's decoded locals updated.
static void Keep(Search* s, const Bone* b, const uint8_t* rates, uint32_t segment)
{
    maniCompressPlan* p = s->plan;
    uint32_t n = p->animatedChannels + p->animatedFloats;
    for (uint32_t i = 0; i < b->count; ++i)
    {
        const maniSubTrack* t = &p->subs[b->subs[i]];
        size_t at = (size_t)segment * n + t->animated;
        p->rates[at] = rates[i];
        memcpy(p->segmentRanges + 6 * at,
               s->ranges + 12 * (size_t)b->subs[i] + (rates[i] == 0 ? 6 : 0), 6);
    }
    for (uint32_t j = 0; j < b->samples; ++j)
    {
        s->decoded[(size_t)j * s->bones + b->bone] = Candidate(s, b, rates, j);
    }
}

maniResult maniSearchRates(maniCompressPlan* p)
{
    Search s;
    maniResult r = Make(p, &s);
    if (r == mani_success)
    {
        Shells(&s);
        for (uint32_t segment = 0; segment < p->segmentCount; ++segment)
        {
            LoadSegment(&s, segment);
            for (uint32_t bone = s.bones; bone-- > 0;)
            {
                Bone b = BoneAt(&s, bone, segment);
                if (b.count == 0)
                {
                    continue;
                }
                Relative(&s, &b);
                uint32_t e = Choose(&s, &b);
                Keep(&s, &b, s.table + kTableStart[b.count - 1] + (size_t)e * b.count, segment);
            }
        }
    }
    Release(&s);
    return r;
}
