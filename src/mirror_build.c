// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a mirror map: the rules checked, the reference
// pose checked and taken to model rotations, the bones paired (pairs
// first, then each remaining bone with the bone the first matching rule
// names, which must name it back), the first bone breaking the
// hierarchy's symmetry found, each bone's corrections composed, and the
// names paired (name pairs both ways, then listed names by the rules),
// sorted by hash.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "mirror_file.h"
#include "sort.h"
#include "text.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorInvalid, reason, item, other);
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

static void StoreF32(uint8_t* p, float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    maniStoreU32(p, bits);
}

static void StoreQuat(uint8_t* p, maniQuat q)
{
    StoreF32(p, q.x);
    StoreF32(p + 4, q.y);
    StoreF32(p + 8, q.z);
    StoreF32(p + 12, q.w);
}

// A name pair: the name's hash and bytes, its partner's bytes and the
// source item.
typedef struct Entry
{
    uint64_t hash;
    const char* name;
    const char* partner;
    uint32_t nameLength;
    uint32_t length;
    uint32_t item;
} Entry;

typedef struct Work
{
    const maniMirrorMapSource* s;
    uint32_t count;
    uint32_t* partner;
    uint32_t* candidate;
    maniQuat* model;
    Entry* entries;
    uint32_t entryCount;
    // The names the rules make, and a name being made.
    char* text;
    size_t textUsed;
    char* name;
    uint32_t unmatched;
} Work;

static bool Utf8(const char* text, uint32_t length)
{
    return (text != NULL || length == 0) && maniValidUtf8((const uint8_t*)text, length);
}

static maniBuildResult CheckRules(const maniMirrorMapSource* s)
{
    for (uint32_t i = 0; i < s->ruleCount; ++i)
    {
        const maniMirrorRuleSource* r = &s->rules[i];
        if (r->findLength == 0 || r->place > maniMirrorAnywhere || r->reserved != 0 ||
            !Utf8(r->find, r->findLength) || !Utf8(r->replace, r->replaceLength))
        {
            return Refuse(maniBuildReasonMirrorRule, i, 0);
        }
    }
    return kOk;
}

// The reference pose's model rotations, each local checked.
static maniBuildResult Reference(Work* w)
{
    const maniSkeleton* skeleton = w->s->skeleton;
    for (uint32_t b = 0; b < w->count; ++b)
    {
        maniTransform t =
            w->s->reference != NULL ? w->s->reference[b] : maniSkeletonRest(skeleton, b);
        maniQuat q = t.rotation;
        bool finite = Finite(t.translation.x) && Finite(t.translation.y) &&
                      Finite(t.translation.z) && Finite(t.scale.x) && Finite(t.scale.y) &&
                      Finite(t.scale.z) && Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w);
        float length = maniQuatDot(q, q);
        if (!finite || !(length >= 0.998f && length <= 1.002f))
        {
            return Refuse(maniBuildReasonMirrorReference, b, 0);
        }
        q = maniQuatNormalize(q);
        uint32_t parent = maniSkeletonParent(skeleton, b);
        w->model[b] = parent == MANI_NO_BONE ? q : maniQuatMul(w->model[parent], q);
    }
    return kOk;
}

// A rule's partner name of a name into out; false when it does not
// match.
static bool Apply(const maniMirrorRuleSource* r, const char* name, uint32_t length, char* out,
                  uint32_t* lengthOut)
{
    uint32_t f = r->findLength;
    if (f > length)
    {
        return false;
    }
    uint32_t at = UINT32_MAX;
    if (r->place == maniMirrorPrefix)
    {
        at = memcmp(name, r->find, f) == 0 ? 0u : UINT32_MAX;
    }
    else if (r->place == maniMirrorSuffix)
    {
        at = memcmp(name + length - f, r->find, f) == 0 ? length - f : UINT32_MAX;
    }
    for (uint32_t i = 0; r->place == maniMirrorAnywhere && at == UINT32_MAX && i + f <= length; ++i)
    {
        at = memcmp(name + i, r->find, f) == 0 ? i : UINT32_MAX;
    }
    if (at == UINT32_MAX)
    {
        return false;
    }
    memcpy(out, name, at);
    if (r->replaceLength > 0)
    {
        memcpy(out + at, r->replace, r->replaceLength);
    }
    memcpy(out + at + r->replaceLength, name + at + f, length - at - f);
    *lengthOut = length - f + r->replaceLength;
    return true;
}

// The bone the first matching rule names for bone b; MANI_NO_BONE for
// none.
static uint32_t RuleBone(const Work* w, uint32_t b)
{
    uint32_t length = 0;
    const char* name = maniSkeletonBoneName(w->s->skeleton, b, &length);
    for (uint32_t i = 0; i < w->s->ruleCount; ++i)
    {
        uint32_t made = 0;
        if (Apply(&w->s->rules[i], name, length, w->name, &made))
        {
            uint32_t bone = maniSkeletonFindBone(w->s->skeleton, w->name, made);
            if (bone != MANI_NO_BONE)
            {
                return bone;
            }
        }
    }
    return MANI_NO_BONE;
}

// The explicit pairs.
static maniBuildResult Pairs(Work* w)
{
    for (uint32_t b = 0; b < w->count; ++b)
    {
        w->partner[b] = MANI_NO_BONE;
    }
    for (uint32_t i = 0; i < w->s->pairCount; ++i)
    {
        uint32_t a = w->s->pairs[i].first;
        uint32_t b = w->s->pairs[i].second;
        if (a >= w->count || b >= w->count)
        {
            return Refuse(maniBuildReasonMirrorPair, a < w->count ? b : a, 0);
        }
        if (w->partner[a] != MANI_NO_BONE || w->partner[b] != MANI_NO_BONE)
        {
            uint32_t twice = w->partner[a] != MANI_NO_BONE ? a : b;
            return Refuse(maniBuildReasonMirrorPair, twice, w->partner[twice]);
        }
        w->partner[a] = b;
        w->partner[b] = a;
    }
    return kOk;
}

// The rules for the bones the pairs leave: each bone with the bone its
// first matching rule names, which must name it back (a paired bone
// names none), or itself.
static maniBuildResult RulePairs(Work* w)
{
    for (uint32_t b = 0; b < w->count; ++b)
    {
        w->candidate[b] = w->partner[b] == MANI_NO_BONE ? RuleBone(w, b) : MANI_NO_BONE;
    }
    for (uint32_t b = 0; b < w->count; ++b)
    {
        uint32_t c = w->candidate[b];
        if (w->partner[b] != MANI_NO_BONE)
        {
            continue;
        }
        if (c == MANI_NO_BONE || c == b)
        {
            w->partner[b] = b;
            continue;
        }
        if (w->candidate[c] != b)
        {
            return Refuse(maniBuildReasonMirrorPair, b, c);
        }
        w->partner[b] = c;
        w->partner[c] = b;
    }
    return kOk;
}

// Whether bone b's parent's partner is not its partner's parent.
static bool BoneUnmatched(const Work* w, uint32_t b)
{
    uint32_t p = maniSkeletonParent(w->s->skeleton, b);
    uint32_t q = maniSkeletonParent(w->s->skeleton, w->partner[b]);
    return p == MANI_NO_BONE ? q != MANI_NO_BONE : q != w->partner[p];
}

// The first bone whose parent's partner is not its partner's parent.
static uint32_t Unmatched(const Work* w)
{
    for (uint32_t b = 0; b < w->count; ++b)
    {
        if (BoneUnmatched(w, b))
        {
            return b;
        }
    }
    return MANI_NO_BONE;
}

static maniBuildResult CheckName(const char* name, uint32_t length, uint32_t item)
{
    return length > 0 && Utf8(name, length) ? kOk : Refuse(maniBuildReasonMirrorName, item, 0);
}

static void AddEntry(Work* w, const char* name, uint32_t length, const char* partner,
                     uint32_t partnerLength, uint32_t item)
{
    w->entries[w->entryCount++] =
        (Entry){maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length),
                name,
                partner,
                length,
                partnerLength,
                item};
}

// The name pairs both ways, then each listed name the rules pair.
static maniBuildResult Names(Work* w)
{
    const maniMirrorMapSource* s = w->s;
    for (uint32_t i = 0; i < s->namePairCount; ++i)
    {
        const maniMirrorNamePairSource* p = &s->namePairs[i];
        maniBuildResult r = CheckName(p->first, p->firstLength, i);
        r = r.result == mani_success ? CheckName(p->second, p->secondLength, i) : r;
        if (r.result != mani_success)
        {
            return r;
        }
        AddEntry(w, p->first, p->firstLength, p->second, p->secondLength, i);
        AddEntry(w, p->second, p->secondLength, p->first, p->firstLength, i);
    }
    for (uint32_t j = 0; j < s->nameCount; ++j)
    {
        const maniMirrorNameSource* n = &s->names[j];
        uint32_t item = s->namePairCount + j;
        maniBuildResult r = CheckName(n->name, n->nameLength, item);
        if (r.result != mani_success || n->reserved != 0)
        {
            return Refuse(maniBuildReasonMirrorName, item, 0);
        }
        char* out = w->text + w->textUsed;
        uint32_t made = 0;
        for (uint32_t i = 0; i < s->ruleCount; ++i)
        {
            if (Apply(&s->rules[i], n->name, n->nameLength, out, &made))
            {
                // A rule that empties the name makes no partner.
                if (made == 0)
                {
                    return Refuse(maniBuildReasonMirrorName, item, 0);
                }
                AddEntry(w, n->name, n->nameLength, out, made, item);
                w->textUsed += made;
                break;
            }
        }
    }
    return kOk;
}

// The entries by hash, a name's repeats with the same partner dropped;
// a refusal for one with another partner, and for two names of one hash.
// order holds the sorted indices.
static maniBuildResult Sorted(Work* w, uint32_t* order, uint32_t* scratch, uint64_t* keys)
{
    for (uint32_t i = 0; i < w->entryCount; ++i)
    {
        keys[i] = w->entries[i].hash;
    }
    maniSortByKey(keys, w->entryCount, order, scratch);
    uint32_t kept = 0;
    for (uint32_t i = 0; i < w->entryCount; ++i)
    {
        const Entry* e = &w->entries[order[i]];
        const Entry* last = kept > 0 ? &w->entries[order[kept - 1]] : NULL;
        if (last != NULL && last->hash == e->hash)
        {
            if (last->nameLength != e->nameLength ||
                memcmp(last->name, e->name, e->nameLength) != 0)
            {
                return Refuse(maniBuildReasonNameHash, e->item, last->item);
            }
            bool same =
                last->length == e->length && memcmp(last->partner, e->partner, e->length) == 0;
            if (!same)
            {
                return Refuse(maniBuildReasonMirrorName, e->item, last->item);
            }
            continue;
        }
        order[kept++] = order[i];
    }
    w->entryCount = kept;
    return kOk;
}

// mirror(q): the two vector components off the normal negated.
static maniQuat Mirror(maniQuat q, maniMirrorAxis axis)
{
    return (maniQuat){axis == maniMirrorAxisX ? q.x : -q.x, axis == maniMirrorAxisY ? q.y : -q.y,
                      axis == maniMirrorAxisZ ? q.z : -q.z, q.w};
}

// How far an entry of a turn's matrix may be from 0 or 1 in magnitude
// for the turn to take axes to axes.
static const double kAxisTolerance = 1e-4;

static double Magnitude(double v)
{
    return v < 0.0 ? -v : v;
}

// A correction's scale flags: each axis i's source p(i) XOR i, two bits
// each, when the turn takes axes to axes; MANI_MIRROR_FLAG_APPROXIMATE
// otherwise. Column i of the turn's matrix holds where axis i goes.
static uint32_t ScaleFlags(maniQuat c)
{
    double x = (double)c.x;
    double y = (double)c.y;
    double z = (double)c.z;
    double w = (double)c.w;
    const double r[3][3] = {
        {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)},
        {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)},
        {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}};
    uint32_t flags = 0;
    uint32_t used = 0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        uint32_t source = 3;
        for (uint32_t j = 0; j < 3; ++j)
        {
            double v = Magnitude(r[j][i]);
            if (v > 1.0 - kAxisTolerance)
            {
                source = j;
            }
            else if (v >= kAxisTolerance)
            {
                return MANI_MIRROR_FLAG_APPROXIMATE;
            }
        }
        if (source == 3 || ((used >> source) & 1u) != 0)
        {
            return MANI_MIRROR_FLAG_APPROXIMATE;
        }
        used |= 1u << source;
        flags |= (source ^ i) << (2 * i);
    }
    return flags;
}

static void LayBones(const Work* w, uint8_t* bones)
{
    const maniSkeleton* skeleton = w->s->skeleton;
    for (uint32_t b = 0; b < w->count; ++b)
    {
        uint32_t m = w->partner[b];
        uint32_t pb = maniSkeletonParent(skeleton, b);
        uint32_t pm = maniSkeletonParent(skeleton, m);
        maniQuat parentB = pb == MANI_NO_BONE ? maniQuatIdentity() : w->model[pb];
        maniQuat parentM = pm == MANI_NO_BONE ? maniQuatIdentity() : w->model[pm];
        maniQuat a = maniQuatMul(maniQuatConjugate(parentB), Mirror(parentM, w->s->axis));
        maniQuat c = maniQuatMul(maniQuatConjugate(Mirror(w->model[m], w->s->axis)), w->model[b]);
        uint8_t* e = bones + (size_t)b * MANI_MIRROR_BONE_SIZE;
        c = maniQuatNormalize(c);
        maniStoreU32(e, m);
        maniStoreU32(e + 4,
                     ScaleFlags(c) | (BoneUnmatched(w, b) ? MANI_MIRROR_FLAG_UNMATCHED : 0u));
        StoreQuat(e + 8, maniQuatNormalize(a));
        StoreQuat(e + 24, c);
    }
}

static void LayHeader(const Work* w, uint8_t* h)
{
    maniStoreU64(h, maniSkeletonIdentity(w->s->skeleton));
    maniStoreU32(h + 8, w->count);
    maniStoreU32(h + 12, w->s->axis);
    maniStoreU32(h + 16, w->entryCount);
    maniStoreU32(h + 20, w->unmatched);
}

static void LayNames(const Work* w, const uint32_t* order, uint8_t* names, uint8_t* strings)
{
    uint32_t at = 0;
    for (uint32_t i = 0; i < w->entryCount; ++i)
    {
        const Entry* e = &w->entries[order[i]];
        uint8_t* p = names + (size_t)i * MANI_MIRROR_NAME_SIZE;
        maniStoreU64(p, e->hash);
        maniStoreU64(p + 8, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)e->partner, e->length));
        maniStoreU32(p + 16, at);
        maniStoreU32(p + 20, e->length);
        memcpy(strings + at, e->partner, e->length);
        at += e->length;
    }
}

static maniBuildResult Write(maniBuilder* builder, const Work* w, const uint32_t* order)
{
    uint64_t text = 0;
    for (uint32_t i = 0; i < w->entryCount; ++i)
    {
        text += w->entries[order[i]].length;
    }
    const uint64_t sizes[MANI_MIRROR_SECTIONS] = {
        MANI_MIRROR_HEADER_SIZE, (uint64_t)w->count * MANI_MIRROR_BONE_SIZE,
        (uint64_t)w->entryCount * MANI_MIRROR_NAME_SIZE, text};
    size_t own = (size_t)(sizes[0] + sizes[1] + sizes[2] + sizes[3]);
    uint8_t* staging = maniAllocate(&builder->allocator, own, 1, 8);
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(staging, 0, own);
    uint8_t* data[MANI_MIRROR_SECTIONS] = {staging, staging + sizes[0],
                                           staging + sizes[0] + sizes[1],
                                           staging + sizes[0] + sizes[1] + sizes[2]};
    const uint8_t* views[MANI_MIRROR_SECTIONS] = {data[0], data[1], data[2], data[3]};
    LayHeader(w, data[0]);
    LayBones(w, data[1]);
    LayNames(w, order, data[2], data[3]);
    // Empty sections are left out of the table.
    maniSectionSource sections[MANI_MIRROR_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_MIRROR_SECTIONS; ++i)
    {
        sections[count] = (maniSectionSource){i + 1, data[i], sizes[i]};
        count += sizes[i] > 0 ? 1u : 0u;
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindMirrorMap, MANI_MIRROR_FORMAT_VERSION,
                           maniMirrorComputeIdentity(views, sizes), sections, count);
        maniBuilderKeep(builder, maniKindMirrorMap, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, own, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

static bool Usable(const maniMirrorMapSource* s)
{
    return s != NULL && s->skeleton != NULL && (s->pairs != NULL || s->pairCount == 0) &&
           (s->rules != NULL || s->ruleCount == 0) &&
           (s->namePairs != NULL || s->namePairCount == 0) &&
           (s->names != NULL || s->nameCount == 0) && s->reserved == 0;
}

// The longest name a rule can make from a bone's or a listed name, and
// the room every listed name's partner takes.
static void Room(const maniMirrorMapSource* s, uint32_t count, size_t* nameOut, size_t* textOut)
{
    size_t replace = 0;
    for (uint32_t i = 0; i < s->ruleCount; ++i)
    {
        replace = s->rules[i].replaceLength > replace ? s->rules[i].replaceLength : replace;
    }
    size_t longest = 0;
    for (uint32_t b = 0; b < count; ++b)
    {
        uint32_t length = 0;
        (void)maniSkeletonBoneName(s->skeleton, b, &length);
        longest = length > longest ? length : longest;
    }
    size_t text = 0;
    for (uint32_t j = 0; j < s->nameCount; ++j)
    {
        text += (size_t)s->names[j].nameLength + replace;
    }
    *nameOut = longest + replace;
    *textOut = text;
}

typedef struct Block
{
    void* p;
    size_t count;
    size_t size;
} Block;

static maniBuildResult Build(maniBuilder* builder, Work* w)
{
    maniBuildResult r = CheckRules(w->s);
    r = r.result == mani_success ? Reference(w) : r;
    r = r.result == mani_success ? Pairs(w) : r;
    r = r.result == mani_success ? RulePairs(w) : r;
    r = r.result == mani_success ? Names(w) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    w->unmatched = Unmatched(w);
    size_t n = w->entryCount;
    uint32_t* order = maniAllocate(&builder->allocator, 2 * n + 1, sizeof(uint32_t), 8);
    uint64_t* keys = maniAllocate(&builder->allocator, n + 1, sizeof(uint64_t), 8);
    r = order != NULL && keys != NULL ? Sorted(w, order, order + n, keys)
                                      : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    r = r.result == mani_success ? Write(builder, w, order) : r;
    maniRelease(&builder->allocator, order, 2 * n + 1, sizeof(uint32_t), 8);
    maniRelease(&builder->allocator, keys, n + 1, sizeof(uint64_t), 8);
    return r;
}

maniBuildResult maniBuildMirrorMap(maniBuilder* builder, const maniMirrorMapSource* source)
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
    if (source->axis > maniMirrorAxisZ)
    {
        return Refuse(maniBuildReasonOption, 0, 0);
    }
    uint32_t count = maniSkeletonBoneCount(source->skeleton);
    size_t nameRoom = 0;
    size_t textRoom = 0;
    Room(source, count, &nameRoom, &textRoom);
    size_t entries = 2 * (size_t)source->namePairCount + source->nameCount;
    const maniAllocator* a = &builder->allocator;
    Block blocks[6] = {{NULL, count, sizeof(uint32_t)},    {NULL, count, sizeof(uint32_t)},
                       {NULL, count, sizeof(maniQuat)},    {NULL, entries + 1, sizeof(Entry)},
                       {NULL, textRoom + 1, sizeof(char)}, {NULL, nameRoom + 1, sizeof(char)}};
    bool made = true;
    for (uint32_t i = 0; i < 6; ++i)
    {
        blocks[i].p = maniAllocate(a, blocks[i].count, blocks[i].size, 8);
        made = made && blocks[i].p != NULL;
    }
    Work w = {source, count,       blocks[0].p, blocks[1].p, blocks[2].p, blocks[3].p,
              0,      blocks[4].p, 0,           blocks[5].p, MANI_NO_BONE};
    maniBuildResult r =
        made ? Build(builder, &w) : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    for (uint32_t i = 0; i < 6; ++i)
    {
        maniRelease(a, blocks[i].p, blocks[i].count, blocks[i].size, 8);
    }
    return r;
}
