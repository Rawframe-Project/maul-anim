// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening checks a skeleton file without scratch memory. Depth-first
// order holds when each bone's parent is the previous bone or one of its
// ancestors; walking up from the previous bone to the new parent closes
// the subtrees the new bone leaves, each bone exactly once over the whole
// file, so the walk is linear and checks every stored subtree end as it
// closes it. Names are unique when the stored name order has strictly
// increasing hashes, which also makes lookups a binary search.

#include "maul-anim/skeleton.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "invariant.h"
#include "skeleton_file.h"
#include "skeleton_view.h"
#include "text.h"
#include "view.h"

#include "maul-anim/base.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static maniFileResult Success(void)
{
    return (maniFileResult){mani_success, 0, 0};
}

// A section's place: its bytes and its offset in the file.
typedef struct Section
{
    const uint8_t* data;
    uint64_t offset;
    uint64_t size;
} Section;

// The seven sections, tags 1 to 7 in order, each of the size the bone
// count gives.
static maniFileResult FindSections(const maniContainer* c, Section sections[MANI_SKELETON_SECTIONS],
                                   uint32_t* countOut)
{
    if (c->sectionCount != MANI_SKELETON_SECTIONS)
    {
        return Fail(12);
    }
    for (uint32_t i = 0; i < MANI_SKELETON_SECTIONS; ++i)
    {
        if (!maniContainerSection(c, i + 1, &sections[i].data, &sections[i].size,
                                  &sections[i].offset))
        {
            return Fail(MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE);
        }
    }
    uint64_t parentsSize = sections[0].size;
    if (parentsSize == 0 || parentsSize % 2 != 0 || parentsSize / 2 > MANI_MAX_BONES)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + 16);
    }
    uint64_t n = parentsSize / 2;
    const uint64_t sizes[MANI_SKELETON_SECTIONS - 1] = {2 * n, MANI_SKELETON_REST_SIZE * n, 8 * n,
                                                        2 * n, MANI_SKELETON_SPAN_SIZE * n};
    for (uint32_t i = 1; i < MANI_SKELETON_SECTIONS - 1; ++i)
    {
        if (sections[i].size != sizes[i - 1])
        {
            return Fail(MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE + 16);
        }
    }
    *countOut = (uint32_t)n;
    return Success();
}

static uint32_t Parent(const Section* parents, uint32_t bone)
{
    return maniLoadU16(parents->data + 2 * (uint64_t)bone);
}

static uint32_t End(const Section* ends, uint32_t bone)
{
    return maniLoadU16(ends->data + 2 * (uint64_t)bone);
}

static maniFileResult CheckHierarchy(const Section* parents, const Section* ends, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t p = Parent(parents, i);
        if (p != MANI_NO_BONE && p >= i)
        {
            return Fail(parents->offset + 2 * (uint64_t)i);
        }
        if (i == 0)
        {
            continue;
        }
        // Close the subtrees between the previous bone and the parent.
        uint32_t b = i - 1;
        while (b != p)
        {
            if (b == MANI_NO_BONE)
            {
                return Fail(parents->offset + 2 * (uint64_t)i);
            }
            if (End(ends, b) != i)
            {
                return Fail(ends->offset + 2 * (uint64_t)b);
            }
            b = Parent(parents, b);
        }
    }
    for (uint32_t b = n - 1; b != MANI_NO_BONE; b = Parent(parents, b))
    {
        if (End(ends, b) != n)
        {
            return Fail(ends->offset + 2 * (uint64_t)b);
        }
    }
    return Success();
}

MANI_NODISCARD static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static maniFileResult CheckRests(const Section* rests, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        float values[10];
        memcpy(values, rests->data + (uint64_t)i * MANI_SKELETON_REST_SIZE, sizeof values);
        for (uint32_t k = 0; k < 10; ++k)
        {
            if (!Finite(values[k]))
            {
                return Fail(rests->offset + (uint64_t)i * MANI_SKELETON_REST_SIZE + 4 * k);
            }
        }
        double x = (double)values[3];
        double y = (double)values[4];
        double z = (double)values[5];
        double w = (double)values[6];
        double squared = x * x + y * y + z * z + w * w;
        if (fabs(squared - 1.0) > 0x1p-20)
        {
            return Fail(rests->offset + (uint64_t)i * MANI_SKELETON_REST_SIZE + 12);
        }
    }
    return Success();
}

static maniFileResult CheckNames(const Section* spans, const Section* names, const Section* hashes,
                                 uint32_t n)
{
    uint64_t next = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        const uint8_t* span = spans->data + (uint64_t)i * MANI_SKELETON_SPAN_SIZE;
        uint64_t offset = maniLoadU32(span);
        uint64_t length = maniLoadU32(span + 4);
        if (offset != next)
        {
            return Fail(spans->offset + (uint64_t)i * MANI_SKELETON_SPAN_SIZE);
        }
        if (length == 0 || length > names->size - offset)
        {
            return Fail(spans->offset + (uint64_t)i * MANI_SKELETON_SPAN_SIZE + 4);
        }
        if (!maniValidUtf8(names->data + offset, (size_t)length))
        {
            return Fail(names->offset + offset);
        }
        if (maniHashBytes(MANI_HASH_INIT, names->data + offset, length) !=
            maniLoadU64(hashes->data + 8 * (uint64_t)i))
        {
            return Fail(hashes->offset + 8 * (uint64_t)i);
        }
        next = offset + length;
    }
    if (next != names->size)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + 6 * MANI_CONTAINER_ENTRY_SIZE + 16);
    }
    return Success();
}

static maniFileResult CheckOrder(const Section* order, const Section* hashes, uint32_t n)
{
    uint64_t last = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        uint32_t bone = maniLoadU16(order->data + 2 * (uint64_t)k);
        if (bone >= n)
        {
            return Fail(order->offset + 2 * (uint64_t)k);
        }
        uint64_t hash = maniLoadU64(hashes->data + 8 * (uint64_t)bone);
        if (k > 0 && hash <= last)
        {
            return Fail(order->offset + 2 * (uint64_t)k);
        }
        last = hash;
    }
    return Success();
}

uint64_t maniSkeletonComputeIdentity(uint32_t boneCount, const uint8_t* parents,
                                     const uint8_t* rests, const uint8_t* spans,
                                     const uint8_t* names, uint64_t nameSize)
{
    uint8_t count[4];
    maniStoreU32(count, boneCount);
    uint64_t hash = maniHash64(MANI_HASH_INIT, count, sizeof count);
    hash = maniHashBytes(hash, parents, 2 * (uint64_t)boneCount);
    hash = maniHashBytes(hash, rests, MANI_SKELETON_REST_SIZE * (uint64_t)boneCount);
    hash = maniHashBytes(hash, spans, MANI_SKELETON_SPAN_SIZE * (uint64_t)boneCount);
    return maniHashBytes(hash, names, nameSize);
}

static MANI_COLD MANI_NOINLINE maniFileResult OpenSkeletonWork(const void* bytes, size_t size,
                                                               void* out)
{
    maniSkeleton* skeletonOut = out;
    if (skeletonOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindSkeleton,
                                         MANI_SKELETON_FORMAT_VERSION, MANI_SKELETON_SECTIONS, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    Section s[MANI_SKELETON_SECTIONS];
    uint32_t n = 0;
    r = FindSections(&c, s, &n);
    if (r.result == mani_success)
    {
        r = CheckHierarchy(&s[0], &s[1], n);
    }
    if (r.result == mani_success)
    {
        r = CheckRests(&s[2], n);
    }
    if (r.result == mani_success)
    {
        r = CheckNames(&s[5], &s[6], &s[3], n);
    }
    if (r.result == mani_success)
    {
        r = CheckOrder(&s[4], &s[3], n);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniSkeletonComputeIdentity(n, s[0].data, s[2].data, s[5].data, s[6].data, s[6].size) !=
        c.identity)
    {
        return Fail(40);
    }
    memset(skeletonOut, 0, sizeof *skeletonOut);
    skeletonOut->base = c.bytes;
    skeletonOut->size = c.size;
    for (uint32_t i = 0; i < MANI_SKELETON_SECTIONS; ++i)
    {
        skeletonOut->sections[i] = s[i].offset;
    }
    skeletonOut->nameSize = s[6].size;
    skeletonOut->identity = c.identity;
    skeletonOut->boneCount = n;
    skeletonOut->cookie = MANI_VIEW_COOKIE(maniKindSkeleton);
    return Success();
}

maniFileResult maniOpenSkeleton(const void* bytes, size_t size, maniSkeleton* skeletonOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = maniContainerOpenTagged(bytes, size, skeletonOut, OpenSkeletonWork);
    maniFloatEnvLeave(env);
    return result;
}

MANI_NOINLINE bool maniSkeletonValid(const maniSkeleton* s)
{
    // Each fixed section's bytes a bone; the names' are their own size.
    static const uint8_t kStrides[MANI_SKELETON_SECTIONS - 1] = {2, 2, MANI_SKELETON_REST_SIZE,
                                                                 8, 2, MANI_SKELETON_SPAN_SIZE};
    if (s == NULL || s->cookie != MANI_VIEW_COOKIE(maniKindSkeleton) || s->base == NULL ||
        s->boneCount - 1u >= MANI_MAX_BONES)
    {
        return false;
    }
    bool fits = maniViewFits(s->sections[MANI_SKELETON_SECTIONS - 1], s->nameSize, s->size);
    for (uint32_t i = 0; i < MANI_SKELETON_SECTIONS - 1; ++i)
    {
        fits = fits && maniViewFits(s->sections[i], (uint64_t)kStrides[i] * s->boneCount, s->size);
    }
    return fits;
}

uint32_t maniSkeletonParentOf(const maniSkeleton* s, uint32_t bone)
{
    if (bone >= s->boneCount)
    {
        return MANI_NO_BONE;
    }
    uint32_t parent = maniLoadU16(maniSkeletonSection(s, maniSkeletonParents) + 2 * (size_t)bone);
    return parent < bone ? parent : MANI_NO_BONE;
}

maniTransform maniSkeletonRestOf(const maniSkeleton* s, uint32_t bone)
{
    maniTransform rest = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    if (bone < s->boneCount)
    {
        memcpy(&rest,
               maniSkeletonSection(s, maniSkeletonRests) + MANI_SKELETON_REST_SIZE * (size_t)bone,
               sizeof rest);
    }
    return rest;
}

uint32_t maniSkeleton_GetBoneCount(const maniSkeleton* skeleton)
{
    return maniSkeletonValid(skeleton) ? skeleton->boneCount : 0;
}

uint64_t maniSkeleton_GetIdentity(const maniSkeleton* skeleton)
{
    return maniSkeletonValid(skeleton) ? skeleton->identity : 0;
}

// A u16 bone index from a section, MANI_NO_BONE when it is not below the
// view's count: the bytes may have changed since the open.
static uint32_t BoneAt(const maniSkeleton* s, uint32_t section, uint32_t i)
{
    uint32_t bone = maniLoadU16(maniSkeletonSection(s, section) + 2 * (size_t)i);
    return bone < s->boneCount ? bone : MANI_NO_BONE;
}

uint32_t maniSkeleton_GetParent(const maniSkeleton* skeleton, uint32_t bone)
{
    return maniSkeletonValid(skeleton) ? maniSkeletonParentOf(skeleton, bone) : MANI_NO_BONE;
}

uint32_t maniSkeleton_GetSubtreeEnd(const maniSkeleton* skeleton, uint32_t bone)
{
    if (!maniSkeletonValid(skeleton) || bone >= skeleton->boneCount)
    {
        return bone;
    }
    uint32_t end =
        maniLoadU16(maniSkeletonSection(skeleton, maniSkeletonSubtreeEnds) + 2 * (size_t)bone);
    return end > bone && end <= skeleton->boneCount ? end : bone;
}

maniTransform maniSkeleton_GetRest(const maniSkeleton* skeleton, uint32_t bone)
{
    if (!maniSkeletonValid(skeleton))
    {
        return (maniTransform){{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    }
    return maniSkeletonRestOf(skeleton, bone);
}

// A bone's name in a checked view, NULL past the count or when its span
// leaves the names' bytes.
static const char* Name(const maniSkeleton* s, uint32_t bone, size_t* lengthOut)
{
    *lengthOut = 0;
    if (bone >= s->boneCount)
    {
        return NULL;
    }
    const uint8_t* span =
        maniSkeletonSection(s, maniSkeletonNameSpans) + MANI_SKELETON_SPAN_SIZE * (size_t)bone;
    uint32_t offset = maniLoadU32(span);
    uint32_t length = maniLoadU32(span + 4);
    if (!maniViewFits(offset, length, s->nameSize))
    {
        return NULL;
    }
    *lengthOut = length;
    return (const char*)(maniSkeletonSection(s, maniSkeletonNameBytes) + offset);
}

const char* maniSkeleton_GetBoneName(const maniSkeleton* skeleton, uint32_t bone, size_t* lengthOut)
{
    size_t length = 0;
    const char* name = maniSkeletonValid(skeleton) ? Name(skeleton, bone, &length) : NULL;
    if (lengthOut != NULL)
    {
        *lengthOut = length;
    }
    return name;
}

uint32_t maniSkeleton_FindBone(const maniSkeleton* skeleton, const char* name, size_t length)
{
    if (!maniSkeletonValid(skeleton))
    {
        return MANI_NO_BONE;
    }
    const maniSkeleton* s = skeleton;
    // An empty name hashes to MANI_HASH_INIT and matches no bone, whose
    // names are never empty, so it needs no case of its own.
    uint64_t hash = maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length);
    uint32_t low = 0;
    uint32_t high = s->boneCount;
    while (low < high)
    {
        uint32_t middle = low + (high - low) / 2;
        uint32_t bone = BoneAt(s, maniSkeletonNameOrder, middle);
        if (bone == MANI_NO_BONE)
        {
            return MANI_NO_BONE;
        }
        uint64_t found =
            maniLoadU64(maniSkeletonSection(s, maniSkeletonNameHashes) + 8 * (size_t)bone);
        if (found < hash)
        {
            low = middle + 1;
        }
        else if (found > hash)
        {
            high = middle;
        }
        else
        {
            size_t foundLength = 0;
            const char* foundName = Name(s, bone, &foundLength);
            bool same = foundName != NULL && foundLength == length &&
                        (length == 0 || memcmp(foundName, name, length) == 0);
            return same ? bone : MANI_NO_BONE;
        }
    }
    return MANI_NO_BONE;
}
