// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a skeleton file (mani-0003): every bone checked, the bones
// ordered depth-first by an explicit stack (children and roots in the
// source's order, so the order is the source's own where it already is
// depth-first), rotations normalized in binary64 and rounded once, names
// sorted by hash with a stable merge sort, so equal hashes meet and are
// told apart as duplicates or collisions.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "skeleton_file.h"
#include "text.h"

#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdckdint.h>
#include <string.h>

// The scratch a build uses: arrays of n u32 each.
enum
{
    ScratchFirstChild,
    ScratchLastChild,
    ScratchNextSibling,
    ScratchStack,
    ScratchOrder,
    ScratchSorted,
    ScratchMerge,
    ScratchArrays,
};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static maniBuildResult Outcome(maniResult result)
{
    return (maniBuildResult){result, maniBuildReasonNone, 0, 0};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static maniBuildResult CheckBones(const maniSkeletonSource* source)
{
    uint32_t n = source->boneCount;
    if (n == 0 || n > MANI_MAX_BONES || source->bones == NULL)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    for (uint32_t i = 0; i < n; ++i)
    {
        const maniBoneSource* bone = &source->bones[i];
        if (bone->parent < -1 || bone->parent >= (int64_t)n || bone->parent == (int64_t)i)
        {
            return Refuse(maniBuildReasonParent, i, 0);
        }
        if (bone->name == NULL || bone->nameLength == 0 ||
            !maniValidUtf8((const uint8_t*)bone->name, bone->nameLength))
        {
            return Refuse(maniBuildReasonName, i, 0);
        }
        const maniTransform* r = &bone->rest;
        const float values[10] = {
            r->translation.x, r->translation.y, r->translation.z, r->rotation.x, r->rotation.y,
            r->rotation.z,    r->rotation.w,    r->scale.x,       r->scale.y,    r->scale.z};
        for (uint32_t k = 0; k < 10; ++k)
        {
            if (!Finite(values[k]))
            {
                return Refuse(maniBuildReasonNotFinite, i, 0);
            }
        }
        if (r->rotation.x == 0.0f && r->rotation.y == 0.0f && r->rotation.z == 0.0f &&
            r->rotation.w == 0.0f)
        {
            return Refuse(maniBuildReasonRotation, i, 0);
        }
    }
    return Outcome(mani_success);
}

// The lowest bone the depth-first walk did not place: it has ancestors
// forever. marks is scratch of n entries.
static uint32_t LowestUnplaced(const uint32_t* order, uint32_t placed, uint32_t n, uint32_t* marks)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        marks[i] = 0;
    }
    for (uint32_t k = 0; k < placed; ++k)
    {
        marks[order[k]] = 1;
    }
    uint32_t bone = 0;
    while (marks[bone] != 0)
    {
        ++bone;
    }
    return bone;
}

// Depth-first order of the source's bones into order (source indices by
// position); false when some bone sits on a cycle, with that bone.
static bool OrderDepthFirst(const maniSkeletonSource* source, uint32_t* scratch[ScratchArrays],
                            uint32_t* cycleBoneOut)
{
    uint32_t n = source->boneCount;
    uint32_t* first = scratch[ScratchFirstChild];
    uint32_t* last = scratch[ScratchLastChild];
    uint32_t* next = scratch[ScratchNextSibling];
    uint32_t* stack = scratch[ScratchStack];
    uint32_t* order = scratch[ScratchOrder];
    const uint32_t none = UINT32_MAX;
    for (uint32_t i = 0; i < n; ++i)
    {
        first[i] = none;
        last[i] = none;
        next[i] = none;
    }
    // Children linked in source order; roots pushed in reverse so the
    // first root is visited first.
    for (uint32_t i = 0; i < n; ++i)
    {
        int32_t parent = source->bones[i].parent;
        if (parent < 0)
        {
            continue;
        }
        uint32_t p = (uint32_t)parent;
        if (last[p] == none)
        {
            first[p] = i;
        }
        else
        {
            next[last[p]] = i;
        }
        last[p] = i;
    }
    uint32_t top = 0;
    for (uint32_t i = n; i-- > 0;)
    {
        if (source->bones[i].parent < 0)
        {
            stack[top++] = i;
        }
    }
    uint32_t placed = 0;
    while (top > 0)
    {
        uint32_t bone = stack[--top];
        order[placed++] = bone;
        // Push the children last to first, so the first is visited next.
        // A bone is on the stack once, so the stack never holds more than
        // n entries.
        uint32_t count = 0;
        for (uint32_t c = first[bone]; c != none; c = next[c])
        {
            ++count;
        }
        top += count;
        uint32_t at = top;
        for (uint32_t c = first[bone]; c != none; c = next[c])
        {
            stack[--at] = c;
        }
    }
    if (placed == n)
    {
        return true;
    }
    *cycleBoneOut = LowestUnplaced(order, placed, n, last);
    return false;
}

static uint64_t NameHash(const maniBoneSource* bone)
{
    return maniHashBytes(MANI_HASH_INIT, (const uint8_t*)bone->name, bone->nameLength);
}

// Sorts the skeleton's bones by name hash, stable, bottom-up.
static void SortByHash(const uint64_t* hashes, uint32_t n, uint32_t* sorted, uint32_t* merge)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        sorted[i] = i;
    }
    uint32_t* from = sorted;
    uint32_t* to = merge;
    for (uint32_t width = 1; width < n; width *= 2)
    {
        for (uint32_t low = 0; low < n; low += 2 * width)
        {
            uint32_t middle = low + width < n ? low + width : n;
            uint32_t high = middle + width < n ? middle + width : n;
            uint32_t a = low;
            uint32_t b = middle;
            for (uint32_t k = low; k < high; ++k)
            {
                bool takeA = a < middle && (b >= high || hashes[from[a]] <= hashes[from[b]]);
                to[k] = takeA ? from[a++] : from[b++];
            }
        }
        uint32_t* swap = from;
        from = to;
        to = swap;
    }
    if (from != sorted)
    {
        memcpy(sorted, from, (size_t)n * sizeof(uint32_t));
    }
}

static maniTransform NormalizedRest(const maniTransform* rest)
{
    maniTransform r = *rest;
    double x = (double)r.rotation.x;
    double y = (double)r.rotation.y;
    double z = (double)r.rotation.z;
    double w = (double)r.rotation.w;
    double length = sqrt(x * x + y * y + z * z + w * w);
    r.rotation = (maniQuat){(float)(x / length), (float)(y / length), (float)(z / length),
                            (float)(w / length)};
    return r;
}

// The sections' bytes, laid out once the order is known.
typedef struct Layout
{
    uint8_t* parents;
    uint8_t* ends;
    uint8_t* rests;
    uint8_t* hashes;
    uint8_t* nameOrder;
    uint8_t* spans;
    uint8_t* names;
    uint64_t nameSize;
} Layout;

static void Fill(const maniSkeletonSource* source, uint32_t* scratch[ScratchArrays],
                 const uint32_t* newIndex, const uint64_t* hashes, const Layout* l)
{
    uint32_t n = source->boneCount;
    const uint32_t* order = scratch[ScratchOrder];
    uint64_t nameAt = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniBoneSource* bone = &source->bones[order[k]];
        uint32_t parent = bone->parent < 0 ? MANI_NO_BONE : newIndex[bone->parent];
        maniStoreU16(l->parents + 2 * (size_t)k, (uint16_t)parent);
        maniTransform rest = NormalizedRest(&bone->rest);
        memcpy(l->rests + MANI_SKELETON_REST_SIZE * (size_t)k, &rest, MANI_SKELETON_REST_SIZE);
        maniStoreU64(l->hashes + 8 * (size_t)k, hashes[k]);
        maniStoreU32(l->spans + MANI_SKELETON_SPAN_SIZE * (size_t)k, (uint32_t)nameAt);
        maniStoreU32(l->spans + MANI_SKELETON_SPAN_SIZE * (size_t)k + 4, bone->nameLength);
        memcpy(l->names + nameAt, bone->name, bone->nameLength);
        nameAt += bone->nameLength;
    }
    // A subtree ends where the next bone outside it starts: walking back,
    // each bone's end is the largest end among its children, or itself + 1.
    uint32_t* end = scratch[ScratchStack];
    for (uint32_t k = n; k-- > 0;)
    {
        end[k] = end[k] > k + 1 ? end[k] : k + 1;
        uint32_t parent = maniLoadU16(l->parents + 2 * (size_t)k);
        if (parent != MANI_NO_BONE && end[parent] < end[k])
        {
            end[parent] = end[k];
        }
    }
    for (uint32_t k = 0; k < n; ++k)
    {
        maniStoreU16(l->ends + 2 * (size_t)k, (uint16_t)end[k]);
    }
    const uint32_t* sorted = scratch[ScratchSorted];
    for (uint32_t k = 0; k < n; ++k)
    {
        maniStoreU16(l->nameOrder + 2 * (size_t)k, (uint16_t)sorted[k]);
    }
}

// Equal hashes in the sorted order: a duplicate name or a collision,
// reported with source indices.
static maniBuildResult CheckNames(const maniSkeletonSource* source,
                                  uint32_t* scratch[ScratchArrays], const uint64_t* hashes)
{
    uint32_t n = source->boneCount;
    const uint32_t* sorted = scratch[ScratchSorted];
    const uint32_t* order = scratch[ScratchOrder];
    for (uint32_t k = 1; k < n; ++k)
    {
        if (hashes[sorted[k]] != hashes[sorted[k - 1]])
        {
            continue;
        }
        uint32_t a = order[sorted[k - 1]];
        uint32_t b = order[sorted[k]];
        uint32_t low = a < b ? a : b;
        uint32_t high = a < b ? b : a;
        const maniBoneSource* x = &source->bones[low];
        const maniBoneSource* y = &source->bones[high];
        bool same = x->nameLength == y->nameLength && memcmp(x->name, y->name, x->nameLength) == 0;
        return Refuse(same ? maniBuildReasonDuplicateName : maniBuildReasonNameHash, high, low);
    }
    return Outcome(mani_success);
}

static maniBuildResult Build(maniBuilder* builder, const maniSkeletonSource* source,
                             uint32_t* scratch[ScratchArrays], uint64_t* hashes, uint32_t* newIndex,
                             uint8_t** fileOut, size_t* sizeOut)
{
    uint32_t n = source->boneCount;
    uint32_t cycleBone = 0;
    if (!OrderDepthFirst(source, scratch, &cycleBone))
    {
        return Refuse(maniBuildReasonCycle, cycleBone, 0);
    }
    const uint32_t* order = scratch[ScratchOrder];
    uint64_t nameSize = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        newIndex[order[k]] = k;
        hashes[k] = NameHash(&source->bones[order[k]]);
        nameSize += source->bones[order[k]].nameLength;
    }
    if (nameSize > UINT32_MAX)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    SortByHash(hashes, n, scratch[ScratchSorted], scratch[ScratchMerge]);
    maniBuildResult checked = CheckNames(source, scratch, hashes);
    if (checked.result != mani_success)
    {
        return checked;
    }

    // The sections, in one block, each at a multiple of 8.
    uint64_t sizes[MANI_SKELETON_SECTIONS] = {
        2 * (uint64_t)n, 2 * (uint64_t)n, MANI_SKELETON_REST_SIZE * (uint64_t)n,
        8 * (uint64_t)n, 2 * (uint64_t)n, MANI_SKELETON_SPAN_SIZE * (uint64_t)n,
        nameSize};
    uint64_t offsets[MANI_SKELETON_SECTIONS];
    uint64_t total = 0;
    for (uint32_t i = 0; i < MANI_SKELETON_SECTIONS; ++i)
    {
        offsets[i] = total;
        total += (sizes[i] + 7) / 8 * 8;
    }
    uint8_t* block = maniAllocate(&builder->allocator, (size_t)total, 1, 8);
    if (block == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    memset(scratch[ScratchStack], 0, (size_t)n * sizeof(uint32_t));
    Layout layout = {block + offsets[0], block + offsets[1], block + offsets[2], block + offsets[3],
                     block + offsets[4], block + offsets[5], block + offsets[6], nameSize};
    Fill(source, scratch, newIndex, hashes, &layout);
    maniSectionSource sections[MANI_SKELETON_SECTIONS];
    for (uint32_t i = 0; i < MANI_SKELETON_SECTIONS; ++i)
    {
        sections[i] = (maniSectionSource){i + 1, block + offsets[i], sizes[i]};
    }
    uint64_t identity = maniSkeletonComputeIdentity(n, layout.parents, layout.rests, layout.spans,
                                                    layout.names, nameSize);
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, MANI_SKELETON_SECTIONS, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindSkeleton, MANI_SKELETON_FORMAT_VERSION, identity, sections,
                           MANI_SKELETON_SECTIONS);
    }
    maniRelease(&builder->allocator, block, (size_t)total, 1, 8);
    if (file == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    *fileOut = file;
    *sizeOut = (size_t)fileSize;
    return Outcome(mani_success);
}

maniBuildResult maniBuildSkeleton(maniBuilder* builder, const maniSkeletonSource* source)
{
    if (builder == NULL)
    {
        return Outcome(mani_errorInvalid);
    }
    if (source == NULL)
    {
        ++builder->misuses;
        return Outcome(mani_errorInvalid);
    }
    maniBuildResult result = CheckBones(source);
    if (result.result != mani_success)
    {
        return result;
    }
    uint32_t n = source->boneCount;
    size_t words = (size_t)n * ScratchArrays;
    uint32_t* words32 = maniAllocate(&builder->allocator, words, sizeof(uint32_t), 8);
    uint64_t* hashes = maniAllocate(&builder->allocator, n, sizeof(uint64_t), 8);
    uint32_t* newIndex = maniAllocate(&builder->allocator, n, sizeof(uint32_t), 8);
    uint8_t* file = NULL;
    size_t fileSize = 0;
    result = Outcome(mani_errorCapacity);
    if (words32 != NULL && hashes != NULL && newIndex != NULL)
    {
        uint32_t* scratch[ScratchArrays];
        for (uint32_t i = 0; i < ScratchArrays; ++i)
        {
            scratch[i] = words32 + (size_t)i * n;
        }
        result = Build(builder, source, scratch, hashes, newIndex, &file, &fileSize);
    }
    maniRelease(&builder->allocator, words32, words, sizeof(uint32_t), 8);
    maniRelease(&builder->allocator, hashes, n, sizeof(uint64_t), 8);
    if (result.result != mani_success)
    {
        maniRelease(&builder->allocator, newIndex, n, sizeof(uint32_t), 8);
        return result;
    }
    maniBuilderKeep(builder, maniKindSkeleton, file, fileSize, newIndex, n);
    return result;
}
