// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binding a graph to its skeleton and clips (mani-0023): each clip slot
// found by identity among the host's clips, in caller memory.

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

size_t maniGraphBindingSize(const maniGraph* graph)
{
    if (graph == NULL)
    {
        return 0;
    }
    return offsetof(maniGraphBinding, clips) + (size_t)graph->clipCount * sizeof(const maniClip*);
}

// The clip of the slot's identity among clips, of the graph's skeleton
// and of the slot's kind; NULL for none.
static const maniClip* Find(const maniGraph* graph, uint32_t slot, const maniClip* const* clips,
                            uint32_t count)
{
    const uint8_t* e = graph->clips + (size_t)slot * MANI_GRAPH_CLIP_SIZE;
    uint64_t identity = maniLoadU64(e);
    bool difference = maniLoadU32(e + 24) != 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        const maniClip* clip = clips[i];
        if (clip != NULL && maniClipIdentity(clip) == identity &&
            maniClipSkeletonIdentity(clip) == graph->skeletonIdentity &&
            (maniClipAdditiveBasis(clip) != maniAdditiveNone) == difference)
        {
            return clip;
        }
    }
    return NULL;
}

static size_t CursorSize(const maniGraphBinding* binding)
{
    size_t largest = 0;
    for (uint32_t s = 0; s < binding->graph.clipCount; ++s)
    {
        size_t size = maniClipCursorSize(binding->clips[s]);
        largest = size > largest ? size : largest;
    }
    return (largest + 7u) & ~(size_t)7u;
}

maniResult maniBindGraph(const maniGraph* graph, const maniSkeleton* skeleton,
                         const maniClip* const* clips, uint32_t count, void* memory, size_t size,
                         maniGraphBinding** bindingOut, uint32_t* unboundOut)
{
    if (unboundOut != NULL)
    {
        *unboundOut = MANI_NO_TRACK;
    }
    if (graph == NULL || skeleton == NULL || (clips == NULL && count > 0) || memory == NULL ||
        bindingOut == NULL || (uintptr_t)memory % MANI_GRAPH_BINDING_ALIGNMENT != 0 ||
        maniSkeletonIdentity(skeleton) != graph->skeletonIdentity ||
        maniSkeletonBoneCount(skeleton) != graph->skeletonBoneCount)
    {
        return mani_errorInvalid;
    }
    if (size < maniGraphBindingSize(graph))
    {
        return mani_errorCapacity;
    }
    maniGraphBinding* binding = memory;
    for (uint32_t s = 0; s < graph->clipCount; ++s)
    {
        binding->clips[s] = Find(graph, s, clips, count);
        if (binding->clips[s] == NULL)
        {
            if (unboundOut != NULL)
            {
                *unboundOut = s;
            }
            return mani_errorInvalid;
        }
    }
    binding->stamp = MANI_GRAPH_BINDING_STAMP;
    binding->reserved = 0;
    binding->graph = *graph;
    binding->skeleton = skeleton;
    binding->cursorSize = CursorSize(binding);
    *bindingOut = binding;
    return mani_success;
}
