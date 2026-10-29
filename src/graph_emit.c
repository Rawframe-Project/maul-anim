// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A stretch's events are the clip's in its window (maniClip_GetEventRange),
// each listed with the node, its clip slot and its weight while the
// buffer lasts, and counted up to UINT32_MAX. A request is kept in the
// inertialize node's block as the least of an update's.

#include "graph_emit.h"

#include "bytes.h"
#include "clip_view.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_instance.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/mirror.h"
#include "maul-anim/rootmotion.h"

#include <stdint.h>
#include <string.h>

// The most whole loops a step lists or counts, as moves bound them.
#define MAX_WRAPS 4294967296.0

void maniGraphEmit(maniGraphUpdate* u, uint32_t node, uint32_t slot, const maniClip* clip,
                   double from, double to)
{
    uint32_t first = 0;
    uint32_t count = 0;
    maniClipEventRangeEntered(clip, (float)from, (float)to, &first, &count);
    const maniGraphNodeState* s = maniGraphUpdateNode(u, node);
    for (uint32_t k = 0; k < count; ++k)
    {
        if (u->count < u->capacity)
        {
            maniClipEvent e = maniClip_GetEvent(clip, first + k);
            maniGraphEvent* out = &u->events[u->count];
            memset(out, 0, sizeof *out);
            out->node = node;
            out->clip = slot;
            out->event = first + k;
            out->payload = e.payload;
            out->nameHash = maniGraphNodeName(u, node, e.nameHash);
            out->weight = s->weight;
            out->time = e.time;
            out->direction = to >= from ? 1 : -1;
        }
        u->count += u->count < UINT32_MAX ? 1u : 0u;
    }
}

void maniGraphEmitLoops(maniGraphUpdate* u, uint32_t node, uint32_t slot, const maniClip* clip,
                        double from, double to, double loops)
{
    uint32_t first = 0;
    uint32_t perLoop = 0;
    maniClipEventRangeEntered(clip, (float)from, (float)to, &first, &perLoop);
    uint64_t n = (uint64_t)(loops < MAX_WRAPS ? loops : MAX_WRAPS);
    for (uint64_t k = 0; k < n; ++k)
    {
        if (u->count >= u->capacity || perLoop == 0)
        {
            // At most 2^32 loops of fewer than 2^32 events: no overflow.
            u->count += (n - k) * perLoop;
            u->count = u->count < UINT32_MAX ? u->count : UINT32_MAX;
            return;
        }
        maniGraphEmit(u, node, slot, clip, from, to);
    }
}

void maniGraphRequest(const maniGraphUpdate* u, uint32_t node, float seconds)
{
    if (node == MANI_GRAPH_NO_NODE || !(seconds >= 0.0f))
    {
        return;
    }
    uint32_t data = maniLoadU32(u->graph->nodes + (size_t)node * MANI_GRAPH_NODE_SIZE + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)u->instance + u->instance->machinesAt) +
                      maniGraphUpdateWord(u, data + maniGraphInertiaBlock);
    float pending = maniGraphInertiaGet(block, maniGraphInertiaPending);
    maniGraphInertiaSet(block, maniGraphInertiaPending, seconds < pending ? seconds : pending);
}

static bool Mirrored(const maniGraphUpdate* u, uint32_t node)
{
    return (maniLoadU16(u->graph->nodes + (size_t)node * MANI_GRAPH_NODE_SIZE + 2) &
            MANI_GRAPH_MIRRORED) != 0;
}

maniRootMove maniGraphNodeMove(const maniGraphUpdate* u, uint32_t node, maniRootMove move)
{
    return Mirrored(u, node) ? maniMirrorMap_MirrorRootMotion(u->binding->mirrorMap, move) : move;
}

uint64_t maniGraphNodeName(const maniGraphUpdate* u, uint32_t node, uint64_t hash)
{
    return Mirrored(u, node) ? maniMirrorMap_MirrorNameHash(u->binding->mirrorMap, hash) : hash;
}
