// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motion database's search tree (match_tree.h): the order by
// median splits, then the sections laid out from it, boxes from the
// leaves up as the opener checks them.

#include "match_tree.h"

#include "bytes.h"
#include "match_file.h"
#include "sort.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// The feature a node's frames spread widest over; the lowest on a tie.
static uint32_t Widest(const float* features, uint32_t n, const uint32_t* order, uint32_t first,
                       uint32_t count)
{
    uint32_t widest = 0;
    float spread = -1.0f;
    for (uint32_t j = 0; j < n; ++j)
    {
        float low = HUGE_VALF;
        float high = -HUGE_VALF;
        for (uint32_t p = first; p < first + count; ++p)
        {
            float v = features[(size_t)order[p] * n + j];
            low = v < low ? v : low;
            high = v > high ? v : high;
        }
        if (high - low > spread)
        {
            spread = high - low;
            widest = j;
        }
    }
    return widest;
}

void maniMatchTreeOrder(const float* features, uint32_t n, uint32_t frames, uint32_t leaf,
                        uint32_t* order, uint64_t* keys, uint32_t* sorted, uint32_t* scratch)
{
    for (uint32_t f = 0; f < frames; ++f)
    {
        order[f] = f;
    }
    maniMatchWalk walk;
    maniMatchVisit v;
    maniMatchWalkStart(&walk, frames, leaf);
    while (maniMatchWalkNext(&walk, &v))
    {
        if (v.count <= leaf)
        {
            continue;
        }
        uint32_t j = Widest(features, n, order, v.first, v.count);
        for (uint32_t k = 0; k < v.count; ++k)
        {
            keys[k] = maniFloatKey(features[(size_t)order[v.first + k] * n + j]);
        }
        maniSortByKey(keys, v.count, sorted, scratch);
        for (uint32_t k = 0; k < v.count; ++k)
        {
            scratch[k] = order[v.first + sorted[k]];
        }
        memcpy(order + v.first, scratch, (size_t)v.count * sizeof(uint32_t));
    }
}

// A leaf's box, lowest frame and least bias over its positions in order.
static void Leaf(const maniMatchTreeSource* s, const float* features, uint32_t first,
                 uint32_t count, float* box, uint8_t* node)
{
    uint32_t n = s->n;
    uint32_t lowest = UINT32_MAX;
    float bias = HUGE_VALF;
    for (uint32_t j = 0; j < n; ++j)
    {
        box[j] = HUGE_VALF;
        box[n + j] = -HUGE_VALF;
    }
    for (uint32_t p = first; p < first + count; ++p)
    {
        uint32_t frame = s->order[p];
        lowest = frame < lowest ? frame : lowest;
        bias = maniMatchLower(bias, s->biases[s->entryOf[frame]]);
        for (uint32_t j = 0; j < n; ++j)
        {
            float v = features[(size_t)p * n + j];
            box[j] = maniMatchLower(box[j], v);
            box[n + j] = maniMatchHigher(box[n + j], v);
        }
    }
    maniStoreU32(node, 0);
    maniStoreU32(node + 4, lowest);
    memcpy(node + 8, &bias, 4);
}

void maniMatchLayTree(const maniMatchTreeSource* s, float* features, float* boxes, uint8_t* nodes,
                      uint8_t* positions, uint8_t* places)
{
    uint32_t n = s->n;
    for (uint32_t p = 0; p < s->frames; ++p)
    {
        uint32_t frame = s->order[p];
        memcpy(features + (size_t)p * n, s->features + (size_t)frame * n,
               (size_t)n * sizeof(float));
        maniStoreU32(positions + (size_t)p * MANI_MATCH_POSITION_SIZE, frame);
        maniStoreU32(positions + (size_t)p * MANI_MATCH_POSITION_SIZE + 4, s->entryOf[frame]);
        maniStoreU32(places + 4 * (size_t)frame, p);
    }
    // The leaves and each node's second half on the walk, then the other
    // nodes from the last up, so both halves come first.
    maniMatchWalk walk;
    maniMatchVisit v;
    maniMatchWalkStart(&walk, s->frames, s->leaf);
    while (maniMatchWalkNext(&walk, &v))
    {
        if (v.halfOf != UINT32_MAX)
        {
            maniStoreU32(nodes + (size_t)v.halfOf * MANI_MATCH_NODE_SIZE, v.node);
        }
        if (v.count <= s->leaf)
        {
            Leaf(s, features, v.first, v.count, boxes + (size_t)v.node * 2 * n,
                 nodes + (size_t)v.node * MANI_MATCH_NODE_SIZE);
        }
    }
    for (uint32_t k = s->nodes; k-- > 0;)
    {
        uint8_t* node = nodes + (size_t)k * MANI_MATCH_NODE_SIZE;
        uint32_t second = maniLoadU32(node);
        if (second == 0)
        {
            continue;
        }
        const uint8_t* a = nodes + (size_t)(k + 1) * MANI_MATCH_NODE_SIZE;
        const uint8_t* b = nodes + (size_t)second * MANI_MATCH_NODE_SIZE;
        uint32_t lowest =
            maniLoadU32(a + 4) < maniLoadU32(b + 4) ? maniLoadU32(a + 4) : maniLoadU32(b + 4);
        float biasA = 0.0f;
        float biasB = 0.0f;
        memcpy(&biasA, a + 8, 4);
        memcpy(&biasB, b + 8, 4);
        float bias = maniMatchLower(biasA, biasB);
        maniStoreU32(node + 4, lowest);
        memcpy(node + 8, &bias, 4);
        float* box = boxes + (size_t)k * 2 * n;
        const float* boxA = boxes + (size_t)(k + 1) * 2 * n;
        const float* boxB = boxes + (size_t)second * 2 * n;
        for (uint32_t j = 0; j < 2 * n; ++j)
        {
            box[j] = j < n ? maniMatchLower(boxA[j], boxB[j]) : maniMatchHigher(boxA[j], boxB[j]);
        }
    }
}
