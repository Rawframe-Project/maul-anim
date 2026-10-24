// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motion database's search tree, for the builder: the
// frames' order (each node's frames split at the median of its widest
// feature) and the index sections laid out from an order (match_file.h).
// The search is exact whatever the order; the median split only makes it
// prune well.

#ifndef MAUL_ANIM_SRC_MATCH_TREE_H
#define MAUL_ANIM_SRC_MATCH_TREE_H

#include <stdint.h>

// The frames' order: order[p] the frame at position p. Each node of more
// than leaf frames has its frames sorted, stably, by the feature its
// frames spread widest over (the lowest such feature on a tie), its
// first half the lower. features holds frames rows of n normalized
// features in frame order; keys and scratch hold frames each, sorted
// 2 frames.
void maniMatchTreeOrder(const float* features, uint32_t n, uint32_t frames, uint32_t leaf,
                        uint32_t* order, uint64_t* keys, uint32_t* sorted, uint32_t* scratch);

// What the index is laid out from: the features in frame order, the
// frames' order, each frame's entry and each entry's cost bias.
typedef struct maniMatchTreeSource
{
    const float* features;
    const uint32_t* order;
    const uint32_t* entryOf;
    const float* biases;
    uint32_t n;
    uint32_t frames;
    uint32_t leaf;
    uint32_t nodes;
} maniMatchTreeSource;

// Lays out the features in tree order (frames rows of n), the boxes
// (2 n floats a node), the nodes (MANI_MATCH_NODE_SIZE bytes each), the
// positions (MANI_MATCH_POSITION_SIZE bytes each) and the places (4 bytes
// a frame).
void maniMatchLayTree(const maniMatchTreeSource* s, float* features, float* boxes, uint8_t* nodes,
                      uint8_t* positions, uint8_t* places);

#endif // MAUL_ANIM_SRC_MATCH_TREE_H
