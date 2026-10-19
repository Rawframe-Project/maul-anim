// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mirror parity of a graph's nodes at build (mani-0033): from the
// output down, each node's readers settled before it, a mirror node
// turning the parity its input takes.

#include "graph_mirror.h"

#include "graph_check.h"

#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>
#include <string.h>

maniBuildResult maniGraphMirrorParity(const maniGraphSource* source, const uint32_t* order,
                                      uint32_t* mirrored)
{
    uint32_t n = source->nodeCount;
    memset(mirrored, 0, n * sizeof(uint32_t));
    mirrored[source->output] = 1;
    for (uint32_t p = n; p-- > 0;)
    {
        uint32_t v = order[p];
        const maniGraphNodeSource* node = &source->nodes[v];
        uint32_t want = node->kind == maniGraphNodeMirror ? 3u - mirrored[v] : mirrored[v];
        for (uint32_t k = 0; k < node->inputCount; ++k)
        {
            uint32_t u = node->inputs[k];
            if (mirrored[u] != 0 && mirrored[u] != want)
            {
                return maniGraphRefuse(maniBuildReasonMirrorGraph, u, 2);
            }
            mirrored[u] = want;
        }
    }
    return (maniBuildResult){mani_success, maniBuildReasonNone, 0, 0};
}
