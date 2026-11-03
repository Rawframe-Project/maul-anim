// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Reading and writing the graph JSON form's searching nodes: a motion
// matching node and a learned node. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_SEARCH_H
#define MAUL_ANIM_SRC_GRAPH_JSON_SEARCH_H

#include "graph_json_read.h"
#include "graph_json_write.h"

#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// A motion matching node (members its form's: kind, database, trigger,
// query, period, jumpThreshold, endExclusion, historyTime,
// continuingBias, transitionCost, historyCount, requiredTags,
// excludedTags, duration).
bool maniGraphJsonMatch(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                        maniGraphMatchNodeSource* out);

// A learned node (members its form's: kind, model, trigger, query,
// period, transitionCost, requiredTags, excludedTags, duration).
bool maniGraphJsonLearned(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                          maniGraphLearnedNodeSource* out);

// A matching node's members after its kind, each optional one only off
// its default.
void maniGraphJsonPutMatch(maniGraphWriter* w, const maniGraphSource* source,
                           const maniGraphMatchNodeSource* m);

// A learned node's members after its kind, each optional one only off
// its default.
void maniGraphJsonPutLearned(maniGraphWriter* w, const maniGraphSource* source,
                             const maniGraphLearnedNodeSource* l);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_SEARCH_H
