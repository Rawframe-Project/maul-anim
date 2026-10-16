// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Checking a graph source's motion databases and motion matching nodes as
// its build does (mani-0024).

#ifndef MAUL_ANIM_SRC_GRAPH_MATCH_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_MATCH_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/match.h"

#include <stdint.h>

// Every database slot: a database of the skeleton, each once, each of
// its clips a clip slot giving poses. Gives the clips the databases list.
maniBuildResult maniCheckGraphDatabases(const maniGraphSource* source, uint64_t* clipsOut);

// The clip slot of a database's clip (one of the source's, by identity),
// or UINT32_MAX.
uint32_t maniGraphDatabaseClipSlot(const maniGraphSource* source,
                                   const maniMotionDatabase* database, uint32_t clip);

// A motion matching node's fields, its query against its database.
maniBuildResult maniCheckGraphMatch(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node);

#endif // MAUL_ANIM_SRC_GRAPH_MATCH_CHECK_H
