// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The exact search for callers holding a query already
// normalized, as a motion matching node does with its continuing frame's
// features.

#ifndef MAUL_ANIM_SRC_MATCH_SEARCH_H
#define MAUL_ANIM_SRC_MATCH_SEARCH_H

#include "maul-anim/base.h"
#include "maul-anim/match.h"

#include <stdbool.h>

// maniSearchMotionDatabase, its query's features normalized already when
// normalized is true (then taken as they are).
maniResult maniMatchSearch(const maniMotionDatabase* database, const maniMatchQuery* query,
                           bool normalized, maniMatchResult* resultOut);

// maniMatchSearch with the continuing frame scored by continuing (its
// features, normalized; NULL for the database's): a node playing between
// frames scores its continuing frame as at its playhead.
maniResult maniMatchSearchFrom(const maniMotionDatabase* database, const maniMatchQuery* query,
                               bool normalized, const float* continuing,
                               maniMatchResult* resultOut);

// Features normalized as the database keeps them, blended between two
// frames (a + w (b - a) a feature): each bone facing, a direction, made
// unit again in the database's raw units (unless shorter than a
// thousandth: directions near opposite), as the blend of two unit
// directions falls inside their arc.
void maniMatchFacings(const maniMotionDatabase* database, float* features);

#endif // MAUL_ANIM_SRC_MATCH_SEARCH_H
