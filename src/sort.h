// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A stable sort of indices by 64-bit keys, for the builders: the same
// order on every platform, ties in index order.

#ifndef MAUL_ANIM_SRC_SORT_H
#define MAUL_ANIM_SRC_SORT_H

#include <stdint.h>

// Sorts the indices 0 to n - 1 by keys[index], stable, bottom-up merge;
// scratch holds n.
void maniSortByKey(const uint64_t* keys, uint32_t n, uint32_t* sorted, uint32_t* scratch);

#endif // MAUL_ANIM_SRC_SORT_H
