// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Kinds, modes and choices are int32_t, so a check of a kind's upper
// bound alone lets a negative value through, and a table indexed by it
// reads below its start. Every suite that refuses a kind past its last
// value also refuses these.

#ifndef MAUL_ANIM_TEST_NEGATIVE_KINDS_H
#define MAUL_ANIM_TEST_NEGATIVE_KINDS_H

#include <stdint.h>

#define NEGATIVE_KIND_COUNT 2

static const int32_t kNegativeKinds[NEGATIVE_KIND_COUNT] = {-1, INT32_MIN};

#endif // MAUL_ANIM_TEST_NEGATIVE_KINDS_H
