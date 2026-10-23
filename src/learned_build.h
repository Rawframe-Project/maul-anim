// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The learned model builder's checks other builders share.

#ifndef MAUL_ANIM_SRC_LEARNED_BUILD_H
#define MAUL_ANIM_SRC_LEARNED_BUILD_H

#include "maul-anim/build.h"
#include "maul-anim/learnedbuild.h"

#include <stdint.h>

// A list's names not empty, UTF-8 and unique, their bytes within what a
// model holds; refused with list combined with maniBuildReasonName or
// maniBuildReasonDuplicateName.
maniBuildResult maniLearnedCheckNames(const maniLearnedNameSource* names, uint32_t count,
                                      maniBuildReason list);

#endif // MAUL_ANIM_SRC_LEARNED_BUILD_H
