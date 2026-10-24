// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The FBX build options: their checks, and the basis they
// turn the file's axes and unit to.

#ifndef MAUL_ANIM_SRC_FBX_OPTIONS_H
#define MAUL_ANIM_SRC_FBX_OPTIONS_H

#include "basis.h"
#include "fbx_document.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stdbool.h>

// Each field in its range.
bool maniFbxOptionsValid(const maniFbxOptions* options);

// The checks every build entry point makes: a NULL builder is refused
// alone, any other bad argument (inRange false included) is a misuse.
bool maniFbxUsable(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options,
                   bool inRange);

// The output's basis: the file's right, up and front axes turned to the
// options', translations from the file's unit to the options'.
maniBasis maniFbxBasis(const maniFbx* fbx, const maniFbxOptions* options);

#endif // MAUL_ANIM_SRC_FBX_OPTIONS_H
