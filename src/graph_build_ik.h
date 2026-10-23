// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Writing an IK or full-body node's data words.

#ifndef MAUL_ANIM_SRC_GRAPH_BUILD_IK_H
#define MAUL_ANIM_SRC_GRAPH_BUILD_IK_H

#include "maul-anim/graphbuild.h"

#include <stdint.h>

// The words of an IK node (kinds two-bone to full-body SDLS), by its
// kind, written into data from word at; the word after them.
uint32_t maniGraphPutIk(const maniGraphSource* source, uint8_t* data, uint32_t at,
                        const maniGraphNodeSource* node);

#endif // MAUL_ANIM_SRC_GRAPH_BUILD_IK_H
