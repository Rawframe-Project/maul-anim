// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A state machine's data words checked on open: their count
// from its transitions', conditions' and states', and each word in
// range: a list's clip nodes before the machine and in node order.

#ifndef MAUL_ANIM_SRC_GRAPH_MACHINE_FILE_H
#define MAUL_ANIM_SRC_GRAPH_MACHINE_FILE_H

#include "maul-anim/graph.h"

#include <stdint.h>

// The data words the machine of states inputs at data word data takes as
// its header, transitions and states count them, or UINT32_MAX when they
// do not fit in available words or name more transitions or conditions
// than allowed.
uint32_t maniGraphMachineDataWords(const maniGraph* g, uint32_t data, uint32_t states,
                                   uint32_t available);

// The first wrong data word of machine node p of states inputs whose
// block starts at block, or UINT32_MAX; its words counted already and
// the nodes before it checked.
uint32_t maniGraphMachineValid(const maniGraph* g, uint32_t p, uint32_t data, uint32_t states,
                               uint32_t block);

#endif // MAUL_ANIM_SRC_GRAPH_MACHINE_FILE_H
