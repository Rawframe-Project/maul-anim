// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What a listening state machine records after the clips move
// (mani-0023): its current state's leading clip (the clip node beneath
// it of greatest effective weight, the first of equals), where it was
// and is and the whole loops it wrapped, the loops since the state was
// entered, whether the state finished, and the event names its clips
// listed; the machine's next update decides on them.

#ifndef MAUL_ANIM_SRC_GRAPH_MACHINE_LISTEN_H
#define MAUL_ANIM_SRC_GRAPH_MACHINE_LISTEN_H

#include "graph_instance.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"

// Records every listening machine of an instance whose clips moved,
// clips the binding's in slot order.
void maniGraphListen(const maniGraph* graph, const maniClip* const* clips,
                     maniGraphInstance* instance);

#endif // MAUL_ANIM_SRC_GRAPH_MACHINE_LISTEN_H
