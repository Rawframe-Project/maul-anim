// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The graph file's layout (mani-0023), shared by its view and its
// builder: kind 3 of the container. Records are little-endian; names are
// offsets into the names section; nodes are in topological order, each
// after its inputs, the output last.

#ifndef MAUL_ANIM_SRC_GRAPH_FILE_H
#define MAUL_ANIM_SRC_GRAPH_FILE_H

#include <stdint.h>

#define MANI_GRAPH_FORMAT_VERSION 1u
#define MANI_GRAPH_SECTIONS       9u

enum
{
    // The skeleton's identity (u64), then u32 each: its bone count, the
    // parameter, node, input, data word, clip, sync group, mask and slot
    // counts, and 0.
    maniGraphTagHeader = 1,
    // MANI_GRAPH_PARAMETER_SIZE each: type, flags (bit 0 bounded), name
    // offset and length (u32 each), name hash (u64), default (16 bytes),
    // minimum and maximum (4 bytes each, for int and float).
    maniGraphTagParameters = 2,
    // MANI_GRAPH_NODE_SIZE each: kind (u16), flags (u16, bit 0 gives
    // differences), first input, input count, first data word, data word
    // count, slot (u32 each), zero (8 bytes).
    maniGraphTagNodes = 3,
    // u32 each: a node before its reader.
    maniGraphTagInputs = 4,
    // u32 words, per kind (MANI_GRAPH_*_WORDS); a scalar is a parameter
    // index (MANI_NO_PARAMETER for a literal) and a binary32's bits.
    maniGraphTagData = 5,
    // MANI_GRAPH_CLIP_SIZE each: identity, name hash (u64 each), name
    // offset and length, flags (bit 0 differences), zero (u32 each).
    maniGraphTagClips = 6,
    // MANI_GRAPH_SYNC_GROUP_SIZE each: name hash (u64), name offset and
    // length, flags, zero (u32 each).
    maniGraphTagSyncGroups = 7,
    // A binary32 per bone per mask, 0 to 1.
    maniGraphTagMaskWeights = 8,
    // The names' bytes, UTF-8.
    maniGraphTagNames = 9,
};

#define MANI_GRAPH_HEADER_SIZE     48u
#define MANI_GRAPH_PARAMETER_SIZE  48u
#define MANI_GRAPH_NODE_SIZE       32u
#define MANI_GRAPH_CLIP_SIZE       32u
#define MANI_GRAPH_SYNC_GROUP_SIZE 24u

// A parameter's and a node's flags.
#define MANI_GRAPH_BOUNDED    1u
#define MANI_GRAPH_DIFFERENCE 1u

// Node data words. A clip node: slot, sync group, loop, flags, speed (a
// scalar). A blend: direct, then a scalar per input. A mask: its mask. An
// additive node: a scalar per layer. A select: its parameter. A time
// scale: the scale. A seek: the trigger, then the time. A blend space:
// its position (a vec2 parameter or MANI_NO_PARAMETER, then x and y as
// scalars, literal 0 with a vec2 and a 1D space's y), a word (a 1D
// space's period, a binary32; a banded space's polar flag; a triangulated
// space's triangle count), then per input in a 1D space its point, then
// the points' order (the inputs by ascending point), or in a plane its
// point's x and y; a triangulated space then reserves
// MANI_GRAPH_MAX_TRIANGLES(n) triangles of three corners (anticlockwise),
// those past its count zero.
#define MANI_GRAPH_CLIP_WORDS       6u
#define MANI_GRAPH_MASK_WORDS       1u
#define MANI_GRAPH_SELECT_WORDS     1u
#define MANI_GRAPH_TIME_SCALE_WORDS 2u
#define MANI_GRAPH_SEEK_WORDS       3u
#define MANI_GRAPH_POSITION_WORDS   5u
#define MANI_GRAPH_SPACE_WORDS      6u
// The most triangles that n points (3 or more) can be corners of with no
// two overlapping, by Euler's formula.
#define MANI_GRAPH_MAX_TRIANGLES(n) (2u * (n) - 5u)

// A state machine's data words: its entry state, chain limit, transition
// count, the first word of its block in an instance's machine state, its
// condition count and whether it listens (1 when a transition has a
// phase, finished or event condition or a match entry, else 0); then per
// transition, in priority order (the source's order among equals), its
// from (a state or MANI_GRAPH_ANY_STATE), to, priority, flags
// (interruption, entry << 4, fade << 8), duration and cooldown
// (binary32), first condition (counted from the node's first) and
// condition count; then per condition its kind, parameter, compare and
// value (the bits of a bool, int or binary32; an event condition's name
// hash, low word in the parameter, high in the compare); then per state
// whether it is final, its nearest machine (a node, or MANI_GRAPH_NO_NODE)
// and its clip list's first word (counted from the lists' first) and
// count; then the lists, each the clip nodes beneath a state in node
// order. A machine that does not listen lists nothing and has no nearest.
#define MANI_GRAPH_MACHINE_WORDS    6u
#define MANI_GRAPH_TRANSITION_WORDS 8u
#define MANI_GRAPH_CONDITION_WORDS  4u
#define MANI_GRAPH_STATE_WORDS      4u
#define MANI_GRAPH_NO_NODE          0xFFFFFFFFu
// A machine's block in an instance: its current state, the transition
// running (MANI_GRAPH_NO_TRANSITION for none), the fade's elapsed seconds
// (binary32), the seconds in the current state, then the record of the
// update before (binary64, two words each, low first: the leading clip's
// normalized position before and after its move, the whole loops it
// wrapped, the loops it had moved since the state was entered before
// this move) and whether the state finished; per state its weight and
// its weight when the fade began; per transition the seconds since it
// fired (+inf before it has); a bit per condition (low bit first), an
// event condition's set when its name was heard.
#define MANI_GRAPH_BLOCK_WORDS   14u
#define MANI_GRAPH_NO_TRANSITION 0xFFFFFFFFu
#define MANI_GRAPH_BLOCK_SIZE(states, transitions, conditions)                                     \
    (MANI_GRAPH_BLOCK_WORDS + 2u * (states) + (transitions) + ((conditions) + 31u) / 32u)

// A one-shot's data words: its fire, fade-out and abort triggers
// (MANI_NO_PARAMETER for the last two when absent), its curve, fade-in and
// fade-out seconds (binary32), its mask (MANI_GRAPH_NO_MASK for the whole
// body), the first word of its block in an instance's machine state, and
// the count of the clip nodes beneath its shot, then those nodes in node
// order.
#define MANI_GRAPH_SHOT_WORDS 9u
#define MANI_GRAPH_NO_MASK    0xFFFFFFFFu
// A one-shot's block: its phase (idle, fading in, playing, fading out),
// the fade's elapsed seconds, the weight when the fade began and its
// weight (binary32), then the clip seconds its leading clip had left
// after the update before (binary64, +inf for none or repeating).
#define MANI_GRAPH_SHOT_BLOCK_WORDS 6u

// An inertialize node's data words: its trigger (MANI_NO_PARAMETER for
// none), its method, its request's duration as a scalar, dead blending's
// half-life scale, minimum and maximum (binary32, 0 for
// inertialization), and the first word of its block in an instance's
// machine state.
#define MANI_GRAPH_INERTIA_WORDS 8u
// Its block: flags (active, last output kept, the one before kept), its
// update count, this update's step, the elapsed and requested seconds,
// the pending request (+inf for none), the last output's count and step,
// the one before's count, a zero word (binary32 for seconds); then per
// bone its last two outputs (translation, rotation, scale: ten binary32
// each) and the capture. Inertialization's per channel (translation,
// rotation, scale): a unit direction or axis, the offset's magnitude or
// angle, its speed and its duration (18 words). Dead blending's: the
// last output (10), its speeds per axis (translation, the rotation's
// scaled axis-angle, the scale's logarithm: 9) and their decay rates
// (9).
#define MANI_GRAPH_INERTIA_HEADER             10u
#define MANI_GRAPH_INERTIA_BONE_WORDS(method) ((method) != 0 ? 48u : 38u)
#define MANI_GRAPH_INERTIA_BLOCK(bones, method)                                                    \
    (MANI_GRAPH_INERTIA_HEADER + MANI_GRAPH_INERTIA_BONE_WORDS(method) * (bones))

// The data words a node of a kind with n inputs holds; 0 for an unknown
// kind, a state machine and a one-shot, whose words their contents
// count.
uint32_t maniGraphDataWords(uint32_t kind, uint32_t inputCount);

// The inputs a kind takes: at least minOut, at most the return (0 with
// minOut 1 for an unknown kind).
uint32_t maniGraphInputRange(uint32_t kind, uint32_t* minOut);

// The identity: maniHash64 over every section in tag order, each
// continuing the last.
uint64_t maniGraphComputeIdentity(const uint8_t* const* sections, const uint64_t* sizes);

#endif // MAUL_ANIM_SRC_GRAPH_FILE_H
