// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The graph file's layout (mani-0023 to mani-0026), shared by its view and
// its builder: kind 3 of the container. Records are little-endian; names are
// offsets into the names section; nodes are in topological order, each
// after its inputs, the output last.

#ifndef MAUL_ANIM_SRC_GRAPH_FILE_H
#define MAUL_ANIM_SRC_GRAPH_FILE_H

#include <stdint.h>

#define MANI_GRAPH_FORMAT_VERSION 4u
#define MANI_GRAPH_SECTIONS       15u

enum
{
    // The skeleton's identity (u64), then u32 each: its bone count, the
    // parameter, node, input, data word, clip, sync group, mask, slot,
    // database, database clip, model, spring set and map counts.
    maniGraphTagHeader = 1,
    // MANI_GRAPH_PARAMETER_SIZE each: type, flags (bit 0 bounded), name
    // offset and length (u32 each), name hash (u64), default (16 bytes),
    // minimum and maximum (4 bytes each, for int and float).
    maniGraphTagParameters = 2,
    // MANI_GRAPH_NODE_SIZE each: kind (u16), flags (u16, bit 0 gives
    // differences, bit 1 under an odd number of mirror nodes), first
    // input, input count, first data word, data word
    // count, slot, its place in the source (subgraphs inlined, as the
    // builder numbers them) (u32 each), zero (4 bytes).
    maniGraphTagNodes = 3,
    // u32 each: a node before its reader.
    maniGraphTagInputs = 4,
    // u32 words, per kind (MANI_GRAPH_*_WORDS); a scalar is a parameter
    // index (MANI_NO_PARAMETER for a literal) and a binary32's bits.
    maniGraphTagData = 5,
    // MANI_GRAPH_CLIP_SIZE each: identity, name hash (u64 each), name
    // offset and length, flags (bit 0 differences), map slot
    // (MANI_GRAPH_NO_MAP for none) (u32 each).
    maniGraphTagClips = 6,
    // MANI_GRAPH_SYNC_GROUP_SIZE each: name hash (u64), name offset and
    // length, flags, zero (u32 each).
    maniGraphTagSyncGroups = 7,
    // A binary32 per bone per mask, 0 to 1.
    maniGraphTagMaskWeights = 8,
    // MANI_GRAPH_DATABASE_SIZE each: identity, name hash (u64 each), name
    // offset and length, its first clip in the database clips and its
    // clip count (u32 each).
    maniGraphTagDatabases = 9,
    // u32 each: the clip slot of each database's clip, in its order, the
    // databases one after another.
    maniGraphTagDatabaseClips = 10,
    // The names' bytes, UTF-8.
    maniGraphTagNames = 11,
    // MANI_GRAPH_MODEL_SIZE each: identity, name hash (u64 each), name
    // offset and length (u32 each).
    maniGraphTagModels = 12,
    // MANI_GRAPH_SPRING_SET_SIZE each: identity, name hash (u64 each),
    // name offset and length (u32 each).
    maniGraphTagSpringSets = 13,
    // MANI_GRAPH_MAP_SIZE each: a retargeting map's identity and its
    // source skeleton's (u64 each), each map once.
    maniGraphTagMaps = 14,
    // The mirror map's identity (u64) when the graph has one, else
    // nothing.
    maniGraphTagMirror = 15,
};

#define MANI_GRAPH_HEADER_SIZE     64u
#define MANI_GRAPH_PARAMETER_SIZE  48u
#define MANI_GRAPH_NODE_SIZE       32u
#define MANI_GRAPH_CLIP_SIZE       32u
#define MANI_GRAPH_SYNC_GROUP_SIZE 24u
#define MANI_GRAPH_DATABASE_SIZE   32u
#define MANI_GRAPH_MODEL_SIZE      24u
#define MANI_GRAPH_SPRING_SET_SIZE 24u
#define MANI_GRAPH_MAP_SIZE        16u

// A clip slot of the graph's skeleton: no map slot.
#define MANI_GRAPH_NO_MAP 0xFFFFFFFFu

// A parameter's and a node's flags.
#define MANI_GRAPH_BOUNDED    1u
#define MANI_GRAPH_DIFFERENCE 1u
#define MANI_GRAPH_MIRRORED   2u
// Every flag a node may carry.
#define MANI_GRAPH_NODE_FLAGS (MANI_GRAPH_DIFFERENCE | MANI_GRAPH_MIRRORED)

// Node data words. A clip node: slot, sync group, loop, flags, speed (a
// scalar). A blend: direct, then a scalar per input. A mask: its mask. An
// additive node: a scalar per layer. A select: its parameter. A time
// scale: the scale. A seek: the trigger, then the time. A pose input:
// its pose parameter. A blend space:
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
#define MANI_GRAPH_POSE_WORDS       1u
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
// Its block in an instance is laid out in graph_inertia.h.

// A motion matching node's data words: its database slot, trigger
// (MANI_NO_PARAMETER for none), period, jump threshold, end exclusion
// and history time, continuing bias and transition cost (binary32), its
// history count, its required and excluded tags (a parameter or
// MANI_NO_PARAMETER, then the literal bits, each), its request's
// duration as a scalar, the first word of its block in an instance's
// machine state, its query count, then each query parameter.
#define MANI_GRAPH_MATCH_WORDS 17u
// Its block in an instance is laid out in graph_match.h.

// A spring bone node's data words: its spring set slot, position,
// rotation and reset parameters (MANI_NO_PARAMETER for none), the first
// word of its block in an instance's machine state, its set's state
// words and its force parameter (or none); the words and block are laid
// out in graph_spring.h.
#define MANI_GRAPH_SPRING_WORDS 7u

// A two-bone IK node's data words: its start, middle and end bones, its
// goal parameter (a vec3), pole parameter (a vec3 or MANI_NO_PARAMETER),
// hinge (three binary32), then its twist, soften and weight as scalars.
#define MANI_GRAPH_TWO_BONE_WORDS 14u
enum
{
    maniGraphTwoBoneGoal = 3,
    maniGraphTwoBonePole = 4,
    maniGraphTwoBoneHinge = 5,
    maniGraphTwoBoneTwist = 8,
    maniGraphTwoBoneSoften = 10,
    maniGraphTwoBoneWeight = 12,
};

// An aim IK node's data words: its bone count, goal and pole parameters
// (as a two-bone node's), MANI_AIM_MAX_BONES bones then as many shares
// (binary32), those past the count 0, its forward, offset and up (three
// binary32 each), then its twist, cone and weight as scalars.
#define MANI_GRAPH_AIM_WORDS 34u
enum
{
    maniGraphAimGoal = 1,
    maniGraphAimPole = 2,
    maniGraphAimBones = 3,
    maniGraphAimShares = 11,
    maniGraphAimForward = 19,
    maniGraphAimOffset = 22,
    maniGraphAimUp = 25,
    maniGraphAimTwist = 28,
    maniGraphAimCone = 30,
    maniGraphAimWeight = 32,
};

// A chain IK node's data words: its root and end bones, its goal (a
// vec3 parameter), pole (a vec3 parameter or MANI_NO_PARAMETER, none
// for CCD) and rotation (a quat parameter or MANI_NO_PARAMETER)
// parameters, its iteration count, tolerance and step (binary32), its
// rotation weight and weight as scalars, its limit count (0, or one per
// turning bone), then each limit's MANI_GRAPH_LIMIT_WORDS: its kind, axis
// (three binary32), cone, low, high, twist low and twist high
// (binary32), unused fields 0.
#define MANI_GRAPH_CHAIN_WORDS 13u
#define MANI_GRAPH_LIMIT_WORDS 9u
enum
{
    maniGraphChainGoal = 2,
    maniGraphChainPole = 3,
    maniGraphChainRotation = 4,
    maniGraphChainIterations = 5,
    maniGraphChainTolerance = 6,
    maniGraphChainStep = 7,
    maniGraphChainRotationWeight = 8,
    maniGraphChainWeight = 10,
    maniGraphChainLimitCount = 12,
};

// A full-body IK node's data words: its top bone and mode, stretch (0
// or 1, 0 for SDLS), iteration count and tolerance (binary32), its
// weight as a scalar, its effector and setting counts; then each
// effector's MANI_GRAPH_EFFECTOR_WORDS: its bone, position (a vec3
// parameter) and rotation (a quat parameter or MANI_NO_PARAMETER)
// parameters, its position and rotation weights as scalars (the
// rotation weight the number 0 without a rotation), its strength and
// pull chain (binary32); then each setting's MANI_GRAPH_SETTING_WORDS:
// its bone, its limit's MANI_GRAPH_LIMIT_WORDS as a chain node's, its
// preferred angles (three binary32) and its rotation and position
// stiffness (binary32).
#define MANI_GRAPH_BODY_WORDS     9u
#define MANI_GRAPH_EFFECTOR_WORDS 9u
#define MANI_GRAPH_SETTING_WORDS  15u
enum
{
    maniGraphBodyTop = 0,
    maniGraphBodyMode = 1,
    maniGraphBodyStretch = 2,
    maniGraphBodyIterations = 3,
    maniGraphBodyTolerance = 4,
    maniGraphBodyWeight = 5,
    maniGraphBodyEffectorCount = 7,
    maniGraphBodySettingCount = 8,
};
// An effector's words, from its first.
enum
{
    maniGraphEffectorBone = 0,
    maniGraphEffectorPosition = 1,
    maniGraphEffectorRotation = 2,
    maniGraphEffectorPositionWeight = 3,
    maniGraphEffectorRotationWeight = 5,
    maniGraphEffectorStrength = 7,
    maniGraphEffectorPull = 8,
};
// A setting's words, from its first.
enum
{
    maniGraphSettingBone = 0,
    maniGraphSettingLimit = 1,
    maniGraphSettingPreferred = 10,
    maniGraphSettingRotationStiffness = 13,
    maniGraphSettingPositionStiffness = 14,
};

// The data words a node of a kind with n inputs holds; 0 for an unknown
// kind, a state machine, a one-shot, a chain IK and a full-body IK node,
// whose words their contents count.
uint32_t maniGraphDataWords(uint32_t kind, uint32_t inputCount);

// The inputs a kind takes: at least minOut, at most the return (0 with
// minOut 1 for an unknown kind).
uint32_t maniGraphInputRange(uint32_t kind, uint32_t* minOut);

// The identity: maniHash64 over every section in tag order, each
// continuing the last.
uint64_t maniGraphComputeIdentity(const uint8_t* const* sections, const uint64_t* sizes);

#endif // MAUL_ANIM_SRC_GRAPH_FILE_H
