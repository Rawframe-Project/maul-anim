// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The offline side: a builder turns source data into the library's files,
// which it keeps until the next build for a copy into caller
// memory. A builder allocates through the allocator of its def and is
// built only when MAUL_ANIM_OFFLINE is on.

#ifndef MAUL_ANIM_BUILD_H
#define MAUL_ANIM_BUILD_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/morph.h"
#include "maul-anim/ragdoll.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A builder: one owner object for every kind of file. Every function
    // taking a builder, and closing a document opened with one, refuses a
    // call back in from the builder's allocator with mani_errorBusy and
    // counts one misuse.
    typedef struct maniBuilder maniBuilder;

    typedef struct maniBuilderDef
    {
        uint32_t cookie;
        // MANI_ABI_VERSION, set by maniDefaultBuilderDef.
        uint32_t version;
        // The allocator the builder uses; zeroed for the C library's.
        maniAllocator allocator;
    } maniBuilderDef;

// The cookie maniDefaultBuilderDef sets and maniCreateBuilder checks.
#define MANI_BUILDER_DEF_COOKIE 0x4D424C44u

    /// Returns a builder def with every setting at its default.
    ///
    /// @return The def, with a valid cookie.
    /// @par Thread safety
    /// Safe from any thread.
    static inline maniBuilderDef maniDefaultBuilderDef(void)
    {
#ifdef __cplusplus
        maniBuilderDef def = {};
#else
    maniBuilderDef def = {0};
#endif
        def.cookie = MANI_BUILDER_DEF_COOKIE;
        def.version = MANI_ABI_VERSION;
        return def;
    }

    /// Makes a builder from a def.
    ///
    /// @param def        The def.
    /// @param builderOut Receives the builder, or NULL on failure.
    /// @return `mani_success`; `mani_errorVersion` for a def built against
    /// another major or minor version; `mani_errorInvalid` for a NULL
    /// argument, a def without its cookie or an allocator with one
    /// function; `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniCreateBuilder(const maniBuilderDef* def,
                                                         maniBuilder** builderOut);

    /// Destroys a builder and the file it holds. Called back in from the
    /// builder's allocator it destroys nothing and counts one misuse.
    ///
    /// @param builder  The builder, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_API void maniDestroyBuilder(maniBuilder* builder);

    /// Returns how many calls on the builder were refused for invalid
    /// arguments (maul-conventions.md, Errors) or as calls back in from its
    /// allocator.
    ///
    /// @param builder  The builder.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_API uint32_t maniBuilderMisuses(const maniBuilder* builder);

    // Why a build refused its source.
    typedef uint32_t maniBuildReason;
    enum
    {
        maniBuildReasonNone = 0,
        // Too few or too many items: no bones, or more than MANI_MAX_BONES
        // (at the limit).
        maniBuildReasonCount = 1,
        // A parent index out of range, or a bone its own parent.
        maniBuildReasonParent = 2,
        // A bone no root reaches: on a cycle of parents, or below one; a
        // graph node on a cycle of inputs.
        maniBuildReasonCycle = 3,
        // An empty name, a NULL one, or one that is not UTF-8.
        maniBuildReasonName = 4,
        // A name a lower item already has; other is that item.
        maniBuildReasonDuplicateName = 5,
        // Two names whose 64-bit hashes are equal; other is the lower item.
        maniBuildReasonNameHash = 6,
        // A translation, rotation or scale component that is not finite.
        maniBuildReasonNotFinite = 7,
        // A rotation of zero length.
        maniBuildReasonRotation = 8,
        // A joint naming a bone the skeleton does not have.
        maniBuildReasonJoint = 9,
        // A track for a bone the skeleton does not have, or a second track
        // for one bone or name; other is the earlier track. For float
        // tracks, discrete tracks, events and markers, item counts from 0
        // within its own list.
        maniBuildReasonTrack = 10,
        // Key times not increasing, or before 0.
        maniBuildReasonKeyTime = 11,
        // A cubic spline channel with fewer than two keys.
        maniBuildReasonKeyCount = 12,
        // A sample rate or duration not finite, a rate not above 0, a
        // duration below 0, or more than MANI_MAX_SAMPLES samples (at the
        // limit).
        maniBuildReasonTiming = 13,
        // An interpolation, loop policy or additive basis the library does not
        // know.
        maniBuildReasonKind = 14,
        // An additive clip's basis with a scale component of 0 on a bone
        // whose track has scale keys.
        maniBuildReasonBasis = 15,
        // A weight below 0, a dual quaternion weight outside 0 to 1, a
        // morph target's full weight not finite, 0 or not above the
        // channel's previous, or a mask chain's weight outside 0 to 1 (item
        // the node, other the chain).
        maniBuildReasonWeight = 16,
        // A joint given twice for one vertex with weight; other is the
        // earlier influence.
        maniBuildReasonRepeatedJoint = 17,
        // A triangle naming a vertex past the vertex count; item is the
        // triangle.
        maniBuildReasonTriangle = 18,
        // An option outside its range (the centers' sigma).
        maniBuildReasonOption = 19,
        // A vertex index past the vertex count; other is its place in the
        // source's indices.
        maniBuildReasonVertex = 20,
        // A morph attribute of an unknown semantic, with a set beyond 0
        // for positions, normals or tangents, or given twice (other the
        // earlier); item is the attribute.
        maniBuildReasonAttribute = 21,
        // A root motion source: a bone outside the skeleton
        // or without a track, no channel chosen, a field out of range,
        // an additive clip, or a motion translation beyond 2^64; item is
        // the bone. A retargeted clip's root motion bone past the target's
        // bones, or none that follows the source's (item MANI_NO_BONE).
        maniBuildReasonRootMotion = 22,
        // A graph parameter of an unknown type, or whose
        // default or bounds are out of form: not finite, a minimum above
        // the maximum, the default outside them, a quaternion of zero
        // length, a bool or trigger not 0 or 1; item is the parameter.
        maniBuildReasonParameter = 23,
        // A graph's clip slot without a clip, with a clip of another
        // skeleton (than the graph's, or its map's source), with a map
        // not made for the graph's skeleton or past
        // MANI_MAX_GRAPH_RETARGET_MAPS maps, or with a clip an earlier slot
        // (other) has. A learned
        // model trainer's clip whose identity is not its database's (item
        // the clip), or a clip count not the database's (item the count,
        // other the database's).
        maniBuildReasonClip = 24,
        // A graph node of an unknown kind, with an input count its kind
        // does not take, or with a field out of range: a clip slot, sync
        // group, loop or flags; a select's parameter not int or bool; a
        // seek's not a trigger; a motion matching node's database slot,
        // trigger, query (a parameter past the count or of the wrong type,
        // or not one per trajectory and float time of the database), tags,
        // history count or times out of range, or a path from the output
        // to it through no inertialize node; item is the node.
        maniBuildReasonNode = 25,
        // A node's input past the node count or the node itself, or the
        // output past the count; item is the node, other the input's place.
        maniBuildReasonInput = 26,
        // A node the output does not reach.
        maniBuildReasonUnreachable = 27,
        // A node's number naming a parameter past the count or not a
        // float, or a literal not finite or out of its range (a weight
        // below 0, a direct or additive weight above 1), or a time scale
        // reading a parameter without bounds or taking a path's product of
        // time scales past MANI_MAX_GRAPH_TIME_FACTOR; item is the node,
        // other the number's place among its numbers.
        maniBuildReasonScalar = 28,
        // A node mixing poses and differences, an additive node's base
        // that is a difference or a layer that is a pose, or an output of
        // differences; item is the node, other the input's place.
        maniBuildReasonDifference = 29,
        // A text that is not JSON (RFC 8259); item is the byte offset of the
        // first byte found wrong, other what broke: 1 the grammar or text
        // past the value, 2 nesting deeper than 128, 3 a text of 4 GiB or
        // more, 4 bytes not UTF-8, 5 an escape not defined or a lone
        // surrogate, 6 a control character in a string.
        maniBuildReasonJson = 30,
        // JSON that is not the form a text reader takes; item is the byte
        // offset of the value at fault (of an object missing a member),
        // other the kind of fault (maniGraphForm for graphs).
        maniBuildReasonForm = 31,
        // A blend space's point not finite, equal to an earlier one,
        // (wrapped) outside one period of the first, or a corner of no
        // authored triangle; item is the node, other the point.
        maniBuildReasonPoint = 32,
        // A blend space's authored triangle with a corner past its points,
        // clockwise or degenerate, or overlapping an earlier one; item is
        // the node, other the triangle.
        maniBuildReasonSpaceTriangle = 33,
        // A state machine's transition (or one of its conditions) out of
        // range, to its own source, or of a priority an earlier candidate
        // of the same state shares; item is the node, other the
        // transition.
        maniBuildReasonTransition = 34,
        // A motion database's channel of an unknown kind or axis, a bone
        // past the skeleton, no times or too many, a time not finite or
        // past MANI_MAX_MATCH_REACH, a weight not above 0, or a float
        // track with no name; item is the channel.
        maniBuildReasonChannel = 35,
        // A motion database's entry naming a clip past the clips, with
        // unknown flags, a span not finite or outside its clip (or empty
        // when it loops), or a cost bias not finite; item is the entry.
        maniBuildReasonEntry = 36,
        // A motion database's channel whose features never vary; item is
        // the channel.
        maniBuildReasonSpread = 37,
        // A float track channel whose track an entry's clip lacks; item is
        // the channel, other the entry.
        maniBuildReasonMissingTrack = 38,
        // A graph's database slot without a database, with one of another
        // skeleton or one an earlier slot (other) has, or with one listing
        // a clip no clip slot has (other is the database's clip); item is
        // the slot. A learned model's or trainer's database of another
        // skeleton (item 0).
        maniBuildReasonDatabase = 39,
        // A learned model's network with no layers or more
        // than MANI_MAX_LEARNED_LAYERS, a layer of no inputs or outputs or
        // past MANI_MAX_LEARNED_WIDTH, layers that do not chain, a first
        // input or last output count its role does not take, weights past
        // MANI_MAX_LEARNED_WEIGHTS in all, or a value not finite or a
        // scale of 0; item is the network, other the layer (the layer
        // count for the normalization).
        maniBuildReasonNetwork = 40,
        // A learned model trainer's Projector whose tag filter no frame of
        // an entry a jump may reach passes (item the Projector), or a
        // graph's learned node whose literal tags are no filter of its
        // model's Projectors (item the node).
        maniBuildReasonTags = 41,
        // A graph's model slot without a model, with one of another
        // skeleton or one an earlier slot (other) has; item is the slot.
        maniBuildReasonModel = 42,
        // A spring set's collider (item) on a bone out of range, of an
        // unknown shape, inside as a plane, with a radius negative or a
        // value not finite, or a plane's normal not of length 1.
        maniBuildReasonCollider = 43,
        // A spring set's collider group (item) naming a collider out of
        // range at its entry other.
        maniBuildReasonColliderGroup = 44,
        // A spring set's spring (item) of fewer than two joints, a joint
        // (other) on a bone out of range or not below the joint before
        // it, a group out of range (other the entry, past the joints), or
        // a center neither the first joint nor above it, or one on or
        // below another spring's joint (other the joint count).
        maniBuildReasonSpring = 45,
        // A spring set's joint (other) of a spring (item) with a value
        // outside its range: hit radius, stiffness or gravity power
        // negative, drag outside 0 to 1, a gravity direction not finite,
        // an unknown limit, a limit's angle negative or not finite or its
        // rotation not of length 1.
        maniBuildReasonSpringJoint = 46,
        // A graph's spring set slot without a set, with one of another
        // skeleton or one an earlier slot (other) has; item is the slot.
        maniBuildReasonSpringSetSlot = 47,
        // A ragdoll's body (item, in the source's order) on a bone out of
        // range or one an earlier body (other) takes, of an unknown shape,
        // a size or mass not positive and finite (a sphere's and
        // capsule's unused sizes 0), a frame not finite or its rotation
        // not of length 1, or a second root (other the first).
        maniBuildReasonRagdollBody = 48,
        // A ragdoll's joint (item the child body): its frame not finite or
        // its rotation not of length 1, its twist range outside [-pi, pi]
        // or empty, a swing half angle outside [0, pi], or a drive value
        // negative or not finite.
        maniBuildReasonRagdollJoint = 49,
        // A ragdoll's facing or up axis not of length 1, or the two not
        // perpendicular (within a thousandth).
        maniBuildReasonRagdollAxes = 50,
        // A retargeting map's reference pose (other 0 the source's, 1 the
        // target's): the bone's (item) local not finite or its rotation not
        // of length 1.
        maniBuildReasonRetargetReference = 51,
        // A retargeting map's pair (item; other its target bone): a bone
        // out of range or the target mapped before, a weight outside
        // [0, 1], an unknown translation.
        maniBuildReasonRetargetPair = 52,
        // A retargeting map's chain (item; other a target bone mapped
        // before, else 0): bones out of range, an end not below its start
        // within MANI_MAX_IK_CHAIN_BONES, an unknown mode or translation,
        // a weight outside [0, 1].
        maniBuildReasonRetargetChain = 53,
        // A retargeting map's pelvis: a bone out of range or its target
        // mapped before, the up axis not of length 1, a factor, blend or
        // weight out of range, a reference pelvis at or below height 0.
        maniBuildReasonRetargetPelvis = 54,
        // A clip to retarget (other 0 the source skeleton, 1 the target,
        // 2 the clip): a skeleton not its map's, a clip not of the map's
        // source.
        maniBuildReasonRetargetClip = 55,
        // A mirror map's reference pose: the bone's (item) local not
        // finite or its rotation not of length 1.
        maniBuildReasonMirrorReference = 56,
        // A mirror map's bone (item) and partner (other): a bone out of
        // range or in two pairs, a pair's bone a rule's partner pairs
        // with, or a rule's partner whose own rules name another bone.
        maniBuildReasonMirrorPair = 57,
        // A mirror map's rule (item): an empty find, an unknown place,
        // a find or replace not UTF-8.
        maniBuildReasonMirrorRule = 58,
        // A mirror map's name (item: a name pair, then a listed name
        // past the pairs; other the earlier): empty or not UTF-8, or
        // paired twice with different partners.
        maniBuildReasonMirrorName = 59,
        // A clip to mirror (other 0 the skeleton, 1 the clip): a skeleton
        // not its map's, a clip not of the map's skeleton.
        maniBuildReasonMirrorClip = 60,
        // A graph's mirroring (other 0 its mirror map of another skeleton,
        // item 0, or a subgraph's map not the graph's, item the subgraph
        // node; 1 a mirror node without a map, item the node; 3 a database
        // slot whose mirrored entries play through another map or none,
        // item the slot).
        maniBuildReasonMirrorGraph = 61,
        // A motion database's mirroring: its mirror map of another
        // skeleton (item 0), or a mirrored entry without a map (item the
        // entry).
        maniBuildReasonMirrorDatabase = 62,
        // A graph node reached both through an odd and an even number of
        // mirror nodes (its moves and events would be needed both ways at
        // once): item the node, other a reader that reaches it the other way
        // from an earlier one. Give each way its own copy of the nodes
        // beneath (a second subgraph instance).
        maniBuildReasonMirrorParity = 63,
        // The list a refused float track, discrete track, event or marker
        // is in: the reason's item indexes it. Bone tracks report plainly.
        maniBuildReasonFloatTrack = 0x100,
        maniBuildReasonDiscreteTrack = 0x200,
        maniBuildReasonEvent = 0x300,
        maniBuildReasonMarker = 0x400,
        maniBuildReasonParameterList = 0x500,
        maniBuildReasonClipList = 0x600,
        maniBuildReasonSyncGroupList = 0x700,
        maniBuildReasonDatabaseList = 0x800,
        maniBuildReasonModelList = 0x900,
        maniBuildReasonSpringList = 0xA00,
        maniBuildReasonSpringSetList = 0xB00,
    };

    // A build's outcome: the status and, when the source was refused
    // (mani_errorInvalid) or passes a bound (mani_errorLimit), the reason,
    // the source item and a second item the reason names.
    typedef struct maniBuildResult
    {
        maniResult result;
        maniBuildReason reason;
        uint32_t item;
        uint32_t other;
    } maniBuildResult;

    // A bone as a source gives it.
    typedef struct maniBoneSource
    {
        // The name, UTF-8, not empty, unique in the skeleton.
        const char* name;
        size_t nameLength;
        // The parent's index in the source's bones, or -1 for a root.
        int32_t parent;
        // The rest transform relative to the parent; the rotation is
        // normalized by the build.
        maniTransform rest;
    } maniBoneSource;

    // A skeleton's bones in any order, each naming its parent by index.
    typedef struct maniSkeletonSource
    {
        const maniBoneSource* bones;
        uint32_t boneCount;
    } maniSkeletonSource;

    /// Builds a skeleton file: checks every bone, orders the bones
    /// depth-first (children and roots in the source's order), normalizes
    /// rotations, and keeps the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The bones.
    /// @return `mani_success`; `mani_errorLimit` with
    /// `maniBuildReasonCount` for more than MANI_MAX_BONES bones (no
    /// misuse); `mani_errorInvalid` for a NULL argument (a misuse) or with
    /// the reason and source bone for a refused source;
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildSkeleton(maniBuilder* builder,
                                                              const maniSkeletonSource* source);

    // A skin: the bones a mesh is bound to, in the mesh's order, with an
    // inverse bind matrix each.
    typedef struct maniSkinSource
    {
        // The opened skeleton the skin is for.
        const maniSkeleton* skeleton;
        // jointCount skeleton bones, 1 to MANI_MAX_BONES of them.
        const uint32_t* joints;
        // jointCount matrices, finite.
        const maniAffine* inverseBinds;
        uint32_t jointCount;
    } maniSkinSource;

    /// Builds a skin file bound to the source's skeleton, and keeps it in
    /// place of any file built before.
    ///
    /// @param builder  The builder.
    /// @param source   The skin.
    /// @return `mani_success`; `mani_errorLimit` with
    /// `maniBuildReasonCount` for more than MANI_MAX_BONES joints (no
    /// misuse); `mani_errorInvalid` for a NULL argument (a misuse), or
    /// with the reason and joint for a refused source (no joints, a bone
    /// past the skeleton's, a matrix not finite); `mani_errorCapacity`
    /// when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildSkin(maniBuilder* builder,
                                                          const maniSkinSource* source);

    // A mesh primitive's influences: any number a vertex,
    // vertex v's in [offsets[v], offsets[v + 1]) of joints and weights.
    typedef struct maniInfluenceSource
    {
        // The opened skin the joints index.
        const maniSkin* skin;
        // vertexCount + 1 offsets, increasing from 0.
        const uint32_t* offsets;
        // Joints of the skin, and their weights (finite, not negative).
        const uint32_t* joints;
        const float* weights;
        // NULL, or per vertex the blend from linear (0) to dual
        // quaternion (1) skinning, FBX's blended method.
        const float* dualQuaternionWeights;
        uint32_t vertexCount;
    } maniInfluenceSource;

    /// Builds an influences file bound to the source's skin: zero weights
    /// dropped, each vertex's influences sorted by decreasing weight then
    /// joint, padded to groups of four with joint 0 at weight 0 and
    /// renormalized to sum 1 (in binary64, then rounded); a vertex with no
    /// weight keeps all 0, and skinning leaves it as it is. Keeps the file
    /// in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The influences.
    /// @return `mani_success`; `mani_errorLimit` with
    /// `maniBuildReasonCount` for more than MANI_MAX_INFLUENCES weighted
    /// influences on a vertex (item the vertex, other the first past the
    /// bound; no misuse); `mani_errorInvalid` for a NULL argument (a
    /// misuse), or with the reason and item for a refused source (item a
    /// vertex, other an influence index): `maniBuildReasonCount` for no
    /// vertices or offsets not increasing from 0, `maniBuildReasonJoint`,
    /// `maniBuildReasonNotFinite`, `maniBuildReasonWeight`,
    /// `maniBuildReasonRepeatedJoint`; `mani_errorCapacity` when memory
    /// runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildInfluences(maniBuilder* builder,
                                                                const maniInfluenceSource* source);

    // What centers of rotation are built from: influences, the
    // rest mesh's positions (binary32 x, y, z a vertex, stride in bytes)
    // and its triangles.
    typedef struct maniCenterSource
    {
        // Opened influences; the centers are added to a copy of them.
        const maniInfluences* influences;
        // The influences' vertex count of positions, at least 12 bytes
        // apart.
        const float* positions;
        size_t stride;
        // triangleCount triples of vertex indices.
        const uint32_t* triangles;
        uint32_t triangleCount;
        // The similarity's width; 0 for 0.1, Le and Hodgins' value.
        float sigma;
    } maniCenterSource;

    /// Builds the influences again with a center of rotation for every
    /// vertex (Le and Hodgins 2016, Eq. 4): the triangles' centroids
    /// weighted by their areas and by the similarity of their average
    /// weights to the vertex's, in binary64. A vertex similar to no
    /// triangle (a single influence, say) takes its rest position. Keeps
    /// the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The influences, mesh and sigma.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument (a
    /// misuse), or with the reason and item for a refused source:
    /// `maniBuildReasonNotFinite` (a position; item the vertex),
    /// `maniBuildReasonTriangle`, `maniBuildReasonOption` (sigma below 0
    /// or not finite), `maniBuildReasonCount` (a stride under 12);
    /// `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildCenters(maniBuilder* builder,
                                                             const maniCenterSource* source);

    // How a channel's keys are joined, as glTF's samplers define it.
    typedef uint32_t maniInterpolation;
    enum
    {
        maniInterpolationLinear = 0,
        maniInterpolationStep = 1,
        maniInterpolationCubicSpline = 2,
    };

    // A channel's keys: keyCount strictly increasing times from 0, and per
    // key one value of 3 floats (translation, scale) or 4 (rotation, x y z
    // w), or for a cubic spline three: in-tangent, value, out-tangent. No
    // keys: the channel is the rest's.
    typedef struct maniChannelSource
    {
        const float* times;
        const float* values;
        uint32_t keyCount;
        maniInterpolation interpolation;
    } maniChannelSource;

    // A bone's channels.
    typedef struct maniBoneTrackSource
    {
        uint32_t bone;
        maniChannelSource translation;
        maniChannelSource rotation;
        maniChannelSource scale;
    } maniBoneTrackSource;

    // A named curve: keys of one float, linear, step or a cubic spline
    // (morph weights, host curves). Sampled on the clip's grid; a step
    // track stays a step at playback, the others interpolate linearly.
    typedef struct maniFloatTrackSource
    {
        // UTF-8, not empty, unique among float tracks.
        const char* name;
        size_t nameLength;
        // One float a key, or three for a cubic spline.
        maniChannelSource keys;
    } maniFloatTrackSource;

    // A named whole number held from each key to the next.
    typedef struct maniDiscreteTrackSource
    {
        // UTF-8, not empty, unique among discrete tracks.
        const char* name;
        size_t nameLength;
        // At least one key, times strictly increasing within [0, duration].
        uint32_t keyCount;
        const float* times;
        const int32_t* values;
    } maniDiscreteTrackSource;

    // An event (with a payload for the host) or a sync marker (payload
    // ignored) at a time within [0, duration].
    typedef struct maniEventSource
    {
        float time;
        int32_t payload;
        // UTF-8, not empty; names repeat freely.
        const char* name;
        size_t nameLength;
    } maniEventSource;

    // A clip: tracks for some of a skeleton's bones, in any order.
    typedef struct maniClipSource
    {
        // The opened skeleton the clip is for.
        const maniSkeleton* skeleton;
        const maniBoneTrackSource* boneTracks;
        const maniFloatTrackSource* floatTracks;
        const maniDiscreteTrackSource* discreteTracks;
        const maniEventSource* events;
        const maniEventSource* markers;
        uint32_t boneTrackCount;
        uint32_t floatTrackCount;
        uint32_t discreteTrackCount;
        uint32_t eventCount;
        uint32_t markerCount;
        // maniLoopClamp or maniLoopRepeat.
        uint32_t loop;
        // Samples per second, 0 for 30.
        float sampleRate;
        // Seconds, 0 for the last key's time.
        float duration;
        // maniAdditiveNone for a clip of poses; otherwise each channel is
        // stored as its difference from the basis.
        maniAdditiveBasis additive;
        // NULL, or the root motion to take out of a bone: kept
        // beside the clip, the bone left without it.
        const maniRootMotionSource* rootMotion;
    } maniClipSource;

    /// Builds a full-precision clip file: evaluates every channel at each
    /// sample time as glTF defines its interpolation (in binary64,
    /// rotations normalized and kept in one hemisphere from sample to
    /// sample), and keeps the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The clip.
    /// @return `mani_success`; `mani_errorLimit` with
    /// `maniBuildReasonTiming` for more than MANI_MAX_SAMPLES samples (no
    /// misuse); `mani_errorInvalid` for a NULL argument (a misuse), or
    /// with the reason and track for a refused source;
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildClip(maniBuilder* builder,
                                                          const maniClipSource* source);

    // The largest error of one kind, where it is: the error (metres or
    // the source's unit for translations, scales and floats; radians for
    // rotations; seconds for step shifts), the time, and the item, a bone
    // for bone channels and a source index for float tracks. Zero error,
    // time and item where there is nothing to measure.
    typedef struct maniClipError
    {
        double error;
        float time;
        uint32_t item;
    } maniClipError;

    // How far a clip plays from its source.
    typedef struct maniClipReport
    {
        // At the sample times: what the clip stores against the source;
        // the rounding to binary32, or a compressed format's error.
        maniClipError sampleTranslation;
        maniClipError sampleRotation;
        maniClipError sampleScale;
        maniClipError sampleFloat;
        // Halfway between samples: playback against the source curve;
        // what raising the rate lowers. Step channels are left out.
        maniClipError midTranslation;
        maniClipError midRotation;
        maniClipError midScale;
        maniClipError midFloat;
        // How late a step key plays, moved to the first sample at or
        // after it; the time is the key's.
        maniClipError boneStepShift;
        maniClipError floatStepShift;
    } maniClipReport;

    /// Measures a clip against the source it was built from, at every
    /// sample time and halfway between samples, through the clip's own
    /// sampling, against the source evaluated in binary64. Rotations are
    /// compared by the angle between them.
    ///
    /// @param builder    A builder, for its allocator.
    /// @param source     The clip's source, as maniBuildClip accepts it.
    /// @param clip       The clip opened from what the build made.
    /// @param reportOut  Receives the report.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// source the build refuses, or a clip not of the source's skeleton or
    /// missing one of its float tracks; `mani_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniMeasureClip(maniBuilder* builder,
                                                       const maniClipSource* source,
                                                       const maniClip* clip,
                                                       maniClipReport* reportOut);

    // How a clip is compressed, in skeleton units: the largest
    // distance a point on a shell around each bone may move, through the
    // hierarchy, and the shell's radius.
    typedef struct maniCompressOptions
    {
        float precision;
        float shellDistance;
        // Per bone in skeleton order, or NULL for the values above.
        const float* bonePrecisions;
        const float* boneShellDistances;
        // A float track's largest error, or per float track in the clip's
        // track order (NULL for the value).
        float floatPrecision;
        const float* floatPrecisions;
    } maniCompressOptions;

    /// Returns the default compression options: precision 0.0001 and shell
    /// distance 0.03 (0.1 mm at 3 cm when the unit is the metre), float
    /// tracks within 0.00001.
    ///
    /// @return The options.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniCompressOptions maniDefaultCompressOptions(void);

    // What a compression made: the file's size, and the largest errors at
    // the samples against the full-precision clip: the farthest a shell
    // point moved (skeleton units), with its sample and bone, and a float
    // track's (with its sample and track).
    typedef struct maniCompressReport
    {
        uint64_t bytes;
        double shellError;
        uint32_t shellSample;
        uint32_t shellBone;
        double floatError;
        uint32_t floatSample;
        uint32_t floatTrack;
    } maniCompressReport;

    /// Compresses a full-precision clip into the builder:
    /// default and constant channels folded, segments of about 16
    /// samples, ranges over the clip and the segment, and per segment the
    /// fewest bits that keep every bone's shell and every float track
    /// within its tolerance at every sample. The same input gives the same
    /// bytes on every platform.
    ///
    /// @param builder    The builder.
    /// @param clip       A full-precision clip.
    /// @param skeleton   Its skeleton.
    /// @param options    The tolerances.
    /// @param reportOut  Receives the report. May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// compressed clip, a skeleton not the clip's, or a tolerance not
    /// finite and above 0; `mani_errorCapacity` when the allocator fails.
    /// A NULL argument or a bad tolerance counts one misuse on the
    /// builder; a compressed clip or another skeleton's counts none, as
    /// the clip's file decides them.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniCompressClip(maniBuilder* builder, const maniClip* clip,
                                                        const maniSkeleton* skeleton,
                                                        const maniCompressOptions* options,
                                                        maniCompressReport* reportOut);

    // A spring bone collider's shape.
    typedef uint32_t maniColliderShape;
    enum
    {
        maniColliderSphere = 0,
        maniColliderCapsule = 1,
        // VRMC_springBone_extended_collider's plane.
        maniColliderPlane = 2,
    };

    // A collider on a bone: a sphere at offset, a capsule from offset to
    // tail, or a plane through offset whose unit normal is tail, in the
    // bone's local space. A sphere or capsule keeps tails out of it, or
    // in it when inside is 1 (the extended collider's inside shapes); a
    // plane keeps them on its normal's side.
    typedef struct maniColliderSource
    {
        uint32_t bone;
        maniColliderShape shape;
        maniVec3 offset;
        maniVec3 tail;
        float radius;
        uint32_t inside;
    } maniColliderSource;

    // Colliders that springs collide with together.
    typedef struct maniColliderGroupSource
    {
        const char* name;
        size_t nameLength;
        uint32_t colliderCount;
        const uint32_t* colliders;
    } maniColliderGroupSource;

    // A spring joint's angle limit (VRMC_springBone_limit): none, a cone
    // (angle), a hinge (angle, turning about the limit's x only) or a
    // spherical limit (pitch about x, yaw about z), each opening along
    // the limit's +y.
    typedef uint32_t maniSpringLimit;
    enum
    {
        maniSpringLimitNone = 0,
        maniSpringLimitCone = 1,
        maniSpringLimitHinge = 2,
        maniSpringLimitSpherical = 3,
    };

    // A spring chain's joint: a bone and the motion of the segment it
    // heads (the last joint is only a tail), as VRM defines them, per
    // frame of the set's reference rate; its limit's angles in radians
    // (angle the cone's or hinge's, or the spherical limit's pitch; yaw
    // the spherical limit's) and the rotation from the limit's default
    // orientation. The last joint's limit is ignored.
    typedef struct maniSpringJointSource
    {
        uint32_t bone;
        float hitRadius;
        float stiffness;
        float gravityPower;
        maniVec3 gravityDirection;
        // Air resistance, 0 to 1.
        float drag;
        maniSpringLimit limit;
        float limitAngle;
        float limitYaw;
        maniQuat limitRotation;
    } maniSpringJointSource;

    // A chain of joints, each an ancestor of the next, colliding with
    // the colliders of its groups, simulated in its center bone's space
    // (MANI_NO_BONE for the model's).
    typedef struct maniSpringSource
    {
        const char* name;
        size_t nameLength;
        uint32_t jointCount;
        const maniSpringJointSource* joints;
        const uint32_t* colliderGroups;
        uint32_t colliderGroupCount;
        uint32_t center;
    } maniSpringSource;

    // A skeleton's spring bones: colliders, collider groups, springs;
    // the rate its joints' values are per frame of (0 for 60 Hz) and the
    // rate it steps at (0 for the reference rate), both up to
    // MANI_MAX_SPRING_RATE.
    typedef struct maniSpringSetSource
    {
        const maniSkeleton* skeleton;
        const maniColliderSource* colliders;
        const maniColliderGroupSource* colliderGroups;
        const maniSpringSource* springs;
        uint32_t colliderCount;
        uint32_t colliderGroupCount;
        uint32_t springCount;
        float referenceRate;
        float stepRate;
    } maniSpringSetSource;

#define MANI_MAX_SPRING_RATE 1000.0f

    /// Builds a spring set: the source checked (bones, shapes,
    /// chains and centers as VRMC_springBone and its extensions require,
    /// every value in its range), each joint's drag, stiffness and gravity
    /// converted from a frame of the reference rate to a step of the step
    /// rate, colliders listed per spring.
    ///
    /// @param builder  A builder; its last output is replaced on success.
    /// @param source   The set's source.
    /// @return `mani_success`; `mani_errorLimit` with
    /// `maniBuildReasonOption` (item 0 the reference rate, 1 the step
    /// rate, past MANI_MAX_SPRING_RATE) or `maniBuildReasonCount` (more
    /// than MANI_MAX_BONES springs), no misuse; `mani_errorInvalid` with
    /// the reason, item and other for a refused source (or NULL
    /// arguments, a misuse); `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildSpringSet(maniBuilder* builder,
                                                               const maniSpringSetSource* source);

    // A ragdoll body's joint to its parent body: its frame in
    // the body (the parent's frame of it is the same place in the rest
    // pose), swing-twist limits about the frame's axes and an optional
    // drive. A root's joint is ignored.
    typedef struct maniRagdollJointSource
    {
        maniVec3 translation;
        maniQuat rotation;
        // Twist about x within [twistMin, twistMax], -pi <= twistMin <=
        // twistMax <= pi (equal, the twist held there); swing about y and
        // z within half angles each in [0, pi] (0 locks that axis, pi
        // leaves it free).
        float twistMin;
        float twistMax;
        float swingY;
        float swingZ;
        // Stiffness, damping and the largest torque, at least 0 and in
        // the host engine's units (passed through); all 0 for no drive.
        float driveStiffness;
        float driveDamping;
        float driveMaxTorque;
    } maniRagdollJointSource;

    // A body on a bone: its frame in the bone's model frame (rotation of
    // length 1), its shape at the frame's origin (maniRagdollShape's
    // sizes) and mass, its joint to the body of the bone's nearest
    // ancestor with one, an optional name (UTF-8).
    typedef struct maniRagdollBodySource
    {
        const char* name;
        size_t nameLength;
        uint32_t bone;
        maniVec3 frameTranslation;
        maniQuat frameRotation;
        maniRagdollShape shape;
        maniVec3 size;
        float mass;
        maniRagdollJointSource joint;
    } maniRagdollBodySource;

    // A skeleton's ragdoll: its bodies in any order (one, the root, with
    // no ancestor body; every other below it), and the root body's facing
    // (out of the chest) and up axes in its frame, for getting up.
    typedef struct maniRagdollSource
    {
        const maniSkeleton* skeleton;
        const maniRagdollBodySource* bodies;
        uint32_t bodyCount;
        maniVec3 facing;
        maniVec3 up;
    } maniRagdollSource;

    /// Builds a ragdoll profile: the source checked (bones,
    /// one root, shapes, sizes, masses, frames, limits, drives, the axes),
    /// the bodies ordered by bone, each joint's frame in its parent body
    /// worked out from the skeleton's rest pose.
    ///
    /// @param builder  A builder; its last output is replaced on success.
    /// @param source   The profile's source.
    /// @return `mani_success`; `mani_errorInvalid` with the reason, item
    /// and other for a refused source (or NULL arguments, a misuse;
    /// maniBuildReasonCount for no bodies or more than the skeleton's
    /// bones); `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildRagdoll(maniBuilder* builder,
                                                             const maniRagdollSource* source);

    /// Copies the last built file, or the last written text, into caller
    /// memory.
    ///
    /// @param builder   The builder.
    /// @param bufferOut Receives the bytes: at least capacity bytes; may be
    ///                  NULL when capacity is 0. Open a file's copy from a
    ///                  16-byte aligned address.
    /// @param capacity  The buffer's size in bytes.
    /// @param sizeOut   Receives the size in bytes. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when the buffer is
    /// smaller than the file or text, with the size still written;
    /// `mani_errorInvalid` for a NULL builder, a NULL buffer with a
    /// positive capacity, or a builder holding neither.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniCopyBuilt(maniBuilder* builder, void* bufferOut,
                                                     size_t capacity, size_t* sizeOut);

    /// Copies where each source bone of the last built skeleton went: the
    /// skeleton's index of source bone i at boneOut[i].
    ///
    /// @param builder   The builder.
    /// @param boneOut   Receives up to capacity indices; may be NULL when
    ///                  capacity is 0.
    /// @param capacity  The array's length.
    /// @param countOut  Receives the source's bone count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when the array is
    /// shorter than the count, with the count still written and the array
    /// filled as far as it goes; `mani_errorInvalid` for a NULL builder, a
    /// NULL array with a positive capacity, or a builder holding no
    /// skeleton.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniCopySkeletonOrder(maniBuilder* builder,
                                                             uint32_t* boneOut, uint32_t capacity,
                                                             uint32_t* countOut);

    // An attribute morph targets move: its semantic and set (0 for
    // positions, normals and tangents).
    typedef struct maniMorphAttributeSource
    {
        maniMorphSemantic semantic;
        uint32_t set;
    } maniMorphAttributeSource;

    // A morph target as a source gives it: deltas for some of its
    // vertices (any order; a vertex given twice takes the sum) or for all.
    typedef struct maniMorphTargetSource
    {
        // The channel weight at which the target applies in full: 1 for
        // glTF, FBX's FullWeights over 100 for in-betweens.
        float fullWeight;
        // The vertices given.
        uint32_t count;
        // count vertex indices, or NULL for every vertex in order (count
        // then the vertex count).
        const uint32_t* indices;
        // A pointer an attribute of the source's: its deltas, its width
        // of floats (3, 2 or 4, maniMorphSemantic) a given vertex, or NULL
        // where the target leaves it. NULL for none at all.
        const float* const* deltas;
        // The name, UTF-8, not empty; NULL for the channel's name followed
        // by '/' and the target's index within the channel past the first.
        const char* name;
        size_t nameLength;
    } maniMorphTargetSource;

    // A named channel: its targets at increasing full weights.
    typedef struct maniMorphChannelSource
    {
        // The name, UTF-8, not empty, unique among the channels.
        const char* name;
        size_t nameLength;
        uint32_t targetCount;
        const maniMorphTargetSource* targets;
    } maniMorphChannelSource;

    // A mesh primitive's morph targets.
    typedef struct maniMorphSource
    {
        const maniMorphChannelSource* channels;
        uint32_t channelCount;
        uint32_t vertexCount;
        // The attributes the targets move, at most
        // MANI_MAX_MORPH_ATTRIBUTES.
        const maniMorphAttributeSource* attributes;
        uint32_t attributeCount;
    } maniMorphSource;

    /// Builds a morph targets file: checks every channel and target, sorts
    /// each sparse target's vertices and sums a vertex given twice (in
    /// binary64, in the source's order), stores a target touching every
    /// vertex densely, and keeps the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The channels.
    /// @return `mani_success`; `mani_errorLimit` with
    /// `maniBuildReasonCount` for more than MANI_MAX_MORPH_ATTRIBUTES
    /// attributes (item 0; no misuse); `mani_errorInvalid` for a NULL
    /// argument (a misuse), or with the reason and item for a refused
    /// source (item a
    /// channel for `maniBuildReasonCount` on a channel without targets,
    /// `maniBuildReasonName` and `maniBuildReasonDuplicateName`; a
    /// target, counted over the channels in order, otherwise):
    /// `maniBuildReasonName` for a target's name has item the target and
    /// other 1.
    /// `maniBuildReasonCount` (no channels, no vertices, or an index-less
    /// target whose count is not the vertex count), `maniBuildReasonAttribute`
    /// (item the attribute), `maniBuildReasonName`,
    /// `maniBuildReasonDuplicateName` (other the earlier channel),
    /// `maniBuildReasonWeight`, `maniBuildReasonVertex`,
    /// `maniBuildReasonNotFinite` (other the given vertex's place);
    /// `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildMorphTargets(maniBuilder* builder,
                                                                  const maniMorphSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_BUILD_H
