// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a graph file, offline: a graph's parameters, clip
// slots, sync groups and nodes, checked against one skeleton and its
// clips, ordered so each node follows its inputs, its poses given slots,
// and written.

#ifndef MAUL_ANIM_GRAPHBUILD_H
#define MAUL_ANIM_GRAPHBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/ik.h"
#include "maul-anim/match.h"
#include "maul-anim/mirror.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A number a node takes: the value of a float parameter, or a literal
    // value when parameter is MANI_NO_PARAMETER.
    typedef struct maniGraphScalar
    {
        uint32_t parameter;
        float value;
    } maniGraphScalar;

    // A parameter: its name (UTF-8, not empty, unique in the graph), type,
    // default and, for int and float when bounded is 1, the bounds that
    // clamp every value set (minimum at most maximum, the default within).
    typedef struct maniGraphParameterSource
    {
        const char* name;
        size_t nameLength;
        maniGraphParameterType type;
        maniGraphValue defaultValue;
        maniGraphValue minimum;
        maniGraphValue maximum;
        uint32_t bounded;
    } maniGraphParameterSource;

    // A clip slot: an opened clip of the graph's skeleton, each clip once,
    // and a name for it (UTF-8, not empty, unique in the graph). The graph
    // keeps the clip's identity; binding finds the clip by it. With a
    // retargeting map, the clip is of the map's source skeleton and plays
    // carried through the map, which is made for the graph's skeleton;
    // the graph keeps a retargeting map slot per map.
    typedef struct maniGraphClipSource
    {
        const maniClip* clip;
        const char* name;
        size_t nameLength;
        const maniRetargetMap* retargetMap;
    } maniGraphClipSource;

    // A motion database slot: an opened database of the graph's skeleton,
    // each database once, every clip it lists among the graph's clip
    // slots, and a name for it (UTF-8, not empty, unique among the
    // graph's databases). The graph keeps the database's identity and its
    // clips' slots; binding finds the database by its identity.
    typedef struct maniGraphDatabaseSource
    {
        const maniMotionDatabase* database;
        const char* name;
        size_t nameLength;
    } maniGraphDatabaseSource;

    // A learned model slot: an opened learned model of the
    // graph's skeleton, each model once, and a name for it (UTF-8, not
    // empty, unique among the graph's models). The graph keeps the
    // model's identity; binding finds the model by it.
    typedef struct maniGraphModelSource
    {
        const maniLearnedModel* model;
        const char* name;
        size_t nameLength;
    } maniGraphModelSource;

    // A spring set slot: an opened spring set of the graph's
    // skeleton, each set once, and a name for it (UTF-8, not empty,
    // unique among the graph's spring sets). The graph keeps the set's
    // identity; binding finds the set by it.
    typedef struct maniGraphSpringSetSource
    {
        const maniSpringSet* set;
        const char* name;
        size_t nameLength;
    } maniGraphSpringSetSource;

    // A sync group's flags.
    enum
    {
        // Every member's events fire, not only the leader's.
        maniGraphSyncAllEvents = 1,
    };

    // A sync group: its name (UTF-8, not empty, unique in the graph) and
    // flags.
    typedef struct maniGraphSyncGroupSource
    {
        const char* name;
        size_t nameLength;
        uint32_t flags;
    } maniGraphSyncGroupSource;

    // How a clip node loops: as its clip says, or overridden.
    typedef uint32_t maniGraphLoop;
    enum
    {
        maniGraphLoopClip = 0,
        maniGraphLoopClamp = 1,
        maniGraphLoopRepeat = 2,
    };

    // A clip node's flags.
    enum
    {
        // Leads its sync group whatever its weight.
        maniGraphClipLeader = 1,
    };

    // A clip node: its clip slot, sync group (or MANI_NO_SYNC_GROUP),
    // speed (negative plays backward), loop and flags.
    typedef struct maniGraphClipNodeSource
    {
        uint32_t clip;
        uint32_t syncGroup;
        maniGraphScalar speed;
        maniGraphLoop loop;
        uint32_t flags;
    } maniGraphClipNodeSource;

    // A blend node: a weight per input (not negative), normalized unless
    // direct is 1 (then each at most 1, for weights set by the host).
    typedef struct maniGraphBlendNodeSource
    {
        const maniGraphScalar* weights;
        uint32_t direct;
    } maniGraphBlendNodeSource;

    // A mask node: chains making per-bone weights (each 0 to 1) as
    // maniSkeleton_BuildMask makes them; its first input takes each bone's weight,
    // its second the remainder.
    typedef struct maniGraphMaskNodeSource
    {
        const maniMaskChain* chains;
        uint32_t chainCount;
        float defaultWeight;
    } maniGraphMaskNodeSource;

    // An additive node: a weight per layer (each input after the first),
    // 0 to 1.
    typedef struct maniGraphAdditiveNodeSource
    {
        const maniGraphScalar* weights;
    } maniGraphAdditiveNodeSource;

    // A select node: the int or bool parameter choosing an input (an int
    // clamped to the inputs).
    typedef struct maniGraphSelectNodeSource
    {
        uint32_t parameter;
    } maniGraphSelectNodeSource;

    // A time scale node: the factor on time below it (negative plays
    // backward). A parameter it reads has bounds, and the product of the
    // time scales' sizes on any path from the output, a parameter's at
    // the larger size of its bounds, is at most MANI_MAX_GRAPH_TIME_FACTOR.
    typedef struct maniGraphTimeScaleNodeSource
    {
        maniGraphScalar scale;
    } maniGraphTimeScaleNodeSource;

    // A seek node: the trigger that sets every playhead below it to time.
    typedef struct maniGraphSeekNodeSource
    {
        uint32_t trigger;
        maniGraphScalar time;
    } maniGraphSeekNodeSource;

    // A source-only node kind: another graph inlined in its place at
    // build (its nodes appended, its output standing where this node is).
    enum
    {
        maniGraphNodeSubgraph = 64,
    };

    // How a condition compares a parameter with its value.
    typedef uint32_t maniGraphCompare;
    enum
    {
        maniGraphCompareEqual = 0,
        maniGraphCompareNotEqual = 1,
        maniGraphCompareLess = 2,
        maniGraphCompareAtMost = 3,
        maniGraphCompareGreater = 4,
        maniGraphCompareAtLeast = 5,
    };

    // What a transition's condition tests.
    typedef uint32_t maniGraphConditionKind;
    enum
    {
        // A bool, int or float parameter compared with value (bool: equal
        // or not equal; float: no equality).
        maniGraphConditionParameter = 0,
        // A trigger parameter set; the transition firing consumes it.
        maniGraphConditionTrigger = 1,
        // At least value.f seconds (finite, not negative) since the
        // machine entered its current state.
        maniGraphConditionTimeInState = 2,
        // The current state's leading clip (the clip node beneath it of
        // greatest effective weight) passed phase value.f (finite, not
        // negative) in the update before: below 1, its normalized
        // position at every loop; 1 or more, its loops since the state
        // was entered, once.
        maniGraphConditionPhase = 3,
        // The current state finished in the update before: its nearest
        // nested machine in a final state with no transition running, or
        // with none, its leading clip, not repeating, at its end.
        maniGraphConditionFinished = 4,
        // A clip node beneath the current state listed an event of the
        // condition's name in the update before.
        maniGraphConditionEvent = 5,
    };

    // A condition of a transition: its kind, the parameter it reads, how
    // it compares and the value it compares with; an event condition's
    // name (UTF-8, not empty, eventLength bytes).
    typedef struct maniGraphConditionSource
    {
        maniGraphConditionKind kind;
        uint32_t parameter;
        maniGraphCompare compare;
        size_t eventLength;
        maniGraphValue value;
        const char* event;
    } maniGraphConditionSource;

    // While a transition runs, which others may fire.
    typedef uint32_t maniGraphInterruption;
    enum
    {
        // None until it completes.
        maniGraphInterruptionNone = 0,
        // Those of a lower priority value.
        maniGraphInterruptionHigher = 1,
        // Any whose conditions hold.
        maniGraphInterruptionAny = 2,
    };

    // How a transition's destination state takes up its time.
    typedef uint32_t maniGraphEntry;
    enum
    {
        // From its start: every playhead below it set to 0 when it is
        // entered at weight 0 (one still weighted keeps its time).
        maniGraphEntryRestart = 0,
        // Where it was left.
        maniGraphEntryKeep = 1,
        // At the source's leading clip's normalized position, each clip
        // below it at that fraction of its duration, when it is entered
        // at weight 0 (as a restart).
        maniGraphEntryMatch = 2,
    };

    // The curve of a cross-fade over its elapsed fraction.
    typedef uint32_t maniGraphFade;
    enum
    {
        maniGraphFadeLinear = 0,
        maniGraphFadeSmoothstep = 1,
        // At once, its duration requested of the inertialize nodes above
        // the machine (one on every path to the output, or the build
        // refuses it).
        maniGraphFadeInertial = 2,
    };

    // A transition of a state machine: from a state (or
    // MANI_GRAPH_ANY_STATE) to another, its priority (lower first; no two
    // equal among a state's own and the any-state transitions), its
    // interruption, entry and fade, the fade's duration in seconds (0
    // completes at once), the seconds before it may fire again, and its
    // conditions (all must hold; none fires at once).
    typedef struct maniGraphTransitionSource
    {
        uint32_t from;
        uint32_t to;
        uint32_t priority;
        maniGraphInterruption interruption;
        maniGraphEntry entry;
        maniGraphFade fade;
        float duration;
        float cooldown;
        const maniGraphConditionSource* conditions;
        uint32_t conditionCount;
    } maniGraphTransitionSource;

    // A state machine node: its inputs are its states; the state it
    // starts in, its transitions, the most transitions it takes in one
    // update (1 to MANI_MAX_GRAPH_CHAIN; 0 for 4), and its final states
    // (a parent machine's finished condition reads them).
    typedef struct maniGraphMachineNodeSource
    {
        const maniGraphTransitionSource* transitions;
        uint32_t transitionCount;
        uint32_t entry;
        uint32_t chainLimit;
        uint32_t finalCount;
        const uint32_t* finals;
    } maniGraphMachineNodeSource;

    // A one-shot node: inputs base and shot. The fire trigger restarts
    // the shot and fades it in over fadeIn seconds; it fades out over
    // fadeOut seconds when its leading clip (the clip node beneath the
    // shot of greatest effective weight), not repeating, has no more
    // than fadeOut seconds of clip time left, or when the fade-out
    // trigger is set; the abort trigger stops it at once (both triggers
    // optional, MANI_NO_PARAMETER for none). The first set of abort, fire
    // and fade-out acts; each is consumed when read. Both fades follow
    // curve; mask, when not NULL, weighs the shot per bone (NULL: the
    // whole body).
    typedef struct maniGraphOneShotNodeSource
    {
        uint32_t fire;
        uint32_t fadeOutTrigger;
        uint32_t abort;
        maniGraphFade curve;
        float fadeIn;
        float fadeOut;
        const maniGraphMaskNodeSource* mask;
    } maniGraphOneShotNodeSource;

    // How an inertialize node eases out a request.
    typedef uint32_t maniGraphInertializeMethod;
    enum
    {
        // Bollo's inertialization: the offset from its last output to its
        // input decays to 0 by a quintic over the requested seconds.
        maniGraphInertialization = 0,
        // Holden's dead blending: its last output, extrapolated by its
        // speed decaying per axis, cross-faded into its input by a
        // smoothstep over the requested seconds.
        maniGraphDeadBlending = 1,
    };

    // An inertialize node: one input, whose discontinuities it eases out
    // by its method. Requests come from the inertial transitions of the
    // machines beneath it (the nearest such node above on their first
    // parents' path takes them) and from its trigger (MANI_NO_PARAMETER
    // for none), for duration seconds (not negative); the least of one
    // update's requests is taken. Dead blending's speed on each axis
    // decays with a half-life of halfLifeScale times the distance to the
    // input over the speed, clamped to halfLifeMin and halfLifeMax
    // seconds (0 < halfLifeMin <= halfLifeMax, halfLifeScale not
    // negative; Holden's 0.3, 0.1 and 1.0 are the JSON form's defaults);
    // inertialization takes all three 0.
    typedef struct maniGraphInertializeNodeSource
    {
        uint32_t trigger;
        maniGraphInertializeMethod method;
        maniGraphScalar duration;
        float halfLifeScale;
        float halfLifeMin;
        float halfLifeMax;
    } maniGraphInertializeNodeSource;

    // Tags a motion matching node reads: the bits of an int parameter, or
    // value when parameter is MANI_NO_PARAMETER.
    typedef struct maniGraphTags
    {
        uint32_t parameter;
        uint32_t value;
    } maniGraphTags;

    // A motion matching node: no inputs; it plays its database slot's
    // frames. Its query is its continuing frame's bone channels,
    // interpolated to the playhead toward its entry's next frame, and, for
    // each time of each trajectory and float track channel in channel
    // order, a parameter (queryCount of them: vec2 on the ground plane
    // for trajectory positions and facings, float for float tracks). It
    // searches when period seconds (above 0) have passed since its last
    // search, when its trigger is set (MANI_NO_PARAMETER for none), when
    // a clamped entry reaches its end, and when it has no entry. A search
    // takes the continuing bias and transition cost (finite), the jump
    // threshold, end exclusion and history time (seconds, not negative),
    // the last historyCount picks (0 to MANI_MAX_MATCH_HISTORY) and the
    // tags (maniMotionDatabase_Search). On a jump it requests duration
    // seconds of the nearest inertialize node above it, which the build
    // requires on every path from the output. It moves by the time its
    // parents give it, forward.
    typedef struct maniGraphMatchNodeSource
    {
        uint32_t database;
        uint32_t trigger;
        const uint32_t* query;
        uint32_t queryCount;
        uint32_t historyCount;
        float period;
        float jumpThreshold;
        float endExclusion;
        float historyTime;
        float continuingBias;
        float transitionCost;
        maniGraphTags requiredTags;
        maniGraphTags excludedTags;
        maniGraphScalar duration;
    } maniGraphMatchNodeSource;

    // A learned motion matching node: no inputs; it plays its
    // model slot's state. Its query is its state's features with, for
    // each time of each trajectory and float track channel of the model's
    // database in channel order, a parameter (queryCount of them: vec2
    // on the ground plane for trajectory positions and facings, float for
    // float tracks). It steps its state at the model's rate, the state
    // between steps taken between the two last, and projects its query
    // with no state, when its trigger is set (MANI_NO_PARAMETER for none)
    // and when period seconds (above 0) have passed since its last
    // projection, through the model's Projector whose filter equals its
    // tags (none such: no projection; literal tags must name one). A
    // projection whose features lie farther than transitionCost (finite,
    // not negative) from the state's becomes the state, and then the
    // node requests duration seconds of the nearest inertialize node
    // above it, which the build requires on every path from the output.
    // It moves by the time its parents give it, forward, and gives the
    // root's motion and the events whose states rise past 0.5.
    typedef struct maniGraphLearnedNodeSource
    {
        uint32_t model;
        uint32_t trigger;
        const uint32_t* query;
        uint32_t queryCount;
        float period;
        float transitionCost;
        maniGraphTags requiredTags;
        maniGraphTags excludedTags;
        maniGraphScalar duration;
    } maniGraphLearnedNodeSource;

    // A spring bone node: one input, a pose of the skeleton
    // (not differences), given with its spring set slot's joints turned
    // toward their tails as maniSpringState_Apply turns them. Its tails step
    // at the set's step rate by the time its parents give it (its
    // magnitude), when the graph is evaluated, toward the input pose just
    // computed, at most MANI_SPRING_MAX_STEPS steps an evaluation. The
    // character's place in the world is position (a vec3 parameter) and
    // rotation (a quat parameter), MANI_NO_PARAMETER for the origin and
    // the identity. Its reset trigger (MANI_NO_PARAMETER for none) and a
    // seek above it drop the motion, as maniSpringState_Reset does: the next
    // evaluation places every tail where the input puts it. Its force (a
    // vec3 parameter, MANI_NO_PARAMETER for none) is the external force
    // maniSpringState_Step takes. The set's state takes at most 2^24 words.
    typedef struct maniGraphSpringNodeSource
    {
        uint32_t springSet;
        uint32_t position;
        uint32_t rotation;
        uint32_t reset;
        uint32_t force;
    } maniGraphSpringNodeSource;

    // A turn limit node: one input, a pose (not
    // differences), given with each bone's local rotation turned from the
    // node's last output toward the input's by at most speed (radians a
    // second, finite and not negative; a parameter's value below 0 counts
    // as 0) times the seconds its parents gave it since its last
    // evaluation, over the bone's weight in mask (NULL for every bone at
    // weight 1), about the shorter arc's axis; translations and scales
    // pass. The weight is how much the limit holds: 0.5 allows twice the
    // turn, 0 leaves the bone as its input gives it. A speed of 0 holds
    // every bone of weight above 0 at the node's last output for as long
    // as it lasts, as the limit's end; to stop limiting, set the speed
    // high (FLT_MAX passes any turn) or the weights to 0. Its first
    // evaluation, the first after its reset trigger (MANI_NO_PARAMETER
    // for none) fires or a seek above it, and the first after an update
    // that did not reach it take the input as it is.
    typedef struct maniGraphTurnLimitNodeSource
    {
        maniGraphScalar speed;
        uint32_t reset;
        const maniGraphMaskNodeSource* mask;
    } maniGraphTurnLimitNodeSource;

    // A pose input node: no inputs, the pose its pose
    // parameter holds as the host last set it (the skeleton's rest until
    // set). Blended over the animation by a blend or mask node, it lays a
    // ragdoll over the whole body or part of it; an inertialize node
    // above eases the way back.
    typedef struct maniGraphPoseNodeSource
    {
        uint32_t parameter;
    } maniGraphPoseNodeSource;

    // A two-bone IK node: one input, a pose of the skeleton
    // (not differences), given with its chain solved as maniPose_SolveTwoBone
    // solves it: start, middle and end bones of the skeleton, each an
    // ancestor of the next; its goal (a vec3 parameter, in the model
    // space of the pose) and pole (a vec3 parameter, MANI_NO_PARAMETER for
    // none); the middle bone's hinge (finite, zero for none); its twist
    // (finite), soften and weight (in [0, 1]) numbers or float parameters,
    // a parameter's soften and weight clamped to [0, 1]. A weight of 0
    // passes the input.
    typedef struct maniGraphTwoBoneNodeSource
    {
        uint32_t start;
        uint32_t middle;
        uint32_t end;
        uint32_t goal;
        uint32_t pole;
        maniVec3 hinge;
        maniGraphScalar twist;
        maniGraphScalar soften;
        maniGraphScalar weight;
    } maniGraphTwoBoneNodeSource;

    // An aim IK node: one input, a pose of the skeleton (not
    // differences), given with its bones aimed as maniPose_SolveAim aims them:
    // count bones (1 to MANI_MAX_AIM_BONES) of the skeleton, each an
    // ancestor of the next, the aiming bone last; their shares (at least
    // 0, finite, not all 0), or NULL for equal shares; its goal and pole
    // parameters as a two-bone node's; the aiming bone's forward (of
    // length 1), offset and up (finite) in its local frame; its twist
    // (finite), cone (in [0, MANI_AIM_MAX_CONE]) and weight (in [0, 1])
    // numbers or float parameters, a parameter's cone and weight clamped.
    // A weight of 0 passes the input.
    typedef struct maniGraphAimNodeSource
    {
        const uint32_t* bones;
        const float* shares;
        uint32_t count;
        uint32_t goal;
        uint32_t pole;
        maniVec3 forward;
        maniVec3 offset;
        maniVec3 up;
        maniGraphScalar twist;
        maniGraphScalar cone;
        maniGraphScalar weight;
    } maniGraphAimNodeSource;

    // A chain IK node, CCD or FABRIK by its kind: one input, a
    // pose of the skeleton (not differences), given with its chain solved
    // as maniPose_SolveCcd or maniPose_SolveFabrik solves it: root and end bones,
    // the root an ancestor of the end, at most MANI_MAX_IK_CHAIN_BONES
    // on the path; its goal (a vec3 parameter, model space); a FABRIK
    // node's pole (a vec3 parameter, MANI_NO_PARAMETER for none; none
    // for CCD); the end's wanted rotation (a quat parameter,
    // MANI_NO_PARAMETER for none); limits, one per turning bone (the
    // path's bones but the end), or NULL for none; the iteration count,
    // tolerance and a CCD node's step (0 for FABRIK) as maniChainSolve
    // takes them; its rotation weight and weight (in [0, 1]) numbers or
    // float parameters, a parameter's value clamped to [0, 1]. A weight
    // of 0 passes the input.
    typedef struct maniGraphChainNodeSource
    {
        uint32_t root;
        uint32_t end;
        uint32_t goal;
        uint32_t pole;
        uint32_t rotation;
        uint32_t iterations;
        const maniJointLimit* limits;
        float tolerance;
        float step;
        maniGraphScalar rotationWeight;
        maniGraphScalar weight;
    } maniGraphChainNodeSource;

    // A full-body node's effector: its bone; its position
    // goal (a vec3 parameter, model space) and rotation goal (a quat
    // parameter, MANI_NO_PARAMETER for none, whose rotation weight is
    // then the number 0); its position and rotation weights (in [0, 1])
    // numbers or float parameters, a parameter's value clamped to [0, 1];
    // its strength and pull-chain weight as maniBodyEffector takes them.
    typedef struct maniGraphEffectorSource
    {
        uint32_t bone;
        uint32_t position;
        uint32_t rotation;
        maniGraphScalar positionWeight;
        maniGraphScalar rotationWeight;
        float strength;
        float pullChain;
    } maniGraphEffectorSource;

    // A full-body IK node, XPBD or SDLS by its kind: one
    // input, a pose of the skeleton (not differences), given with the
    // bones from its top to its effectors solved as maniPose_SolveBodyXpbd or
    // maniPose_SolveBodySdls solves them: the top and its mode; the effectors
    // (1 to MANI_MAX_BODY_IK_EFFECTORS); bone settings as maniBodyBone
    // (NULL for none); stretch (XPBD only), the iteration count and
    // tolerance as maniBodySolve takes them; its weight (in [0, 1]) a
    // number or a float parameter, a parameter's value clamped to [0, 1].
    // A weight of 0 passes the input, as does a solve the bound skeleton
    // refuses.
    typedef struct maniGraphBodyNodeSource
    {
        uint32_t top;
        maniBodyTopMode topMode;
        const maniGraphEffectorSource* effectors;
        const maniBodyBone* bones;
        uint32_t effectorCount;
        uint32_t boneCount;
        uint32_t iterations;
        bool stretch;
        float tolerance;
        maniGraphScalar weight;
    } maniGraphBodyNodeSource;

    typedef struct maniGraphSource maniGraphSource;

    // Where a blend space reads its position: a vec2 parameter, or when
    // vector is MANI_NO_PARAMETER the numbers x and y (a 1D space reads x
    // alone, its y unused).
    typedef struct maniGraphPosition
    {
        uint32_t vector;
        maniGraphScalar x;
        maniGraphScalar y;
    } maniGraphPosition;

    // A 1D blend space: a point per input (finite, no two equal), and a
    // period: 0 to clamp the position to the ends, or above 0 to wrap it
    // and blend across the seam (the points then within one period).
    typedef struct maniGraphBlendSpace1DNodeSource
    {
        maniGraphPosition position;
        const float* points;
        float period;
    } maniGraphBlendSpace1DNodeSource;

    // A 2D blend space by gradient bands: a point (x, y) per input
    // (finite, no two equal); polar 1 weighs directions and speeds about
    // the origin, 0 the coordinates.
    typedef struct maniGraphBandsNodeSource
    {
        maniGraphPosition position;
        const float* points;
        uint32_t polar;
    } maniGraphBandsNodeSource;

    // A 2D blend space by triangles: a point (x, y) per input (finite, no
    // two equal, 3 or more, not all on one line), and triangleCount
    // triangles of three inputs each, anticlockwise and not degenerate,
    // no two overlapping, every input a corner of one, at most 2n - 5 of
    // them for n points; or none, for the points' Delaunay triangulation.
    typedef struct maniGraphTrianglesNodeSource
    {
        maniGraphPosition position;
        const float* points;
        const uint32_t* triangles;
        uint32_t triangleCount;
    } maniGraphTrianglesNodeSource;

    // A subgraph node: the graph (of the same skeleton; its clips and
    // databases joined to this graph's by identity, its sync groups by
    // name, its mirror map this graph's or none; its lists held to the
    // limits of this graph's, and each slot and sync group its nodes name
    // within its own lists, as this graph's nodes name only this graph's),
    // and for each of its parameters the parameter of this graph it reads,
    // of the same type. It takes no inputs.
    typedef struct maniGraphSubgraphNodeSource
    {
        const maniGraphSource* graph;
        const uint32_t* parameters;
    } maniGraphSubgraphNodeSource;

    // A node: its kind, its inputs by index in the source's nodes, and the
    // part of the union its kind reads.
    typedef struct maniGraphNodeSource
    {
        maniGraphNodeKind kind;
        uint32_t inputCount;
        const uint32_t* inputs;
        union
        {
            maniGraphClipNodeSource clip;
            maniGraphBlendNodeSource blend;
            maniGraphMaskNodeSource mask;
            maniGraphAdditiveNodeSource additive;
            maniGraphSelectNodeSource select;
            maniGraphTimeScaleNodeSource timeScale;
            maniGraphSeekNodeSource seek;
            maniGraphBlendSpace1DNodeSource blendSpace1D;
            maniGraphBandsNodeSource bands;
            maniGraphTrianglesNodeSource triangulated;
            maniGraphMachineNodeSource machine;
            maniGraphOneShotNodeSource oneShot;
            maniGraphInertializeNodeSource inertialize;
            maniGraphMatchNodeSource match;
            maniGraphLearnedNodeSource learned;
            maniGraphSpringNodeSource spring;
            maniGraphTurnLimitNodeSource turnLimit;
            maniGraphPoseNodeSource pose;
            maniGraphTwoBoneNodeSource twoBone;
            maniGraphAimNodeSource aim;
            maniGraphChainNodeSource chain;
            maniGraphBodyNodeSource body;
            maniGraphSubgraphNodeSource subgraph;
        };
    } maniGraphNodeSource;

    // A graph: everything it holds, and the node whose pose it outputs.
    // Every node must reach the output.
    struct maniGraphSource
    {
        const maniSkeleton* skeleton;
        const maniGraphParameterSource* parameters;
        const maniGraphClipSource* clips;
        const maniGraphSyncGroupSource* syncGroups;
        const maniGraphNodeSource* nodes;
        uint32_t parameterCount;
        uint32_t clipCount;
        uint32_t syncGroupCount;
        uint32_t nodeCount;
        uint32_t output;
        uint32_t databaseCount;
        const maniGraphDatabaseSource* databases;
        uint32_t modelCount;
        uint32_t springSetCount;
        const maniGraphModelSource* models;
        const maniGraphSpringSetSource* springSets;
        // The mirror map of the graph's skeleton its mirror
        // nodes mirror through; NULL for none.
        const maniMirrorMap* mirrorMap;
    };

    /// Builds a graph file: inlines subgraphs (their nodes after the
    /// source's, in order, nested ones after them; refusals name nodes by
    /// their place there), checks every parameter, clip slot, sync group,
    /// database slot, model slot, spring set slot and node, gives each
    /// retargeting map the clip slots name a map slot, orders the nodes
    /// so each
    /// follows its inputs (the output last, the source's order kept where
    /// it may be), makes each mask's
    /// bone weights, marks each node as giving poses or differences and as
    /// mirrored or not (the parity of the mirror nodes above it), gives
    /// each node's pose a slot reused once its last reader has run, and
    /// keeps the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The graph.
    /// @return `mani_success`; `mani_errorLimit` with the reason and item
    /// for a source past a bound (no misuse): a name past
    /// MANI_MAX_NAME_BYTES (the reason and item a refused name gets; a
    /// condition's event at its Transition), Count (more items than the
    /// named limits, or subgraphs nested deeper than
    /// MANI_MAX_GRAPH_DEPTH), Clip (more maps than MANI_MAX_GRAPH_RETARGET_MAPS),
    /// Scalar (a time factor past MANI_MAX_GRAPH_TIME_FACTOR), Node (a
    /// node's inputs, points, bones, iterations, effectors, queries or
    /// transitions past their bounds) and Transition (conditions past
    /// MANI_MAX_GRAPH_CONDITIONS); `mani_errorInvalid` for a NULL argument
    /// (a misuse) or with the reason, the item and another the reason
    /// names for a refused source: Count (none of nodes, or a list of
    /// items not given, in the source or a subgraph, or a subgraph's sync
    /// group without its name), Name,
    /// DuplicateName and NameHash with the list bits of parameters, clips,
    /// sync groups, databases, models or spring sets, Parameter, Clip,
    /// Database, Model, SpringSetSlot, Node, Input, Cycle, Unreachable,
    /// Scalar, Weight, Difference, MirrorGraph and MirrorParity (a node
    /// under both a mirrored and an unmirrored path, item the node, other
    /// the reader reaching it the other way: give each path its own copy
    /// of the nodes beneath); and `mani_errorCapacity` when the allocator
    /// fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildGraph(maniBuilder* builder,
                                                           const maniGraphSource* source);

    /// Writes a graph source as its JSON text form (UTF-8, two-space
    /// indents, a line per list item, ending in a new line), building it
    /// first: a source maniBuildGraph refuses is refused the same way and
    /// not written. Parameters, clip slots, sync groups, database slots
    /// (a "databases" list, when the graph has any), model slots (a
    /// "models" list, when it has any), spring set slots (a "springSets"
    /// list, when it has any) and the bones of
    /// mask chains are written by name, nodes and the output by their
    /// place in the source, subgraphs as nested graphs with their
    /// parameters mapped by name, and numbers as the shortest text that
    /// reads back to the same binary32 (ECMAScript's layout). The text is
    /// kept in place of any file or text kept before; maniBuilder_CopyBuilt
    /// copies it.
    ///
    /// @param builder  The builder.
    /// @param source   The graph.
    /// @return As maniBuildGraph.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniWriteGraphJson(maniBuilder* builder,
                                                               const maniGraphSource* source);

    // Why a JSON text is not a graph's form: the other of a
    // maniBuildReasonForm refusal.
    typedef uint32_t maniGraphForm;
    enum
    {
        // A member the object does not take.
        maniGraphFormMember = 1,
        // A member given twice, or a bone a map's reference lists twice.
        maniGraphFormRepeat = 2,
        // A member the object needs is missing.
        maniGraphFormMissing = 3,
        // A value of the wrong kind, count or range.
        maniGraphFormValue = 4,
        // A name no parameter, clip slot, sync group, database slot, model
        // slot, spring set slot or bone of the graph has, or no bone of a
        // map's skeleton has.
        maniGraphFormName = 5,
        // A format not the form's ("maul-anim graph", "maul-anim retarget
        // map" or "maul-anim mirror map") or a version not 1.
        maniGraphFormVersion = 6,
        // A clip slot the host's clips do not name, a database slot the
        // host's databases do not, a model slot the host's models do not,
        // or a spring set slot the host's spring sets do not.
        maniGraphFormUnbound = 7,
    };

    // What a graph's JSON text names: its skeleton, and clips, databases,
    // models and spring sets with their slot names; each list may be NULL
    // when its count is 0.
    typedef struct maniGraphNamedAssets
    {
        const maniSkeleton* skeleton;
        const maniGraphClipSource* clips;
        const maniGraphDatabaseSource* databases;
        const maniGraphModelSource* models;
        const maniGraphSpringSetSource* springSets;
        uint32_t clipCount;
        uint32_t databaseCount;
        uint32_t modelCount;
        uint32_t springSetCount;
        // The mirror map every graph read takes, or NULL.
        const maniMirrorMap* mirrorMap;
    } maniGraphNamedAssets;

    /// Builds a graph file from its JSON text form, as maniWriteGraphJson
    /// writes it: members an object does not take are refused; those with
    /// an off value may be left out (a parameter's default: false, 0, the
    /// zero vector or the identity quaternion, and none for a pose
    /// parameter; bounds, both or neither;
    /// allEvents false; a clip node's syncGroup null, speed 1, loop
    /// "clip", leader false; a blend's direct false; a mask's chains empty
    /// and defaultWeight 0; a chain's subtree false; a seek's time 0; a
    /// 1D space's period 0, a banded space's polar false, a triangulated
    /// space's triangles none (its Delaunay triangulation), a position's
    /// y 0; a machine's entry 0, chainLimit 0 (4), transitions and finals
    /// empty; a transition's priority 0, interruption "none", entry
    /// "restart", fade "linear", duration and cooldown 0 and conditions
    /// empty; a one-shot's fadeOutTrigger and abort none, curve "linear",
    /// fadeIn and fadeOut 0, mask none (the whole body); an
    /// inertialize node's trigger none, method "inertialization",
    /// duration 0 and, for "deadBlending", halfLifeScale 0.3, halfLifeMin
    /// 0.1 and halfLifeMax 1.0; a motion matching node's trigger none,
    /// query empty, period 0.1, jumpThreshold, endExclusion, historyTime,
    /// continuingBias, transitionCost and historyCount 0, requiredTags
    /// and excludedTags 0 (a whole number, or {"parameter": name}) and
    /// duration 0; parameters, clips, syncGroups, databases, models and
    /// springSets empty). Each clip slot name, at any nesting, takes the
    /// host's clip of that name (and its map), each database slot name the host's
    /// database of that name, each model slot name the host's model of
    /// that name and each spring set slot name the host's spring set of
    /// that name (the first of equal names); every graph read takes the
    /// host's mirror map. The source read is then built
    /// as maniBuildGraph builds it.
    ///
    /// @param builder    The builder.
    /// @param text       The UTF-8 text (a leading byte order mark
    ///                   skipped); may be NULL when length is 0.
    /// @param length     The text's length in bytes.
    /// @param assets     The graph's skeleton (mask chains name its
    ///                   bones), the clips, databases, models and
    ///                   spring sets with their slot names, and the
    ///                   mirror map.
    /// @return As maniBuildGraph, items by place in the text's lists; or
    /// `mani_errorInvalid` with maniBuildReasonJson or
    /// maniBuildReasonForm (other a maniGraphForm), item the byte offset.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildGraphJson(maniBuilder* builder,
                                                               const char* text, size_t length,
                                                               const maniGraphNamedAssets* assets);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_GRAPHBUILD_H
