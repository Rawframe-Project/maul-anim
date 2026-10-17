// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs (mani-0023): a graph file built offline against one skeleton,
// opened in place, bound to its clips, and played by instances in caller
// memory. Its nodes form a DAG stored in topological order, each after
// its inputs, the output last.

#ifndef MAUL_ANIM_GRAPH_H
#define MAUL_ANIM_GRAPH_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// What a lookup by name finds for no parameter, and a scalar's parameter
// when it is a literal.
#define MANI_NO_PARAMETER 0xFFFFFFFFu
// A clip node in no sync group.
#define MANI_NO_SYNC_GROUP 0xFFFFFFFFu
// Named limits: nodes, inputs of one node, parameters, clips, sync
// groups and motion databases of one graph.
#define MANI_GRAPH_MAX_NODES       65536u
#define MANI_GRAPH_MAX_INPUTS      4096u
#define MANI_GRAPH_MAX_PARAMETERS  65536u
#define MANI_GRAPH_MAX_CLIPS       65536u
#define MANI_GRAPH_MAX_SYNC_GROUPS 65536u
#define MANI_GRAPH_MAX_DATABASES   256u
#define MANI_GRAPH_MAX_MODELS      256u
#define MANI_GRAPH_MAX_SPRING_SETS 256u
// The most points (and inputs) a blend space has.
#define MANI_GRAPH_MAX_SPACE_POINTS 256u
// The most transitions of one state machine, conditions of one
// transition, and transitions one machine takes in an update.
#define MANI_GRAPH_MAX_TRANSITIONS 4096u
#define MANI_GRAPH_MAX_CONDITIONS  16u
#define MANI_GRAPH_MAX_CHAIN       16u
// A transition's source when it leaves from any state.
#define MANI_GRAPH_ANY_STATE 0xFFFFFFFFu

    // A parameter's type. A trigger is set by the host and consumed by the
    // first node that reads it within an update.
    typedef uint32_t maniGraphParameterType;
    enum
    {
        maniGraphParameterBool = 0,
        maniGraphParameterInt = 1,
        maniGraphParameterFloat = 2,
        maniGraphParameterVec2 = 3,
        maniGraphParameterVec3 = 4,
        // Normalized when set.
        maniGraphParameterQuat = 5,
        maniGraphParameterTrigger = 6,
    };

    // A parameter's value: f for float (f[0]) and the vectors, i for int,
    // b (0 or 1) for bool and trigger.
    typedef union maniGraphValue
    {
        float f[4];
        int32_t i;
        uint32_t b;
    } maniGraphValue;

    // A node's kind.
    typedef uint32_t maniGraphNodeKind;
    enum
    {
        // Plays a clip; no inputs.
        maniGraphNodeClip = 1,
        // Blends its inputs by weights, normalized or direct.
        maniGraphNodeBlend = 2,
        // Its first input on the bones a mask weighs, its second on the
        // rest, blended per bone.
        maniGraphNodeMask = 3,
        // Its first input with each other, a layer of differences, added.
        maniGraphNodeAdditive = 4,
        // One of its inputs, chosen by an int or bool parameter.
        maniGraphNodeSelect = 5,
        // Its input with time scaled.
        maniGraphNodeTimeScale = 6,
        // Its input, its playheads set to a time when a trigger fires.
        maniGraphNodeSeek = 7,
        // A blend space on a line: its inputs blended by where a position
        // falls among their points.
        maniGraphNodeBlendSpace1D = 8,
        // A blend space in a plane by triangles: the corners of the
        // triangle holding the position, or the ends of the nearest edge.
        maniGraphNodeBlendSpaceTriangles = 9,
        // A blend space in a plane by gradient bands, cartesian or polar.
        maniGraphNodeBlendSpaceBands = 10,
        // A state machine: its inputs are its states, blended by the
        // weights its transitions give them.
        maniGraphNodeMachine = 11,
        // A one-shot: a shot (input 1) played over a base (input 0) when
        // a trigger fires it, faded in and out by its weight.
        maniGraphNodeOneShot = 12,
        // An inertialize node: its input with the discontinuities
        // requested beneath it eased out over the requested time.
        maniGraphNodeInertialize = 13,
        // A motion matching node: plays the frame of a motion database
        // that best matches its query, searching again as it plays; no
        // inputs.
        maniGraphNodeMatch = 14,
        // A learned motion matching node: plays the state of a learned
        // model, stepping it on and projecting its query again as it
        // plays; no inputs.
        maniGraphNodeLearned = 15,
    };

// The most steps of its model a learned node takes in one update: a
// longer update plays as this many steps' seconds and loses the rest, as
// fixed-step simulations lose time past their limit.
#define MANI_GRAPH_LEARNED_MAX_STEPS 20u

    // A graph over a file's bytes, which the caller keeps alive and
    // unchanged while the view is used. Its fields are the library's; read
    // a graph through the functions below. A view is immutable and
    // copyable.
    typedef struct maniGraph
    {
        const uint8_t* parameters;
        const uint8_t* nodes;
        const uint8_t* inputs;
        const uint8_t* data;
        const uint8_t* clips;
        const uint8_t* syncGroups;
        const uint8_t* maskWeights;
        const uint8_t* databases;
        const uint8_t* databaseClips;
        const uint8_t* names;
        const uint8_t* models;
        const uint8_t* springSets;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint64_t namesSize;
        uint32_t skeletonBoneCount;
        uint32_t parameterCount;
        uint32_t nodeCount;
        uint32_t inputCount;
        uint32_t dataWords;
        uint32_t clipCount;
        uint32_t syncGroupCount;
        uint32_t maskCount;
        uint32_t slotCount;
        uint32_t machineWords;
        uint32_t databaseCount;
        uint32_t databaseClipCount;
        uint32_t modelCount;
        uint32_t springSetCount;
    } maniGraph;

    /// Opens a graph file in place: checks the container, every record,
    /// index, count, float and name, and that every node's inputs come
    /// before it.
    ///
    /// @param bytes     The file, 16-byte aligned.
    /// @param size      Its size in bytes; bytes past the file are ignored.
    /// @param graphOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenGraph(const void* bytes, size_t size,
                                                         maniGraph* graphOut);

    /// Returns the graph's identity, the hash of its contents.
    ///
    /// @param graph  An opened graph.
    /// @return The identity; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphIdentity(const maniGraph* graph);

    /// Returns the identity of the skeleton the graph was built against.
    ///
    /// @param graph  An opened graph.
    /// @return The identity; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphSkeletonIdentity(const maniGraph* graph);

    /// Returns the number of nodes.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphNodeCount(const maniGraph* graph);

    /// Returns the number of parameters.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphParameterCount(const maniGraph* graph);

    /// Finds a parameter by its name.
    ///
    /// @param graph   An opened graph.
    /// @param name    The name's bytes, UTF-8.
    /// @param length  Its length in bytes.
    /// @return The parameter, or MANI_NO_PARAMETER.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphFindParameter(const maniGraph* graph, const char* name,
                                             size_t length);

    /// Returns a parameter's type.
    ///
    /// @param graph      An opened graph.
    /// @param parameter  The parameter.
    /// @param typeOut    Receives the type.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// parameter past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniGraphParameterTypeOf(const maniGraph* graph,
                                                                uint32_t parameter,
                                                                maniGraphParameterType* typeOut);

    /// Returns a parameter's name, its bytes inside the file.
    ///
    /// @param graph      An opened graph.
    /// @param parameter  The parameter.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphParameterName(const maniGraph* graph, uint32_t parameter,
                                                uint32_t* lengthOut);

    /// Returns the number of clip slots the graph plays.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphClipCount(const maniGraph* graph);

    /// Returns the identity of the clip a slot names.
    ///
    /// @param graph  An opened graph.
    /// @param clip   The slot.
    /// @return The clip's identity; 0 past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphClipIdentity(const maniGraph* graph, uint32_t clip);

    /// Returns the name a slot gives its clip, its bytes inside the file.
    ///
    /// @param graph      An opened graph.
    /// @param clip       The slot.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphClipName(const maniGraph* graph, uint32_t clip,
                                           uint32_t* lengthOut);

    /// Returns the number of motion database slots the graph's motion
    /// matching nodes search.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphDatabaseCount(const maniGraph* graph);

    /// Returns the identity of the motion database a slot names.
    ///
    /// @param graph     An opened graph.
    /// @param database  The slot.
    /// @return The database's identity; 0 past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphDatabaseIdentity(const maniGraph* graph, uint32_t database);

    /// Returns the name a slot gives its motion database, its bytes inside
    /// the file.
    ///
    /// @param graph      An opened graph.
    /// @param database   The slot.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphDatabaseName(const maniGraph* graph, uint32_t database,
                                               uint32_t* lengthOut);

    /// Returns the number of learned model slots the graph's learned
    /// motion matching nodes play.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphModelCount(const maniGraph* graph);

    /// Returns the identity of the learned model a slot names.
    ///
    /// @param graph  An opened graph.
    /// @param model  The slot.
    /// @return The model's identity; 0 past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphModelIdentity(const maniGraph* graph, uint32_t model);

    /// Returns the name a slot gives its learned model, its bytes inside
    /// the file.
    ///
    /// @param graph      An opened graph.
    /// @param model      The slot.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphModelName(const maniGraph* graph, uint32_t model,
                                            uint32_t* lengthOut);

    /// Returns the number of spring set slots the graph's spring nodes
    /// read.
    ///
    /// @param graph  An opened graph.
    /// @return The count; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGraphSpringSetCount(const maniGraph* graph);

    /// Returns the identity of the spring set a slot names.
    ///
    /// @param graph  An opened graph.
    /// @param slot   The slot.
    /// @return The set's identity; 0 past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniGraphSpringSetIdentity(const maniGraph* graph, uint32_t slot);

    /// Returns the name a slot gives its spring set, its bytes inside the
    /// file.
    ///
    /// @param graph      An opened graph.
    /// @param slot       The slot.
    /// @param lengthOut  Receives the name's length; 0 past the count.
    /// @return The name, not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGraphSpringSetName(const maniGraph* graph, uint32_t slot,
                                                uint32_t* lengthOut);

    // A graph bound to its skeleton and clips, in caller memory: shared by
    // every instance of the graph, read only.
    typedef struct maniGraphBinding maniGraphBinding;

    // An instance of a bound graph, in caller memory: its parameters and
    // the state of its nodes.
    typedef struct maniGraphInstance maniGraphInstance;

#define MANI_GRAPH_BINDING_ALIGNMENT  8u
#define MANI_GRAPH_INSTANCE_ALIGNMENT 8u

    /// Returns the bytes a binding of the graph needs.
    ///
    /// @param graph  An opened graph.
    /// @return The size; 0 for a NULL graph.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniGraphBindingSize(const maniGraph* graph);

    // What a graph binds to: its skeleton, and opened clips, motion
    // databases, learned models and spring sets in any order (others
    // ignored); each list may be NULL when its count is 0.
    typedef struct maniGraphAssets
    {
        const maniSkeleton* skeleton;
        const maniClip* const* clips;
        const maniMotionDatabase* const* databases;
        const maniLearnedModel* const* models;
        const maniSpringSet* const* springSets;
        uint32_t clipCount;
        uint32_t databaseCount;
        uint32_t modelCount;
        uint32_t springSetCount;
    } maniGraphAssets;

    /// Binds a graph to its assets: each clip slot to the clip of its
    /// identity among the clips, of the graph's skeleton and of the kind
    /// the slot was built with; each database slot to the database of its
    /// identity among the databases, of the graph's skeleton, whose clips
    /// are the slots the graph lists for it and whose channels take the
    /// graph's query parameters; each model slot to the learned model of
    /// its identity among the models, of the graph's skeleton; each
    /// spring set slot to the spring set of its identity among the sets,
    /// of the graph's skeleton. The graph and its assets must outlive the
    /// binding.
    ///
    /// @param graph       An opened graph.
    /// @param assets      The skeleton the graph was built against, and
    ///                    the clips, databases, models and spring sets.
    /// @param memory      At least maniGraphBindingSize bytes,
    ///                    MANI_GRAPH_BINDING_ALIGNMENT-aligned.
    /// @param size        The memory's size.
    /// @param bindingOut  Receives the binding.
    /// @param unboundOut  Receives the first slot left unbound on failure:
    ///                    a clip slot, the graph's clip count plus a
    ///                    database slot, the clip and database counts
    ///                    plus a model slot, or the clip, database and
    ///                    model counts plus a spring set slot
    ///                    (MANI_NO_TRACK otherwise). May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL or
    /// misaligned argument, a list NULL with a count, a skeleton that is
    /// not the graph's, a slot no asset binds, or clips whose root motion
    /// names different bones; `mani_errorCapacity` when the memory is too
    /// small.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniBindGraph(const maniGraph* graph,
                                                     const maniGraphAssets* assets, void* memory,
                                                     size_t size, maniGraphBinding** bindingOut,
                                                     uint32_t* unboundOut);

    /// Returns the bytes an instance of a bound graph needs.
    ///
    /// @param binding  A binding.
    /// @return The size; 0 for a NULL binding.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniGraphInstanceSize(const maniGraphBinding* binding);

    /// Makes an instance in caller memory: parameters at their defaults,
    /// every playhead at 0, nothing weighed until the first update. The
    /// instance holds no pointer into itself, so a `memcpy` of its bytes
    /// is a copy.
    ///
    /// @param binding      A binding, which must outlive the instance.
    /// @param memory       At least maniGraphInstanceSize bytes,
    ///                     MANI_GRAPH_INSTANCE_ALIGNMENT-aligned.
    /// @param size         The memory's size.
    /// @param instanceOut  Receives the instance.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL or
    /// misaligned argument; `mani_errorCapacity` when the memory is too
    /// small.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniInitGraphInstance(const maniGraphBinding* binding,
                                                             void* memory, size_t size,
                                                             maniGraphInstance** instanceOut);

    /// Sets a parameter: bool and trigger 0 or 1 (b), int (i) and float
    /// (f[0]) clamped to their bounds, the vectors (f) finite, a
    /// quaternion (f) normalized.
    ///
    /// @param instance   An instance.
    /// @param parameter  The parameter.
    /// @param type       Its type, as the graph declares it.
    /// @param value      The value.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// parameter past the count, a type that is not the parameter's, or a
    /// value out of form (not finite, a bool past 1, a quaternion of zero
    /// length; nothing changed).
    /// @par Thread safety
    /// Safe from any thread; the instance is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSetGraphParameter(maniGraphInstance* instance,
                                                             uint32_t parameter,
                                                             maniGraphParameterType type,
                                                             maniGraphValue value);

    /// Reads a parameter's value.
    ///
    /// @param instance   An instance.
    /// @param parameter  The parameter.
    /// @param valueOut   Receives the value.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a parameter past the count.
    /// @par Thread safety
    /// Safe from any thread; the instance is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniGetGraphParameter(const maniGraphInstance* instance,
                                                             uint32_t parameter,
                                                             maniGraphValue* valueOut);

    /// Returns the bytes an instance's saved state takes: its parameters,
    /// its nodes' state and its state machines' and one-shots', with the
    /// graph's identity.
    ///
    /// @param binding  A binding.
    /// @return The size; 0 for a NULL binding.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniGraphStateSize(const maniGraphBinding* binding);

    /// Writes an instance's state as bytes that hold no pointer, to be
    /// restored into any instance of the same graph.
    ///
    /// @param instance  An instance.
    /// @param out       At least maniGraphStateSize bytes.
    /// @param capacity  The bytes out holds.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorCapacity` when capacity is too small (nothing written).
    /// @par Thread safety
    /// Safe from any thread; the instance is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSaveGraphInstance(const maniGraphInstance* instance,
                                                             void* out, size_t capacity);

    /// Restores a saved state, checked as external bytes: the graph's
    /// identity, the size, every parameter in form and every node's, state
    /// machine's and one-shot's state finite and in range. The instance's
    /// cursors start over (they cache nothing a pose depends on).
    ///
    /// @param instance  An instance of the graph the state was saved from.
    /// @param bytes     The state.
    /// @param size      Its size in bytes.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorFormat` with the offset of the first byte found wrong
    /// (the instance unchanged).
    /// @par Thread safety
    /// Safe from any thread; the instance is used by one thread at a time.
    MANI_NODISCARD MANI_API maniFileResult maniRestoreGraphInstance(maniGraphInstance* instance,
                                                                    const void* bytes, size_t size);

// An event fires only from a clip node weighed past this.
#define MANI_GRAPH_EVENT_THRESHOLD 1e-5f

    // An event a clip node passed in an update, or one a learned node's
    // model raised.
    typedef struct maniGraphEvent
    {
        // The clip node, its clip slot and the event's index in the clip;
        // for a learned node, MANI_NO_TRACK and the index in its model.
        uint32_t node;
        uint32_t clip;
        uint32_t event;
        int32_t payload;
        uint64_t nameHash;
        // The node's effective weight and the event's time in its clip
        // (a learned node's: its time since it began).
        float weight;
        float time;
        // 1 passed forward, -1 backward.
        int32_t direction;
        uint32_t reserved;
    } maniGraphEvent;

    /// Moves an instance through time: shares each node's weight down from
    /// the output (normalized blends, masks by their largest bone weight,
    /// additive layers by their weights, the selected input), moves every
    /// clip node's playhead by the step times its speed and the time
    /// scales above it (a seek whose trigger is set moving its playheads to
    /// its time instead, and consuming the trigger), moves learned nodes'
    /// states (stepped, at most MANI_GRAPH_LEARNED_MAX_STEPS steps an
    /// update, and projected), lists the events clip nodes and
    /// learned nodes' models weighed past MANI_GRAPH_EVENT_THRESHOLD give
    /// in node order, and blends the clips' and learned nodes' root motion
    /// by the weights the output gives it (masks by their weight at the
    /// root motion bone, additive layers none).
    ///
    /// @param instance      An instance.
    /// @param delta         The step in seconds, finite, either sign.
    /// @param events        Receives up to capacity events. May be NULL
    ///                      with a capacity of 0.
    /// @param capacity      The events events holds.
    /// @param eventCountOut Receives the true count, which may pass the
    ///                      capacity (saturating at UINT32_MAX). May be NULL.
    /// @param rootMotionOut Receives the update's root motion. May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL instance,
    /// events NULL with a capacity, or a step not finite (nothing changed).
    /// @par Thread safety
    /// Safe from any thread; the instance is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniUpdateGraph(maniGraphInstance* instance, float delta,
                                                       maniGraphEvent* events, uint32_t capacity,
                                                       uint32_t* eventCountOut,
                                                       maniRootMove* rootMotionOut);

#define MANI_GRAPH_SCRATCH_ALIGNMENT 32u

    /// Returns the bytes of the scratch an evaluation of a bound graph's
    /// instances needs: a pose per slot and room for one node's layers.
    /// One scratch serves any number of instances, one at a time.
    ///
    /// @param binding  A binding.
    /// @return The size; 0 for a NULL binding.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniGraphScratchSize(const maniGraphBinding* binding);

    /// Writes an instance's pose as its last update weighed it: each node
    /// of weight above 0, in order, into its slot (a clip sampled at its
    /// playhead through its cursor; a learned node's model decompressed
    /// at its state; blends, masks and additive layers by
    /// the blending functions over their weighed inputs; a select, time
    /// scale or seek passing its input on), then the output's. The rest
    /// pose before the first update.
    ///
    /// @param instance  An instance.
    /// @param scratch   At least maniGraphScratchSize bytes,
    ///                  MANI_GRAPH_SCRATCH_ALIGNMENT-aligned, used by one
    ///                  thread at a time.
    /// @param size      The scratch's size.
    /// @param pose      A pose of the graph's skeleton.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL or
    /// misaligned argument or a pose of another skeleton;
    /// `mani_errorCapacity` when the scratch is too small.
    /// @par Thread safety
    /// Safe from any thread; the instance, scratch and pose are used by
    /// one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniEvaluateGraph(maniGraphInstance* instance, void* scratch,
                                                         size_t size, maniPose* pose);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_GRAPH_H
