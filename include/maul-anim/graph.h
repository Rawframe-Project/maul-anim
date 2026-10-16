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
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

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
// Named limits: nodes, inputs of one node, parameters, clips and sync
// groups of one graph.
#define MANI_GRAPH_MAX_NODES       65536u
#define MANI_GRAPH_MAX_INPUTS      4096u
#define MANI_GRAPH_MAX_PARAMETERS  65536u
#define MANI_GRAPH_MAX_CLIPS       65536u
#define MANI_GRAPH_MAX_SYNC_GROUPS 65536u

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
    };

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
        const uint8_t* names;
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
        uint32_t reserved;
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

    /// Binds a graph to its skeleton and clips: each clip slot to the
    /// clip of its identity among clips (in any order, others ignored),
    /// each of the graph's skeleton and of the kind the slot was built
    /// with. The graph, skeleton and clips must outlive the binding.
    ///
    /// @param graph       An opened graph.
    /// @param skeleton    The skeleton the graph was built against.
    /// @param clips       count opened clips.
    /// @param count       The number of clips.
    /// @param memory      At least maniGraphBindingSize bytes,
    ///                    MANI_GRAPH_BINDING_ALIGNMENT-aligned.
    /// @param size        The memory's size.
    /// @param bindingOut  Receives the binding.
    /// @param unboundOut  Receives the first slot left unbound on failure
    ///                    (MANI_NO_TRACK otherwise). May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL or
    /// misaligned argument, a skeleton that is not the graph's, a slot no
    /// clip binds, or clips whose root motion names different bones;
    /// `mani_errorCapacity` when the memory is too small.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniBindGraph(const maniGraph* graph,
                                                     const maniSkeleton* skeleton,
                                                     const maniClip* const* clips, uint32_t count,
                                                     void* memory, size_t size,
                                                     maniGraphBinding** bindingOut,
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

    /// Returns the bytes an instance's saved state takes: its parameters
    /// and its nodes' state, with the graph's identity.
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
    /// identity, the size, every parameter in form and every node's state
    /// finite and in range. The instance's cursors start over (they cache
    /// nothing a pose depends on).
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

    // An event a clip node passed in an update.
    typedef struct maniGraphEvent
    {
        // The clip node, its clip slot and the event's index in the clip.
        uint32_t node;
        uint32_t clip;
        uint32_t event;
        int32_t payload;
        uint64_t nameHash;
        // The node's effective weight and the event's time in its clip.
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
    /// its time instead, and consuming the trigger), lists the events clip
    /// nodes weighed past MANI_GRAPH_EVENT_THRESHOLD pass in node order,
    /// and blends the clips' root motion by the weights the output gives it
    /// (masks by their weight at the root motion bone, additive layers
    /// none).
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

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_GRAPH_H
