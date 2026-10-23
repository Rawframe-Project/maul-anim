// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The inertialize suites' helpers: an inertialize node over a
// machine of two clips switched inertially by a trigger, updates with
// their evaluation, the node's block read, and dead blending's node
// source. Included by one suite each.

#ifndef MAUL_ANIM_TEST_GRAPH_INERTIA_FIXTURE_H
#define MAUL_ANIM_TEST_GRAPH_INERTIA_FIXTURE_H

#include "graph_inertia.h"
#include "graph_space_fixture.h"
#include "vecmath.h"

#include "maul-anim/pose.h"

// Parameters: go and push (triggers), d (float).
static maniGraphParameterSource s_parameters[3];

static inline void Parameters(void)
{
    s_parameters[0] = Parameter("go", maniGraphParameterTrigger, Int(0));
    s_parameters[1] = Parameter("push", maniGraphParameterTrigger, Int(0));
    s_parameters[2] = Parameter("d", maniGraphParameterFloat, Float(0.2f));
}

static const maniGraphConditionSource kGo[1] = {{maniGraphConditionTrigger, 0, 0, 0, {{0}}, NULL}};

static inline maniGraphTransitionSource Inertial(float duration)
{
    return (maniGraphTransitionSource){0,
                                       1,
                                       0,
                                       maniGraphInterruptionNone,
                                       maniGraphEntryRestart,
                                       maniGraphFadeInertial,
                                       duration,
                                       0.0f,
                                       kGo,
                                       1,
                                       0};
}

static inline maniGraphNodeSource Node(uint32_t trigger, maniGraphScalar duration)
{
    static const uint32_t kNode[1] = {1};
    return (maniGraphNodeSource){maniGraphNodeInertialize, 1, kNode,
                                 .inertialize = {trigger, 0, duration, 0.0f, 0.0f, 0.0f}};
}

// An inertialize node (its trigger and duration given) over a machine of
// clips from and to (repeating, from at speed), go taking it inertially
// over half a second: from 0, to 1, the machine 2, the node 3.
static inline maniGraphInstance* Eased(maniGraphNodeSource node, uint32_t from, float speed,
                                       uint32_t to, maniGraph* g)
{
    static const uint32_t kStates[2] = {2, 3};
    static maniGraphTransitionSource t[1];
    t[0] = Inertial(0.5f);
    Parameters();
    maniGraphNodeSource nodes[4] = {
        node,
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(from, speed, maniGraphLoopRepeat),
        ClipNode(to, 1.0f, maniGraphLoopRepeat)};
    return Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 3, .nodeCount = 4},
        g);
}

// The walk (at speed) to the run, the node's trigger push and duration d.
static inline maniGraphInstance* Walked(float speed, maniGraph* g)
{
    return Eased(Node(1, (maniGraphScalar){2, 0.0f}), 3, speed, 1, g);
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[4096];
static maniPose* s_pose;

// One update of delta and its evaluation; the output's bone b.
static inline maniTransform Step(maniGraphInstance* instance, float delta, uint32_t b)
{
    Update(instance, delta);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "evaluated");
    return maniPoseLocal(s_pose, b);
}

static inline void Steps(maniGraphInstance* instance, int count)
{
    for (int k = 0; k < count; ++k)
    {
        (void)Step(instance, 0.1f, 0);
    }
}

static inline uint32_t* Block(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    uint32_t data = maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    return (uint32_t*)((uint8_t*)instance + instance->machinesAt) +
           maniLoadU32(g->data + 4 * (size_t)(data + maniGraphInertiaBlock));
}

// Bone b's capture's first word in a block of a method (the fixture's
// three bones).
static inline uint32_t CaptureAt(uint32_t method, uint32_t b)
{
    maniGraphInertia node = {3, method, 0.0f, 0.0f, 0.0f, NULL};
    return maniGraphInertiaCapture(&node, b);
}

// Bone b's channel c (translation 0, rotation 1, scale 2) as captured:
// direction, magnitude, speed, duration.
static inline float Captured(maniGraphInstance* instance, const maniGraph* g, uint32_t b,
                             uint32_t c, uint32_t k)
{
    return maniGraphInertiaGet(Block(instance, g, 3),
                               CaptureAt(maniGraphInertialization, b) + 6 * c + k);
}

static inline void Pull(maniGraphInstance* instance, uint32_t trigger)
{
    Set(instance, trigger, maniGraphParameterTrigger, Int(1));
}

// The run's x at its time t: 3 t.
static inline bool Run(float x, double t)
{
    return fabs((double)x - 3.0 * t) <= 1e-5;
}

// Dead blending with Holden's half-lives, the node's trigger push and
// duration d.
static inline maniGraphNodeSource Dead(void)
{
    maniGraphNodeSource n = Node(1, (maniGraphScalar){2, 0.0f});
    n.inertialize.method = maniGraphDeadBlending;
    n.inertialize.halfLifeScale = 0.3f;
    n.inertialize.halfLifeMin = 0.1f;
    n.inertialize.halfLifeMax = 1.0f;
    return n;
}

static inline maniQuat AboutZ(float angle)
{
    return maniQuatFromAxisAngle((maniVec3){0.0f, 0.0f, 1.0f}, angle);
}

#endif // MAUL_ANIM_TEST_GRAPH_INERTIA_FIXTURE_H
