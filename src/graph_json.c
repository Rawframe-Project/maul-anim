// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph source written as its JSON text form: built first,
// so only a source that builds is written; then each list item on a line
// of its own, parameters, clips, sync groups, databases (when
// it has any) and bones by name, nodes by their place in the source,
// subgraphs as nested graphs, and numbers as the shortest binary32 text.
// Counted once, then written.

#include "allocator.h"
#include "builder.h"
#include "fpenv.h"
#include "graph_json_body.h"
#include "graph_json_ik.h"
#include "graph_json_write.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef maniGraphWriter Writer;

static const char* const kTypes[] = {"bool", "int",  "float",   "vec2",
                                     "vec3", "quat", "trigger", "pose"};

// A parameter's value of its type: true or false, an integer, a number
// or an array of 2 to 4 numbers.
static void PutValue(Writer* w, maniGraphParameterType type, maniGraphValue v)
{
    if (type == maniGraphParameterBool || type == maniGraphParameterTrigger)
    {
        maniGraphJsonPutBool(w, v.b != 0);
        return;
    }
    if (type == maniGraphParameterInt)
    {
        maniGraphJsonPutInt(w, v.i);
        return;
    }
    if (type == maniGraphParameterFloat)
    {
        maniGraphJsonPutFloat(w, v.f[0]);
        return;
    }
    uint32_t n = type == maniGraphParameterVec2 ? 2u : (type == maniGraphParameterVec3 ? 3u : 4u);
    maniGraphJsonPutText(w, "[");
    for (uint32_t k = 0; k < n; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutFloat(w, v.f[k]);
    }
    maniGraphJsonPutText(w, "]");
}

static void PutParameter(Writer* w, const maniGraphParameterSource* p)
{
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "name");
    maniGraphJsonPutString(w, p->name, p->nameLength);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "type");
    maniGraphJsonPutString(w, kTypes[p->type], (uint32_t)strlen(kTypes[p->type]));
    if (p->type != maniGraphParameterPose)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "default");
        PutValue(w, p->type, p->defaultValue);
    }
    if (p->bounded != 0)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "minimum");
        PutValue(w, p->type, p->minimum);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "maximum");
        PutValue(w, p->type, p->maximum);
    }
    maniGraphJsonPutText(w, "}");
}

static void PutClipNode(Writer* w, const maniGraphSource* source, const maniGraphClipNodeSource* c)
{
    static const char* const kLoops[] = {"\"clip\"", "\"clamp\"", "\"repeat\""};
    const maniGraphClipSource* clip = &source->clips[c->clip];
    maniGraphJsonPutKey(w, "clip");
    maniGraphJsonPutString(w, clip->name, clip->nameLength);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "syncGroup");
    if (c->syncGroup == MANI_NO_SYNC_GROUP)
    {
        maniGraphJsonPutText(w, "null");
    }
    else
    {
        const maniGraphSyncGroupSource* g = &source->syncGroups[c->syncGroup];
        maniGraphJsonPutString(w, g->name, g->nameLength);
    }
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "speed");
    maniGraphJsonPutScalar(w, source, c->speed);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "loop");
    maniGraphJsonPutText(w, kLoops[c->loop]);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "leader");
    maniGraphJsonPutBool(w, (c->flags & maniGraphClipLeader) != 0);
}

static void PutMaskNode(Writer* w, const maniGraphMaskNodeSource* m)
{
    maniGraphJsonPutKey(w, "chains");
    maniGraphJsonPutText(w, "[");
    for (uint32_t k = 0; k < m->chainCount; ++k)
    {
        uint32_t length = 0;
        const char* name = maniSkeletonBoneName(w->skeleton, m->chains[k].bone, &length);
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutText(w, "{");
        maniGraphJsonPutKey(w, "bone");
        maniGraphJsonPutString(w, name, length);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "weight");
        maniGraphJsonPutFloat(w, m->chains[k].weight);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "subtree");
        maniGraphJsonPutBool(w, m->chains[k].subtree);
        maniGraphJsonPutText(w, "}");
    }
    maniGraphJsonPutText(w, "], ");
    maniGraphJsonPutKey(w, "defaultWeight");
    maniGraphJsonPutFloat(w, m->defaultWeight);
}

// An inertialize node: its method and half-lives for dead blending only.
static void PutInertialize(Writer* w, const maniGraphSource* source,
                           const maniGraphInertializeNodeSource* s)
{
    if (s->trigger != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutKey(w, "trigger");
        maniGraphJsonPutParameterName(w, source, s->trigger);
        maniGraphJsonPutText(w, ", ");
    }
    if (s->method == maniGraphDeadBlending)
    {
        maniGraphJsonPutKey(w, "method");
        maniGraphJsonPutText(w, "\"deadBlending\", ");
    }
    maniGraphJsonPutKey(w, "duration");
    maniGraphJsonPutScalar(w, source, s->duration);
    if (s->method == maniGraphDeadBlending)
    {
        const char* const keys[3] = {"halfLifeScale", "halfLifeMin", "halfLifeMax"};
        const float values[3] = {s->halfLifeScale, s->halfLifeMin, s->halfLifeMax};
        for (uint32_t k = 0; k < 3; ++k)
        {
            maniGraphJsonPutText(w, ", ");
            maniGraphJsonPutKey(w, keys[k]);
            maniGraphJsonPutFloat(w, values[k]);
        }
    }
}

// A one-shot's triggers (those it has), curve, fades and mask (when it
// has one).
static void PutShot(Writer* w, const maniGraphSource* source, const maniGraphOneShotNodeSource* s)
{
    static const char* const kKeys[3] = {"fire", "fadeOutTrigger", "abort"};
    const uint32_t triggers[3] = {s->fire, s->fadeOutTrigger, s->abort};
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (triggers[k] != MANI_NO_PARAMETER)
        {
            maniGraphJsonPutKey(w, kKeys[k]);
            maniGraphJsonPutParameterName(w, source, triggers[k]);
            maniGraphJsonPutText(w, ", ");
        }
    }
    maniGraphJsonPutKey(w, "curve");
    maniGraphJsonPutText(w,
                         s->curve == maniGraphFadeSmoothstep ? "\"smoothstep\", " : "\"linear\", ");
    maniGraphJsonPutKey(w, "fadeIn");
    maniGraphJsonPutFloat(w, s->fadeIn);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "fadeOut");
    maniGraphJsonPutFloat(w, s->fadeOut);
    if (s->mask != NULL)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "mask");
        maniGraphJsonPutText(w, "{");
        PutMaskNode(w, s->mask);
        maniGraphJsonPutText(w, "}");
    }
}

// A turn limit node: its speed, its reset trigger and mask when it has
// them.
static void PutTurn(Writer* w, const maniGraphSource* source, const maniGraphTurnLimitNodeSource* s)
{
    maniGraphJsonPutKey(w, "speed");
    maniGraphJsonPutScalar(w, source, s->speed);
    if (s->reset != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "reset");
        maniGraphJsonPutParameterName(w, source, s->reset);
    }
    if (s->mask != NULL)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "mask");
        maniGraphJsonPutText(w, "{");
        PutMaskNode(w, s->mask);
        maniGraphJsonPutText(w, "}");
    }
}

// A seek node: its trigger and time.
static void PutSeek(Writer* w, const maniGraphSource* source, const maniGraphSeekNodeSource* s)
{
    maniGraphJsonPutKey(w, "trigger");
    maniGraphJsonPutParameterName(w, source, s->trigger);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "time");
    maniGraphJsonPutScalar(w, source, s->time);
}

// Tags: their bits, or {"parameter": name}.
static void PutTags(Writer* w, const maniGraphSource* source, maniGraphTags tags)
{
    if (tags.parameter == MANI_NO_PARAMETER)
    {
        maniGraphJsonPutUnsigned(w, tags.value);
        return;
    }
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "parameter");
    maniGraphJsonPutParameterName(w, source, tags.parameter);
    maniGraphJsonPutText(w, "}");
}

// A motion matching node: its database by name, its trigger (when it has
// one), its query's parameters, its numbers, tags and duration.
static void PutMatch(Writer* w, const maniGraphSource* source, const maniGraphMatchNodeSource* m)
{
    const maniGraphDatabaseSource* d = &source->databases[m->database];
    maniGraphJsonPutKey(w, "database");
    maniGraphJsonPutString(w, d->name, d->nameLength);
    maniGraphJsonPutText(w, ", ");
    if (m->trigger != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutKey(w, "trigger");
        maniGraphJsonPutParameterName(w, source, m->trigger);
        maniGraphJsonPutText(w, ", ");
    }
    maniGraphJsonPutKey(w, "query");
    maniGraphJsonPutText(w, "[");
    for (uint32_t q = 0; q < m->queryCount; ++q)
    {
        maniGraphJsonPutText(w, q > 0 ? ", " : "");
        maniGraphJsonPutParameterName(w, source, m->query[q]);
    }
    maniGraphJsonPutText(w, "]");
    static const char* const kKeys[6] = {"period",      "jumpThreshold",  "endExclusion",
                                         "historyTime", "continuingBias", "transitionCost"};
    const float values[6] = {m->period,      m->jumpThreshold,  m->endExclusion,
                             m->historyTime, m->continuingBias, m->transitionCost};
    for (uint32_t k = 0; k < 6; ++k)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, kKeys[k]);
        maniGraphJsonPutFloat(w, values[k]);
    }
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "historyCount");
    maniGraphJsonPutUnsigned(w, m->historyCount);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "requiredTags");
    PutTags(w, source, m->requiredTags);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "excludedTags");
    PutTags(w, source, m->excludedTags);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "duration");
    maniGraphJsonPutScalar(w, source, m->duration);
}

// A spring node's set by its slot's name, and the parameters it has.
static void PutSpring(Writer* w, const maniGraphSource* source, const maniGraphSpringNodeSource* s)
{
    const maniGraphSpringSetSource* set = &source->springSets[s->springSet];
    maniGraphJsonPutKey(w, "springSet");
    maniGraphJsonPutString(w, set->name, set->nameLength);
    const char* const keys[4] = {"position", "rotation", "reset", "force"};
    const uint32_t parameters[4] = {s->position, s->rotation, s->reset, s->force};
    for (uint32_t k = 0; k < 4; ++k)
    {
        if (parameters[k] != MANI_NO_PARAMETER)
        {
            maniGraphJsonPutText(w, ", ");
            maniGraphJsonPutKey(w, keys[k]);
            maniGraphJsonPutParameterName(w, source, parameters[k]);
        }
    }
}

// A learned node: its model by name, its trigger (when it has one), its
// query's parameters, its period and transition cost, tags and duration.
static void PutLearned(Writer* w, const maniGraphSource* source,
                       const maniGraphLearnedNodeSource* l)
{
    const maniGraphModelSource* m = &source->models[l->model];
    maniGraphJsonPutKey(w, "model");
    maniGraphJsonPutString(w, m->name, m->nameLength);
    maniGraphJsonPutText(w, ", ");
    if (l->trigger != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutKey(w, "trigger");
        maniGraphJsonPutParameterName(w, source, l->trigger);
        maniGraphJsonPutText(w, ", ");
    }
    maniGraphJsonPutKey(w, "query");
    maniGraphJsonPutText(w, "[");
    for (uint32_t q = 0; q < l->queryCount; ++q)
    {
        maniGraphJsonPutText(w, q > 0 ? ", " : "");
        maniGraphJsonPutParameterName(w, source, l->query[q]);
    }
    maniGraphJsonPutText(w, "], ");
    maniGraphJsonPutKey(w, "period");
    maniGraphJsonPutFloat(w, l->period);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "transitionCost");
    maniGraphJsonPutFloat(w, l->transitionCost);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "requiredTags");
    PutTags(w, source, l->requiredTags);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "excludedTags");
    PutTags(w, source, l->excludedTags);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "duration");
    maniGraphJsonPutScalar(w, source, l->duration);
}

static void PutGraph(Writer* w, const maniGraphSource* source, uint32_t depth, bool top);

// A blend space's position: a vec2 parameter, or x (and, in a plane,
// y).
static void PutPosition(Writer* w, const maniGraphSource* source, const maniGraphPosition* p,
                        bool plane)
{
    maniGraphJsonPutKey(w, "position");
    maniGraphJsonPutText(w, "{");
    if (p->vector != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutKey(w, "vector");
        maniGraphJsonPutParameterName(w, source, p->vector);
    }
    else
    {
        maniGraphJsonPutKey(w, "x");
        maniGraphJsonPutScalar(w, source, p->x);
        if (plane)
        {
            maniGraphJsonPutText(w, ", ");
            maniGraphJsonPutKey(w, "y");
            maniGraphJsonPutScalar(w, source, p->y);
        }
    }
    maniGraphJsonPutText(w, "}, ");
}

// A blend space's points: numbers on a line, [x, y] pairs in a plane.
static void PutPoints(Writer* w, const float* points, uint32_t count, bool pairs)
{
    maniGraphJsonPutKey(w, "points");
    maniGraphJsonPutText(w, "[");
    for (uint32_t k = 0; k < count; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        if (!pairs)
        {
            maniGraphJsonPutFloat(w, points[k]);
            continue;
        }
        maniGraphJsonPutText(w, "[");
        maniGraphJsonPutFloat(w, points[2 * k]);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutFloat(w, points[2 * k + 1]);
        maniGraphJsonPutText(w, "]");
    }
    maniGraphJsonPutText(w, "]");
}

// A triangulated space's authored triangles, three inputs each; nothing
// for a Delaunay triangulation.
static void PutTriangles(Writer* w, const maniGraphTrianglesNodeSource* s)
{
    if (s->triangleCount == 0)
    {
        return;
    }
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "triangles");
    maniGraphJsonPutText(w, "[");
    for (uint32_t t = 0; t < s->triangleCount; ++t)
    {
        maniGraphJsonPutText(w, t > 0 ? ", [" : "[");
        for (uint32_t k = 0; k < 3; ++k)
        {
            maniGraphJsonPutText(w, k > 0 ? ", " : "");
            maniGraphJsonPutUnsigned(w, s->triangles[3 * (size_t)t + k]);
        }
        maniGraphJsonPutText(w, "]");
    }
    maniGraphJsonPutText(w, "]");
}

// A condition in its form: a parameter compared with a value of its
// type, a trigger, or time in state.
static void PutCondition(Writer* w, const maniGraphSource* source,
                         const maniGraphConditionSource* c)
{
    static const char* const kCompares[] = {"\"==\"", "\"!=\"", "\"<\"",
                                            "\"<=\"", "\">\"",  "\">=\""};
    maniGraphJsonPutText(w, "{");
    if (c->kind == maniGraphConditionTimeInState || c->kind == maniGraphConditionPhase)
    {
        maniGraphJsonPutKey(w, c->kind == maniGraphConditionPhase ? "phase" : "timeInState");
        maniGraphJsonPutFloat(w, c->value.f[0]);
    }
    else if (c->kind == maniGraphConditionFinished)
    {
        maniGraphJsonPutKey(w, "finished");
        maniGraphJsonPutBool(w, true);
    }
    else if (c->kind == maniGraphConditionEvent)
    {
        maniGraphJsonPutKey(w, "event");
        maniGraphJsonPutString(w, c->event, c->eventLength);
    }
    else if (c->kind == maniGraphConditionTrigger)
    {
        maniGraphJsonPutKey(w, "trigger");
        maniGraphJsonPutParameterName(w, source, c->parameter);
    }
    else
    {
        maniGraphParameterType type = source->parameters[c->parameter].type;
        maniGraphJsonPutKey(w, "parameter");
        maniGraphJsonPutParameterName(w, source, c->parameter);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "compare");
        maniGraphJsonPutText(w, kCompares[c->compare]);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "value");
        if (type == maniGraphParameterBool)
        {
            maniGraphJsonPutBool(w, c->value.b != 0);
        }
        else if (type == maniGraphParameterInt)
        {
            maniGraphJsonPutInt(w, c->value.i);
        }
        else
        {
            maniGraphJsonPutFloat(w, c->value.f[0]);
        }
    }
    maniGraphJsonPutText(w, "}");
}

static void PutTransition(Writer* w, const maniGraphSource* source,
                          const maniGraphTransitionSource* t)
{
    static const char* const kInterruptions[] = {"\"none\"", "\"higher\"", "\"any\""};
    static const char* const kEntries[] = {"\"restart\"", "\"keep\"", "\"match\""};
    static const char* const kFades[] = {"\"linear\"", "\"smoothstep\"", "\"inertial\""};
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "from");
    if (t->from == MANI_GRAPH_ANY_STATE)
    {
        maniGraphJsonPutText(w, "\"any\"");
    }
    else
    {
        maniGraphJsonPutUnsigned(w, t->from);
    }
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "to");
    maniGraphJsonPutUnsigned(w, t->to);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "priority");
    maniGraphJsonPutUnsigned(w, t->priority);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "interruption");
    maniGraphJsonPutText(w, kInterruptions[t->interruption]);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "entry");
    maniGraphJsonPutText(w, kEntries[t->entry]);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "fade");
    maniGraphJsonPutText(w, kFades[t->fade]);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "duration");
    maniGraphJsonPutFloat(w, t->duration);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "cooldown");
    maniGraphJsonPutFloat(w, t->cooldown);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "conditions");
    maniGraphJsonPutText(w, "[");
    for (uint32_t c = 0; c < t->conditionCount; ++c)
    {
        maniGraphJsonPutText(w, c > 0 ? ", " : "");
        PutCondition(w, source, &t->conditions[c]);
    }
    maniGraphJsonPutText(w, "]}");
}

// A machine's entry, chain limit, transitions and finals, on the node's
// line.
static void PutMachine(Writer* w, const maniGraphSource* source,
                       const maniGraphMachineNodeSource* m)
{
    maniGraphJsonPutKey(w, "entry");
    maniGraphJsonPutUnsigned(w, m->entry);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "chainLimit");
    maniGraphJsonPutUnsigned(w, m->chainLimit);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "transitions");
    maniGraphJsonPutText(w, "[");
    for (uint32_t t = 0; t < m->transitionCount; ++t)
    {
        maniGraphJsonPutText(w, t > 0 ? ", " : "");
        PutTransition(w, source, &m->transitions[t]);
    }
    maniGraphJsonPutText(w, "], ");
    maniGraphJsonPutKey(w, "finals");
    maniGraphJsonPutText(w, "[");
    for (uint32_t k = 0; k < m->finalCount; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutUnsigned(w, m->finals[k]);
    }
    maniGraphJsonPutText(w, "]");
}

static void PutSubgraphNode(Writer* w, const maniGraphSource* source,
                            const maniGraphSubgraphNodeSource* s, uint32_t depth)
{
    maniGraphJsonPutKey(w, "parameters");
    maniGraphJsonPutText(w, "{");
    for (uint32_t k = 0; k < s->graph->parameterCount; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutParameterName(w, s->graph, k);
        maniGraphJsonPutText(w, ": ");
        maniGraphJsonPutParameterName(w, source, s->parameters[k]);
    }
    maniGraphJsonPutText(w, "}, ");
    maniGraphJsonPutKey(w, "graph");
    PutGraph(w, s->graph, depth, false);
}

// A node's own fields, by kind, after its kind and inputs.
// A blend's weights and direct flag, or an additive's weights.
static void PutWeights(Writer* w, const maniGraphSource* source, const maniGraphNodeSource* n)
{
    maniGraphJsonPutKey(w, "weights");
    if (n->kind == maniGraphNodeAdditive)
    {
        maniGraphJsonPutScalars(w, source, n->additive.weights, n->inputCount - 1);
        return;
    }
    maniGraphJsonPutScalars(w, source, n->blend.weights, n->inputCount);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "direct");
    maniGraphJsonPutBool(w, n->blend.direct != 0);
}

// An IK node's members after its inputs, by its kind.
static void PutIkNode(Writer* w, const maniGraphSource* source, const maniGraphNodeSource* n)
{
    if (n->kind == maniGraphNodeCcd || n->kind == maniGraphNodeFabrik)
    {
        maniGraphJsonPutChain(w, source, n);
    }
    else if (n->kind == maniGraphNodeBodyXpbd || n->kind == maniGraphNodeBodySdls)
    {
        maniGraphJsonPutBody(w, source, n);
    }
    else
    {
        maniGraphJsonPutIk(w, source, n);
    }
}

static void PutFields(Writer* w, const maniGraphSource* source, const maniGraphNodeSource* n,
                      uint32_t depth)
{
    switch (n->kind)
    {
    case maniGraphNodeClip:
        PutClipNode(w, source, &n->clip);
        return;
    case maniGraphNodeBlend:
    case maniGraphNodeAdditive:
        PutWeights(w, source, n);
        return;
    case maniGraphNodeMask:
        PutMaskNode(w, &n->mask);
        return;
    case maniGraphNodeOneShot:
        PutShot(w, source, &n->oneShot);
        return;
    case maniGraphNodeInertialize:
        PutInertialize(w, source, &n->inertialize);
        return;
    case maniGraphNodeMatch:
        PutMatch(w, source, &n->match);
        return;
    case maniGraphNodeLearned:
        PutLearned(w, source, &n->learned);
        return;
    case maniGraphNodeSpring:
        PutSpring(w, source, &n->spring);
        return;
    case maniGraphNodeTwoBone:
    case maniGraphNodeAim:
    case maniGraphNodeCcd:
    case maniGraphNodeFabrik:
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        PutIkNode(w, source, n);
        return;
    case maniGraphNodeSelect:
    case maniGraphNodePose:
        maniGraphJsonPutKey(w, "parameter");
        maniGraphJsonPutParameterName(
            w, source, n->kind == maniGraphNodePose ? n->pose.parameter : n->select.parameter);
        return;
    case maniGraphNodeTimeScale:
        maniGraphJsonPutKey(w, "scale");
        maniGraphJsonPutScalar(w, source, n->timeScale.scale);
        return;
    case maniGraphNodeBlendSpace1D:
        PutPosition(w, source, &n->blendSpace1D.position, false);
        PutPoints(w, n->blendSpace1D.points, n->inputCount, false);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "period");
        maniGraphJsonPutFloat(w, n->blendSpace1D.period);
        return;
    case maniGraphNodeBlendSpaceTriangles:
        PutPosition(w, source, &n->triangulated.position, true);
        PutPoints(w, n->triangulated.points, n->inputCount, true);
        PutTriangles(w, &n->triangulated);
        return;
    case maniGraphNodeBlendSpaceBands:
        PutPosition(w, source, &n->bands.position, true);
        PutPoints(w, n->bands.points, n->inputCount, true);
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "polar");
        maniGraphJsonPutBool(w, n->bands.polar != 0);
        return;
    case maniGraphNodeMachine:
        PutMachine(w, source, &n->machine);
        return;
    case maniGraphNodeMirror:
        return;
    case maniGraphNodeTurnLimit:
        PutTurn(w, source, &n->turnLimit);
        return;
    case maniGraphNodeSeek:
        PutSeek(w, source, &n->seek);
        return;
    default:
        PutSubgraphNode(w, source, &n->subgraph, depth);
        return;
    }
}

static const char* KindName(maniGraphNodeKind kind)
{
    static const char* const kKinds[] = {"clip",
                                         "blend",
                                         "mask",
                                         "additive",
                                         "select",
                                         "timeScale",
                                         "seek",
                                         "blendSpace1D",
                                         "blendSpaceTriangles",
                                         "blendSpaceBands",
                                         "machine",
                                         "oneShot",
                                         "inertialize",
                                         "match",
                                         "learned",
                                         "spring",
                                         "pose",
                                         "twoBone",
                                         "aim",
                                         "ccd",
                                         "fabrik",
                                         "bodyXpbd",
                                         "bodySdls",
                                         "mirror",
                                         "turnLimit"};
    return kind == maniGraphNodeSubgraph ? "subgraph" : kKinds[kind - maniGraphNodeClip];
}

static void PutNode(Writer* w, const maniGraphSource* source, const maniGraphNodeSource* n,
                    uint32_t depth)
{
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "kind");
    maniGraphJsonPutText(w, "\"");
    maniGraphJsonPutText(w, KindName(n->kind));
    maniGraphJsonPutText(w, "\", ");
    if (n->kind != maniGraphNodeClip && n->kind != maniGraphNodeMatch &&
        n->kind != maniGraphNodeLearned && n->kind != maniGraphNodePose &&
        n->kind != maniGraphNodeSubgraph)
    {
        maniGraphJsonPutKey(w, "inputs");
        maniGraphJsonPutText(w, "[");
        for (uint32_t k = 0; k < n->inputCount; ++k)
        {
            maniGraphJsonPutText(w, k > 0 ? ", " : "");
            maniGraphJsonPutUnsigned(w, n->inputs[k]);
        }
        // A mirror node takes no other members.
        maniGraphJsonPutText(w, n->kind == maniGraphNodeMirror ? "]" : "], ");
    }
    PutFields(w, source, n, depth);
    maniGraphJsonPutText(w, "}");
}

// The name of a clip (list 1), database (4), model (5) or spring set (6)
// slot.
static void PutSlotName(Writer* w, const maniGraphSource* source, uint32_t list, uint32_t i)
{
    if (list == 1)
    {
        maniGraphJsonPutString(w, source->clips[i].name, source->clips[i].nameLength);
    }
    else if (list == 4)
    {
        maniGraphJsonPutString(w, source->databases[i].name, source->databases[i].nameLength);
    }
    else if (list == 5)
    {
        maniGraphJsonPutString(w, source->models[i].name, source->models[i].nameLength);
    }
    else
    {
        maniGraphJsonPutString(w, source->springSets[i].name, source->springSets[i].nameLength);
    }
}

// A list's opening, its items each on a line, and its closing; an empty
// list on one line.
static void PutList(Writer* w, const maniGraphSource* source, const char* key, uint32_t list,
                    uint32_t depth)
{
    uint32_t counts[] = {source->parameterCount, source->clipCount,     source->syncGroupCount,
                         source->nodeCount,      source->databaseCount, source->modelCount,
                         source->springSetCount};
    maniGraphJsonLine(w, depth);
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutText(w, "[");
    for (uint32_t i = 0; i < counts[list]; ++i)
    {
        maniGraphJsonPutText(w, i > 0 ? "," : "");
        maniGraphJsonLine(w, depth + 1);
        if (list == 0)
        {
            PutParameter(w, &source->parameters[i]);
        }
        else if (list == 1 || list >= 4)
        {
            PutSlotName(w, source, list, i);
        }
        else if (list == 2)
        {
            const maniGraphSyncGroupSource* g = &source->syncGroups[i];
            maniGraphJsonPutText(w, "{");
            maniGraphJsonPutKey(w, "name");
            maniGraphJsonPutString(w, g->name, g->nameLength);
            maniGraphJsonPutText(w, ", ");
            maniGraphJsonPutKey(w, "allEvents");
            maniGraphJsonPutBool(w, (g->flags & maniGraphSyncAllEvents) != 0);
            maniGraphJsonPutText(w, "}");
        }
        else
        {
            PutNode(w, source, &source->nodes[i], depth + 1);
        }
    }
    if (counts[list] > 0)
    {
        maniGraphJsonLine(w, depth);
    }
    maniGraphJsonPutText(w, "],");
}

static void PutGraph(Writer* w, const maniGraphSource* source, uint32_t depth, bool top)
{
    maniGraphJsonPutText(w, "{");
    if (top)
    {
        maniGraphJsonLine(w, depth + 1);
        maniGraphJsonPutKey(w, "format");
        maniGraphJsonPutText(w, "\"maul-anim graph\",");
        maniGraphJsonLine(w, depth + 1);
        maniGraphJsonPutKey(w, "version");
        maniGraphJsonPutText(w, "1,");
    }
    PutList(w, source, "parameters", 0, depth + 1);
    PutList(w, source, "clips", 1, depth + 1);
    PutList(w, source, "syncGroups", 2, depth + 1);
    if (source->databaseCount > 0)
    {
        PutList(w, source, "databases", 4, depth + 1);
    }
    if (source->modelCount > 0)
    {
        PutList(w, source, "models", 5, depth + 1);
    }
    if (source->springSetCount > 0)
    {
        PutList(w, source, "springSets", 6, depth + 1);
    }
    PutList(w, source, "nodes", 3, depth + 1);
    maniGraphJsonLine(w, depth + 1);
    maniGraphJsonPutKey(w, "output");
    maniGraphJsonPutUnsigned(w, source->output);
    maniGraphJsonLine(w, depth);
    maniGraphJsonPutText(w, "}");
}

static MANI_NOINLINE maniBuildResult WriteGraphJsonWork(maniBuilder* builder,
                                                        const maniGraphSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result != mani_success)
    {
        return r;
    }
    Writer w = {NULL, 0, source->skeleton};
    PutGraph(&w, source, 0, true);
    maniGraphJsonPutText(&w, "\n");
    size_t size = w.at;
    // Kept as files are, 8-byte aligned.
    char* text = maniAllocate(&builder->allocator, size, 1, 8);
    if (text == NULL)
    {
        maniBuilderClear(builder);
        return (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0};
    }
    w = (Writer){text, 0, source->skeleton};
    PutGraph(&w, source, 0, true);
    maniGraphJsonPutText(&w, "\n");
    maniBuilderKeep(builder, MANI_BUILT_TEXT, (uint8_t*)text, size, NULL, 0);
    return r;
}

maniBuildResult maniWriteGraphJson(maniBuilder* builder, const maniGraphSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = WriteGraphJsonWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
