// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph source written as its JSON text form (mani-0023): built first,
// so only a source that builds is written; then each list item on a line
// of its own, parameters, clips, sync groups, databases (mani-0024, when
// it has any) and bones by name, nodes by their place in the source,
// subgraphs as nested graphs, and numbers as the shortest binary32 text.
// Counted once, then written.

#include "allocator.h"
#include "builder.h"
#include "decimal_format.h"

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

// The text's bytes so far; out is NULL while counting.
typedef struct Writer
{
    char* out;
    size_t at;
    const maniSkeleton* skeleton;
} Writer;

static void Put(Writer* w, const char* bytes, size_t length)
{
    if (w->out != NULL)
    {
        memcpy(w->out + w->at, bytes, length);
    }
    w->at += length;
}

static void PutText(Writer* w, const char* text)
{
    Put(w, text, strlen(text));
}

// A new line indented by depth steps of two spaces.
static void Line(Writer* w, uint32_t depth)
{
    PutText(w, "\n");
    for (uint32_t k = 0; k < depth; ++k)
    {
        PutText(w, "  ");
    }
}

// A JSON string: quote, backslash and the controls escaped.
static void PutString(Writer* w, const char* bytes, uint32_t length)
{
    static const char kHex[] = "0123456789abcdef";
    PutText(w, "\"");
    for (uint32_t i = 0; i < length; ++i)
    {
        uint8_t c = (uint8_t)bytes[i];
        char escape[6] = {'\\', 'u', '0', '0', kHex[c >> 4], kHex[c & 15u]};
        const char* named = c == '"'    ? "\\\""
                            : c == '\\' ? "\\\\"
                            : c == '\n' ? "\\n"
                            : c == '\r' ? "\\r"
                            : c == '\t' ? "\\t"
                                        : NULL;
        if (named != NULL)
        {
            PutText(w, named);
        }
        else if (c < 0x20u)
        {
            Put(w, escape, sizeof escape);
        }
        else
        {
            Put(w, bytes + i, 1);
        }
    }
    PutText(w, "\"");
}

// A key and the separator after it.
static void PutKey(Writer* w, const char* key)
{
    PutText(w, "\"");
    PutText(w, key);
    PutText(w, "\": ");
}

static void PutFloat(Writer* w, float value)
{
    char text[MANI_DECIMAL_MAX];
    Put(w, text, maniFormatBinary32(value, text));
}

static void PutUnsigned(Writer* w, uint64_t value)
{
    char reversed[20];
    uint32_t n = 0;
    do
    {
        reversed[n++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value != 0);
    while (n > 0)
    {
        Put(w, &reversed[--n], 1);
    }
}

static void PutInt(Writer* w, int32_t value)
{
    if (value < 0)
    {
        PutText(w, "-");
    }
    PutUnsigned(w, value < 0 ? (uint64_t)(-(int64_t)value) : (uint64_t)value);
}

static void PutBool(Writer* w, bool value)
{
    PutText(w, value ? "true" : "false");
}

static void PutParameterName(Writer* w, const maniGraphSource* source, uint32_t parameter)
{
    const maniGraphParameterSource* p = &source->parameters[parameter];
    PutString(w, p->name, p->nameLength);
}

// A number a node takes: a literal, or the parameter it reads.
static void PutScalar(Writer* w, const maniGraphSource* source, maniGraphScalar s)
{
    if (s.parameter == MANI_NO_PARAMETER)
    {
        PutFloat(w, s.value);
        return;
    }
    PutText(w, "{");
    PutKey(w, "parameter");
    PutParameterName(w, source, s.parameter);
    PutText(w, "}");
}

static void PutScalars(Writer* w, const maniGraphSource* source, const maniGraphScalar* scalars,
                       uint32_t count)
{
    PutText(w, "[");
    for (uint32_t k = 0; k < count; ++k)
    {
        PutText(w, k > 0 ? ", " : "");
        PutScalar(w, source, scalars[k]);
    }
    PutText(w, "]");
}

static const char* const kTypes[] = {"bool", "int", "float", "vec2", "vec3", "quat", "trigger"};

// A parameter's value of its type: true or false, an integer, a number
// or an array of 2 to 4 numbers.
static void PutValue(Writer* w, maniGraphParameterType type, maniGraphValue v)
{
    if (type == maniGraphParameterBool || type == maniGraphParameterTrigger)
    {
        PutBool(w, v.b != 0);
        return;
    }
    if (type == maniGraphParameterInt)
    {
        PutInt(w, v.i);
        return;
    }
    if (type == maniGraphParameterFloat)
    {
        PutFloat(w, v.f[0]);
        return;
    }
    uint32_t n = type == maniGraphParameterVec2 ? 2u : (type == maniGraphParameterVec3 ? 3u : 4u);
    PutText(w, "[");
    for (uint32_t k = 0; k < n; ++k)
    {
        PutText(w, k > 0 ? ", " : "");
        PutFloat(w, v.f[k]);
    }
    PutText(w, "]");
}

static void PutParameter(Writer* w, const maniGraphParameterSource* p)
{
    PutText(w, "{");
    PutKey(w, "name");
    PutString(w, p->name, p->nameLength);
    PutText(w, ", ");
    PutKey(w, "type");
    PutString(w, kTypes[p->type], (uint32_t)strlen(kTypes[p->type]));
    PutText(w, ", ");
    PutKey(w, "default");
    PutValue(w, p->type, p->defaultValue);
    if (p->bounded != 0)
    {
        PutText(w, ", ");
        PutKey(w, "minimum");
        PutValue(w, p->type, p->minimum);
        PutText(w, ", ");
        PutKey(w, "maximum");
        PutValue(w, p->type, p->maximum);
    }
    PutText(w, "}");
}

static void PutClipNode(Writer* w, const maniGraphSource* source, const maniGraphClipNodeSource* c)
{
    static const char* const kLoops[] = {"\"clip\"", "\"clamp\"", "\"repeat\""};
    const maniGraphClipSource* clip = &source->clips[c->clip];
    PutKey(w, "clip");
    PutString(w, clip->name, clip->nameLength);
    PutText(w, ", ");
    PutKey(w, "syncGroup");
    if (c->syncGroup == MANI_NO_SYNC_GROUP)
    {
        PutText(w, "null");
    }
    else
    {
        const maniGraphSyncGroupSource* g = &source->syncGroups[c->syncGroup];
        PutString(w, g->name, g->nameLength);
    }
    PutText(w, ", ");
    PutKey(w, "speed");
    PutScalar(w, source, c->speed);
    PutText(w, ", ");
    PutKey(w, "loop");
    PutText(w, kLoops[c->loop]);
    PutText(w, ", ");
    PutKey(w, "leader");
    PutBool(w, (c->flags & maniGraphClipLeader) != 0);
}

static void PutMaskNode(Writer* w, const maniGraphMaskNodeSource* m)
{
    PutKey(w, "chains");
    PutText(w, "[");
    for (uint32_t k = 0; k < m->chainCount; ++k)
    {
        uint32_t length = 0;
        const char* name = maniSkeletonBoneName(w->skeleton, m->chains[k].bone, &length);
        PutText(w, k > 0 ? ", " : "");
        PutText(w, "{");
        PutKey(w, "bone");
        PutString(w, name, length);
        PutText(w, ", ");
        PutKey(w, "weight");
        PutFloat(w, m->chains[k].weight);
        PutText(w, ", ");
        PutKey(w, "subtree");
        PutBool(w, m->chains[k].subtree);
        PutText(w, "}");
    }
    PutText(w, "], ");
    PutKey(w, "defaultWeight");
    PutFloat(w, m->defaultWeight);
}

// An inertialize node: its method and half-lives for dead blending only.
static void PutInertialize(Writer* w, const maniGraphSource* source,
                           const maniGraphInertializeNodeSource* s)
{
    if (s->trigger != MANI_NO_PARAMETER)
    {
        PutKey(w, "trigger");
        PutParameterName(w, source, s->trigger);
        PutText(w, ", ");
    }
    if (s->method == maniGraphDeadBlending)
    {
        PutKey(w, "method");
        PutText(w, "\"deadBlending\", ");
    }
    PutKey(w, "duration");
    PutScalar(w, source, s->duration);
    if (s->method == maniGraphDeadBlending)
    {
        const char* const keys[3] = {"halfLifeScale", "halfLifeMin", "halfLifeMax"};
        const float values[3] = {s->halfLifeScale, s->halfLifeMin, s->halfLifeMax};
        for (uint32_t k = 0; k < 3; ++k)
        {
            PutText(w, ", ");
            PutKey(w, keys[k]);
            PutFloat(w, values[k]);
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
            PutKey(w, kKeys[k]);
            PutParameterName(w, source, triggers[k]);
            PutText(w, ", ");
        }
    }
    PutKey(w, "curve");
    PutText(w, s->curve == maniGraphFadeSmoothstep ? "\"smoothstep\", " : "\"linear\", ");
    PutKey(w, "fadeIn");
    PutFloat(w, s->fadeIn);
    PutText(w, ", ");
    PutKey(w, "fadeOut");
    PutFloat(w, s->fadeOut);
    if (s->mask != NULL)
    {
        PutText(w, ", ");
        PutKey(w, "mask");
        PutText(w, "{");
        PutMaskNode(w, s->mask);
        PutText(w, "}");
    }
}

// Tags: their bits, or {"parameter": name}.
static void PutTags(Writer* w, const maniGraphSource* source, maniGraphTags tags)
{
    if (tags.parameter == MANI_NO_PARAMETER)
    {
        PutUnsigned(w, tags.value);
        return;
    }
    PutText(w, "{");
    PutKey(w, "parameter");
    PutParameterName(w, source, tags.parameter);
    PutText(w, "}");
}

// A motion matching node: its database by name, its trigger (when it has
// one), its query's parameters, its numbers, tags and duration.
static void PutMatch(Writer* w, const maniGraphSource* source, const maniGraphMatchNodeSource* m)
{
    const maniGraphDatabaseSource* d = &source->databases[m->database];
    PutKey(w, "database");
    PutString(w, d->name, d->nameLength);
    PutText(w, ", ");
    if (m->trigger != MANI_NO_PARAMETER)
    {
        PutKey(w, "trigger");
        PutParameterName(w, source, m->trigger);
        PutText(w, ", ");
    }
    PutKey(w, "query");
    PutText(w, "[");
    for (uint32_t q = 0; q < m->queryCount; ++q)
    {
        PutText(w, q > 0 ? ", " : "");
        PutParameterName(w, source, m->query[q]);
    }
    PutText(w, "]");
    static const char* const kKeys[6] = {"period",      "jumpThreshold",  "endExclusion",
                                         "historyTime", "continuingBias", "transitionCost"};
    const float values[6] = {m->period,      m->jumpThreshold,  m->endExclusion,
                             m->historyTime, m->continuingBias, m->transitionCost};
    for (uint32_t k = 0; k < 6; ++k)
    {
        PutText(w, ", ");
        PutKey(w, kKeys[k]);
        PutFloat(w, values[k]);
    }
    PutText(w, ", ");
    PutKey(w, "historyCount");
    PutUnsigned(w, m->historyCount);
    PutText(w, ", ");
    PutKey(w, "requiredTags");
    PutTags(w, source, m->requiredTags);
    PutText(w, ", ");
    PutKey(w, "excludedTags");
    PutTags(w, source, m->excludedTags);
    PutText(w, ", ");
    PutKey(w, "duration");
    PutScalar(w, source, m->duration);
}

// A learned node: its model by name, its trigger (when it has one), its
// query's parameters, its period and transition cost, tags and duration.
static void PutLearned(Writer* w, const maniGraphSource* source,
                       const maniGraphLearnedNodeSource* l)
{
    const maniGraphModelSource* m = &source->models[l->model];
    PutKey(w, "model");
    PutString(w, m->name, m->nameLength);
    PutText(w, ", ");
    if (l->trigger != MANI_NO_PARAMETER)
    {
        PutKey(w, "trigger");
        PutParameterName(w, source, l->trigger);
        PutText(w, ", ");
    }
    PutKey(w, "query");
    PutText(w, "[");
    for (uint32_t q = 0; q < l->queryCount; ++q)
    {
        PutText(w, q > 0 ? ", " : "");
        PutParameterName(w, source, l->query[q]);
    }
    PutText(w, "], ");
    PutKey(w, "period");
    PutFloat(w, l->period);
    PutText(w, ", ");
    PutKey(w, "transitionCost");
    PutFloat(w, l->transitionCost);
    PutText(w, ", ");
    PutKey(w, "requiredTags");
    PutTags(w, source, l->requiredTags);
    PutText(w, ", ");
    PutKey(w, "excludedTags");
    PutTags(w, source, l->excludedTags);
    PutText(w, ", ");
    PutKey(w, "duration");
    PutScalar(w, source, l->duration);
}

static void PutGraph(Writer* w, const maniGraphSource* source, uint32_t depth, bool top);

// A blend space's position: a vec2 parameter, or x (and, in a plane,
// y).
static void PutPosition(Writer* w, const maniGraphSource* source, const maniGraphPosition* p,
                        bool plane)
{
    PutKey(w, "position");
    PutText(w, "{");
    if (p->vector != MANI_NO_PARAMETER)
    {
        PutKey(w, "vector");
        PutParameterName(w, source, p->vector);
    }
    else
    {
        PutKey(w, "x");
        PutScalar(w, source, p->x);
        if (plane)
        {
            PutText(w, ", ");
            PutKey(w, "y");
            PutScalar(w, source, p->y);
        }
    }
    PutText(w, "}, ");
}

// A blend space's points: numbers on a line, [x, y] pairs in a plane.
static void PutPoints(Writer* w, const float* points, uint32_t count, bool pairs)
{
    PutKey(w, "points");
    PutText(w, "[");
    for (uint32_t k = 0; k < count; ++k)
    {
        PutText(w, k > 0 ? ", " : "");
        if (!pairs)
        {
            PutFloat(w, points[k]);
            continue;
        }
        PutText(w, "[");
        PutFloat(w, points[2 * k]);
        PutText(w, ", ");
        PutFloat(w, points[2 * k + 1]);
        PutText(w, "]");
    }
    PutText(w, "]");
}

// A triangulated space's authored triangles, three inputs each; nothing
// for a Delaunay triangulation.
static void PutTriangles(Writer* w, const maniGraphTrianglesNodeSource* s)
{
    if (s->triangleCount == 0)
    {
        return;
    }
    PutText(w, ", ");
    PutKey(w, "triangles");
    PutText(w, "[");
    for (uint32_t t = 0; t < s->triangleCount; ++t)
    {
        PutText(w, t > 0 ? ", [" : "[");
        for (uint32_t k = 0; k < 3; ++k)
        {
            PutText(w, k > 0 ? ", " : "");
            PutUnsigned(w, s->triangles[3 * (size_t)t + k]);
        }
        PutText(w, "]");
    }
    PutText(w, "]");
}

// A condition in its form: a parameter compared with a value of its
// type, a trigger, or time in state.
static void PutCondition(Writer* w, const maniGraphSource* source,
                         const maniGraphConditionSource* c)
{
    static const char* const kCompares[] = {"\"==\"", "\"!=\"", "\"<\"",
                                            "\"<=\"", "\">\"",  "\">=\""};
    PutText(w, "{");
    if (c->kind == maniGraphConditionTimeInState || c->kind == maniGraphConditionPhase)
    {
        PutKey(w, c->kind == maniGraphConditionPhase ? "phase" : "timeInState");
        PutFloat(w, c->value.f[0]);
    }
    else if (c->kind == maniGraphConditionFinished)
    {
        PutKey(w, "finished");
        PutBool(w, true);
    }
    else if (c->kind == maniGraphConditionEvent)
    {
        PutKey(w, "event");
        PutString(w, c->event, c->eventLength);
    }
    else if (c->kind == maniGraphConditionTrigger)
    {
        PutKey(w, "trigger");
        PutParameterName(w, source, c->parameter);
    }
    else
    {
        maniGraphParameterType type = source->parameters[c->parameter].type;
        PutKey(w, "parameter");
        PutParameterName(w, source, c->parameter);
        PutText(w, ", ");
        PutKey(w, "compare");
        PutText(w, kCompares[c->compare]);
        PutText(w, ", ");
        PutKey(w, "value");
        if (type == maniGraphParameterBool)
        {
            PutBool(w, c->value.b != 0);
        }
        else if (type == maniGraphParameterInt)
        {
            PutInt(w, c->value.i);
        }
        else
        {
            PutFloat(w, c->value.f[0]);
        }
    }
    PutText(w, "}");
}

static void PutTransition(Writer* w, const maniGraphSource* source,
                          const maniGraphTransitionSource* t)
{
    static const char* const kInterruptions[] = {"\"none\"", "\"higher\"", "\"any\""};
    static const char* const kEntries[] = {"\"restart\"", "\"keep\"", "\"match\""};
    static const char* const kFades[] = {"\"linear\"", "\"smoothstep\"", "\"inertial\""};
    PutText(w, "{");
    PutKey(w, "from");
    if (t->from == MANI_GRAPH_ANY_STATE)
    {
        PutText(w, "\"any\"");
    }
    else
    {
        PutUnsigned(w, t->from);
    }
    PutText(w, ", ");
    PutKey(w, "to");
    PutUnsigned(w, t->to);
    PutText(w, ", ");
    PutKey(w, "priority");
    PutUnsigned(w, t->priority);
    PutText(w, ", ");
    PutKey(w, "interruption");
    PutText(w, kInterruptions[t->interruption]);
    PutText(w, ", ");
    PutKey(w, "entry");
    PutText(w, kEntries[t->entry]);
    PutText(w, ", ");
    PutKey(w, "fade");
    PutText(w, kFades[t->fade]);
    PutText(w, ", ");
    PutKey(w, "duration");
    PutFloat(w, t->duration);
    PutText(w, ", ");
    PutKey(w, "cooldown");
    PutFloat(w, t->cooldown);
    PutText(w, ", ");
    PutKey(w, "conditions");
    PutText(w, "[");
    for (uint32_t c = 0; c < t->conditionCount; ++c)
    {
        PutText(w, c > 0 ? ", " : "");
        PutCondition(w, source, &t->conditions[c]);
    }
    PutText(w, "]}");
}

// A machine's entry, chain limit, transitions and finals, on the node's
// line.
static void PutMachine(Writer* w, const maniGraphSource* source,
                       const maniGraphMachineNodeSource* m)
{
    PutKey(w, "entry");
    PutUnsigned(w, m->entry);
    PutText(w, ", ");
    PutKey(w, "chainLimit");
    PutUnsigned(w, m->chainLimit);
    PutText(w, ", ");
    PutKey(w, "transitions");
    PutText(w, "[");
    for (uint32_t t = 0; t < m->transitionCount; ++t)
    {
        PutText(w, t > 0 ? ", " : "");
        PutTransition(w, source, &m->transitions[t]);
    }
    PutText(w, "], ");
    PutKey(w, "finals");
    PutText(w, "[");
    for (uint32_t k = 0; k < m->finalCount; ++k)
    {
        PutText(w, k > 0 ? ", " : "");
        PutUnsigned(w, m->finals[k]);
    }
    PutText(w, "]");
}

static void PutSubgraphNode(Writer* w, const maniGraphSource* source,
                            const maniGraphSubgraphNodeSource* s, uint32_t depth)
{
    PutKey(w, "parameters");
    PutText(w, "{");
    for (uint32_t k = 0; k < s->graph->parameterCount; ++k)
    {
        PutText(w, k > 0 ? ", " : "");
        PutParameterName(w, s->graph, k);
        PutText(w, ": ");
        PutParameterName(w, source, s->parameters[k]);
    }
    PutText(w, "}, ");
    PutKey(w, "graph");
    PutGraph(w, s->graph, depth, false);
}

// A node's own fields, by kind, after its kind and inputs.
static void PutFields(Writer* w, const maniGraphSource* source, const maniGraphNodeSource* n,
                      uint32_t depth)
{
    switch (n->kind)
    {
    case maniGraphNodeClip:
        PutClipNode(w, source, &n->clip);
        return;
    case maniGraphNodeBlend:
        PutKey(w, "weights");
        PutScalars(w, source, n->blend.weights, n->inputCount);
        PutText(w, ", ");
        PutKey(w, "direct");
        PutBool(w, n->blend.direct != 0);
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
    case maniGraphNodeAdditive:
        PutKey(w, "weights");
        PutScalars(w, source, n->additive.weights, n->inputCount - 1);
        return;
    case maniGraphNodeSelect:
        PutKey(w, "parameter");
        PutParameterName(w, source, n->select.parameter);
        return;
    case maniGraphNodeTimeScale:
        PutKey(w, "scale");
        PutScalar(w, source, n->timeScale.scale);
        return;
    case maniGraphNodeBlendSpace1D:
        PutPosition(w, source, &n->blendSpace1D.position, false);
        PutPoints(w, n->blendSpace1D.points, n->inputCount, false);
        PutText(w, ", ");
        PutKey(w, "period");
        PutFloat(w, n->blendSpace1D.period);
        return;
    case maniGraphNodeBlendSpaceTriangles:
        PutPosition(w, source, &n->triangulated.position, true);
        PutPoints(w, n->triangulated.points, n->inputCount, true);
        PutTriangles(w, &n->triangulated);
        return;
    case maniGraphNodeBlendSpaceBands:
        PutPosition(w, source, &n->bands.position, true);
        PutPoints(w, n->bands.points, n->inputCount, true);
        PutText(w, ", ");
        PutKey(w, "polar");
        PutBool(w, n->bands.polar != 0);
        return;
    case maniGraphNodeMachine:
        PutMachine(w, source, &n->machine);
        return;
    case maniGraphNodeSeek:
        PutKey(w, "trigger");
        PutParameterName(w, source, n->seek.trigger);
        PutText(w, ", ");
        PutKey(w, "time");
        PutScalar(w, source, n->seek.time);
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
                                         "learned"};
    return kind == maniGraphNodeSubgraph ? "subgraph" : kKinds[kind - maniGraphNodeClip];
}

static void PutNode(Writer* w, const maniGraphSource* source, const maniGraphNodeSource* n,
                    uint32_t depth)
{
    PutText(w, "{");
    PutKey(w, "kind");
    PutText(w, "\"");
    PutText(w, KindName(n->kind));
    PutText(w, "\", ");
    if (n->kind != maniGraphNodeClip && n->kind != maniGraphNodeMatch &&
        n->kind != maniGraphNodeLearned && n->kind != maniGraphNodeSubgraph)
    {
        PutKey(w, "inputs");
        PutText(w, "[");
        for (uint32_t k = 0; k < n->inputCount; ++k)
        {
            PutText(w, k > 0 ? ", " : "");
            PutUnsigned(w, n->inputs[k]);
        }
        PutText(w, "], ");
    }
    PutFields(w, source, n, depth);
    PutText(w, "}");
}

// A list's opening, its items each on a line, and its closing; an empty
// list on one line.
static void PutList(Writer* w, const maniGraphSource* source, const char* key, uint32_t list,
                    uint32_t depth)
{
    uint32_t counts[] = {source->parameterCount, source->clipCount,     source->syncGroupCount,
                         source->nodeCount,      source->databaseCount, source->modelCount};
    Line(w, depth);
    PutKey(w, key);
    PutText(w, "[");
    for (uint32_t i = 0; i < counts[list]; ++i)
    {
        PutText(w, i > 0 ? "," : "");
        Line(w, depth + 1);
        if (list == 0)
        {
            PutParameter(w, &source->parameters[i]);
        }
        else if (list == 1)
        {
            PutString(w, source->clips[i].name, source->clips[i].nameLength);
        }
        else if (list == 4)
        {
            PutString(w, source->databases[i].name, source->databases[i].nameLength);
        }
        else if (list == 5)
        {
            PutString(w, source->models[i].name, source->models[i].nameLength);
        }
        else if (list == 2)
        {
            const maniGraphSyncGroupSource* g = &source->syncGroups[i];
            PutText(w, "{");
            PutKey(w, "name");
            PutString(w, g->name, g->nameLength);
            PutText(w, ", ");
            PutKey(w, "allEvents");
            PutBool(w, (g->flags & maniGraphSyncAllEvents) != 0);
            PutText(w, "}");
        }
        else
        {
            PutNode(w, source, &source->nodes[i], depth + 1);
        }
    }
    if (counts[list] > 0)
    {
        Line(w, depth);
    }
    PutText(w, "],");
}

static void PutGraph(Writer* w, const maniGraphSource* source, uint32_t depth, bool top)
{
    PutText(w, "{");
    if (top)
    {
        Line(w, depth + 1);
        PutKey(w, "format");
        PutText(w, "\"maul-anim graph\",");
        Line(w, depth + 1);
        PutKey(w, "version");
        PutText(w, "1,");
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
    PutList(w, source, "nodes", 3, depth + 1);
    Line(w, depth + 1);
    PutKey(w, "output");
    PutUnsigned(w, source->output);
    Line(w, depth);
    PutText(w, "}");
}

maniBuildResult maniWriteGraphJson(maniBuilder* builder, const maniGraphSource* source)
{
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result != mani_success)
    {
        return r;
    }
    Writer w = {NULL, 0, source->skeleton};
    PutGraph(&w, source, 0, true);
    PutText(&w, "\n");
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
    PutText(&w, "\n");
    maniBuilderKeep(builder, MANI_BUILT_TEXT, (uint8_t*)text, size, NULL, 0);
    return r;
}
