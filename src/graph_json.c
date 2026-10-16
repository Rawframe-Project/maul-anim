// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph source written as its JSON text form (mani-0023): built first,
// so only a source that builds is written; then each list item on a line
// of its own, parameters, clips, sync groups and bones by name, nodes by
// their place in the source, subgraphs as nested graphs, and numbers as
// the shortest binary32 text. Counted once, then written.

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

static void PutGraph(Writer* w, const maniGraphSource* source, uint32_t depth, bool top);

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
    static const char* const kKinds[] = {"clip",   "blend",     "mask", "additive",
                                         "select", "timeScale", "seek"};
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
    if (n->kind != maniGraphNodeClip && n->kind != maniGraphNodeSubgraph)
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
    uint32_t counts[] = {source->parameterCount, source->clipCount, source->syncGroupCount,
                         source->nodeCount};
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
