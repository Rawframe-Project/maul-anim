// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph source read from its JSON form (mani-0023): parameters, clip
// slots bound to the host's clips by name, sync groups, database slots
// bound to the host's databases by name (mani-0024), model slots bound to
// the host's models by name (mani-0025), nodes by kind and
// nested subgraphs, each list's names in a table before the nodes refer
// to them; then built as maniBuildGraph builds any source.

#include "maul-anim/graph.h"

#include "arena.h"
#include "builder.h"
#include "graph_json_machine.h"
#include "graph_json_read.h"
#include "graph_json_search.h"
#include "graph_json_space.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef maniGraphReader Reader;
typedef maniGraphScope Scope;

#define ABSENT MANI_JSON_NONE

// A value of a parameter's type.
static bool Value(Reader* r, maniGraphParameterType type, uint32_t value, maniGraphValue* out)
{
    *out = (maniGraphValue){{0}};
    if (type == maniGraphParameterBool || type == maniGraphParameterTrigger)
    {
        bool b = false;
        bool ok = maniGraphJsonBool(r, value, &b);
        out->b = b ? 1u : 0u;
        return ok;
    }
    if (type == maniGraphParameterInt)
    {
        int64_t whole = 0;
        bool ok = maniJsonInteger(r->json, value, INT32_MIN, INT32_MAX, &whole);
        out->i = (int32_t)whole;
        return ok || maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    if (type == maniGraphParameterFloat)
    {
        return maniGraphJsonFloat(r, value, &out->f[0]);
    }
    uint32_t n = type == maniGraphParameterVec2 ? 2u : (type == maniGraphParameterVec3 ? 3u : 4u);
    return maniGraphJsonFloats(r, value, out->f, n);
}

static bool Parameter(Reader* r, uint32_t object, maniGraphParameterSource* p)
{
    static const char* const kKeys[] = {"name", "type", "default", "minimum", "maximum"};
    static const char* const kTypes[] = {"bool", "int",  "float",   "vec2",
                                         "vec3", "quat", "trigger", "pose"};
    uint32_t f[5];
    if (!maniGraphJsonMembers(r, object, kKeys, 5, 3u, f) ||
        !maniGraphJsonName(r, f[0], &p->name, &p->nameLength))
    {
        return false;
    }
    p->type = maniGraphJsonWord(r, f[1], kTypes, 8);
    if (p->type == 8)
    {
        return maniGraphJsonFault(r, f[1], maniGraphFormValue);
    }
    bool number = p->type == maniGraphParameterInt || p->type == maniGraphParameterFloat;
    if ((f[3] == ABSENT) != (f[4] == ABSENT))
    {
        return maniGraphJsonFault(r, object, maniGraphFormMissing);
    }
    if (f[3] != ABSENT && !number)
    {
        return maniGraphJsonFault(r, f[3] - 1, maniGraphFormMember);
    }
    if (f[2] != ABSENT && p->type == maniGraphParameterPose)
    {
        // A pose parameter has no default: the rest pose stands for it.
        return maniGraphJsonFault(r, f[2] - 1, maniGraphFormMember);
    }
    p->defaultValue = p->type == maniGraphParameterQuat ? (maniGraphValue){{0.0f, 0.0f, 0.0f, 1.0f}}
                                                        : (maniGraphValue){{0}};
    p->bounded = f[3] != ABSENT ? 1u : 0u;
    return (f[2] == ABSENT || Value(r, p->type, f[2], &p->defaultValue)) &&
           (f[3] == ABSENT ||
            (Value(r, p->type, f[3], &p->minimum) && Value(r, p->type, f[4], &p->maximum)));
}

// A list's elements and the names of its items into a table.
typedef struct List
{
    uint32_t count;
    const char** names;
    uint32_t* lengths;
} List;

static bool Begin(Reader* r, uint32_t value, List* list)
{
    list->count = 0;
    if (value != ABSENT && !maniGraphJsonArray(r, value, &list->count))
    {
        return false;
    }
    list->names = (const char**)maniArenaTake(&r->arena, list->count, sizeof(const char*));
    list->lengths = maniArenaTake(&r->arena, list->count, sizeof(uint32_t));
    return (list->names != NULL && list->lengths != NULL) || maniGraphJsonNoMemory(r);
}

static bool Parameters(Reader* r, uint32_t value, maniGraphSource* out, Scope* scope)
{
    List list;
    if (!Begin(r, value, &list))
    {
        return false;
    }
    maniGraphParameterSource* p =
        maniArenaTake(&r->arena, list.count, sizeof(maniGraphParameterSource));
    if (p == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < list.count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        if (!Parameter(r, e, &p[i]))
        {
            return false;
        }
        list.names[i] = p[i].name;
        list.lengths[i] = p[i].nameLength;
    }
    out->parameters = p;
    out->parameterCount = list.count;
    scope->sources = p;
    return maniGraphJsonTable(r, list.names, list.lengths, list.count, &scope->parameters);
}

// Clip slots by name, each bound to the host's clip of that name.
static bool Clips(Reader* r, uint32_t value, maniGraphSource* out, Scope* scope)
{
    List list;
    if (!Begin(r, value, &list))
    {
        return false;
    }
    maniGraphClipSource* c = maniArenaTake(&r->arena, list.count, sizeof(maniGraphClipSource));
    if (c == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < list.count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        if (!maniGraphJsonName(r, e, &c[i].name, &c[i].nameLength))
        {
            return false;
        }
        uint32_t host = maniGraphJsonLookup(&r->host, c[i].name, c[i].nameLength);
        if (host == ABSENT)
        {
            return maniGraphJsonFault(r, e, maniGraphFormUnbound);
        }
        c[i].clip = r->hostClips[host].clip;
        list.names[i] = c[i].name;
        list.lengths[i] = c[i].nameLength;
    }
    out->clips = c;
    out->clipCount = list.count;
    return maniGraphJsonTable(r, list.names, list.lengths, list.count, &scope->clips);
}

// Database slots by name, each bound to the host's database of that
// name.
static bool Databases(Reader* r, uint32_t value, maniGraphSource* out, Scope* scope)
{
    List list;
    if (!Begin(r, value, &list))
    {
        return false;
    }
    maniGraphDatabaseSource* d =
        maniArenaTake(&r->arena, list.count, sizeof(maniGraphDatabaseSource));
    if (d == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < list.count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        d[i] = (maniGraphDatabaseSource){0};
        if (!maniGraphJsonName(r, e, &d[i].name, &d[i].nameLength))
        {
            return false;
        }
        uint32_t host = maniGraphJsonLookup(&r->hostDatabaseNames, d[i].name, d[i].nameLength);
        if (host == ABSENT)
        {
            return maniGraphJsonFault(r, e, maniGraphFormUnbound);
        }
        d[i].database = r->hostDatabases[host].database;
        list.names[i] = d[i].name;
        list.lengths[i] = d[i].nameLength;
    }
    out->databases = d;
    out->databaseCount = list.count;
    return maniGraphJsonTable(r, list.names, list.lengths, list.count, &scope->databases);
}

// Model or spring set slots by name, each the host's slot of that name
// (stride bytes each, its asset first): the slots in itemsOut, their
// count in countOut, their names in table.
static bool HostSlots(Reader* r, uint32_t value, const void* host, size_t stride,
                      const maniNameTable* hostNames, const void** itemsOut, uint32_t* countOut,
                      maniNameTable* table)
{
    List list;
    if (!Begin(r, value, &list))
    {
        return false;
    }
    uint8_t* items = maniArenaTake(&r->arena, list.count, stride);
    if (items == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < list.count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        if (!maniGraphJsonName(r, e, &list.names[i], &list.lengths[i]))
        {
            return false;
        }
        uint32_t found = maniGraphJsonLookup(hostNames, list.names[i], list.lengths[i]);
        if (found == ABSENT)
        {
            return maniGraphJsonFault(r, e, maniGraphFormUnbound);
        }
        memcpy(items + (size_t)i * stride, (const uint8_t*)host + (size_t)found * stride, stride);
    }
    *itemsOut = items;
    *countOut = list.count;
    return maniGraphJsonTable(r, list.names, list.lengths, list.count, table);
}

static bool Models(Reader* r, uint32_t value, maniGraphSource* out, Scope* scope)
{
    const void* items = NULL;
    bool ok = HostSlots(r, value, r->hostModels, sizeof(maniGraphModelSource), &r->hostModelNames,
                        &items, &out->modelCount, &scope->models);
    out->models = items;
    return ok;
}

static bool SpringSets(Reader* r, uint32_t value, maniGraphSource* out, Scope* scope)
{
    const void* items = NULL;
    bool ok = HostSlots(r, value, r->hostSpringSets, sizeof(maniGraphSpringSetSource),
                        &r->hostSpringSetNames, &items, &out->springSetCount, &scope->springSets);
    out->springSets = items;
    return ok;
}

static bool SyncGroups(Reader* r, uint32_t value, maniGraphSource* out, Scope* scope)
{
    static const char* const kKeys[] = {"name", "allEvents"};
    List list;
    if (!Begin(r, value, &list))
    {
        return false;
    }
    maniGraphSyncGroupSource* g =
        maniArenaTake(&r->arena, list.count, sizeof(maniGraphSyncGroupSource));
    if (g == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < list.count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        uint32_t f[2];
        bool all = false;
        if (!maniGraphJsonMembers(r, e, kKeys, 2, 1u, f) ||
            !maniGraphJsonName(r, f[0], &g[i].name, &g[i].nameLength) ||
            (f[1] != ABSENT && !maniGraphJsonBool(r, f[1], &all)))
        {
            return false;
        }
        g[i].flags = all ? (uint32_t)maniGraphSyncAllEvents : 0u;
        list.names[i] = g[i].name;
        list.lengths[i] = g[i].nameLength;
    }
    out->syncGroups = g;
    out->syncGroupCount = list.count;
    return maniGraphJsonTable(r, list.names, list.lengths, list.count, &scope->syncGroups);
}

static bool Inputs(Reader* r, uint32_t value, maniGraphNodeSource* n)
{
    uint32_t count = 0;
    if (!maniGraphJsonArray(r, value, &count))
    {
        return false;
    }
    uint32_t* inputs = maniArenaTake(&r->arena, count, sizeof(uint32_t));
    if (inputs == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!maniGraphJsonIndex(r, e, &inputs[k]))
        {
            return false;
        }
    }
    n->inputCount = count;
    n->inputs = inputs;
    return true;
}

// count scalars.
static bool Weights(Reader* r, const Scope* scope, uint32_t value, uint32_t count,
                    const maniGraphScalar** out)
{
    uint32_t n = 0;
    if (!maniGraphJsonArray(r, value, &n))
    {
        return false;
    }
    if (n != count)
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    maniGraphScalar* weights = maniArenaTake(&r->arena, count, sizeof(maniGraphScalar));
    if (weights == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!maniGraphJsonScalar(r, scope, e, &weights[k]))
        {
            return false;
        }
    }
    *out = weights;
    return true;
}

static bool Chain(Reader* r, uint32_t object, maniMaskChain* chain)
{
    static const char* const kKeys[] = {"bone", "weight", "subtree"};
    uint32_t f[3];
    const char* name = NULL;
    uint32_t length = 0;
    bool subtree = false;
    if (!maniGraphJsonMembers(r, object, kKeys, 3, 3u, f) ||
        !maniGraphJsonName(r, f[0], &name, &length) ||
        !maniGraphJsonFloat(r, f[1], &chain->weight) ||
        (f[2] != ABSENT && !maniGraphJsonBool(r, f[2], &subtree)))
    {
        return false;
    }
    chain->subtree = subtree;
    chain->bone = maniSkeletonFindBone(r->skeleton, name, length);
    return chain->bone != MANI_NO_BONE || maniGraphJsonFault(r, f[0], maniGraphFormName);
}

static bool Mask(Reader* r, const uint32_t* f, maniGraphMaskNodeSource* m)
{
    uint32_t count = 0;
    if (f[2] != ABSENT && !maniGraphJsonArray(r, f[2], &count))
    {
        return false;
    }
    maniMaskChain* chains = maniArenaTake(&r->arena, count, sizeof(maniMaskChain));
    if (chains == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = f[2];
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Chain(r, e, &chains[k]))
        {
            return false;
        }
    }
    m->chains = chains;
    m->chainCount = count;
    m->defaultWeight = 0.0f;
    return f[3] == ABSENT || maniGraphJsonFloat(r, f[3], &m->defaultWeight);
}

// A one-shot: fire, and optionally fadeOutTrigger and abort, triggers by
// name; curve "linear" | "smoothstep"; fadeIn and fadeOut seconds; mask
// {"chains", "defaultWeight"} as a mask node's (left out: the whole
// body).
static bool Shot(Reader* r, const Scope* scope, const uint32_t* f, maniGraphOneShotNodeSource* s)
{
    static const char* const kCurves[] = {"linear", "smoothstep"};
    static const char* const kMask[] = {"chains", "defaultWeight"};
    *s = (maniGraphOneShotNodeSource){0, MANI_NO_PARAMETER, MANI_NO_PARAMETER, 0, 0.0f, 0.0f, NULL};
    s->curve = f[5] == ABSENT ? 0u : maniGraphJsonWord(r, f[5], kCurves, 2);
    if (!maniGraphJsonRefer(r, &scope->parameters, f[2], &s->fire) ||
        (f[3] != ABSENT && !maniGraphJsonRefer(r, &scope->parameters, f[3], &s->fadeOutTrigger)) ||
        (f[4] != ABSENT && !maniGraphJsonRefer(r, &scope->parameters, f[4], &s->abort)) ||
        (s->curve >= 2 && !maniGraphJsonFault(r, f[5], maniGraphFormValue)) ||
        (f[6] != ABSENT && !maniGraphJsonFloat(r, f[6], &s->fadeIn)) ||
        (f[7] != ABSENT && !maniGraphJsonFloat(r, f[7], &s->fadeOut)))
    {
        return false;
    }
    if (f[8] == ABSENT)
    {
        return true;
    }
    uint32_t members[4] = {ABSENT, ABSENT, ABSENT, ABSENT};
    maniGraphMaskNodeSource* mask = maniArenaTake(&r->arena, 1, sizeof(maniGraphMaskNodeSource));
    if (mask == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    s->mask = mask;
    return maniGraphJsonMembers(r, f[8], kMask, 2, 0u, members + 2) && Mask(r, members, mask);
}

// A spring node: its set by its slot's name; its position, rotation,
// reset and force parameters by name, none when left out.
static bool Spring(Reader* r, const Scope* scope, const uint32_t* f, maniGraphSpringNodeSource* s)
{
    uint32_t* parameters[4] = {&s->position, &s->rotation, &s->reset, &s->force};
    if (!maniGraphJsonRefer(r, &scope->springSets, f[2], &s->springSet))
    {
        return false;
    }
    for (uint32_t k = 0; k < 4; ++k)
    {
        *parameters[k] = MANI_NO_PARAMETER;
        if (f[3 + k] != ABSENT &&
            !maniGraphJsonRefer(r, &scope->parameters, f[3 + k], parameters[k]))
        {
            return false;
        }
    }
    return true;
}

// An inertialize node; dead blending's half-lives default to Holden's.
static bool Inertialize(Reader* r, const Scope* scope, const uint32_t* f,
                        maniGraphInertializeNodeSource* s)
{
    static const char* const kMethods[] = {"inertialization", "deadBlending"};
    *s = (maniGraphInertializeNodeSource){
        MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.0f}, 0.0f, 0.0f, 0.0f};
    s->method = f[3] == ABSENT ? 0u : maniGraphJsonWord(r, f[3], kMethods, 2);
    if (s->method == maniGraphDeadBlending)
    {
        s->halfLifeScale = 0.3f;
        s->halfLifeMin = 0.1f;
        s->halfLifeMax = 1.0f;
    }
    return (f[2] == ABSENT || maniGraphJsonRefer(r, &scope->parameters, f[2], &s->trigger)) &&
           (s->method < 2 || maniGraphJsonFault(r, f[3], maniGraphFormValue)) &&
           (f[4] == ABSENT || maniGraphJsonScalar(r, scope, f[4], &s->duration)) &&
           (f[5] == ABSENT || maniGraphJsonFloat(r, f[5], &s->halfLifeScale)) &&
           (f[6] == ABSENT || maniGraphJsonFloat(r, f[6], &s->halfLifeMin)) &&
           (f[7] == ABSENT || maniGraphJsonFloat(r, f[7], &s->halfLifeMax));
}

static bool ClipNode(Reader* r, const Scope* scope, const uint32_t* f, maniGraphClipNodeSource* c)
{
    static const char* const kLoops[] = {"clip", "clamp", "repeat"};
    bool leader = false;
    *c = (maniGraphClipNodeSource){0, MANI_NO_SYNC_GROUP, {MANI_NO_PARAMETER, 1.0f}, 0, 0};
    bool grouped = f[2] != ABSENT && r->json->values[f[2]].kind != maniJsonNull;
    if (!maniGraphJsonRefer(r, &scope->clips, f[1], &c->clip) ||
        (grouped && !maniGraphJsonRefer(r, &scope->syncGroups, f[2], &c->syncGroup)) ||
        (f[3] != ABSENT && !maniGraphJsonScalar(r, scope, f[3], &c->speed)) ||
        (f[5] != ABSENT && !maniGraphJsonBool(r, f[5], &leader)))
    {
        return false;
    }
    c->flags = leader ? (uint32_t)maniGraphClipLeader : 0u;
    c->loop = f[4] == ABSENT ? 0u : maniGraphJsonWord(r, f[4], kLoops, 3);
    return c->loop < 3u || maniGraphJsonFault(r, f[4], maniGraphFormValue);
}

static bool ReadGraph(Reader* r, uint32_t object, bool top, maniGraphSource* out, Scope* scope);

// A subgraph: its graph, and each of its parameters mapped by name to
// one of the scope's.
static bool Subgraph(Reader* r, const Scope* scope, uint32_t node, const uint32_t* f,
                     maniGraphSubgraphNodeSource* s)
{
    maniGraphSource* graph = maniArenaTake(&r->arena, 1, sizeof(maniGraphSource));
    Scope inner;
    if (graph == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    if (!ReadGraph(r, f[1], false, graph, &inner))
    {
        return false;
    }
    uint32_t* map = maniArenaTake(&r->arena, graph->parameterCount, sizeof(uint32_t));
    if (map == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    memset(map, 0xFF, sizeof(uint32_t) * (size_t)graph->parameterCount);
    s->graph = graph;
    s->parameters = map;
    if (f[2] != ABSENT && r->json->values[f[2]].kind != maniJsonObject)
    {
        return maniGraphJsonFault(r, f[2], maniGraphFormValue);
    }
    uint32_t members = f[2] != ABSENT ? r->json->values[f[2]].count : 0;
    uint32_t key = f[2] + 1;
    for (uint32_t m = 0; m < members; ++m)
    {
        uint32_t k = 0;
        if (!maniGraphJsonRefer(r, &inner.parameters, key, &k))
        {
            return false;
        }
        if (map[k] != ABSENT)
        {
            return maniGraphJsonFault(r, key, maniGraphFormRepeat);
        }
        if (!maniGraphJsonRefer(r, &scope->parameters, key + 1, &map[k]))
        {
            return false;
        }
        key = r->json->values[key + 1].next;
    }
    for (uint32_t k = 0; k < graph->parameterCount; ++k)
    {
        if (map[k] == ABSENT)
        {
            return maniGraphJsonFault(r, f[2] != ABSENT ? f[2] : node, maniGraphFormMissing);
        }
    }
    return true;
}

// Each kind's name, members (kind first) and the bits of those required.
typedef struct KindForm
{
    const char* name;
    maniGraphNodeKind kind;
    const char* keys[14];
    uint32_t count;
    uint32_t required;
} KindForm;

static const KindForm kForms[] = {
    {"clip", maniGraphNodeClip, {"kind", "clip", "syncGroup", "speed", "loop", "leader"}, 6, 3u},
    {"blend", maniGraphNodeBlend, {"kind", "inputs", "weights", "direct"}, 4, 7u},
    {"mask", maniGraphNodeMask, {"kind", "inputs", "chains", "defaultWeight"}, 4, 3u},
    {"additive", maniGraphNodeAdditive, {"kind", "inputs", "weights"}, 3, 7u},
    {"select", maniGraphNodeSelect, {"kind", "inputs", "parameter"}, 3, 7u},
    {"timeScale", maniGraphNodeTimeScale, {"kind", "inputs", "scale"}, 3, 7u},
    {"seek", maniGraphNodeSeek, {"kind", "inputs", "trigger", "time"}, 4, 7u},
    {"blendSpace1D",
     maniGraphNodeBlendSpace1D,
     {"kind", "inputs", "position", "points", "period"},
     5,
     15u},
    {"blendSpaceTriangles",
     maniGraphNodeBlendSpaceTriangles,
     {"kind", "inputs", "position", "points", "triangles"},
     5,
     15u},
    {"blendSpaceBands",
     maniGraphNodeBlendSpaceBands,
     {"kind", "inputs", "position", "points", "polar"},
     5,
     15u},
    {"machine",
     maniGraphNodeMachine,
     {"kind", "inputs", "entry", "chainLimit", "transitions", "finals"},
     6,
     3u},
    {"oneShot",
     maniGraphNodeOneShot,
     {"kind", "inputs", "fire", "fadeOutTrigger", "abort", "curve", "fadeIn", "fadeOut", "mask"},
     9,
     7u},
    {"inertialize",
     maniGraphNodeInertialize,
     {"kind", "inputs", "trigger", "method", "duration", "halfLifeScale", "halfLifeMin",
      "halfLifeMax"},
     8,
     3u},
    {"match",
     maniGraphNodeMatch,
     {"kind", "database", "trigger", "query", "period", "jumpThreshold", "endExclusion",
      "historyTime", "continuingBias", "transitionCost", "historyCount", "requiredTags",
      "excludedTags", "duration"},
     14,
     3u},
    {"learned",
     maniGraphNodeLearned,
     {"kind", "model", "trigger", "query", "period", "transitionCost", "requiredTags",
      "excludedTags", "duration"},
     9,
     3u},
    {"spring",
     maniGraphNodeSpring,
     {"kind", "inputs", "springSet", "position", "rotation", "reset", "force"},
     7,
     7u},
    {"pose", maniGraphNodePose, {"kind", "parameter"}, 2, 3u},
    {"subgraph", maniGraphNodeSubgraph, {"kind", "graph", "parameters"}, 3, 3u},
};

#define FORMS (sizeof kForms / sizeof kForms[0])

// A node's fields of its kind (f the members of kForms[form]).
static bool Fields(Reader* r, const Scope* scope, uint32_t node, const uint32_t* f,
                   maniGraphNodeSource* n)
{
    bool direct = false;
    switch (n->kind)
    {
    case maniGraphNodeClip:
        return ClipNode(r, scope, f, &n->clip);
    case maniGraphNodeBlend:
        n->blend.direct = 0;
        if (f[3] != ABSENT && !maniGraphJsonBool(r, f[3], &direct))
        {
            return false;
        }
        n->blend.direct = direct ? 1u : 0u;
        return Weights(r, scope, f[2], n->inputCount, &n->blend.weights);
    case maniGraphNodeMask:
        return Mask(r, f, &n->mask);
    case maniGraphNodeAdditive:
        return Weights(r, scope, f[2], n->inputCount > 0 ? n->inputCount - 1 : 0,
                       &n->additive.weights);
    case maniGraphNodeSelect:
        return maniGraphJsonRefer(r, &scope->parameters, f[2], &n->select.parameter);
    case maniGraphNodePose:
        return maniGraphJsonRefer(r, &scope->parameters, f[1], &n->pose.parameter);
    case maniGraphNodeTimeScale:
        return maniGraphJsonScalar(r, scope, f[2], &n->timeScale.scale);
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceTriangles:
    case maniGraphNodeBlendSpaceBands:
        return maniGraphJsonSpace(r, scope, f, n);
    case maniGraphNodeMachine:
        return maniGraphJsonMachine(r, scope, f + 2, n);
    case maniGraphNodeOneShot:
        return Shot(r, scope, f, &n->oneShot);
    case maniGraphNodeInertialize:
        return Inertialize(r, scope, f, &n->inertialize);
    case maniGraphNodeMatch:
        return maniGraphJsonMatch(r, scope, f, &n->match);
    case maniGraphNodeLearned:
        return maniGraphJsonLearned(r, scope, f, &n->learned);
    case maniGraphNodeSpring:
        return Spring(r, scope, f, &n->spring);
    case maniGraphNodeSeek:
        n->seek.time = (maniGraphScalar){MANI_NO_PARAMETER, 0.0f};
        return maniGraphJsonRefer(r, &scope->parameters, f[2], &n->seek.trigger) &&
               (f[3] == ABSENT || maniGraphJsonScalar(r, scope, f[3], &n->seek.time));
    default:
        return Subgraph(r, scope, node, f, &n->subgraph);
    }
}

static bool Node(Reader* r, const Scope* scope, uint32_t object, maniGraphNodeSource* n)
{
    uint32_t kind = maniJsonFind(r->json, object, "kind", 4);
    if (r->json->values[object].kind != maniJsonObject || kind == ABSENT)
    {
        return maniGraphJsonFault(r, object,
                                  kind == ABSENT && r->json->values[object].kind == maniJsonObject
                                      ? maniGraphFormMissing
                                      : maniGraphFormValue);
    }
    uint32_t form = 0;
    while (form < FORMS && maniGraphJsonWord(r, kind, &kForms[form].name, 1) != 0)
    {
        ++form;
    }
    if (form == FORMS)
    {
        return maniGraphJsonFault(r, kind, maniGraphFormValue);
    }
    const KindForm* k = &kForms[form];
    uint32_t f[14];
    if (!maniGraphJsonMembers(r, object, k->keys, k->count, k->required, f))
    {
        return false;
    }
    *n = (maniGraphNodeSource){0};
    n->kind = k->kind;
    bool takesInputs = k->kind != maniGraphNodeClip && k->kind != maniGraphNodeMatch &&
                       k->kind != maniGraphNodeLearned && k->kind != maniGraphNodePose &&
                       k->kind != maniGraphNodeSubgraph;
    return (!takesInputs || Inputs(r, f[1], n)) && Fields(r, scope, object, f, n);
}

static bool Nodes(Reader* r, uint32_t value, maniGraphSource* out, const Scope* scope)
{
    uint32_t count = 0;
    if (!maniGraphJsonArray(r, value, &count))
    {
        return false;
    }
    maniGraphNodeSource* nodes = maniArenaTake(&r->arena, count, sizeof(maniGraphNodeSource));
    if (nodes == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t i = 0; i < count; ++i)
    {
        e = maniGraphJsonNext(r->json, e, i == 0);
        if (!Node(r, scope, e, &nodes[i]))
        {
            return false;
        }
    }
    out->nodes = nodes;
    out->nodeCount = count;
    return true;
}

static bool Header(Reader* r, const uint32_t* f)
{
    int64_t version = 0;
    bool format = r->json->values[f[0]].kind == maniJsonString &&
                  maniJsonStringEquals(r->json, f[0], "maul-anim graph", 15);
    if (!format)
    {
        return maniGraphJsonFault(r, f[0], maniGraphFormVersion);
    }
    return (maniJsonInteger(r->json, f[1], 1, 1, &version)) ||
           maniGraphJsonFault(r, f[1], maniGraphFormVersion);
}

static bool ReadGraph(Reader* r, uint32_t object, bool top, maniGraphSource* out, Scope* scope)
{
    static const char* const kKeys[] = {"format", "version", "parameters", "clips",  "syncGroups",
                                        "nodes",  "output",  "databases",  "models", "springSets"};
    // A nested graph takes the keys after format and version.
    uint32_t skip = top ? 0u : 2u;
    uint32_t f[10];
    if (!maniGraphJsonMembers(r, object, kKeys + skip, 10 - skip, 0x60u >> skip | (top ? 3u : 0u),
                              f + skip) ||
        (top && !Header(r, f)))
    {
        return false;
    }
    *out = (maniGraphSource){.skeleton = r->skeleton};
    return Parameters(r, f[2], out, scope) && Clips(r, f[3], out, scope) &&
           SyncGroups(r, f[4], out, scope) && Databases(r, f[7], out, scope) &&
           Models(r, f[8], out, scope) && SpringSets(r, f[9], out, scope) &&
           Nodes(r, f[5], out, scope) && maniGraphJsonIndex(r, f[6], &out->output);
}

maniBuildResult maniBuildGraphJson(maniBuilder* builder, const char* text, size_t length,
                                   const maniGraphNamedAssets* assets)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    const maniGraphNamedAssets* a = assets;
    if ((text == NULL && length > 0) || a == NULL || a->skeleton == NULL ||
        (a->clips == NULL && a->clipCount > 0) || (a->databases == NULL && a->databaseCount > 0) ||
        (a->models == NULL && a->modelCount > 0) ||
        (a->springSets == NULL && a->springSetCount > 0))
    {
        maniBuilderMisuse(builder);
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    maniJson json;
    maniJsonResult read = maniReadJson(&builder->allocator, (const uint8_t*)text, length, &json);
    if (read.result != mani_success)
    {
        maniReleaseJson(&builder->allocator, &json);
        return read.result == mani_errorCapacity
                   ? (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0}
                   : (maniBuildResult){mani_errorInvalid, maniBuildReasonJson,
                                       (uint32_t)read.offset, read.reason};
    }
    Reader r = {.json = &json,
                .arena = maniArenaMake(&builder->allocator),
                .skeleton = a->skeleton,
                .hostClips = a->clips,
                .hostDatabases = a->databases,
                .hostModels = a->models,
                .hostSpringSets = a->springSets};
    maniGraphSource source;
    Scope scope;
    maniBuildResult result =
        maniGraphJsonHostNames(&r, a->clips, sizeof *a->clips, a->clipCount, &r.host) &&
                maniGraphJsonHostNames(&r, a->databases, sizeof *a->databases, a->databaseCount,
                                       &r.hostDatabaseNames) &&
                maniGraphJsonHostNames(&r, a->models, sizeof *a->models, a->modelCount,
                                       &r.hostModelNames) &&
                maniGraphJsonHostNames(&r, a->springSets, sizeof *a->springSets, a->springSetCount,
                                       &r.hostSpringSetNames) &&
                ReadGraph(&r, 0, true, &source, &scope)
            ? maniBuildGraph(builder, &source)
            : r.fault;
    maniArenaRelease(&r.arena);
    maniReleaseJson(&builder->allocator, &json);
    return result;
}
