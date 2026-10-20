// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX animation stacks (mani-0016), as ufbx binds and evaluates them
// (mani-0016). A stack's layers are its AnimationLayer children in
// connection order; a layer's curve nodes are its AnimationCurveNode
// children, each bound through its property connections to the
// properties it drives, its curves connected to its channel properties
// (X or d|X component 0, Y 1, Z 2, any other name 0). Layer 0 sets a
// property; each later layer combines with the value so far.

#include "fbx_anim.h"

#include "allocator.h"
#include "fbx_curve.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_transform.h"
#include "fbx_value.h"
#include "quat64.h"
#include "scalar.h"
#include "sort.h"

#include <math.h>
#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

// Before 6.0, the stack of the models' own channels (mani-0017), its
// source the root node.
static const char kInternal[] = "(internal)";

// Whether a stack's source is the root standing for the internal stack
// (from 7.0 a source is an object, and object 0 is a stack like any).
static bool IsInternal(const maniFbx* fbx, uint32_t source)
{
    return fbx->version < 6000 && source == 0;
}

// Whether a file before 6.0 has a top-level model with channels.
static bool HasInternal(const maniFbx* fbx)
{
    for (uint32_t m = fbx->version < 6000 ? fbx->nodes[0].firstChild : MANI_FBX_NONE;
         m != MANI_FBX_NONE; m = fbx->nodes[m].next)
    {
        if (maniFbxNameIs(fbx, m, "Model", 5) &&
            maniFbxFindChild(fbx, m, "Channel", 7) != MANI_FBX_NONE)
        {
            return true;
        }
    }
    return false;
}

// A stack's name, decoded: an AnimationStack's (7.x) or a Take's (6.x);
// counted with out NULL.
static size_t StackName(const maniFbx* fbx, uint32_t source, uint8_t* out, size_t capacity)
{
    return fbx->version >= 7000
               ? maniFbxObjectName(fbx, source, out, capacity)
               : maniFbxStringDecode(fbx, maniFbxNodeProperty(fbx, source, 0), out, capacity);
}

// Each stack's source: counted (out NULL) or listed.
static uint32_t EachStack(const maniFbx* fbx, uint32_t* out)
{
    uint32_t n = 0;
    uint32_t takes = fbx->version < 7000 ? maniFbxFindChild(fbx, 0, "Takes", 5) : MANI_FBX_NONE;
    uint32_t take = takes != MANI_FBX_NONE ? fbx->nodes[takes].firstChild : MANI_FBX_NONE;
    uint32_t count = fbx->version >= 7000 ? fbx->objectCount : 0;
    for (uint32_t o = 0; o < count; ++o)
    {
        if (maniFbxObjectKindIs(fbx, o, "AnimationStack", 14))
        {
            if (out != NULL)
            {
                out[n] = o;
            }
            ++n;
        }
    }
    for (; take != MANI_FBX_NONE; take = fbx->nodes[take].next)
    {
        if (maniFbxNameIs(fbx, take, "Take", 4))
        {
            if (out != NULL)
            {
                out[n] = take;
            }
            ++n;
        }
    }
    if (HasInternal(fbx))
    {
        if (out != NULL)
        {
            out[n] = 0;
        }
        ++n;
    }
    return n;
}

maniImportResult maniFbxReadStacks(maniFbx* fbx)
{
    uint32_t n = EachStack(fbx, NULL);
    fbx->stacks = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint32_t), 4);
    fbx->stackNameOffsets = maniAllocate(&fbx->allocator, (size_t)n + 2, sizeof(size_t), 8);
    // Counted now, so a release after a later failure frees what was taken.
    fbx->stackCount = n;
    if (fbx->stacks == NULL || fbx->stackNameOffsets == NULL)
    {
        return Capacity();
    }
    (void)EachStack(fbx, fbx->stacks);
    // The names' raw bytes, at least their decoded ones.
    size_t text = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        uint32_t source = fbx->stacks[k];
        text += IsInternal(fbx, source) ? sizeof kInternal - 1 : StackName(fbx, source, NULL, 0);
    }
    fbx->stackNamesSize = text;
    fbx->stackNames = maniAllocate(&fbx->allocator, text + 1, 1, 1);
    if (fbx->stackNames == NULL)
    {
        return Capacity();
    }
    size_t at = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        uint32_t source = fbx->stacks[k];
        fbx->stackNameOffsets[k] = at;
        if (IsInternal(fbx, source))
        {
            memcpy(fbx->stackNames + at, kInternal, sizeof kInternal - 1);
            at += sizeof kInternal - 1;
            continue;
        }
        at += StackName(fbx, source, (uint8_t*)fbx->stackNames + at, text - at);
    }
    fbx->stackNameOffsets[n] = at;
    return maniFbxSuccess();
}

void maniFbxReleaseStacks(maniFbx* fbx)
{
    maniRelease(&fbx->allocator, fbx->stacks, (size_t)fbx->stackCount + 1, sizeof(uint32_t), 4);
    maniRelease(&fbx->allocator, fbx->stackNameOffsets, (size_t)fbx->stackCount + 2, sizeof(size_t),
                8);
    maniRelease(&fbx->allocator, fbx->stackNames, fbx->stackNamesSize + 1, 1, 1);
    fbx->stacks = NULL;
    fbx->stackNameOffsets = NULL;
    fbx->stackNames = NULL;
    fbx->stackCount = 0;
}

uint32_t maniFbxStackCount(const maniFbx* fbx)
{
    return fbx->stackCount;
}

uint32_t maniFbxStackSource(const maniFbx* fbx, uint32_t k)
{
    return k < fbx->stackCount ? fbx->stacks[k] : MANI_FBX_NONE;
}

// A number property of an object, its fallback when absent; false for
// one present but not a finite number.
static bool Number(const maniFbx* fbx, uint32_t object, const char* name, size_t length,
                   double* out)
{
    uint32_t entry = maniFbxFindProperty(fbx, object, name, length);
    return entry == MANI_FBX_NONE || maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, 0), out);
}

// The children of a kind under an object, in connection order: counted
// (objects NULL) or listed.
typedef struct Children
{
    const char* kind;
    size_t length;
    uint32_t* objects;
    uint32_t count;
} Children;

static void VisitChild(const maniFbx* fbx, uint32_t child, void* context)
{
    Children* c = context;
    if (child < fbx->objectCount && maniFbxObjectKindIs(fbx, child, c->kind, c->length))
    {
        if (c->objects != NULL)
        {
            c->objects[c->count] = child;
        }
        ++c->count;
    }
}

static maniFbxLayer ReadLayer(const maniFbx* fbx, uint32_t object)
{
    double weight = 100.0;
    double blend = 0.0;
    double rotation = 0.0;
    double scale = 0.0;
    // Values not numbers keep their defaults, as ufbx's lookups do.
    (void)Number(fbx, object, "Weight", 6, &weight);
    (void)Number(fbx, object, "BlendMode", 9, &blend);
    (void)Number(fbx, object, "RotationAccumulationMode", 24, &rotation);
    (void)Number(fbx, object, "ScaleAccumulationMode", 21, &scale);
    weight /= 100.0;
    weight = weight < 0.0 ? 0.0 : (weight > 0.99999 ? 1.0 : weight);
    uint32_t mode = blend == 0.0   ? maniFbxBlendAdditive
                    : blend == 2.0 ? maniFbxBlendPassthrough
                                   : maniFbxBlendOverride;
    return (maniFbxLayer){object, mode, weight, rotation == 0.0, scale == 0.0};
}

static bool NameIs(const maniFbx* fbx, uint32_t property, const char* name)
{
    return maniFbxStringEquals(fbx, property, name, strlen(name));
}

// The component a channel property names.
static uint32_t Component(const maniFbx* fbx, uint32_t property)
{
    if (NameIs(fbx, property, "Y") || NameIs(fbx, property, "d|Y"))
    {
        return 1;
    }
    return NameIs(fbx, property, "Z") || NameIs(fbx, property, "d|Z") ? 2 : 0;
}

// A curve node's static channels: X then d|X (and Y, Z); refused
// (Property) for one not a finite number.
static maniImportResult NodeValues(const maniFbx* fbx, uint32_t node, double values[3])
{
    static const char* const kNames[6] = {"X", "d|X", "Y", "d|Y", "Z", "d|Z"};
    for (uint32_t k = 0; k < 6; ++k)
    {
        if (!Number(fbx, node, kNames[k], strlen(kNames[k]), &values[k / 2]))
        {
            return maniFbxFail(maniImportReasonProperty,
                               fbx->nodes[fbx->objects[node].node].nameOffset, node);
        }
    }
    return maniFbxSuccess();
}

// A channel's curves: each AnimationCurve connected to a property of the
// node, the property's own value its component's static value; a later
// curve on a component replaces an earlier.
static maniImportResult NodeCurves(const maniFbx* fbx, uint32_t node, maniFbxChannel* channel)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxChildConnections(fbx, node, &list);
    maniImportResult r = maniFbxSuccess();
    for (uint32_t i = 0; r.result == mani_success && i < n; ++i)
    {
        const maniFbxConnection* c = &fbx->connections[list[i]];
        if (c->childProperty != MANI_FBX_NONE || c->parentProperty == MANI_FBX_NONE ||
            !maniFbxObjectKindIs(fbx, c->child, "AnimationCurve", 14))
        {
            continue;
        }
        uint32_t k = Component(fbx, c->parentProperty);
        // The channel's own value, by its decoded name (a longer name than
        // any channel's keeps the node's value).
        uint8_t name[128];
        size_t length = maniFbxStringDecode(fbx, c->parentProperty, name, sizeof name);
        if (length < sizeof name &&
            !Number(fbx, node, (const char*)name, length, &channel->values[k]))
        {
            return maniFbxFail(maniImportReasonProperty,
                               fbx->nodes[fbx->objects[node].node].nameOffset, node);
        }
        maniFbxReleaseCurve(fbx, &channel->curves[k]);
        r = maniFbxReadCurve(fbx, c->child, &channel->curves[k]);
    }
    return r;
}

// The property connections of a curve node to what it drives: counted
// (out NULL) or bound as channels of a layer, never past capacity. A
// node under several layers binds once in each, so the count can pass
// 32 bits: it is kept in 64.
static maniImportResult Bind(const maniFbx* fbx, uint32_t node, uint32_t layer, maniFbxChannel* out,
                             uint64_t capacity, uint64_t* count)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxParentConnections(fbx, node, &list);
    maniImportResult r = maniFbxSuccess();
    for (uint32_t i = 0; r.result == mani_success && i < n; ++i)
    {
        const maniFbxConnection* c = &fbx->connections[list[i]];
        if (c->childProperty != MANI_FBX_NONE || c->parentProperty == MANI_FBX_NONE ||
            c->parent >= fbx->objectCount)
        {
            continue;
        }
        if (out != NULL && *count < capacity)
        {
            maniFbxChannel* channel = &out[*count];
            memset(channel, 0, sizeof *channel);
            channel->nameHash = maniFbxStringHash(fbx, c->parentProperty);
            channel->target = c->parent;
            channel->name = c->parentProperty;
            channel->layer = layer;
            channel->item = node;
            channel->source = fbx->objects[node].node;
            r = NodeValues(fbx, node, channel->values);
            r = r.result == mani_success ? NodeCurves(fbx, node, channel) : r;
        }
        ++*count;
    }
    return r;
}

static void ReleaseChannel(const maniFbx* fbx, maniFbxChannel* channel)
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        maniFbxReleaseCurve(fbx, &channel->curves[k]);
    }
}

void maniFbxReleaseAnimation(const maniFbx* fbx, maniFbxAnimation* animation)
{
    for (uint32_t i = 0; animation->channels != NULL && i < animation->channelCount; ++i)
    {
        ReleaseChannel(fbx, &animation->channels[i]);
    }
    maniRelease(&fbx->allocator, animation->channels, (size_t)animation->channelCount + 1,
                sizeof(maniFbxChannel), 8);
    maniRelease(&fbx->allocator, animation->layers, (size_t)animation->layerCount + 1,
                sizeof(maniFbxLayer), 8);
    if (animation->names != NULL)
    {
        maniRelease(&fbx->allocator, animation->names, animation->namesSize + 1, 1, 1);
    }
    memset(animation, 0, sizeof *animation);
}

// Every layer's curve nodes bound: counted (out NULL) or read.
static maniImportResult EachNode(const maniFbx* fbx, const maniFbxAnimation* a, maniFbxChannel* out,
                                 uint64_t capacity, uint64_t* count)
{
    maniImportResult r = maniFbxSuccess();
    for (uint32_t l = 0; r.result == mani_success && l < a->layerCount; ++l)
    {
        Children nodes = {"AnimationCurveNode", 18, NULL, 0};
        maniFbxEachChild(fbx, a->layers[l].object, VisitChild, &nodes);
        uint32_t all = nodes.count;
        nodes.objects = maniAllocate(&fbx->allocator, (size_t)all + 1, sizeof(uint32_t), 4);
        if (nodes.objects == NULL)
        {
            return Capacity();
        }
        nodes.count = 0;
        maniFbxEachChild(fbx, a->layers[l].object, VisitChild, &nodes);
        for (uint32_t i = 0; r.result == mani_success && i < nodes.count; ++i)
        {
            r = Bind(fbx, nodes.objects[i], l, out, capacity, count);
        }
        maniRelease(&fbx->allocator, nodes.objects, (size_t)all + 1, sizeof(uint32_t), 4);
    }
    return r;
}

static maniImportResult ReadLayers(const maniFbx* fbx, uint32_t stack, maniFbxAnimation* a)
{
    Children layers = {"AnimationLayer", 14, NULL, 0};
    maniFbxEachChild(fbx, stack, VisitChild, &layers);
    a->layers = maniAllocate(&fbx->allocator, (size_t)layers.count + 1, sizeof(maniFbxLayer), 8);
    uint32_t* objects =
        maniAllocate(&fbx->allocator, (size_t)layers.count + 1, sizeof(uint32_t), 4);
    uint32_t all = layers.count;
    // Counted with the array, so a release after a failure frees it whole.
    a->layerCount = a->layers != NULL ? all : 0;
    if (a->layers != NULL && objects != NULL)
    {
        layers.objects = objects;
        layers.count = 0;
        maniFbxEachChild(fbx, stack, VisitChild, &layers);
        for (uint32_t l = 0; l < all; ++l)
        {
            a->layers[l] = ReadLayer(fbx, objects[l]);
        }
    }
    maniRelease(&fbx->allocator, objects, (size_t)all + 1, sizeof(uint32_t), 4);
    return a->layers != NULL && objects != NULL ? maniFbxSuccess() : Capacity();
}

// The channels in the order given.
static maniImportResult Reorder(const maniFbx* fbx, maniFbxAnimation* a, const uint32_t* order)
{
    uint32_t n = a->channelCount;
    maniFbxChannel* sorted =
        maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(maniFbxChannel), 8);
    if (sorted == NULL)
    {
        return Capacity();
    }
    for (uint32_t i = 0; i < n; ++i)
    {
        sorted[i] = a->channels[order[i]];
    }
    memcpy(a->channels, sorted, (size_t)n * sizeof *sorted);
    maniRelease(&fbx->allocator, sorted, (size_t)n + 1, sizeof(maniFbxChannel), 8);
    return maniFbxSuccess();
}

// Sorted by target, layer and name hash: by the hash first, then stably
// by target and layer.
static maniImportResult SortChannels(const maniFbx* fbx, maniFbxAnimation* a)
{
    uint32_t n = a->channelCount;
    uint64_t* keys = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint64_t), 8);
    uint32_t* byHash = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint32_t), 4);
    uint32_t* order = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint32_t), 4);
    uint32_t* scratch = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint32_t), 4);
    maniImportResult r = keys != NULL && byHash != NULL && order != NULL && scratch != NULL
                             ? maniFbxSuccess()
                             : Capacity();
    if (r.result == mani_success)
    {
        for (uint32_t i = 0; i < n; ++i)
        {
            keys[i] = a->channels[i].nameHash;
        }
        maniSortByKey(keys, n, byHash, scratch);
        for (uint32_t i = 0; i < n; ++i)
        {
            const maniFbxChannel* c = &a->channels[byHash[i]];
            keys[i] = ((uint64_t)c->target << 32) + c->layer;
        }
        maniSortByKey(keys, n, order, scratch);
        for (uint32_t i = 0; i < n; ++i)
        {
            scratch[i] = byHash[order[i]];
        }
        r = Reorder(fbx, a, scratch);
    }
    maniRelease(&fbx->allocator, keys, (size_t)n + 1, sizeof(uint64_t), 8);
    maniRelease(&fbx->allocator, byHash, (size_t)n + 1, sizeof(uint32_t), 4);
    maniRelease(&fbx->allocator, order, (size_t)n + 1, sizeof(uint32_t), 4);
    maniRelease(&fbx->allocator, scratch, (size_t)n + 1, sizeof(uint32_t), 4);
    return r;
}

static bool SameSlot(const maniFbxChannel* a, const maniFbxChannel* b)
{
    return a->target == b->target && a->layer == b->layer && a->nameHash == b->nameHash;
}

// Sorted, then a property driven twice in one layer refused: equal names
// share a hash, so they lie together.
maniImportResult maniFbxSortChannels(const maniFbx* fbx, maniFbxAnimation* a)
{
    maniImportResult r = SortChannels(fbx, a);
    for (uint32_t i = 0; r.result == mani_success && i < a->channelCount; ++i)
    {
        for (uint32_t j = i + 1; j < a->channelCount && SameSlot(&a->channels[i], &a->channels[j]);
             ++j)
        {
            if (maniFbxSameChannelName(fbx, &a->channels[i], &a->channels[j]))
            {
                const maniFbxChannel* c = &a->channels[j];
                return maniFbxFail(maniImportReasonProperty, fbx->nodes[c->source].nameOffset,
                                   c->item);
            }
        }
    }
    return r;
}

// Two integer properties of an object, both present.
static bool Pair(const maniFbx* fbx, uint32_t object, const char* first, const char* second,
                 int64_t* a, int64_t* b)
{
    uint32_t x = maniFbxFindProperty(fbx, object, first, strlen(first));
    uint32_t y = maniFbxFindProperty(fbx, object, second, strlen(second));
    return x != MANI_FBX_NONE && y != MANI_FBX_NONE &&
           maniFbxInteger(fbx, maniFbxPropertyValue(fbx, x, 0), INT64_MIN, INT64_MAX, a) &&
           maniFbxInteger(fbx, maniFbxPropertyValue(fbx, y, 0), INT64_MIN, INT64_MAX, b);
}

bool maniFbxTakeTimes(const maniFbx* fbx, uint32_t take, const char* name, int64_t* a, int64_t* b)
{
    uint32_t node = maniFbxFindChild(fbx, take, name, strlen(name));
    return node != MANI_FBX_NONE &&
           maniFbxInteger(fbx, maniFbxNodeProperty(fbx, node, 0), INT64_MIN, INT64_MAX, a) &&
           maniFbxInteger(fbx, maniFbxNodeProperty(fbx, node, 1), INT64_MIN, INT64_MAX, b);
}

// The take of the stack's name in the top-level Takes, or MANI_FBX_NONE.
static uint32_t FindTake(const maniFbx* fbx, uint32_t stack)
{
    uint8_t name[256];
    size_t length = maniFbxObjectName(fbx, stack, name, sizeof name);
    uint32_t takes = maniFbxFindChild(fbx, 0, "Takes", 5);
    uint32_t take = takes != MANI_FBX_NONE ? fbx->nodes[takes].firstChild : MANI_FBX_NONE;
    for (; take != MANI_FBX_NONE; take = fbx->nodes[take].next)
    {
        if (maniFbxNameIs(fbx, take, "Take", 4) &&
            maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, take, 0), (const char*)name, length))
        {
            return take;
        }
    }
    return MANI_FBX_NONE;
}

void maniFbxKeySpan(const maniFbxAnimation* a, int64_t* start, int64_t* stop)
{
    bool any = false;
    *start = 0;
    *stop = 0;
    for (uint32_t i = 0; i < a->channelCount; ++i)
    {
        for (uint32_t k = 0; k < 3; ++k)
        {
            const maniFbxCurve* c = &a->channels[i].curves[k];
            if (c->count == 0)
            {
                continue;
            }
            *start = !any || c->firstTick < *start ? c->firstTick : *start;
            *stop = !any || c->lastTick > *stop ? c->lastTick : *stop;
            any = true;
        }
    }
}

// The stack's times, as ufbx finds them: its LocalStart and LocalStop,
// else ReferenceStart and ReferenceStop, each its own or its template's;
// a stack with no properties of its own takes its take's LocalTime and
// ReferenceTime as its own, ahead of the template. Else its key span.
static void Range(const maniFbx* fbx, uint32_t stack, maniFbxAnimation* a)
{
    uint32_t list = fbx->objects[stack].properties;
    bool bare = list == MANI_FBX_NONE || fbx->nodes[list].firstChild == MANI_FBX_NONE;
    uint32_t take = bare ? FindTake(fbx, stack) : MANI_FBX_NONE;
    bool timed =
        take != MANI_FBX_NONE && maniFbxTakeTimes(fbx, take, "LocalTime", &a->start, &a->stop);
    timed = timed || Pair(fbx, stack, "LocalStart", "LocalStop", &a->start, &a->stop);
    timed = timed || (take != MANI_FBX_NONE &&
                      maniFbxTakeTimes(fbx, take, "ReferenceTime", &a->start, &a->stop));
    timed = timed || Pair(fbx, stack, "ReferenceStart", "ReferenceStop", &a->start, &a->stop);
    if (!timed)
    {
        maniFbxKeySpan(a, &a->start, &a->stop);
    }
}

maniImportResult maniFbxReadAnimation(const maniFbx* fbx, uint32_t stack, maniFbxAnimation* out)
{
    memset(out, 0, sizeof *out);
    maniImportResult r = ReadLayers(fbx, stack, out);
    uint64_t count = 0;
    r = r.result == mani_success ? EachNode(fbx, out, NULL, 0, &count) : r;
    // Indices and the sort's keys stay within 32 bits.
    r = r.result == mani_success && count >= UINT32_MAX ? Capacity() : r;
    if (r.result == mani_success)
    {
        out->channels = maniAllocate(&fbx->allocator, (size_t)count + 1, sizeof(maniFbxChannel), 8);
        r = out->channels != NULL ? r : Capacity();
    }
    if (r.result == mani_success)
    {
        // Zeroed, so a refusal partway releases only what was read.
        memset(out->channels, 0, ((size_t)count + 1) * sizeof(maniFbxChannel));
        out->channelCount = (uint32_t)count;
        uint64_t filled = 0;
        r = EachNode(fbx, out, out->channels, count, &filled);
    }
    r = r.result == mani_success ? maniFbxSortChannels(fbx, out) : r;
    if (r.result != mani_success)
    {
        maniFbxReleaseAnimation(fbx, out);
        return r;
    }
    Range(fbx, stack, out);
    return r;
}

uint32_t maniFbxAnimatedRun(const maniFbxAnimation* animation, uint32_t object, uint32_t* first)
{
    uint32_t low = 0;
    uint32_t high = animation->channelCount;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (animation->channels[mid].target < object)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    uint32_t end = low;
    while (end < animation->channelCount && animation->channels[end].target == object)
    {
        ++end;
    }
    *first = low;
    return end - low;
}

static void Evaluate(const maniFbxChannel* c, double time, double out[3])
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        out[k] = maniFbxEvaluateCurve(&c->curves[k], time, c->values[k]);
    }
}

// The channel of a run on a property in a layer, or NULL.
static const maniFbxChannel* Find(const maniFbxAnimation* a, const maniFbx* fbx, uint32_t first,
                                  uint32_t count, uint32_t layer, const char* name, size_t length)
{
    for (uint32_t i = first; i < first + count; ++i)
    {
        const maniFbxChannel* c = &a->channels[i];
        if (c->layer == layer && maniFbxChannelNameIs(fbx, c, name, length))
        {
            return c;
        }
    }
    return NULL;
}

// A layer's weight at a time: its own Weight's channel in that layer for
// a blending layer, as ufbx reads it.
static double Weight(const maniFbxAnimation* a, const maniFbx* fbx, uint32_t layer, double time)
{
    const maniFbxLayer* l = &a->layers[layer];
    uint32_t first = 0;
    uint32_t count = maniFbxAnimatedRun(a, l->object, &first);
    const maniFbxChannel* c =
        l->blend != maniFbxBlendOverride ? Find(a, fbx, first, count, layer, "Weight", 6) : NULL;
    if (c == NULL)
    {
        return l->weight;
    }
    double v[3];
    Evaluate(c, time, v);
    double w = v[0] / 100.0;
    return w < 0.0 ? 0.0 : (w > 0.99999 ? 1.0 : w);
}

// |v|^e with v's sign, 1 for e <= 0 and v for e >= 1, as ufbx scales.
static double PowAbs(double v, double e)
{
    if (e <= 0.0)
    {
        return 1.0;
    }
    if (e >= 1.0)
    {
        return v;
    }
    double magnitude = fabs(v);
    double p = magnitude > 0.0 ? maniPowDouble(magnitude, e) : 0.0;
    return v < 0.0 ? -p : p;
}

// Rotations composed: Euler angles to quaternions, slerped (from the
// identity for an additive layer, multiplied on) and back.
static void ComposeRotation(bool additive, double weight, uint32_t order, double result[3],
                            const double value[3])
{
    double a[4];
    double b[4];
    double out[4];
    maniFbxEulerQuat(result, order, a);
    maniFbxEulerQuat(value, order, b);
    if (additive)
    {
        static const double kIdentity[4] = {0.0, 0.0, 0.0, 1.0};
        maniSlerpDouble(kIdentity, b, weight, b);
        maniFbxQuatMul(a, b, out);
    }
    else
    {
        maniSlerpDouble(a, b, weight, out);
    }
    maniFbxQuatEuler(out, order, result);
}

static void Combine(const maniFbxLayer* l, double weight, bool scale, bool rotation, uint32_t order,
                    double result[3], const double value[3])
{
    bool additive = l->blend == maniFbxBlendAdditive;
    if (l->blend == maniFbxBlendOverride)
    {
        memcpy(result, value, 3 * sizeof *result);
    }
    else if (l->composeRotation && rotation)
    {
        ComposeRotation(additive, weight, order, result, value);
    }
    else
    {
        for (uint32_t k = 0; k < 3; ++k)
        {
            bool geometric = l->composeScale && scale;
            if (geometric)
            {
                result[k] = additive ? result[k] * PowAbs(value[k], weight)
                                     : PowAbs(result[k], 1.0 - weight) * PowAbs(value[k], weight);
            }
            else
            {
                result[k] = additive ? result[k] + value[k] * weight
                                     : result[k] * (1.0 - weight) + value[k] * weight;
            }
        }
    }
}

bool maniFbxChannelNameIs(const maniFbx* fbx, const maniFbxChannel* channel, const char* name,
                          size_t length)
{
    if (channel->literal != NULL)
    {
        return channel->literalLength == length && memcmp(channel->literal, name, length) == 0;
    }
    return maniFbxStringEquals(fbx, channel->name, name, length);
}

bool maniFbxSameChannelName(const maniFbx* fbx, const maniFbxChannel* a, const maniFbxChannel* b)
{
    if (a->literal != NULL)
    {
        return maniFbxChannelNameIs(fbx, b, a->literal, a->literalLength);
    }
    if (b->literal != NULL)
    {
        return maniFbxChannelNameIs(fbx, a, b->literal, b->literalLength);
    }
    return maniFbxSameString(fbx, a->name, b->name);
}

size_t maniFbxChannelName(const maniFbx* fbx, const maniFbxChannel* channel, uint8_t* out,
                          size_t capacity)
{
    if (channel->literal != NULL)
    {
        size_t n = channel->literalLength < capacity ? channel->literalLength : capacity;
        if (out != NULL)
        {
            memcpy(out, channel->literal, n);
        }
        return out != NULL ? n : channel->literalLength;
    }
    return maniFbxStringDecode(fbx, channel->name, out, capacity);
}

// A channel's value at a time applied: the base layer's taken, a later
// layer's combined.
static void Apply(const maniFbxAnimation* animation, const maniFbx* fbx, const maniFbxChannel* c,
                  double time, bool scale, bool rotation, uint32_t order, double value[3])
{
    double v[3];
    Evaluate(c, time, v);
    if (c->layer == 0)
    {
        memcpy(value, v, sizeof v);
    }
    else
    {
        Combine(&animation->layers[c->layer], Weight(animation, fbx, c->layer, time), scale,
                rotation, order, value, v);
    }
}

// A property's value through a run of channels of one side: the model's
// own or (6.x) its attribute's.
static bool AnimateRun(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t first,
                       uint32_t count, const char* name, size_t length, uint64_t hash, double time,
                       uint32_t order, bool attribute, double value[3])
{
    bool scale = length == 11 && memcmp(name, "Lcl Scaling", 11) == 0;
    bool rotation = length == 12 && memcmp(name, "Lcl Rotation", 12) == 0;
    bool animated = false;
    // The run is in layer order: one pass applies each layer's channel.
    for (uint32_t i = first; i < first + count; ++i)
    {
        const maniFbxChannel* c = &animation->channels[i];
        if (c->nameHash == hash && c->attribute == attribute &&
            maniFbxChannelNameIs(fbx, c, name, length))
        {
            Apply(animation, fbx, c, time, scale, rotation, order, value);
            animated = true;
        }
    }
    return animated;
}

static bool Animate(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t object,
                    const char* name, size_t length, double time, uint32_t order, bool attribute,
                    double value[3])
{
    uint32_t first = 0;
    uint32_t count = maniFbxAnimatedRun(animation, object, &first);
    return AnimateRun(animation, fbx, first, count, name, length, maniFbxNameHash(name, length),
                      time, order, attribute, value);
}

bool maniFbxAnimateMarked(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t first,
                          uint32_t count, const uint8_t* marks, uint8_t mark, bool scale,
                          bool rotation, double time, uint32_t order, double value[3])
{
    bool animated = false;
    for (uint32_t i = first; i < first + count; ++i)
    {
        if (marks[i] == mark)
        {
            Apply(animation, fbx, &animation->channels[i], time, scale, rotation, order, value);
            animated = true;
        }
    }
    return animated;
}

bool maniFbxAnimateProperty(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t object,
                            const char* name, size_t length, double time, uint32_t order,
                            double value[3])
{
    return Animate(animation, fbx, object, name, length, time, order, false, value);
}

bool maniFbxAnimateAttribute(const maniFbxAnimation* animation, const maniFbx* fbx, uint32_t model,
                             const char* name, size_t length, double time, double value[3])
{
    return Animate(animation, fbx, model, name, length, time, 0, true, value);
}
