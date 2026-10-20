// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX 6.x takes (mani-0017), as ufbx reads them. A Take holds Model
// nodes naming their targets by 6.x key; each Channel under one is a
// property, `Transform`'s T, R and S being the Lcl ones. A channel with
// Key or Default is one component, else its first three such children
// are. ufbx splits a 6.x model's properties between it and its
// attribute, and a channel on a property the model does not keep drives
// the attribute's; such channels are marked so model lookups pass them.

#include "fbx_take.h"

#include "allocator.h"
#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_keys.h"
#include "fbx_object.h"
#include "fbx_value.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Refuse(const maniFbx* fbx, uint32_t node, uint32_t item)
{
    return maniFbxFail(maniImportReasonProperty, fbx->nodes[node].nameOffset, item);
}

// Channels counted (out NULL) or read, never past capacity; the count
// in 64 bits, as for 7.x.
typedef struct Reader
{
    const maniFbx* fbx;
    maniFbxChannel* out;
    uint64_t capacity;
    uint64_t count;
    // Before 6.0: the pool cut names are decoded into, its size, and what
    // is used of it (counted at most each name's raw length).
    uint8_t* names;
    size_t namesSize;
    size_t namesUsed;
} Reader;

// Whether a node holds a component: a Key or a Default.
static bool Holds(const maniFbx* fbx, uint32_t node)
{
    return maniFbxFindChild(fbx, node, "Key", 3) != MANI_FBX_NONE ||
           maniFbxFindChild(fbx, node, "Default", 7) != MANI_FBX_NONE;
}

static bool Named(const maniFbx* fbx, uint32_t node)
{
    const uint8_t* bytes = NULL;
    uint32_t length = 0;
    return maniFbxString(fbx, maniFbxNodeProperty(fbx, node, 0), &bytes, &length);
}

uint32_t maniFbxChannelComponents(const maniFbx* fbx, uint32_t node, uint32_t out[3])
{
    if (Holds(fbx, node))
    {
        out[0] = node;
        return 1;
    }
    uint32_t n = 0;
    for (uint32_t c = fbx->nodes[node].firstChild; c != MANI_FBX_NONE && n < 3;
         c = fbx->nodes[c].next)
    {
        if (maniFbxNameIs(fbx, c, "Channel", 7) && Holds(fbx, c) && Named(fbx, c))
        {
            out[n++] = c;
        }
    }
    return n;
}

double maniFbxChannelDefault(const maniFbx* fbx, uint32_t component)
{
    // A Default not a number counts as none.
    double value = 0.0;
    uint32_t node = maniFbxFindChild(fbx, component, "Default", 7);
    if (node != MANI_FBX_NONE)
    {
        (void)maniFbxNumber(fbx, maniFbxNodeProperty(fbx, node, 0), &value);
    }
    return value;
}

// Whether a channel drives the target's attribute: a 6.x model with one,
// on a property not among the node properties nor user-defined on it.
static bool OnAttribute(const maniFbx* fbx, const maniFbxChannel* channel)
{
    uint32_t model = channel->target;
    if (!maniFbxObjectKindIs(fbx, model, "Model", 5) || !maniFbxModelHasAttribute(fbx, model))
    {
        return false;
    }
    uint8_t name[128];
    size_t length = maniFbxChannelName(fbx, channel, name, sizeof name);
    if (length < sizeof name && maniFbxIsNodeProperty(name, length))
    {
        return false;
    }
    uint32_t entry = length < sizeof name
                         ? maniFbxFindProperty(fbx, model, (const char*)name, length)
                         : MANI_FBX_NONE;
    return entry == MANI_FBX_NONE || !maniFbxEntryUserDefined(fbx, entry);
}

// A channel's components: each Default its static value, each Key list
// its curve.
static maniImportResult ReadComponents(const maniFbx* fbx, const uint32_t* components, uint32_t n,
                                       maniFbxChannel* channel)
{
    maniImportResult r = maniFbxSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < n; ++k)
    {
        channel->values[k] = maniFbxChannelDefault(fbx, components[k]);
        r = maniFbxReadKeys(fbx, components[k], channel->target, &channel->curves[k]);
    }
    return r;
}

// Before 6.0, a channel name ending in " (Shape)" names the shape
// without it (mani-0017): decoded into the pool, its literal.
static void CutShape(Reader* r, maniFbxChannel* channel)
{
    static const char kSuffix[] = " (Shape)";
    const size_t suffix = sizeof kSuffix - 1;
    uint8_t* at = r->names + r->namesUsed;
    size_t n = maniFbxStringDecode(r->fbx, channel->name, at, r->namesSize - r->namesUsed);
    if (n > suffix && memcmp(at + n - suffix, kSuffix, suffix) == 0)
    {
        channel->literal = (const char*)at;
        channel->literalLength = (uint32_t)(n - suffix);
        r->namesUsed += n - suffix;
    }
}

// A property channel of a target, named by a string property or a
// literal.
static maniImportResult Property(Reader* r, uint32_t node, uint32_t target, uint32_t name,
                                 const char* literal)
{
    uint32_t components[3];
    uint32_t n = maniFbxChannelComponents(r->fbx, node, components);
    if (n == 0)
    {
        return maniFbxSuccess();
    }
    maniImportResult result = maniFbxSuccess();
    bool cut = r->fbx->version < 6000 && literal == NULL;
    if (cut && r->out == NULL)
    {
        r->namesUsed += maniFbxStringDecode(r->fbx, name, NULL, 0);
    }
    if (r->out != NULL && r->count < r->capacity)
    {
        maniFbxChannel* c = &r->out[r->count];
        memset(c, 0, sizeof *c);
        c->target = target;
        c->name = name;
        c->literal = literal;
        c->literalLength = literal != NULL ? (uint32_t)strlen(literal) : 0;
        if (cut)
        {
            CutShape(r, c);
        }
        c->nameHash = c->literal != NULL ? maniFbxNameHash(c->literal, c->literalLength)
                                         : maniFbxStringHash(r->fbx, name);
        c->item = target;
        c->source = node;
        c->attribute = OnAttribute(r->fbx, c);
        result = ReadComponents(r->fbx, components, n, c);
    }
    ++r->count;
    return result;
}

// Transform's T, R and S channels as the Lcl properties.
static maniImportResult Transform(Reader* r, uint32_t node, uint32_t target)
{
    static const char* const kShort[3] = {"T", "R", "S"};
    static const char* const kLong[3] = {"Lcl Translation", "Lcl Rotation", "Lcl Scaling"};
    const maniFbx* fbx = r->fbx;
    maniImportResult result = maniFbxSuccess();
    for (uint32_t c = fbx->nodes[node].firstChild;
         result.result == mani_success && c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        if (!maniFbxNameIs(fbx, c, "Channel", 7))
        {
            continue;
        }
        if (!Named(fbx, c))
        {
            return Refuse(fbx, c, target);
        }
        for (uint32_t k = 0; k < 3; ++k)
        {
            if (maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, c, 0), kShort[k], 1))
            {
                result = Property(r, c, target, MANI_FBX_NONE, kLong[k]);
            }
        }
    }
    return result;
}

// A take's Model node: its named channels.
static maniImportResult Model(Reader* r, uint32_t node)
{
    const maniFbx* fbx = r->fbx;
    if (!Named(fbx, node))
    {
        return Refuse(fbx, node, 0);
    }
    uint32_t target = maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, node, 0));
    if (target >= fbx->objectCount)
    {
        return maniFbxSuccess();
    }
    maniImportResult result = maniFbxSuccess();
    for (uint32_t c = fbx->nodes[node].firstChild;
         result.result == mani_success && c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        uint32_t name = maniFbxNodeProperty(fbx, c, 0);
        if (!maniFbxNameIs(fbx, c, "Channel", 7) || !Named(fbx, c))
        {
            continue;
        }
        result = maniFbxStringEquals(fbx, name, "Transform", 9)
                     ? Transform(r, c, target)
                     : Property(r, c, target, name, NULL);
    }
    return result;
}

static maniImportResult EachModel(Reader* r, uint32_t take)
{
    maniImportResult result = maniFbxSuccess();
    for (uint32_t c = r->fbx->nodes[take].firstChild;
         result.result == mani_success && c != MANI_FBX_NONE; c = r->fbx->nodes[c].next)
    {
        if (maniFbxNameIs(r->fbx, c, "Model", 5))
        {
            result = Model(r, c);
        }
    }
    return result;
}

// A take, or (source 0, mani-0017) the top-level models' own channels.
static maniImportResult ReadTake(const maniFbx* fbx, uint32_t take, maniFbxAnimation* out)
{
    memset(out, 0, sizeof *out);
    if (take != 0 && !Named(fbx, take))
    {
        return Refuse(fbx, take, 0);
    }
    out->layers = maniAllocate(&fbx->allocator, 2, sizeof(maniFbxLayer), 8);
    if (out->layers == NULL)
    {
        return Capacity();
    }
    out->layerCount = 1;
    out->layers[0] = (maniFbxLayer){MANI_FBX_NONE, maniFbxBlendAdditive, 1.0, true, true};
    Reader counter = {fbx, NULL, 0, 0, NULL, 0, 0};
    maniImportResult r = EachModel(&counter, take);
    r = r.result == mani_success && counter.count >= UINT32_MAX ? Capacity() : r;
    if (r.result == mani_success)
    {
        out->names = maniAllocate(&fbx->allocator, counter.namesUsed + 1, 1, 1);
        out->namesSize = counter.namesUsed;
        r = out->names != NULL ? r : Capacity();
    }
    if (r.result == mani_success)
    {
        size_t n = (size_t)counter.count + 1;
        out->channels = maniAllocate(&fbx->allocator, n, sizeof(maniFbxChannel), 8);
        r = out->channels != NULL ? r : Capacity();
    }
    if (r.result == mani_success)
    {
        // Zeroed, so a refusal partway releases only what was read.
        memset(out->channels, 0, ((size_t)counter.count + 1) * sizeof(maniFbxChannel));
        out->channelCount = (uint32_t)counter.count;
        Reader reader = {fbx, out->channels, counter.count, 0, out->names, out->namesSize, 0};
        r = EachModel(&reader, take);
    }
    r = r.result == mani_success ? maniFbxSortChannels(fbx, out) : r;
    if (r.result != mani_success)
    {
        maniFbxReleaseAnimation(fbx, out);
        return r;
    }
    if (take == 0 || (!maniFbxTakeTimes(fbx, take, "LocalTime", &out->start, &out->stop) &&
                      !maniFbxTakeTimes(fbx, take, "ReferenceTime", &out->start, &out->stop)))
    {
        maniFbxKeySpan(out, &out->start, &out->stop);
    }
    return r;
}

maniImportResult maniFbxReadStack(const maniFbx* fbx, uint32_t k, maniFbxAnimation* out)
{
    uint32_t source = maniFbxStackSource(fbx, k);
    if (source == MANI_FBX_NONE)
    {
        memset(out, 0, sizeof *out);
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    return fbx->version >= 7000 ? maniFbxReadAnimation(fbx, source, out)
                                : ReadTake(fbx, source, out);
}

maniImportResult maniFbxReadStackOf(const maniFbx* fbx, uint32_t k, const char* name, size_t length,
                                    maniFbxAnimation* out)
{
    uint32_t source = maniFbxStackSource(fbx, k);
    if (source == MANI_FBX_NONE || fbx->version < 7000)
    {
        return maniFbxReadStack(fbx, k, out);
    }
    return maniFbxReadAnimationOf(fbx, source, name, length, out);
}
