// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX objects, connections, templated properties and settings
// (mani-0013). Objects are the children of `Objects`, found by id
// through a sorted index (7.x ids; 6.x objects keyed by a hash of their
// `Class::Name`); connections are indexed from both ends by counting
// sorts; a property an object lacks comes from its template in
// `Definitions`; `GlobalSettings` gives axes, units and the frame rate,
// NTSC rates as exact rationals.

#include "fbx_object.h"

#include "allocator.h"
#include "container.h"
#include "fbx_document.h"
#include "fbx_value.h"
#include "sort.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Refuse(const maniFbx* fbx, uint32_t property, uint32_t item)
{
    uint64_t at = property < fbx->propertyCount ? fbx->properties[property].offset : 0;
    return maniFbxFail(maniImportReasonProperty, at, item);
}

static uint32_t* Words(maniFbx* fbx, size_t count)
{
    return maniAllocate(&fbx->allocator, count + 1, sizeof(uint32_t), 8);
}

static void FreeWords(maniFbx* fbx, uint32_t* words, size_t count)
{
    maniRelease(&fbx->allocator, words, count + 1, sizeof(uint32_t), 8);
}

// A string property's raw bytes as a 6.x key.
static int64_t NameKey(const maniFbx* fbx, uint32_t property, bool* ok)
{
    const uint8_t* bytes = NULL;
    uint32_t length = 0;
    *ok = maniFbxString(fbx, property, &bytes, &length) && length > 0;
    return *ok ? (int64_t)maniHashBytes(MANI_HASH_INIT, bytes, length) : 0;
}

// An object's key: its id (7.x) or its name's hash (6.x).
static bool ObjectKey(const maniFbx* fbx, uint32_t node, int64_t* key)
{
    uint32_t first = maniFbxNodeProperty(fbx, node, 0);
    if (fbx->version >= 7000)
    {
        return maniFbxInteger(fbx, first, INT64_MIN, INT64_MAX, key);
    }
    bool ok = false;
    *key = NameKey(fbx, first, &ok);
    return ok;
}

static uint64_t SortKey(int64_t id)
{
    return (uint64_t)id ^ 0x8000000000000000u;
}

static maniImportResult IndexObjects(maniFbx* fbx)
{
    uint32_t n = fbx->objectCount;
    uint64_t* keys = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint64_t), 8);
    uint32_t* scratch = Words(fbx, n);
    fbx->objectsById = Words(fbx, n);
    maniImportResult r = maniFbxSuccess();
    if (keys == NULL || scratch == NULL || fbx->objectsById == NULL)
    {
        r = Capacity();
    }
    for (uint32_t i = 0; r.result == mani_success && i < n; ++i)
    {
        keys[i] = SortKey(fbx->objects[i].id);
    }
    if (r.result == mani_success)
    {
        maniSortByKey(keys, n, fbx->objectsById, scratch);
    }
    for (uint32_t k = 1; r.result == mani_success && k < n; ++k)
    {
        uint32_t later = fbx->objectsById[k];
        if (keys[later] == keys[fbx->objectsById[k - 1]])
        {
            r = Refuse(fbx, maniFbxNodeProperty(fbx, fbx->objects[later].node, 0), later);
        }
    }
    maniRelease(&fbx->allocator, keys, (size_t)n + 1, sizeof(uint64_t), 8);
    FreeWords(fbx, scratch, n);
    return r;
}

uint32_t maniFbxFindObject(const maniFbx* fbx, int64_t id)
{
    uint32_t low = 0;
    uint32_t high = fbx->objectCount;
    uint64_t key = SortKey(id);
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        uint64_t at = SortKey(fbx->objects[fbx->objectsById[mid]].id);
        if (at < key)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    return low < fbx->objectCount && fbx->objects[fbx->objectsById[low]].id == id
               ? fbx->objectsById[low]
               : MANI_FBX_NONE;
}

// The node's Properties70 child, else its Properties60 one.
static uint32_t PropertiesOf(const maniFbx* fbx, uint32_t node)
{
    uint32_t p = maniFbxFindChild(fbx, node, "Properties70", 12);
    return p != MANI_FBX_NONE ? p : maniFbxFindChild(fbx, node, "Properties60", 12);
}

static bool StringIs(const maniFbx* fbx, uint32_t property, const char* s, size_t n)
{
    return maniFbxStringEquals(fbx, property, s, n);
}

// The subclass property: third in 7.x, second in 6.x.
static uint32_t SubclassOf(const maniFbx* fbx, uint32_t node)
{
    return maniFbxNodeProperty(fbx, node, fbx->version >= 7000 ? 2 : 1);
}

// Whether a kind takes any template of its kind, as ufbx's
// ufbxi_find_template has it.
static bool AnyTemplate(const maniFbx* fbx, uint32_t node)
{
    return maniFbxNameIs(fbx, node, "Model", 5) || maniFbxNameIs(fbx, node, "Material", 8) ||
           maniFbxNameIs(fbx, node, "AnimationStack", 14) ||
           maniFbxNameIs(fbx, node, "AnimationLayer", 14);
}

// Whether a PropertyTemplate's class, without "Fbx", is the subclass.
static bool ClassIs(const maniFbx* fbx, uint32_t tmpl, uint32_t subclass)
{
    const uint8_t* text = NULL;
    uint32_t length = 0;
    if (subclass == MANI_FBX_NONE ||
        !maniFbxString(fbx, maniFbxNodeProperty(fbx, tmpl, 0), &text, &length))
    {
        return false;
    }
    uint32_t skip = length >= 3 && memcmp(text, "Fbx", 3) == 0 ? 3 : 0;
    const uint8_t* own = NULL;
    uint32_t ownLength = 0;
    if (!maniFbxString(fbx, subclass, &own, &ownLength))
    {
        return false;
    }
    uint32_t ownSkip = ownLength >= 3 && memcmp(own, "Fbx", 3) == 0 ? 3 : 0;
    return length - skip == ownLength - ownSkip &&
           memcmp(text + skip, own + ownSkip, length - skip) == 0;
}

// The template for an object: a PropertyTemplate of an ObjectType of
// its kind whose class is its subclass, or the first of any for the
// kinds that take any.
static uint32_t TemplateOf(const maniFbx* fbx, uint32_t node)
{
    uint32_t definitions = maniFbxFindChild(fbx, 0, "Definitions", 11);
    if (definitions == MANI_FBX_NONE)
    {
        return MANI_FBX_NONE;
    }
    const maniFbxNode* object = &fbx->nodes[node];
    const char* kind = (const char*)fbx->bytes + object->nameOffset;
    bool any = AnyTemplate(fbx, node);
    uint32_t subclass = SubclassOf(fbx, node);
    for (uint32_t t = fbx->nodes[definitions].firstChild; t != MANI_FBX_NONE;
         t = fbx->nodes[t].next)
    {
        if (!maniFbxNameIs(fbx, t, "ObjectType", 10) ||
            !StringIs(fbx, maniFbxNodeProperty(fbx, t, 0), kind, object->nameLength))
        {
            continue;
        }
        for (uint32_t p = fbx->nodes[t].firstChild; p != MANI_FBX_NONE; p = fbx->nodes[p].next)
        {
            if (maniFbxNameIs(fbx, p, "PropertyTemplate", 16) && (any || ClassIs(fbx, p, subclass)))
            {
                return PropertiesOf(fbx, p);
            }
        }
    }
    return MANI_FBX_NONE;
}

static maniImportResult ReadObjectList(maniFbx* fbx)
{
    uint32_t objects = maniFbxFindChild(fbx, 0, "Objects", 7);
    uint32_t count = 0;
    for (uint32_t c = objects != MANI_FBX_NONE ? fbx->nodes[objects].firstChild : MANI_FBX_NONE;
         c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        ++count;
    }
    fbx->objects = maniAllocate(&fbx->allocator, (size_t)count + 1, sizeof(maniFbxObject), 8);
    if (fbx->objects == NULL)
    {
        return Capacity();
    }
    fbx->objectCapacity = count;
    // An object with no id or name is skipped and counted, as ufbx skips
    // "weird objects mixed in".
    for (uint32_t c = count > 0 ? fbx->nodes[objects].firstChild : MANI_FBX_NONE;
         c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        maniFbxObject o = {0, c, PropertiesOf(fbx, c), TemplateOf(fbx, c), 0};
        if (ObjectKey(fbx, c, &o.id))
        {
            fbx->objects[fbx->objectCount++] = o;
        }
        else
        {
            ++fbx->skippedObjects;
        }
    }
    return IndexObjects(fbx);
}

uint32_t maniFbxReferencedObject(const maniFbx* fbx, uint32_t property)
{
    int64_t key = 0;
    if (fbx->version >= 7000)
    {
        if (!maniFbxInteger(fbx, property, INT64_MIN, INT64_MAX, &key))
        {
            return MANI_FBX_NONE;
        }
        return key == 0 ? fbx->objectCount : maniFbxFindObject(fbx, key);
    }
    if (maniFbxStringEquals(fbx, property, "Model::Scene", 12))
    {
        return fbx->objectCount;
    }
    bool ok = false;
    key = NameKey(fbx, property, &ok);
    return ok ? maniFbxFindObject(fbx, key) : MANI_FBX_NONE;
}

// One connection entry as child, parent and property names; false to
// skip it.
static bool ReadConnection(const maniFbx* fbx, uint32_t node, maniFbxConnection* c)
{
    uint32_t type = maniFbxNodeProperty(fbx, node, 0);
    bool childProperty = StringIs(fbx, type, "PO", 2) || StringIs(fbx, type, "PP", 2);
    bool parentProperty = StringIs(fbx, type, "OP", 2) || StringIs(fbx, type, "PP", 2);
    if (!childProperty && !parentProperty && !StringIs(fbx, type, "OO", 2))
    {
        return false;
    }
    uint32_t k = 1;
    c->child = maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, node, k++));
    c->childProperty = childProperty ? maniFbxNodeProperty(fbx, node, k++) : MANI_FBX_NONE;
    c->parent = maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, node, k++));
    c->parentProperty = parentProperty ? maniFbxNodeProperty(fbx, node, k) : MANI_FBX_NONE;
    const uint8_t* text = NULL;
    uint32_t length = 0;
    bool names = (c->childProperty == MANI_FBX_NONE ||
                  maniFbxString(fbx, c->childProperty, &text, &length)) &&
                 (c->parentProperty == MANI_FBX_NONE ||
                  maniFbxString(fbx, c->parentProperty, &text, &length));
    return names && c->child < fbx->objectCount && c->parent != MANI_FBX_NONE;
}

// Indices of the connections sorted by one end, and where each object's
// run starts (the root's last).
static maniImportResult IndexEnds(maniFbx* fbx, bool byParent, uint32_t** sortedOut,
                                  uint32_t** startsOut)
{
    uint32_t n = fbx->connectionCount;
    uint32_t ends = fbx->objectCount + 1;
    uint32_t* starts = Words(fbx, (size_t)ends + 1);
    uint32_t* sorted = Words(fbx, n);
    *sortedOut = sorted;
    *startsOut = starts;
    if (starts == NULL || sorted == NULL)
    {
        return Capacity();
    }
    memset(starts, 0, ((size_t)ends + 2) * sizeof(uint32_t));
    for (uint32_t i = 0; i < n; ++i)
    {
        const maniFbxConnection* c = &fbx->connections[i];
        ++starts[(byParent ? c->parent : c->child) + 1];
    }
    for (uint32_t e = 0; e < ends; ++e)
    {
        starts[e + 1] += starts[e];
    }
    // A counting sort: each end's run in file order. starts[e] moves to
    // the run's end while filling, then back.
    for (uint32_t i = 0; i < n; ++i)
    {
        const maniFbxConnection* c = &fbx->connections[i];
        sorted[starts[byParent ? c->parent : c->child]++] = i;
    }
    for (uint32_t e = ends; e-- > 0;)
    {
        starts[e + 1] = starts[e];
    }
    starts[0] = 0;
    return maniFbxSuccess();
}

static maniImportResult ReadConnections(maniFbx* fbx)
{
    uint32_t list = maniFbxFindChild(fbx, 0, "Connections", 11);
    uint32_t count = 0;
    for (uint32_t c = list != MANI_FBX_NONE ? fbx->nodes[list].firstChild : MANI_FBX_NONE;
         c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        ++count;
    }
    fbx->connections =
        maniAllocate(&fbx->allocator, (size_t)count + 1, sizeof(maniFbxConnection), 8);
    fbx->connectionCapacity = count;
    if (fbx->connections == NULL)
    {
        return Capacity();
    }
    for (uint32_t c = count > 0 ? fbx->nodes[list].firstChild : MANI_FBX_NONE; c != MANI_FBX_NONE;
         c = fbx->nodes[c].next)
    {
        maniFbxConnection connection;
        bool entry = maniFbxNameIs(fbx, c, "C", 1) || maniFbxNameIs(fbx, c, "Connect", 7);
        if (entry && ReadConnection(fbx, c, &connection))
        {
            fbx->connections[fbx->connectionCount++] = connection;
        }
        else
        {
            ++fbx->skippedConnections;
        }
    }
    maniImportResult r = IndexEnds(fbx, true, &fbx->byParent, &fbx->parentStarts);
    return r.result == mani_success ? IndexEnds(fbx, false, &fbx->byChild, &fbx->childStarts) : r;
}

uint32_t maniFbxChildConnections(const maniFbx* fbx, uint32_t object, const uint32_t** out)
{
    *out = fbx->byParent + fbx->parentStarts[object];
    return fbx->parentStarts[object + 1] - fbx->parentStarts[object];
}

uint32_t maniFbxParentConnections(const maniFbx* fbx, uint32_t object, const uint32_t** out)
{
    *out = fbx->byChild + fbx->childStarts[object];
    return fbx->childStarts[object + 1] - fbx->childStarts[object];
}

// The last entry of a properties node with the name given.
static uint32_t FindIn(const maniFbx* fbx, uint32_t properties, const char* name, size_t length)
{
    uint32_t found = MANI_FBX_NONE;
    if (properties == MANI_FBX_NONE)
    {
        return found;
    }
    for (uint32_t e = fbx->nodes[properties].firstChild; e != MANI_FBX_NONE; e = fbx->nodes[e].next)
    {
        bool entry = maniFbxNameIs(fbx, e, "P", 1) || maniFbxNameIs(fbx, e, "Property", 8);
        if (entry && StringIs(fbx, maniFbxNodeProperty(fbx, e, 0), name, length))
        {
            found = e;
        }
    }
    return found;
}

uint32_t maniFbxFindProperty(const maniFbx* fbx, uint32_t object, const char* name, size_t length)
{
    const maniFbxObject* o = &fbx->objects[object];
    uint32_t own = FindIn(fbx, o->properties, name, length);
    return own != MANI_FBX_NONE ? own : FindIn(fbx, o->templateProperties, name, length);
}

uint32_t maniFbxPropertyValue(const maniFbx* fbx, uint32_t entry, uint32_t k)
{
    if (entry >= fbx->nodeCount)
    {
        return MANI_FBX_NONE;
    }
    // Properties70's entries hold a label after the type; Properties60's
    // do not.
    uint32_t first = maniFbxNameIs(fbx, entry, "P", 1) ? 4 : 3;
    return maniFbxNodeProperty(fbx, entry, first + k);
}

bool maniFbxObjectKindIs(const maniFbx* fbx, uint32_t object, const char* kind, size_t length)
{
    return maniFbxNameIs(fbx, fbx->objects[object].node, kind, length);
}

bool maniFbxObjectSubclassIs(const maniFbx* fbx, uint32_t object, const char* name, size_t length)
{
    return maniFbxStringEquals(fbx, SubclassOf(fbx, fbx->objects[object].node), name, length);
}

size_t maniFbxObjectName(const maniFbx* fbx, uint32_t object, uint8_t* out, size_t capacity)
{
    uint32_t k = fbx->version >= 7000 ? 1 : 0;
    uint32_t property = maniFbxNodeProperty(fbx, fbx->objects[object].node, k);
    size_t length = maniFbxStringDecode(fbx, property, out, capacity);
    // Binary: Name\x00\x01Class. ASCII: Class::Name.
    for (size_t i = 0; i + 1 < length; ++i)
    {
        if (!fbx->ascii && out[i] == 0 && out[i + 1] == 1)
        {
            return i;
        }
        if (fbx->ascii && out[i] == ':' && out[i + 1] == ':')
        {
            memmove(out, out + i + 2, length - i - 2);
            return length - i - 2;
        }
    }
    return length;
}

// A setting's first value, or MANI_FBX_NONE when GlobalSettings lacks it.
static uint32_t Setting(const maniFbx* fbx, uint32_t properties, const char* name)
{
    uint32_t entry = FindIn(fbx, properties, name, strlen(name));
    return entry != MANI_FBX_NONE ? maniFbxPropertyValue(fbx, entry, 0) : MANI_FBX_NONE;
}

// An integer setting in [low, high], its default when absent.
static bool IntegerSetting(const maniFbx* fbx, uint32_t properties, const char* name, int64_t low,
                           int64_t high, int64_t* value, uint32_t* where)
{
    *where = Setting(fbx, properties, name);
    return *where == MANI_FBX_NONE || maniFbxInteger(fbx, *where, low, high, value);
}

// One axis: its index and sign.
static maniImportResult ReadAxis(maniFbx* fbx, uint32_t properties, const char* axis,
                                 const char* sign, uint32_t slot)
{
    int64_t a = fbx->settings.axes[slot];
    int64_t s = fbx->settings.signs[slot] < 0 ? -1 : 1;
    uint32_t where = MANI_FBX_NONE;
    if (!IntegerSetting(fbx, properties, axis, 0, 2, &a, &where) ||
        !IntegerSetting(fbx, properties, sign, -1, 1, &s, &where) || s == 0)
    {
        return Refuse(fbx, where, slot);
    }
    fbx->settings.axes[slot] = (uint8_t)a;
    fbx->settings.signs[slot] = (int8_t)s;
    return maniFbxSuccess();
}

// FbxTime::EMode's rates as numerator and denominator; custom (14) is
// CustomFrameRate's.
static const uint32_t kRates[19][2] = {
    {30, 1}, {120, 1},      {100, 1},      {60, 1},       {50, 1},        {48, 1},   {30, 1},
    {30, 1}, {30000, 1001}, {30000, 1001}, {25, 1},       {24, 1},        {1000, 1}, {24000, 1001},
    {0, 0},  {96, 1},       {72, 1},       {60000, 1001}, {120000, 1001},
};

static maniImportResult ReadRate(maniFbx* fbx, uint32_t properties)
{
    int64_t mode = 0;
    uint32_t where = MANI_FBX_NONE;
    if (!IntegerSetting(fbx, properties, "TimeMode", 0, 18, &mode, &where))
    {
        return Refuse(fbx, where, 0);
    }
    maniFbxGlobalSettings* s = &fbx->settings;
    s->timeMode = (uint32_t)mode;
    s->rateNumerator = kRates[mode][0];
    s->rateDenominator = kRates[mode][1];
    if (mode == 14)
    {
        where = Setting(fbx, properties, "CustomFrameRate");
        if (!maniFbxNumber(fbx, where, &s->customRate) || !(s->customRate > 0.0))
        {
            return Refuse(fbx, where, 0);
        }
    }
    where = MANI_FBX_NONE;
    if (!IntegerSetting(fbx, properties, "TimeSpanStart", INT64_MIN, INT64_MAX, &s->spanStart,
                        &where) ||
        !IntegerSetting(fbx, properties, "TimeSpanStop", INT64_MIN, INT64_MAX, &s->spanStop,
                        &where))
    {
        return Refuse(fbx, where, 0);
    }
    return maniFbxSuccess();
}

// A positive scale, its default when absent.
static maniImportResult ReadScale(const maniFbx* fbx, uint32_t properties, const char* name,
                                  double* scale)
{
    uint32_t where = Setting(fbx, properties, name);
    if (where != MANI_FBX_NONE && (!maniFbxNumber(fbx, where, scale) || !(*scale > 0.0)))
    {
        return Refuse(fbx, where, 0);
    }
    return maniFbxSuccess();
}

// FBX's time unit, as ufbx reads it: 46186158000 ticks a second before
// 8000, unless a header version from 1004 on gives OtherFlags a
// TCDefinition other than 127, which selects 141120000.
static int64_t TicksPerSecond(const maniFbx* fbx)
{
    uint32_t header = maniFbxFindChild(fbx, 0, "FBXHeaderExtension", 18);
    if (header == MANI_FBX_NONE)
    {
        return 46186158000;
    }
    int64_t version = 0;
    int64_t definition = 127;
    uint32_t node = maniFbxFindChild(fbx, header, "FBXHeaderVersion", 16);
    bool versioned =
        node != MANI_FBX_NONE &&
        maniFbxInteger(fbx, maniFbxNodeProperty(fbx, node, 0), 0, INT32_MAX, &version) &&
        version >= 1004;
    uint32_t flags = maniFbxFindChild(fbx, header, "OtherFlags", 10);
    node =
        flags != MANI_FBX_NONE ? maniFbxFindChild(fbx, flags, "TCDefinition", 12) : MANI_FBX_NONE;
    bool defined = node != MANI_FBX_NONE && maniFbxInteger(fbx, maniFbxNodeProperty(fbx, node, 0),
                                                           INT32_MIN, INT32_MAX, &definition);
    return versioned && defined && definition != 127 ? 141120000 : 46186158000;
}

// GlobalSettings, FBX's defaults where it is silent: X right, Y up, Z
// front, all positive; one centimetre a unit; 30 frames a second.
static maniImportResult ReadSettings(maniFbx* fbx)
{
    fbx->settings = (maniFbxGlobalSettings){
        {0, 1, 2}, {1, 1, 1}, 1.0, 1.0, 0, 30, 1, 0.0, 0, 0, TicksPerSecond(fbx)};
    uint32_t global = maniFbxFindChild(fbx, 0, "GlobalSettings", 14);
    uint32_t properties = global != MANI_FBX_NONE ? PropertiesOf(fbx, global) : MANI_FBX_NONE;
    maniImportResult r = ReadAxis(fbx, properties, "CoordAxis", "CoordAxisSign", 0);
    if (r.result == mani_success)
    {
        r = ReadAxis(fbx, properties, "UpAxis", "UpAxisSign", 1);
    }
    if (r.result == mani_success)
    {
        r = ReadAxis(fbx, properties, "FrontAxis", "FrontAxisSign", 2);
    }
    const uint8_t* axes = fbx->settings.axes;
    if (r.result == mani_success &&
        (axes[0] == axes[1] || axes[1] == axes[2] || axes[0] == axes[2]))
    {
        r = Refuse(fbx, Setting(fbx, properties, "FrontAxis"), 2);
    }
    if (r.result == mani_success)
    {
        r = ReadScale(fbx, properties, "UnitScaleFactor", &fbx->settings.unitScale);
    }
    fbx->settings.originalUnitScale = fbx->settings.unitScale;
    if (r.result == mani_success)
    {
        r = ReadScale(fbx, properties, "OriginalUnitScaleFactor", &fbx->settings.originalUnitScale);
    }
    return r.result == mani_success ? ReadRate(fbx, properties) : r;
}

maniImportResult maniFbxReadObjects(maniFbx* fbx)
{
    maniImportResult r = ReadObjectList(fbx);
    if (r.result == mani_success)
    {
        r = ReadConnections(fbx);
    }
    return r.result == mani_success ? ReadSettings(fbx) : r;
}

void maniFbxReleaseObjects(maniFbx* fbx)
{
    uint32_t ends = fbx->objectCount + 1;
    FreeWords(fbx, fbx->parentStarts, (size_t)ends + 1);
    FreeWords(fbx, fbx->childStarts, (size_t)ends + 1);
    FreeWords(fbx, fbx->byParent, fbx->connectionCount);
    FreeWords(fbx, fbx->byChild, fbx->connectionCount);
    maniRelease(&fbx->allocator, fbx->connections, (size_t)fbx->connectionCapacity + 1,
                sizeof(maniFbxConnection), 8);
    FreeWords(fbx, fbx->objectsById, fbx->objectCount);
    maniRelease(&fbx->allocator, fbx->objects, (size_t)fbx->objectCapacity + 1,
                sizeof(maniFbxObject), 8);
}

maniResult maniFbxReadSettings(const maniFbx* fbx, maniFbxSettings* settingsOut)
{
    if (fbx == NULL || settingsOut == NULL)
    {
        return mani_errorInvalid;
    }
    const maniFbxGlobalSettings* s = &fbx->settings;
    for (uint32_t k = 0; k < 3; ++k)
    {
        settingsOut->axes[k] = s->axes[k] + (s->signs[k] < 0 ? maniAxisNegativeX : maniAxisX);
    }
    settingsOut->timeMode = s->timeMode;
    settingsOut->unitMeters = s->unitScale * 0.01;
    settingsOut->originalUnitMeters = s->originalUnitScale * 0.01;
    settingsOut->rateNumerator = s->rateNumerator;
    settingsOut->rateDenominator = s->rateDenominator;
    settingsOut->customRate = s->customRate;
    settingsOut->spanStart = s->spanStart;
    settingsOut->spanStop = s->spanStop;
    settingsOut->ticksPerSecond = s->ticksPerSecond;
    return mani_success;
}

bool maniFbxDeformerIs(const maniFbx* fbx, uint32_t object, const char* subclass, size_t length)
{
    return object < fbx->objectCount && maniFbxObjectKindIs(fbx, object, "Deformer", 8) &&
           maniFbxObjectSubclassIs(fbx, object, subclass, length);
}

void maniFbxEachChild(const maniFbx* fbx, uint32_t object, maniFbxVisit visit, void* context)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxChildConnections(fbx, object, &list);
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniFbxConnection* c = &fbx->connections[list[k]];
        if (c->childProperty == MANI_FBX_NONE && c->parentProperty == MANI_FBX_NONE)
        {
            visit(fbx, c->child, context);
        }
    }
}

// The properties ufbx keeps on a 6.x model (ufbxi_node_prop_names).
static const char* const kNodeProperties[] = {
    "AxisLen",
    "DefaultAttributeIndex",
    "Freeze",
    "GeometricRotation",
    "GeometricScaling",
    "GeometricTranslation",
    "InheritType",
    "LODBox",
    "Lcl Rotation",
    "Lcl Scaling",
    "Lcl Translation",
    "LookAtProperty",
    "MaxDampRangeX",
    "MaxDampRangeY",
    "MaxDampRangeZ",
    "MaxDampStrengthX",
    "MaxDampStrengthY",
    "MaxDampStrengthZ",
    "MinDampRangeX",
    "MinDampRangeY",
    "MinDampRangeZ",
    "MinDampStrengthX",
    "MinDampStrengthY",
    "MinDampStrengthZ",
    "NegativePercentShapeSupport",
    "PostRotation",
    "PreRotation",
    "PreferedAngleX",
    "PreferedAngleY",
    "PreferedAngleZ",
    "QuaternionInterpolate",
    "RotationActive",
    "RotationMax",
    "RotationMaxX",
    "RotationMaxY",
    "RotationMaxZ",
    "RotationMin",
    "RotationMinX",
    "RotationMinY",
    "RotationMinZ",
    "RotationOffset",
    "RotationOrder",
    "RotationPivot",
    "RotationSpaceForLimitOnly",
    "RotationStiffnessX",
    "RotationStiffnessY",
    "RotationStiffnessZ",
    "ScalingActive",
    "ScalingMax",
    "ScalingMaxX",
    "ScalingMaxY",
    "ScalingMaxZ",
    "ScalingMin",
    "ScalingMinX",
    "ScalingMinY",
    "ScalingMinZ",
    "ScalingOffset",
    "ScalingPivot",
    "Show",
    "TranslationActive",
    "TranslationMax",
    "TranslationMaxX",
    "TranslationMaxY",
    "TranslationMaxZ",
    "TranslationMin",
    "TranslationMinX",
    "TranslationMinY",
    "TranslationMinZ",
    "UpVectorProperty",
    "Visibility Inheritance",
    "Visibility",
    "notes",
};

bool maniFbxIsNodeProperty(const uint8_t* name, size_t length)
{
    for (size_t k = 0; k < sizeof kNodeProperties / sizeof kNodeProperties[0]; ++k)
    {
        const char* n = kNodeProperties[k];
        if (strlen(n) == length && memcmp(n, name, length) == 0)
        {
            return true;
        }
    }
    return false;
}

bool maniFbxEntryUserDefined(const maniFbx* fbx, uint32_t entry)
{
    // The flags follow the type, and the label in Properties70.
    uint32_t k = maniFbxNameIs(fbx, entry, "P", 1) ? 3 : 2;
    const uint8_t* flags = NULL;
    uint32_t length = 0;
    return maniFbxString(fbx, maniFbxNodeProperty(fbx, entry, k), &flags, &length) &&
           memchr(flags, 'U', length) != NULL;
}

bool maniFbxModelHasAttribute(const maniFbx* fbx, uint32_t object)
{
    uint32_t node = fbx->objects[object].node;
    uint32_t subclass = SubclassOf(fbx, node);
    const uint8_t* bytes = NULL;
    uint32_t length = 0;
    bool none = !maniFbxString(fbx, subclass, &bytes, &length) || length == 0;
    if (none)
    {
        return maniFbxFindChild(fbx, node, "Vertices", 8) != MANI_FBX_NONE &&
               maniFbxFindChild(fbx, node, "PolygonVertexIndex", 18) != MANI_FBX_NONE;
    }
    return !maniFbxStringEquals(fbx, subclass, "Model", 5);
}
