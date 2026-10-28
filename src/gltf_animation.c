// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Animations: samplers and channels checked at open against
// the specification's table of targets and accessor types, frames and
// keys counted, a target used twice refused; `KHR_animation_pointer`
// pointers to a node's TRS or weights read as those channels.

#include "gltf_animation.h"

#include "allocator.h"
#include "arena.h"
#include "container.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"
#include "sort.h"

#include "maul-anim/build.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

static maniImportResult Fail(const maniGltf* gltf, maniImportReason reason, uint32_t value,
                             uint32_t item)
{
    return maniImportFail(reason, maniGltfOffset(&gltf->g, value), item);
}

uint32_t maniGltfMeshTargets(const maniGltf* gltf, uint32_t mesh)
{
    const maniJson* json = &gltf->json;
    uint32_t primitives = maniJsonFind(json, gltf->meshValues[mesh], "primitives", 10);
    if (primitives == MANI_JSON_NONE || json->values[primitives].kind != maniJsonArray)
    {
        return 0;
    }
    uint32_t element = primitives + 1;
    for (uint32_t p = 0; p < json->values[primitives].count; ++p)
    {
        uint32_t targets = maniJsonFind(json, element, "targets", 7);
        if (targets != MANI_JSON_NONE && json->values[targets].kind == maniJsonArray &&
            json->values[targets].count > 0)
        {
            return json->values[targets].count;
        }
        element = json->values[element].next;
    }
    return 0;
}

uint32_t maniGltfMorphTargets(const maniGltf* gltf, uint32_t node)
{
    int64_t mesh = -1;
    uint32_t index = maniJsonFind(&gltf->json, gltf->nodeValues[node], "mesh", 4);
    if (gltf->meshCount == 0 ||
        !maniJsonInteger(&gltf->json, index, 0, (int64_t)gltf->meshCount - 1, &mesh))
    {
        return 0;
    }
    return maniGltfMeshTargets(gltf, (uint32_t)mesh);
}

uint32_t maniGltfTargetNames(const maniGltf* gltf, uint32_t mesh, uint32_t targets)
{
    const maniJson* json = &gltf->json;
    uint32_t extras = maniJsonFind(json, gltf->meshValues[mesh], "extras", 6);
    uint32_t names = maniJsonFind(json, extras, "targetNames", 11);
    if (names == MANI_JSON_NONE || json->values[names].kind != maniJsonArray ||
        json->values[names].count != targets)
    {
        return MANI_JSON_NONE;
    }
    uint32_t element = names + 1;
    for (uint32_t i = 0; i < targets; ++i)
    {
        if (json->values[element].kind != maniJsonString || json->values[element].length == 0)
        {
            return MANI_JSON_NONE;
        }
        element = json->values[element].next;
    }
    return names;
}

// A pointer of the form /nodes/<index>/<property> for a node's TRS or
// weights: the target and node; Pointer otherwise.
static maniGltfTarget NodePointer(const uint8_t* text, size_t length, uint32_t* nodeOut)
{
    static const char* const kProperties[4] = {"translation", "rotation", "scale", "weights"};
    if (length < 8 || memcmp(text, "/nodes/", 7) != 0)
    {
        return maniGltfTargetPointer;
    }
    size_t at = 7;
    uint64_t node = 0;
    while (at < length && text[at] >= '0' && text[at] <= '9' && node <= UINT32_MAX)
    {
        node = node * 10 + (uint64_t)(text[at++] - '0');
    }
    // No leading zeros: "/nodes/01" names nothing.
    if (at == 7 || at >= length || text[at] != '/' || node > UINT32_MAX ||
        (text[7] == '0' && at > 8))
    {
        return maniGltfTargetPointer;
    }
    ++at;
    for (uint32_t k = 0; k < 4; ++k)
    {
        size_t n = strlen(kProperties[k]);
        if (length - at == n && memcmp(text + at, kProperties[k], n) == 0)
        {
            *nodeOut = (uint32_t)node;
            return (maniGltfTarget)(k + 1);
        }
    }
    return maniGltfTargetPointer;
}

// The target object: its kind, node and pointer.
static maniImportResult ReadTarget(const maniGltf* gltf, uint32_t channel, uint32_t item,
                                   maniGltfChannel* out)
{
    static const char* const kPaths[5] = {"translation", "rotation", "scale", "weights", "pointer"};
    const maniGltfJson* g = &gltf->g;
    uint32_t target = MANI_JSON_NONE;
    uint32_t path = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, channel, "target", maniJsonObject, true, item, &target);
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, target, "path", maniJsonString, true, item, &path);
    }
    if (r.result == mani_success)
    {
        r = maniGltfIndex(g, target, "node", false, gltf->nodeCount, item, &out->node);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t kind = 5;
    for (uint32_t k = 0; k < 5; ++k)
    {
        kind = kind == 5 && maniJsonStringEquals(&gltf->json, path, kPaths[k], strlen(kPaths[k]))
                   ? k
                   : kind;
    }
    if (kind == 5)
    {
        return Fail(gltf, maniImportReasonProperty, path, item);
    }
    if (kind < 4)
    {
        out->target =
            out->node == MANI_JSON_NONE ? maniGltfTargetIgnored : (maniGltfTarget)(kind + 1);
        return maniImportSuccess();
    }
    uint32_t extensions = MANI_JSON_NONE;
    uint32_t extension = MANI_JSON_NONE;
    r = maniGltfMember(g, target, "extensions", maniJsonObject, true, item, &extensions);
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, extensions, "KHR_animation_pointer", maniJsonObject, true, item,
                           &extension);
    }
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, extension, "pointer", maniJsonString, true, item, &out->pointer);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint8_t text[64];
    const maniJsonValue* p = &gltf->json.values[out->pointer];
    size_t n = maniJsonStringDecode(&gltf->json, out->pointer, text, sizeof text);
    if (n == 0 || text[0] != '/')
    {
        return Fail(gltf, maniImportReasonProperty, out->pointer, item);
    }
    out->target =
        p->length < sizeof text ? NodePointer(text, n, &out->node) : maniGltfTargetPointer;
    if (out->target != maniGltfTargetPointer && out->node >= gltf->nodeCount)
    {
        return Fail(gltf, maniImportReasonIndex, out->pointer, item);
    }
    return maniImportSuccess();
}

static bool IsFloat(const maniGltfAccessor* a)
{
    return a->component == maniGltfFloat && !a->normalized;
}

static bool IsNormalizedOrFloat(const maniGltfAccessor* a)
{
    return IsFloat(a) || (a->normalized && a->component <= maniGltfUnsignedShort);
}

static bool Strided(const maniGltf* gltf, const maniGltfAccessor* a)
{
    return !maniGltfPacked(gltf, a);
}

// The output accessor against the target's row of the specification's
// table; the key width.
static bool OutputFits(const maniGltf* gltf, const maniGltfAccessor* a, maniGltfChannel* c)
{
    switch (c->target)
    {
    case maniGltfTargetTranslation:
    case maniGltfTargetScale:
        c->width = 3;
        return a->rows == 3 && a->columns == 1 && IsFloat(a);
    case maniGltfTargetRotation:
        c->width = 4;
        return a->rows == 4 && a->columns == 1 && IsNormalizedOrFloat(a);
    case maniGltfTargetWeights:
        c->width = maniGltfMorphTargets(gltf, c->node);
        return c->width > 0 && a->rows == 1 && a->columns == 1 && IsNormalizedOrFloat(a);
    default:
        c->width = (uint32_t)a->rows * a->columns;
        return IsNormalizedOrFloat(a);
    }
}

static maniImportResult ReadSampler(const maniGltf* gltf, uint32_t sampler, uint32_t item,
                                    maniGltfChannel* out)
{
    static const char* const kModes[3] = {"LINEAR", "STEP", "CUBICSPLINE"};
    const maniGltfJson* g = &gltf->g;
    uint32_t mode = MANI_JSON_NONE;
    maniImportResult r =
        maniGltfIndex(g, sampler, "input", true, gltf->accessorCount, item, &out->input);
    if (r.result == mani_success)
    {
        r = maniGltfIndex(g, sampler, "output", true, gltf->accessorCount, item, &out->output);
    }
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, sampler, "interpolation", maniJsonString, false, item, &mode);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    out->interpolation = maniInterpolationLinear;
    if (mode != MANI_JSON_NONE)
    {
        out->interpolation = 3;
        for (uint32_t k = 0; k < 3; ++k)
        {
            out->interpolation =
                maniJsonStringEquals(&gltf->json, mode, kModes[k], strlen(kModes[k]))
                    ? k
                    : out->interpolation;
        }
        if (out->interpolation == 3)
        {
            return Fail(gltf, maniImportReasonProperty, mode, item);
        }
    }
    const maniGltfAccessor* input = &gltf->accessors[out->input];
    if (input->rows != 1 || input->columns != 1 || !IsFloat(input) || Strided(gltf, input))
    {
        uint32_t at = maniJsonFind(&gltf->json, sampler, "input", 5);
        return Fail(gltf, maniImportReasonProperty, at, item);
    }
    out->keys = input->count;
    return maniImportSuccess();
}

maniImportResult maniGltfReadChannel(const maniGltf* gltf, uint32_t animation, uint32_t channel,
                                     uint32_t item, maniGltfChannel* out)
{
    const maniGltfJson* g = &gltf->g;
    *out = (maniGltfChannel){.node = MANI_JSON_NONE, .pointer = MANI_JSON_NONE, .value = channel};
    uint32_t first = gltf->samplerStarts[animation];
    uint32_t index = MANI_JSON_NONE;
    maniImportResult r = maniGltfIndex(g, channel, "sampler", true,
                                       gltf->samplerStarts[animation + 1] - first, item, &index);
    if (r.result == mani_success)
    {
        r = ReadSampler(gltf, gltf->samplerValues[first + index], item, out);
    }
    if (r.result == mani_success)
    {
        r = ReadTarget(gltf, channel, item, out);
    }
    if (r.result != mani_success || out->target == maniGltfTargetIgnored)
    {
        return r;
    }
    bool trs = out->target <= maniGltfTargetScale;
    if (trs &&
        maniJsonFind(&gltf->json, gltf->nodeValues[out->node], "matrix", 6) != MANI_JSON_NONE)
    {
        return Fail(gltf, maniImportReasonProperty, channel, item);
    }
    const maniGltfAccessor* output = &gltf->accessors[out->output];
    uint64_t parts = out->interpolation == maniInterpolationCubicSpline ? 3 : 1;
    uint32_t at = maniJsonFind(&gltf->json, channel, "sampler", 7);
    if (!OutputFits(gltf, output, out) || Strided(gltf, output))
    {
        return Fail(gltf, maniImportReasonProperty, at, item);
    }
    // Morph frames are one scalar per target; other outputs a vector a key.
    uint64_t perKey = out->target == maniGltfTargetWeights ? parts * out->width : parts;
    if ((uint64_t)output->count != perKey * out->keys || (parts == 3 && out->keys < 2))
    {
        return Fail(gltf, maniImportReasonProperty, at, item);
    }
    return maniImportSuccess();
}

uint32_t maniGltfAnimationChannels(const maniGltf* gltf, uint32_t animation, uint32_t* countOut)
{
    uint32_t channels = maniJsonFind(&gltf->json, gltf->animationValues[animation], "channels", 8);
    *countOut = gltf->json.values[channels].count;
    return channels;
}

// A target's key for finding repeats: a node and path, or a pointer's
// hash with the top bit set.
static uint64_t TargetKey(const maniGltf* gltf, const maniGltfChannel* c, maniArena* arena)
{
    if (c->target != maniGltfTargetPointer)
    {
        return ((uint64_t)c->node << 3) | (uint64_t)c->target;
    }
    size_t length = gltf->json.values[c->pointer].length;
    uint8_t* text = maniArenaTake(arena, length + 1, 1);
    size_t n = text != NULL ? maniJsonStringDecode(&gltf->json, c->pointer, text, length) : 0;
    return maniHashBytes(MANI_HASH_INIT, text, n) | ((uint64_t)1 << 63);
}

static maniImportResult ReadAnimation(maniGltf* gltf, uint32_t a, maniArena* arena,
                                      maniGltfChannel* kept)
{
    const maniGltfJson* g = &gltf->g;
    uint32_t value = gltf->animationValues[a];
    uint32_t samplers = MANI_JSON_NONE;
    uint32_t channels = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, value, "samplers", maniJsonArray, true, a, &samplers);
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, value, "channels", maniJsonArray, true, a, &channels);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t count = gltf->json.values[channels].count;
    if (count == 0 || gltf->json.values[samplers].count == 0)
    {
        return Fail(gltf, maniImportReasonProperty, count == 0 ? channels : samplers, a);
    }
    uint64_t* keys = maniArenaTake(arena, count, sizeof(uint64_t));
    uint32_t* sorted = maniArenaTake(arena, count, sizeof(uint32_t));
    uint32_t* scratch = maniArenaTake(arena, count, sizeof(uint32_t));
    uint32_t* values = maniArenaTake(arena, count, sizeof(uint32_t));
    if (keys == NULL || sorted == NULL || scratch == NULL || values == NULL)
    {
        return Capacity();
    }
    uint32_t channel = channels + 1;
    for (uint32_t c = 0; c < count; ++c)
    {
        if (gltf->json.values[channel].kind != maniJsonObject)
        {
            return Fail(gltf, maniImportReasonProperty, channel, a);
        }
        maniGltfChannel parsed;
        r = maniGltfReadChannel(gltf, a, channel, a, &parsed);
        if (r.result != mani_success)
        {
            return r;
        }
        kept[c] = parsed;
        // Ignored channels never repeat a target.
        keys[c] = parsed.target == maniGltfTargetIgnored ? ((uint64_t)1 << 62) + c
                                                         : TargetKey(gltf, &parsed, arena);
        values[c] = channel;
        channel = gltf->json.values[channel].next;
    }
    maniSortByKey(keys, count, sorted, scratch);
    for (uint32_t k = 1; k < count; ++k)
    {
        if (keys[sorted[k]] == keys[sorted[k - 1]])
        {
            return Fail(gltf, maniImportReasonProperty, values[sorted[k]], a);
        }
    }
    return maniImportSuccess();
}

// Names: decoded, end to end; empty for an animation without one.
static maniImportResult ReadNames(maniGltf* gltf)
{
    maniBoneNames* names = &gltf->animationNames;
    size_t total = 0;
    for (uint32_t a = 0; a < gltf->animationCount; ++a)
    {
        uint32_t name = maniJsonFind(&gltf->json, gltf->animationValues[a], "name", 4);
        if (name != MANI_JSON_NONE && gltf->json.values[name].kind != maniJsonString)
        {
            return Fail(gltf, maniImportReasonProperty, name, a);
        }
        total += name != MANI_JSON_NONE ? gltf->json.values[name].length : 0;
    }
    names->count = gltf->animationCount;
    names->textSize = total + 1;
    names->text = maniAllocate(&gltf->allocator, names->textSize, 1, 8);
    names->starts = maniAllocate(&gltf->allocator, (size_t)names->count + 1, sizeof(uint32_t), 8);
    if (names->text == NULL || names->starts == NULL)
    {
        return Capacity();
    }
    uint32_t at = 0;
    for (uint32_t a = 0; a < gltf->animationCount; ++a)
    {
        uint32_t name = maniJsonFind(&gltf->json, gltf->animationValues[a], "name", 4);
        names->starts[a] = at;
        if (name != MANI_JSON_NONE)
        {
            at += (uint32_t)maniJsonStringDecode(&gltf->json, name, (uint8_t*)names->text + at,
                                                 gltf->json.values[name].length);
        }
    }
    names->starts[gltf->animationCount] = at;
    return maniImportSuccess();
}

// Every value of an array member of each object, end to end, in a table;
// starts[i] is object i's first. Elements must be objects.
static maniImportResult Table(maniGltf* gltf, const uint32_t* objects, uint32_t count,
                              const char* name, uint32_t** tableOut, uint32_t** startsOut,
                              uint32_t* totalOut)
{
    const maniGltfJson* g = &gltf->g;
    uint64_t total = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t array = MANI_JSON_NONE;
        maniImportResult r = maniGltfMember(g, objects[i], name, maniJsonArray, true, i, &array);
        if (r.result != mani_success)
        {
            return r;
        }
        total += gltf->json.values[array].count;
    }
    *totalOut = (uint32_t)total;
    *tableOut = maniAllocate(&gltf->allocator, total + 1, sizeof(uint32_t), 8);
    *startsOut = maniAllocate(&gltf->allocator, (size_t)count + 1, sizeof(uint32_t), 8);
    if (*tableOut == NULL || *startsOut == NULL)
    {
        return Capacity();
    }
    uint32_t at = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t array = maniJsonFind(&gltf->json, objects[i], name, strlen(name));
        (*startsOut)[i] = at;
        uint32_t element = array + 1;
        for (uint32_t k = 0; k < gltf->json.values[array].count; ++k)
        {
            if (gltf->json.values[element].kind != maniJsonObject)
            {
                return Fail(gltf, maniImportReasonProperty, element, i);
            }
            (*tableOut)[at++] = element;
            element = gltf->json.values[element].next;
        }
    }
    (*startsOut)[count] = at;
    return maniImportSuccess();
}

// The samplers' and meshes' values, so channels find theirs at once.
static maniImportResult ReadTables(maniGltf* gltf)
{
    maniImportResult r = Table(gltf, gltf->animationValues, gltf->animationCount, "samplers",
                               &gltf->samplerValues, &gltf->samplerStarts, &gltf->samplerTotal);
    uint32_t meshes = MANI_JSON_NONE;
    uint32_t count = 0;
    if (r.result == mani_success)
    {
        r = maniGltfArray(&gltf->g, "meshes", maniJsonObject, &meshes, &count);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    gltf->meshValues = maniAllocate(&gltf->allocator, (size_t)count + 1, sizeof(uint32_t), 8);
    if (gltf->meshValues == NULL)
    {
        return Capacity();
    }
    gltf->meshCount = count;
    uint32_t value = meshes + 1;
    for (uint32_t m = 0; m < count; ++m)
    {
        gltf->meshValues[m] = value;
        value = gltf->json.values[value].next;
    }
    return maniImportSuccess();
}

maniImportResult maniGltfReadAnimations(maniGltf* gltf)
{
    uint32_t array = MANI_JSON_NONE;
    uint32_t count = 0;
    maniImportResult r = maniGltfArray(&gltf->g, "animations", maniJsonObject, &array, &count);
    if (r.result != mani_success)
    {
        return r;
    }
    gltf->animationValues = maniAllocate(&gltf->allocator, (size_t)count + 1, sizeof(uint32_t), 8);
    if (gltf->animationValues == NULL)
    {
        return Capacity();
    }
    gltf->animationCount = count;
    uint32_t value = array + 1;
    for (uint32_t a = 0; a < count; ++a)
    {
        gltf->animationValues[a] = value;
        value = gltf->json.values[value].next;
    }
    r = ReadTables(gltf);
    if (r.result != mani_success)
    {
        return r;
    }
    // Room for every channel an animation's array holds; the arrays
    // themselves are checked as each animation is read.
    uint64_t total = 0;
    for (uint32_t a = 0; a < count; ++a)
    {
        uint32_t channels = maniJsonFind(&gltf->json, gltf->animationValues[a], "channels", 8);
        total += channels != MANI_JSON_NONE && gltf->json.values[channels].kind == maniJsonArray
                     ? gltf->json.values[channels].count
                     : 0;
    }
    gltf->channelTotal = (uint32_t)total;
    gltf->channels = maniAllocate(&gltf->allocator, total + 1, sizeof(maniGltfChannel), 8);
    gltf->channelStarts = maniAllocate(&gltf->allocator, (size_t)count + 1, sizeof(uint32_t), 8);
    if (gltf->channels == NULL || gltf->channelStarts == NULL)
    {
        return Capacity();
    }
    maniArena arena = maniArenaMake(&gltf->allocator);
    uint32_t at = 0;
    for (uint32_t a = 0; r.result == mani_success && a < count; ++a)
    {
        gltf->channelStarts[a] = at;
        r = ReadAnimation(gltf, a, &arena, gltf->channels + at);
        if (r.result == mani_success)
        {
            uint32_t n = 0;
            (void)maniGltfAnimationChannels(gltf, a, &n);
            at += n;
        }
    }
    gltf->channelStarts[count] = at;
    maniArenaRelease(&arena);
    return r.result == mani_success ? ReadNames(gltf) : r;
}

void maniGltfReleaseAnimations(maniGltf* gltf)
{
    maniBoneNames* names = &gltf->animationNames;
    maniRelease(&gltf->allocator, names->text, names->textSize, 1, 8);
    maniRelease(&gltf->allocator, names->starts, (size_t)names->count + 1, sizeof(uint32_t), 8);
    maniRelease(&gltf->allocator, gltf->animationValues, (size_t)gltf->animationCount + 1,
                sizeof(uint32_t), 8);
    maniRelease(&gltf->allocator, gltf->samplerValues, (size_t)gltf->samplerTotal + 1,
                sizeof(uint32_t), 8);
    maniRelease(&gltf->allocator, gltf->samplerStarts, (size_t)gltf->animationCount + 1,
                sizeof(uint32_t), 8);
    maniRelease(&gltf->allocator, gltf->meshValues, (size_t)gltf->meshCount + 1, sizeof(uint32_t),
                8);
    maniRelease(&gltf->allocator, gltf->channels, (size_t)gltf->channelTotal + 1,
                sizeof(maniGltfChannel), 8);
    maniRelease(&gltf->allocator, gltf->channelStarts, (size_t)gltf->animationCount + 1,
                sizeof(uint32_t), 8);
}
