// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's meshes: primitives' joints and weights
// read into influences with what glTF forbids refused, and their morph
// targets read into morph target files: every attribute glTF morphs,
// positions, normals and tangents in the output's basis.

#include "arena.h"
#include "builder.h"
#include "decimal.h"
#include "fpenv.h"
#include "gltf_animation.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_draco_import.h"
#include "gltf_layout.h"
#include "gltf_property.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/morph.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// The most influence sets a primitive may have: MANI_INFLUENCE_MAX_GROUPS
// of four.
#define MAX_SETS 16u

static maniImportResult Fail(const maniGltf* gltf, maniImportReason reason, uint32_t value,
                             uint32_t item)
{
    return maniImportFail(reason, maniGltfOffset(&gltf->g, value), item);
}

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

// A builder's outcome as an import's: a refusal as Build, item the reason.
static maniImportResult Built(maniBuildResult b, const maniGltf* gltf, uint32_t value)
{
    if (b.result == mani_success || b.result == mani_errorCapacity)
    {
        return (maniImportResult){b.result, maniImportReasonNone, 0, 0};
    }
    return Fail(gltf, maniImportReasonBuild, value, (uint32_t)b.reason);
}

// The array's element at index, by walking it.
static uint32_t Element(const maniJson* json, uint32_t array, uint32_t index)
{
    uint32_t element = array + 1;
    for (uint32_t i = 0; i < index; ++i)
    {
        element = json->values[element].next;
    }
    return element;
}

// A mesh's primitives array, or MANI_JSON_NONE.
static uint32_t Primitives(const maniGltf* gltf, uint32_t mesh)
{
    if (mesh >= gltf->meshCount)
    {
        return MANI_JSON_NONE;
    }
    uint32_t p = maniJsonFind(&gltf->json, gltf->meshValues[mesh], "primitives", 10);
    return p != MANI_JSON_NONE && gltf->json.values[p].kind == maniJsonArray ? p : MANI_JSON_NONE;
}

// A primitive's object, or MANI_JSON_NONE.
static uint32_t Primitive(const maniGltf* gltf, uint32_t mesh, uint32_t primitive)
{
    uint32_t p = Primitives(gltf, mesh);
    if (p == MANI_JSON_NONE || primitive >= gltf->json.values[p].count)
    {
        return MANI_JSON_NONE;
    }
    uint32_t e = Element(&gltf->json, p, primitive);
    return gltf->json.values[e].kind == maniJsonObject ? e : MANI_JSON_NONE;
}

// A primitive's targets array, or MANI_JSON_NONE.
static uint32_t Targets(const maniGltf* gltf, uint32_t primitive)
{
    uint32_t t = maniJsonFind(&gltf->json, primitive, "targets", 7);
    return t != MANI_JSON_NONE && gltf->json.values[t].kind == maniJsonArray ? t : MANI_JSON_NONE;
}

uint32_t maniGltf_GetMeshCount(const maniGltf* gltf)
{
    return gltf->meshCount;
}

uint32_t maniGltf_GetPrimitiveCount(const maniGltf* gltf, uint32_t mesh)
{
    uint32_t p = Primitives(gltf, mesh);
    return p == MANI_JSON_NONE ? 0 : gltf->json.values[p].count;
}

uint32_t maniGltf_GetPrimitiveTargetCount(const maniGltf* gltf, uint32_t mesh, uint32_t primitive)
{
    uint32_t p = Primitive(gltf, mesh, primitive);
    uint32_t t = p == MANI_JSON_NONE ? MANI_JSON_NONE : Targets(gltf, p);
    return t == MANI_JSON_NONE ? 0 : gltf->json.values[t].count;
}

// A node's index member below count, or UINT32_MAX.
static uint32_t NodeIndex(const maniGltf* gltf, uint32_t node, const char* name, uint32_t count)
{
    int64_t index = -1;
    if (node >= gltf->nodeCount || count == 0 ||
        !maniJsonInteger(&gltf->json,
                         maniJsonFind(&gltf->json, gltf->nodeValues[node], name, strlen(name)), 0,
                         (int64_t)count - 1, &index))
    {
        return UINT32_MAX;
    }
    return (uint32_t)index;
}

uint32_t maniGltf_GetNodeMesh(const maniGltf* gltf, uint32_t node)
{
    return NodeIndex(gltf, node, "mesh", gltf->meshCount);
}

uint32_t maniGltf_GetNodeSkin(const maniGltf* gltf, uint32_t node)
{
    return NodeIndex(gltf, node, "skin", gltf->skinCount);
}

// count numbers of an array into out; false for another kind, another
// count or a value not a binary32.
static bool ReadNumbers(const maniJson* json, uint32_t array, uint32_t count, float* out)
{
    if (json->values[array].kind != maniJsonArray || json->values[array].count != count)
    {
        return false;
    }
    uint32_t e = array + 1;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint64_t bits = 0;
        if (!maniJsonBinary(json, e, maniBinary32, &bits))
        {
            return false;
        }
        uint32_t narrow = (uint32_t)bits;
        memcpy(&out[i], &narrow, sizeof narrow);
        e = json->values[e].next;
    }
    return true;
}

static MANI_NOINLINE maniImportResult GltfMorphWeightsWork(const maniGltf* gltf, uint32_t node,
                                                           float* weightsOut, uint32_t capacity,
                                                           uint32_t* countOut)
{
    uint32_t mesh = gltf == NULL ? UINT32_MAX : maniGltf_GetNodeMesh(gltf, node);
    uint32_t count = mesh == UINT32_MAX ? 0 : maniGltfMeshTargets(gltf, mesh);
    if ((weightsOut == NULL && capacity > 0) || count == 0)
    {
        return Invalid();
    }
    if (countOut != NULL)
    {
        *countOut = count;
    }
    if (weightsOut == NULL || capacity < count)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    const uint32_t owners[2] = {gltf->nodeValues[node], gltf->meshValues[mesh]};
    for (int k = 0; k < 2; ++k)
    {
        uint32_t weights = maniJsonFind(&gltf->json, owners[k], "weights", 7);
        if (weights != MANI_JSON_NONE)
        {
            return ReadNumbers(&gltf->json, weights, count, weightsOut)
                       ? maniImportSuccess()
                       : Fail(gltf, maniImportReasonProperty, weights, node);
        }
    }
    memset(weightsOut, 0, (size_t)count * sizeof(float));
    return maniImportSuccess();
}

maniImportResult maniGltf_GetMorphWeights(const maniGltf* gltf, uint32_t node, float* weightsOut,
                                          uint32_t capacity, uint32_t* countOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = GltfMorphWeightsWork(gltf, node, weightsOut, capacity, countOut);
    maniFloatEnvLeave(env);
    return result;
}

// The primitive's vertex count: its POSITION accessor's, or its first
// attribute's.
static maniImportResult VertexCount(const maniGltf* gltf, uint32_t attributes, uint32_t* countOut)
{
    *countOut = 0;
    uint32_t accessor = MANI_JSON_NONE;
    maniImportResult r =
        maniGltfIndex(&gltf->g, attributes, "POSITION", false, gltf->accessorCount, 0, &accessor);
    if (r.result == mani_success && accessor == MANI_JSON_NONE &&
        gltf->json.values[attributes].count > 0)
    {
        int64_t index = -1;
        if (!maniJsonInteger(&gltf->json, attributes + 2, 0, (int64_t)gltf->accessorCount - 1,
                             &index))
        {
            return Fail(gltf, maniImportReasonIndex, attributes + 2, 0);
        }
        accessor = (uint32_t)index;
    }
    if (r.result == mani_success && accessor == MANI_JSON_NONE)
    {
        return Fail(gltf, maniImportReasonProperty, attributes, 0);
    }
    *countOut = r.result == mani_success ? gltf->accessors[accessor].count : 0;
    return r;
}

// The accessor of an attribute: present (when required), with these rows
// and the vertex count.
static maniImportResult Accessor(const maniGltf* gltf, uint32_t attributes, const char* name,
                                 uint32_t rowsLow, uint32_t rowsHigh, uint32_t vertices,
                                 uint32_t* accessorOut)
{
    maniImportResult r =
        maniGltfIndex(&gltf->g, attributes, name, true, gltf->accessorCount, 0, accessorOut);
    if (r.result != mani_success)
    {
        return r;
    }
    const maniGltfAccessor* a = &gltf->accessors[*accessorOut];
    if (a->columns != 1 || a->rows < rowsLow || a->rows > rowsHigh || a->count != vertices)
    {
        return Fail(gltf, maniImportReasonProperty, a->value, *accessorOut);
    }
    return r;
}

// Influences: what the sets give, in the arena.
typedef struct Sets
{
    uint32_t count;
    uint32_t vertices;
    uint32_t joints[MAX_SETS];
    uint32_t weights[MAX_SETS];
    float* jointValues;
    float* weightValues;
} Sets;

// An indexed semantic's name: the prefix, then n in decimal; its length.
static size_t SetName(char* out, const char* prefix, uint32_t n)
{
    size_t length = strlen(prefix);
    memcpy(out, prefix, length);
    char digits[10];
    size_t count = 0;
    do
    {
        digits[count++] = (char)('0' + n % 10);
        n /= 10;
    } while (n != 0);
    for (size_t i = 0; i < count; ++i)
    {
        out[length++] = digits[count - 1 - i];
    }
    out[length] = '\0';
    return length;
}

// Set n's component types: joints u8 or u16, weights binary32 or
// normalized u8 or u16.
static maniImportResult SetTypes(const maniGltf* gltf, const Sets* s, uint32_t n)
{
    const maniGltfAccessor* a = &gltf->accessors[s->joints[n]];
    const maniGltfAccessor* b = &gltf->accessors[s->weights[n]];
    bool small = b->component == maniGltfUnsignedByte || b->component == maniGltfUnsignedShort;
    bool jointsOk =
        (a->component == maniGltfUnsignedByte || a->component == maniGltfUnsignedShort) &&
        !a->normalized;
    bool weightsOk = b->component == maniGltfFloat || (small && b->normalized);
    if (jointsOk && weightsOk)
    {
        return maniImportSuccess();
    }
    uint32_t bad = jointsOk ? s->weights[n] : s->joints[n];
    return Fail(gltf, maniImportReasonProperty, gltf->accessors[bad].value, bad);
}

// The JOINTS_n and WEIGHTS_n pairs from n = 0, each checked: joints u8 or
// u16, weights binary32 or normalized u8 or u16, VEC4, the vertex count.
static maniImportResult FindSets(const maniGltf* gltf, uint32_t attributes, Sets* s)
{
    maniImportResult r = VertexCount(gltf, attributes, &s->vertices);
    s->count = 0;
    for (uint32_t n = 0; r.result == mani_success && n <= MAX_SETS; ++n)
    {
        char joints[24];
        char weights[24];
        size_t jl = SetName(joints, "JOINTS_", n);
        size_t wl = SetName(weights, "WEIGHTS_", n);
        uint32_t j = maniJsonFind(&gltf->json, attributes, joints, jl);
        uint32_t w = maniJsonFind(&gltf->json, attributes, weights, wl);
        if (j == MANI_JSON_NONE && w == MANI_JSON_NONE)
        {
            break;
        }
        if (j == MANI_JSON_NONE || w == MANI_JSON_NONE || n == MAX_SETS)
        {
            return Fail(gltf, maniImportReasonProperty, j == MANI_JSON_NONE ? w : j, n);
        }
        r = Accessor(gltf, attributes, joints, 4, 4, s->vertices, &s->joints[n]);
        if (r.result == mani_success)
        {
            r = Accessor(gltf, attributes, weights, 4, 4, s->vertices, &s->weights[n]);
        }
        if (r.result != mani_success)
        {
            return r;
        }
        r = SetTypes(gltf, s, n);
        s->count = n + 1;
    }
    return r.result == mani_success && s->count == 0
               ? Fail(gltf, maniImportReasonProperty, attributes, 0)
               : r;
}

// A normalized weight back in units of 1/65535: a byte's value times 257
// (255 * 257 = 65535), a short's as it is.
static uint32_t Units(const maniGltfAccessor* a, float w)
{
    return a->component == maniGltfUnsignedByte ? 257u * (uint32_t)floorf(w * 255.0f + 0.5f)
                                                : (uint32_t)floorf(w * 65535.0f + 0.5f);
}

// One vertex's weighted influences into joints and weights (count out),
// glTF's rules checked.
static maniImportResult Gather(const maniGltf* gltf, const Sets* s, uint32_t v, uint32_t skinJoints,
                               uint32_t* joints, float* weights, uint32_t* countOut)
{
    uint32_t n = 0;
    uint64_t units = 0;
    bool normalized = true;
    for (uint32_t set = 0; set < s->count; ++set)
    {
        const maniGltfAccessor* a = &gltf->accessors[s->weights[set]];
        const float* j = s->jointValues + 4 * ((size_t)set * s->vertices + v);
        const float* w = s->weightValues + 4 * ((size_t)set * s->vertices + v);
        normalized = normalized && a->normalized;
        for (int k = 0; k < 4; ++k)
        {
            bool repeated = false;
            for (uint32_t i = 0; i < n && w[k] != 0.0f; ++i)
            {
                repeated = repeated || joints[i] == (uint32_t)j[k];
            }
            if (j[k] >= (float)skinJoints || w[k] < 0.0f || repeated)
            {
                return Fail(gltf, maniImportReasonInfluence, a->value, v);
            }
            units += a->normalized ? Units(a, w[k]) : 0u;
            if (w[k] != 0.0f)
            {
                joints[n] = (uint32_t)j[k];
                weights[n++] = w[k];
            }
        }
    }
    if (normalized && units != 65535u)
    {
        return Fail(gltf, maniImportReasonInfluence, gltf->accessors[s->weights[0]].value, v);
    }
    *countOut = n;
    return maniImportSuccess();
}

// Each set's joints and weights, from a Draco primitive's stream where
// the extension names them.
static maniImportResult ReadSets(const maniGltf* gltf, const maniGltfDraco* draco, maniArena* arena,
                                 Sets* s)
{
    size_t floats = (size_t)s->count * s->vertices * 4;
    s->jointValues = maniArenaTake(arena, floats, sizeof(float));
    s->weightValues = maniArenaTake(arena, floats, sizeof(float));
    if (s->jointValues == NULL || s->weightValues == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniImportSuccess();
    for (uint32_t set = 0; r.result == mani_success && set < s->count; ++set)
    {
        size_t at = (size_t)set * s->vertices * 4;
        char joints[24];
        char weights[24];
        size_t jl = SetName(joints, "JOINTS_", set);
        size_t wl = SetName(weights, "WEIGHTS_", set);
        r = maniGltfDracoFloats(gltf, draco, joints, jl, s->joints[set], s->jointValues + at);
        if (r.result == mani_success)
        {
            r = maniGltfDracoFloats(gltf, draco, weights, wl, s->weights[set],
                                    s->weightValues + at);
        }
    }
    return r;
}

static maniImportResult BuildInfluences(maniBuilder* builder, const maniGltf* gltf,
                                        uint32_t primitive, const maniSkin* skin, maniArena* arena)
{
    uint32_t attributes = MANI_JSON_NONE;
    maniImportResult r =
        maniGltfMember(&gltf->g, primitive, "attributes", maniJsonObject, true, 0, &attributes);
    Sets s = {0};
    if (r.result == mani_success)
    {
        r = FindSets(gltf, attributes, &s);
    }
    maniGltfDraco draco;
    if (r.result == mani_success)
    {
        r = maniGltfDracoDecode(gltf, arena, primitive, attributes, s.vertices, &draco);
    }
    if (r.result == mani_success)
    {
        r = ReadSets(gltf, &draco, arena, &s);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    size_t slots = (size_t)s.count * 4 * s.vertices;
    uint32_t* offsets = maniArenaTake(arena, (size_t)s.vertices + 1, sizeof(uint32_t));
    uint32_t* joints = maniArenaTake(arena, slots, sizeof(uint32_t));
    float* weights = maniArenaTake(arena, slots, sizeof(float));
    if (offsets == NULL || joints == NULL || weights == NULL)
    {
        return Capacity();
    }
    uint32_t total = 0;
    for (uint32_t v = 0; r.result == mani_success && v < s.vertices; ++v)
    {
        uint32_t n = 0;
        offsets[v] = total;
        r = Gather(gltf, &s, v, maniSkin_GetJointCount(skin), joints + total, weights + total, &n);
        total += n;
    }
    offsets[s.vertices] = total;
    if (r.result != mani_success)
    {
        return r;
    }
    maniInfluenceSource source = {skin, offsets, joints, weights, NULL, s.vertices};
    return Built(maniBuildInfluences(builder, &source), gltf, primitive);
}

static MANI_NOINLINE maniImportResult GltfBuildInfluencesWork(maniBuilder* builder,
                                                              const maniGltf* gltf, uint32_t mesh,
                                                              uint32_t primitive,
                                                              const maniSkin* skin)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorBusy, maniImportReasonNone, 0, 0};
    }
    uint32_t p = gltf == NULL ? MANI_JSON_NONE : Primitive(gltf, mesh, primitive);
    if (builder == NULL || skin == NULL || p == MANI_JSON_NONE)
    {
        return Invalid();
    }
    maniArena arena = maniArenaMake(&gltf->allocator);
    maniImportResult r = BuildInfluences(builder, gltf, p, skin, &arena);
    maniArenaRelease(&arena);
    return r;
}

maniImportResult maniBuildGltfInfluences(maniBuilder* builder, const maniGltf* gltf, uint32_t mesh,
                                         uint32_t primitive, const maniSkin* skin)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = GltfBuildInfluencesWork(builder, gltf, mesh, primitive, skin);
    maniFloatEnvLeave(env);
    return result;
}

// A morph attribute: its semantic and set, the member name it has in
// targets and attributes.
typedef struct Semantic
{
    maniMorphAttributeSource attribute;
    char name[24];
    size_t length;
} Semantic;

// n in decimal, no leading zeros, below 65536; false otherwise.
static bool SetIndex(const char* text, size_t length, uint32_t* setOut)
{
    uint32_t n = 0;
    if (length == 0 || length > 5 || (text[0] == '0' && length > 1))
    {
        return false;
    }
    for (size_t i = 0; i < length; ++i)
    {
        if (text[i] < '0' || text[i] > '9')
        {
            return false;
        }
        n = n * 10 + (uint32_t)(text[i] - '0');
    }
    *setOut = n;
    return n < 65536;
}

// A target member's semantic, if it is one glTF morphs; false for others
// (extension and application semantics, left).
static bool ParseSemantic(const maniJson* json, uint32_t key, Semantic* out)
{
    char text[24];
    if (json->values[key].length >= sizeof text)
    {
        return false;
    }
    size_t length = maniJsonStringDecode(json, key, (uint8_t*)text, sizeof text);
    static const char* const kFixed[3] = {"POSITION", "NORMAL", "TANGENT"};
    uint32_t set = 0;
    bool found = false;
    for (uint32_t k = 0; k < 3 && !found; ++k)
    {
        found = length == strlen(kFixed[k]) && memcmp(text, kFixed[k], length) == 0;
        out->attribute = (maniMorphAttributeSource){k, 0};
    }
    if (!found && length > 9 && memcmp(text, "TEXCOORD_", 9) == 0 &&
        SetIndex(text + 9, length - 9, &set))
    {
        found = true;
        out->attribute = (maniMorphAttributeSource){maniMorphTexcoord, set};
    }
    if (!found && length > 6 && memcmp(text, "COLOR_", 6) == 0 &&
        SetIndex(text + 6, length - 6, &set))
    {
        found = true;
        out->attribute = (maniMorphAttributeSource){maniMorphColor, set};
    }
    memcpy(out->name, text, length);
    out->name[length] = '\0';
    out->length = length;
    return found;
}

// The morph import's state.
typedef struct Morph
{
    const maniGltf* gltf;
    const maniGltfOptions* options;
    maniArena* arena;
    uint32_t primitive;
    uint32_t attributes;
    uint32_t targets;
    uint32_t targetCount;
    uint32_t vertices;
    Semantic table[MANI_MAX_MORPH_ATTRIBUTES];
    maniMorphAttributeSource attributeSources[MANI_MAX_MORPH_ATTRIBUTES];
    uint32_t attributeCount;
    // Each attribute's deltas for every vertex and the moved vertices,
    // read into once a target and kept for the next.
    float* full[MANI_MAX_MORPH_ATTRIBUTES];
    uint32_t* moved;
} Morph;

// One target's semantics into the table: each the primitive has, the
// table at most 32.
static maniImportResult TableTarget(Morph* m, uint32_t target)
{
    const maniJson* json = &m->gltf->json;
    uint32_t key = target + 1;
    for (uint32_t i = 0; i < json->values[target].count; ++i)
    {
        Semantic semantic;
        if (ParseSemantic(json, key, &semantic))
        {
            if (maniJsonFind(json, m->attributes, semantic.name, semantic.length) == MANI_JSON_NONE)
            {
                return Fail(m->gltf, maniImportReasonProperty, key, i);
            }
            uint32_t at = 0;
            while (at < m->attributeCount &&
                   (m->table[at].attribute.semantic != semantic.attribute.semantic ||
                    m->table[at].attribute.set != semantic.attribute.set))
            {
                ++at;
            }
            if (at == MANI_MAX_MORPH_ATTRIBUTES)
            {
                return Fail(m->gltf, maniImportReasonProperty, key, i);
            }
            m->table[at] = semantic;
            m->attributeSources[at] = semantic.attribute;
            m->attributeCount += at == m->attributeCount ? 1 : 0;
        }
        key = json->values[key + 1].next;
    }
    return maniImportSuccess();
}

static maniImportResult Table(Morph* m)
{
    const maniJson* json = &m->gltf->json;
    uint32_t target = m->targets + 1;
    m->attributeCount = 0;
    for (uint32_t t = 0; t < m->targetCount; ++t)
    {
        if (json->values[target].kind != maniJsonObject)
        {
            return Fail(m->gltf, maniImportReasonProperty, target, t);
        }
        maniImportResult r = TableTarget(m, target);
        if (r.result != mani_success)
        {
            return r;
        }
        target = json->values[target].next;
    }
    return maniImportSuccess();
}

static uint32_t Width(uint32_t semantic)
{
    return semantic == maniMorphTexcoord ? 2u : (semantic == maniMorphColor ? 4u : 3u);
}

// One attribute of one target read: its deltas, width floats a vertex,
// in the output's basis (colors of three given alpha 0); NULL out when
// the target leaves it.
static maniImportResult ReadAttribute(Morph* m, uint32_t target, uint32_t a, float** out)
{
    *out = NULL;
    const Semantic* s = &m->table[a];
    if (maniJsonFind(&m->gltf->json, target, s->name, s->length) == MANI_JSON_NONE)
    {
        return maniImportSuccess();
    }
    uint32_t semantic = s->attribute.semantic;
    uint32_t width = Width(semantic);
    uint32_t low = semantic == maniMorphColor ? 3u : width;
    uint32_t accessor = 0;
    maniImportResult r = Accessor(m->gltf, target, s->name, low, width, m->vertices, &accessor);
    if (r.result == mani_success && m->full[a] == NULL)
    {
        m->full[a] = maniArenaTake(m->arena, (size_t)m->vertices * 4, 4);
    }
    float* raw = r.result == mani_success ? m->full[a] : NULL;
    if (r.result == mani_success && raw == NULL)
    {
        return Capacity();
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadFloats(m->gltf, accessor, raw);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t rows = m->gltf->accessors[accessor].rows;
    // Widen colors of three in place, from the last vertex down.
    for (uint32_t v = m->vertices; rows == 3 && width == 4 && v-- > 0;)
    {
        const float rgb[3] = {raw[3 * v], raw[3 * v + 1], raw[3 * v + 2]};
        memcpy(raw + 4 * (size_t)v, rgb, sizeof rgb);
        raw[4 * (size_t)v + 3] = 0.0f;
    }
    if (semantic <= maniMorphTangent)
    {
        maniGltfBasisVectors(m->options, raw, m->vertices, true);
    }
    *out = raw;
    return maniImportSuccess();
}

// Each vertex's word in any made non-zero where its width floats hold a
// delta not zero (a NaN counts, either zero does not), as `!= 0.0f`
// tests them; a width constant in each loop, so each vectorizes.
static void Moves(uint32_t* any, const float* full, uint32_t vertices, uint32_t width)
{
    const uint32_t mask = 0x7FFFFFFFu;
    for (uint32_t v = 0; v < vertices; ++v)
    {
        uint32_t u[4] = {0, 0, 0, 0};
        switch (width)
        {
        case 2:
            memcpy(u, full + 2 * (size_t)v, 2 * sizeof(float));
            break;
        case 3:
            memcpy(u, full + 3 * (size_t)v, 3 * sizeof(float));
            break;
        default:
            memcpy(u, full + 4 * (size_t)v, 4 * sizeof(float));
            break;
        }
        any[v] |= (u[0] | u[1] | u[2] | u[3]) & mask;
    }
}

// The kept rows of width floats copied out in order.
static void Rows(float* out, const float* full, const uint32_t* indices, uint32_t kept,
                 uint32_t width)
{
    for (uint32_t i = 0; i < kept; ++i)
    {
        const float* row = full + width * (size_t)indices[i];
        float* to = out + width * (size_t)i;
        switch (width)
        {
        case 2:
            memcpy(to, row, 2 * sizeof(float));
            break;
        case 3:
            memcpy(to, row, 3 * sizeof(float));
            break;
        default:
            memcpy(to, row, 4 * sizeof(float));
            break;
        }
    }
}

// The vertices a target moves (a delta not zero) and their deltas packed.
static maniImportResult Pack(Morph* m, float* const* full, maniMorphTargetSource* out)
{
    if (m->moved == NULL)
    {
        m->moved = maniArenaTake(m->arena, (size_t)m->vertices + 1, sizeof(uint32_t));
    }
    const float** deltas =
        (const float**)maniArenaTake(m->arena, MANI_MAX_MORPH_ATTRIBUTES, sizeof(void*));
    if (m->moved == NULL || deltas == NULL)
    {
        return Capacity();
    }
    memset(m->moved, 0, (size_t)m->vertices * sizeof(uint32_t));
    for (uint32_t a = 0; a < m->attributeCount; ++a)
    {
        if (full[a] != NULL)
        {
            Moves(m->moved, full[a], m->vertices, Width(m->table[a].attribute.semantic));
        }
    }
    // The moved vertices' indices over the words, in place: the index
    // written never passes the word read.
    uint32_t kept = 0;
    for (uint32_t v = 0; v < m->vertices; ++v)
    {
        uint32_t moves = m->moved[v];
        m->moved[kept] = v;
        kept += moves != 0 ? 1 : 0;
    }
    uint32_t* indices = maniArenaTake(m->arena, (size_t)kept + 1, sizeof(uint32_t));
    if (indices == NULL)
    {
        return Capacity();
    }
    memcpy(indices, m->moved, (size_t)kept * sizeof(uint32_t));
    for (uint32_t a = 0; a < m->attributeCount; ++a)
    {
        uint32_t width = Width(m->table[a].attribute.semantic);
        float* packed = NULL;
        if (full[a] != NULL)
        {
            packed = maniArenaTake(m->arena, (size_t)kept * width + 1, sizeof(float));
            if (packed == NULL)
            {
                return Capacity();
            }
            Rows(packed, full[a], indices, kept, width);
        }
        deltas[a] = packed;
    }
    *out = (maniMorphTargetSource){1.0f, kept, indices, (const float* const*)deltas, NULL, 0};
    return maniImportSuccess();
}

static maniImportResult ReadTargets(Morph* m, maniMorphTargetSource* sources)
{
    const maniJson* json = &m->gltf->json;
    uint32_t target = m->targets + 1;
    maniImportResult r = maniImportSuccess();
    for (uint32_t t = 0; r.result == mani_success && t < m->targetCount; ++t)
    {
        float* full[MANI_MAX_MORPH_ATTRIBUTES];
        for (uint32_t a = 0; r.result == mani_success && a < m->attributeCount; ++a)
        {
            r = ReadAttribute(m, target, a, &full[a]);
        }
        if (r.result == mani_success)
        {
            r = Pack(m, full, &sources[t]);
        }
        target = json->values[target].next;
    }
    return r;
}

// A channel a target, named by targetNames or the index.
static maniImportResult Channels(Morph* m, uint32_t mesh, const maniMorphTargetSource* sources,
                                 maniMorphChannelSource* channels)
{
    const maniJson* json = &m->gltf->json;
    uint32_t names = maniGltfTargetNames(m->gltf, mesh, m->targetCount);
    uint32_t element = names == MANI_JSON_NONE ? MANI_JSON_NONE : names + 1;
    for (uint32_t t = 0; t < m->targetCount; ++t)
    {
        size_t capacity = element == MANI_JSON_NONE ? 10 : json->values[element].length;
        char* name = maniArenaTake(m->arena, capacity + 1, 1);
        if (name == NULL)
        {
            return Capacity();
        }
        size_t length = 0;
        if (element != MANI_JSON_NONE)
        {
            length = maniJsonStringDecode(json, element, (uint8_t*)name, capacity);
            element = json->values[element].next;
        }
        else
        {
            char digits[10];
            uint32_t n = t;
            do
            {
                digits[length++] = (char)('0' + n % 10);
                n /= 10;
            } while (n != 0);
            for (size_t i = 0; i < length; ++i)
            {
                name[i] = digits[length - 1 - i];
            }
        }
        channels[t] = (maniMorphChannelSource){name, length, 1, &sources[t]};
    }
    return maniImportSuccess();
}

// A Draco primitive's targets are plain accessors in its decoded vertex
// order, read as any other.
static maniImportResult BuildMorph(maniBuilder* builder, Morph* m, uint32_t mesh)
{
    maniImportResult r = maniGltfMember(&m->gltf->g, m->primitive, "attributes", maniJsonObject,
                                        true, 0, &m->attributes);
    if (r.result == mani_success)
    {
        r = VertexCount(m->gltf, m->attributes, &m->vertices);
    }
    if (r.result == mani_success)
    {
        r = Table(m);
    }
    maniMorphTargetSource* sources =
        maniArenaTake(m->arena, m->targetCount, sizeof(maniMorphTargetSource));
    maniMorphChannelSource* channels =
        maniArenaTake(m->arena, m->targetCount, sizeof(maniMorphChannelSource));
    if (r.result == mani_success && (sources == NULL || channels == NULL))
    {
        return Capacity();
    }
    if (r.result == mani_success)
    {
        r = ReadTargets(m, sources);
    }
    if (r.result == mani_success)
    {
        r = Channels(m, mesh, sources, channels);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    maniMorphSource source = {channels, m->targetCount, m->vertices, m->attributeSources,
                              m->attributeCount};
    return Built(maniBuildMorphTargets(builder, &source), m->gltf, m->targets);
}

static MANI_NOINLINE maniImportResult GltfBuildMorphTargetsWork(maniBuilder* builder,
                                                                const maniGltf* gltf, uint32_t mesh,
                                                                uint32_t primitive,
                                                                const maniGltfOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorBusy, maniImportReasonNone, 0, 0};
    }
    uint32_t p = gltf == NULL ? MANI_JSON_NONE : Primitive(gltf, mesh, primitive);
    uint32_t targets = p == MANI_JSON_NONE ? MANI_JSON_NONE : Targets(gltf, p);
    if (builder == NULL || options == NULL || targets == MANI_JSON_NONE ||
        gltf->json.values[targets].count == 0 || !maniGltfOptionsValid(options))
    {
        return Invalid();
    }
    maniArena arena = maniArenaMake(&gltf->allocator);
    Morph m = {.gltf = gltf,
               .options = options,
               .arena = &arena,
               .primitive = p,
               .targets = targets,
               .targetCount = gltf->json.values[targets].count};
    maniImportResult r = BuildMorph(builder, &m, mesh);
    maniArenaRelease(&arena);
    return r;
}

maniImportResult maniBuildGltfMorphTargets(maniBuilder* builder, const maniGltf* gltf,
                                           uint32_t mesh, uint32_t primitive,
                                           const maniGltfOptions* options)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = GltfBuildMorphTargetsWork(builder, gltf, mesh, primitive, options);
    maniFloatEnvLeave(env);
    return result;
}
