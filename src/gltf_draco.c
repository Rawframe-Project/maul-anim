// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco bitstream 2.2's mesh decoding, the specification's order with
// Google's decoder deciding where they differ: the header
// and metadata, the connectivity (sequential, or Edgebreaker's), then
// each attribute decoder's attributes, over Edgebreaker in its
// traversal's order: their portable values (raw, or symbols with a
// prediction reverted), the data their transforms need, and the
// transform to their own type. Integer arithmetic is in 32-bit two's complement without
// overflow (uint32_t, or int64_t where a sum is compared), so any input
// is defined; floats are binary32 in Google's order.

#include "gltf_draco.h"

#include "arena.h"
#include "bytes.h"
#include "gltf_draco_edgebreaker.h"
#include "gltf_draco_predict.h"
#include "gltf_draco_read.h"
#include "gltf_draco_transform.h"
#include "gltf_draco_traverse.h"
#include "invariant.h"
#include "sort.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define METADATA_FLAG      0x8000u
#define MAX_METADATA_LEVEL 1000u

enum
{
    Sequential = 0,
    Edgebreaker = 1,
};

enum
{
    DecoderGeneric = 0,
    DecoderInteger = 1,
    DecoderQuantization = 2,
    DecoderNormals = 3,
};

enum
{
    PredictionNone = -2,
    PredictionDifference = 0,
};

// An Edgebreaker attribute decoder: the table its traversal walks (an
// attribute data's for a corner attribute, else the mesh's), how long its
// vertex map is, the traversal; then its values' order and each point's
// value.
typedef struct Group
{
    const maniDracoTable* table;
    uint32_t mapSize;
    uint8_t traversal;
    maniDracoOrder order;
    uint32_t* pointToValue;
} Group;

struct Value;

typedef struct Decoder
{
    maniDracoReader r;
    maniArena* arena;
    uint32_t maxPoints;
    uint32_t maxFaces;
    maniDracoMesh* mesh;
    uint8_t method;
    // Edgebreaker's connectivity, its tables for the attributes; its
    // attribute decoders; the first position (texture coordinates' and
    // normals' parent).
    maniDracoEdgebreaker eb;
    Group* groups;
    struct Value* position;
} Decoder;

// An attribute while it decodes: its sequential decoder, its portable
// values (pcomps a value, two's complement), and its transforms' data.
typedef struct Value
{
    maniDracoAttribute* a;
    // Its values, and their portable form decoded (a parent may be read).
    uint32_t entries;
    bool ready;
    uint8_t decoder;
    int8_t scheme;
    maniDracoTransform xf;
    uint32_t pcomps;
    uint32_t* portable;
    float minimum[MANI_DRACO_MAX_COMPONENTS];
    float range;
    uint8_t bits;
} Value;

uint32_t maniDracoTypeSize(uint8_t dataType)
{
    static const uint8_t kSizes[12] = {0, 1, 1, 2, 2, 4, 4, 8, 8, 4, 8, 1};
    return dataType < 12 ? kSizes[dataType] : 0;
}

static int32_t Signed(uint32_t u)
{
    return u <= INT32_MAX ? (int32_t)u : -(int32_t)(~u) - 1;
}

// A metadata name: its length byte and bytes.
static bool Name(maniDracoReader* r, const uint8_t** name, uint8_t* length)
{
    return maniDracoU8(r, length) && maniDracoBytes(r, *length, name);
}

typedef struct Names
{
    uint64_t* keys;
    const uint8_t** names;
    uint8_t* lengths;
    uint32_t* owners;
    uint32_t count;
    uint32_t capacity;
} Names;

static bool AddName(Names* n, uint32_t owner, const uint8_t* name, uint8_t length)
{
    if (n->count == n->capacity)
    {
        return false;
    }
    // FNV-1a of the owner and the name.
    uint64_t h = 0xcbf29ce484222325ull ^ owner;
    for (uint8_t i = 0; i < length; ++i)
    {
        h = (h ^ name[i]) * 0x100000001b3ull;
    }
    n->keys[n->count] = h;
    n->names[n->count] = name;
    n->lengths[n->count] = length;
    n->owners[n->count++] = owner;
    return true;
}

// Whether two names of one element are the same, among equal keys.
static maniDracoStatus Unique(maniArena* arena, const Names* n)
{
    uint32_t* sorted = maniArenaTake(arena, n->count, sizeof(uint32_t));
    uint32_t* scratch = maniArenaTake(arena, n->count, sizeof(uint32_t));
    if (sorted == NULL || scratch == NULL)
    {
        return maniDracoCapacity;
    }
    maniSortByKey(n->keys, n->count, sorted, scratch);
    for (uint32_t i = 0; i < n->count; ++i)
    {
        for (uint32_t j = i + 1; j < n->count && n->keys[sorted[j]] == n->keys[sorted[i]]; ++j)
        {
            uint32_t a = sorted[i];
            uint32_t b = sorted[j];
            if (n->owners[a] == n->owners[b] && n->lengths[a] == n->lengths[b] &&
                memcmp(n->names[a], n->names[b], n->lengths[a]) == 0)
            {
                return maniDracoInvalid;
            }
        }
    }
    return maniDracoDone;
}

// One element's entries (names unique) and its count of sub-elements;
// entries and sub-elements are named apart (owners 2e and 2e + 1).
static bool Element(maniDracoReader* r, Names* names, uint32_t owner, uint32_t* subs)
{
    uint32_t entries = 0;
    if (!maniDracoLeb32(r, &entries))
    {
        return false;
    }
    for (uint32_t i = 0; i < entries; ++i)
    {
        const uint8_t* name = NULL;
        uint8_t length = 0;
        uint32_t size = 0;
        const uint8_t* value = NULL;
        if (!Name(r, &name, &length) || !AddName(names, 2u * owner, name, length) ||
            !maniDracoLeb32(r, &size) || size == 0 || !maniDracoBytes(r, size, &value))
        {
            return false;
        }
    }
    return maniDracoLeb32(r, subs) && *subs <= maniDracoLeft(r);
}

// The metadata, skipped once checked: the attributes' elements, then the
// file's, each element's sub-elements depth first (as Google reads
// them), at most 1,000 levels deep, names unique in each element.
static maniDracoStatus Metadata(Decoder* d)
{
    maniDracoReader* r = &d->r;
    uint32_t attributes = 0;
    // Every element and every name costs a byte at least.
    size_t most = maniDracoLeft(r) + 2u;
    uint32_t* stack = maniArenaTake(d->arena, most, sizeof(uint32_t) * 2u);
    Names names = {maniArenaTake(d->arena, most, sizeof(uint64_t)),
                   (const uint8_t**)maniArenaTake(d->arena, most, sizeof(const uint8_t*)),
                   maniArenaTake(d->arena, most, 1),
                   maniArenaTake(d->arena, most, sizeof(uint32_t)),
                   0,
                   (uint32_t)(most < UINT32_MAX ? most : UINT32_MAX)};
    if (stack == NULL || names.keys == NULL || names.names == NULL || names.lengths == NULL ||
        names.owners == NULL)
    {
        return maniDracoCapacity;
    }
    if (!maniDracoLeb32(r, &attributes) || attributes > maniDracoLeft(r))
    {
        return maniDracoInvalid;
    }
    uint32_t elements = 0;
    for (uint32_t i = 0; i <= attributes; ++i)
    {
        uint32_t id = 0;
        size_t top = 0;
        if (i < attributes && !maniDracoLeb32(r, &id))
        {
            return maniDracoInvalid;
        }
        // An entry: (level, parent element) of an element still to read;
        // the root has no name.
        stack[top++] = 0;
        stack[top++] = UINT32_MAX;
        while (top > 0)
        {
            uint32_t parent = stack[--top];
            uint32_t level = stack[--top];
            const uint8_t* name = NULL;
            uint8_t length = 0;
            uint32_t self = elements++;
            uint32_t subs = 0;
            if ((parent != UINT32_MAX && (level > MAX_METADATA_LEVEL || !Name(r, &name, &length) ||
                                          !AddName(&names, 2u * parent + 1u, name, length))) ||
                !Element(r, &names, self, &subs) || subs > (2u * most - top) / 2u)
            {
                return maniDracoInvalid;
            }
            for (uint32_t k = 0; k < subs; ++k)
            {
                stack[top++] = parent == UINT32_MAX ? level : level + 1u;
                stack[top++] = self;
            }
        }
    }
    return Unique(d->arena, &names);
}

static maniDracoStatus Header(Decoder* d, uint16_t* flags)
{
    const uint8_t* magic = NULL;
    uint8_t version[2] = {0};
    uint8_t type = 0;
    if (!maniDracoBytes(&d->r, 5, &magic) || memcmp(magic, "DRACO", 5) != 0 ||
        !maniDracoU8(&d->r, &version[0]) || !maniDracoU8(&d->r, &version[1]) ||
        !maniDracoU8(&d->r, &type) || !maniDracoU8(&d->r, &d->method) ||
        !maniDracoU16(&d->r, flags))
    {
        return maniDracoInvalid;
    }
    // Bitstream 2.2 (the extension's), a mesh, sequential or Edgebreaker.
    return version[0] == 2 && version[1] == 2 && type == 1 && d->method <= Edgebreaker
               ? maniDracoDone
               : maniDracoInvalid;
}

// Sequential indices stored in 1, 2, a LEB128 of up to 4, or 4 bytes.
static bool PlainIndex(maniDracoReader* r, uint32_t points, uint32_t* out)
{
    uint8_t u8 = 0;
    uint16_t u16 = 0;
    bool read = points < 256u         ? maniDracoU8(r, &u8)
                : points < (1u << 16) ? maniDracoU16(r, &u16)
                : points < (1u << 21) ? maniDracoLeb32(r, out)
                                      : maniDracoU32(r, out);
    *out = points < 256u ? u8 : (points < (1u << 16) ? u16 : *out);
    return read && *out < points;
}

// Compressed sequential indices: symbols of zigzag deltas.
static maniDracoStatus DeltaIndices(Decoder* d, uint32_t* faces, uint32_t count, uint32_t points)
{
    maniDracoStatus s = maniDracoSymbols(&d->r, d->arena, count, 1, faces);
    int64_t last = 0;
    for (uint32_t i = 0; s == maniDracoDone && i < count; ++i)
    {
        int64_t diff = (int64_t)(faces[i] >> 1);
        last += (faces[i] & 1u) != 0 ? -diff : diff;
        if (last < 0 || last >= (int64_t)points || last > INT32_MAX)
        {
            return maniDracoInvalid;
        }
        faces[i] = (uint32_t)last;
    }
    return s;
}

static maniDracoStatus SequentialConnectivity(Decoder* d)
{
    maniDracoMesh* m = d->mesh;
    uint8_t method = 0;
    if (!maniDracoLeb32(&d->r, &m->faceCount) || !maniDracoLeb32(&d->r, &m->pointCount) ||
        m->faceCount > UINT32_MAX / 3u || m->faceCount > maniDracoLeft(&d->r) / 3u ||
        m->faceCount > d->maxFaces || m->pointCount > d->maxPoints || !maniDracoU8(&d->r, &method))
    {
        return maniDracoInvalid;
    }
    uint32_t count = m->faceCount * 3u;
    m->faces = maniArenaTake(d->arena, count, sizeof(uint32_t));
    if (m->faces == NULL)
    {
        return maniDracoCapacity;
    }
    if (method == 0)
    {
        return DeltaIndices(d, m->faces, count, m->pointCount);
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!PlainIndex(&d->r, m->pointCount, &m->faces[i]))
        {
            return maniDracoInvalid;
        }
    }
    return maniDracoDone;
}

// Edgebreaker: the faces are the corners' points.
static maniDracoStatus EdgebreakerConnectivity(Decoder* d)
{
    maniDracoStatus s =
        maniDracoReadEdgebreaker(&d->r, d->arena, d->maxPoints, d->maxFaces, &d->eb);
    if (s == maniDracoDone)
    {
        d->mesh->faceCount = d->eb.mesh.corners / 3u;
        d->mesh->pointCount = d->eb.pointCount;
        d->mesh->faces = d->eb.cornerPoint;
    }
    return s;
}

// One attribute's description and its sequential decoder's type, as the
// decoder it names may take it.
static bool Describe(maniDracoReader* r, maniDracoAttribute* a)
{
    uint8_t normalized = 0;
    bool read = maniDracoU8(r, &a->type) && maniDracoU8(r, &a->dataType) &&
                maniDracoU8(r, &a->components) && maniDracoU8(r, &normalized);
    a->normalized = normalized != 0;
    return read && a->type <= maniDracoGeneric && maniDracoTypeSize(a->dataType) != 0 &&
           a->components != 0 && a->components <= MANI_DRACO_MAX_COMPONENTS &&
           maniDracoLeb32(r, &a->uniqueId);
}

static bool Takes(const Value* v)
{
    const maniDracoAttribute* a = v->a;
    switch (v->decoder)
    {
    case DecoderGeneric:
        return true;
    case DecoderInteger:
        return a->dataType <= maniDracoUInt32;
    case DecoderQuantization:
        return a->dataType == maniDracoFloat32;
    case DecoderNormals:
        return a->dataType == maniDracoFloat32 && a->components == 3;
    default:
        return false;
    }
}

// The attributes one decoder holds.
static maniDracoStatus Describes(Decoder* d, Value** out, uint32_t* countOut)
{
    // Each attribute takes at least six bytes (its description and its
    // decoder's type), a tighter bound than Google's five per byte left.
    uint32_t count = 0;
    if (!maniDracoLeb32(&d->r, &count) || count == 0 || count > maniDracoLeft(&d->r) / 6u)
    {
        return maniDracoInvalid;
    }
    Value* values = maniArenaTake(d->arena, count, sizeof(Value));
    maniDracoAttribute* attributes = maniArenaTake(d->arena, count, sizeof(maniDracoAttribute));
    if (values == NULL || attributes == NULL)
    {
        return maniDracoCapacity;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        values[i].a = &attributes[i];
        if (!Describe(&d->r, &attributes[i]))
        {
            return maniDracoInvalid;
        }
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!maniDracoU8(&d->r, &values[i].decoder) || !Takes(&values[i]))
        {
            return maniDracoInvalid;
        }
        values[i].pcomps = values[i].decoder == DecoderNormals ? 2u : attributes[i].components;
    }
    *out = values;
    *countOut = count;
    return maniDracoDone;
}

// A symbol back to a signed value: zigzag.
static uint32_t Unzigzag(uint32_t v)
{
    return (v & 1u) != 0 ? ~(v >> 1) : (v >> 1);
}

// The integer values: symbols, or raw of 0 to 4 bytes each.
static maniDracoStatus Integers(Decoder* d, Value* v, uint32_t count)
{
    uint8_t compressed = 0;
    if (!maniDracoU8(&d->r, &compressed))
    {
        return maniDracoInvalid;
    }
    if (compressed > 0)
    {
        return maniDracoSymbols(&d->r, d->arena, count, v->pcomps, v->portable);
    }
    uint8_t width = 0;
    const uint8_t* raw = NULL;
    if (!maniDracoU8(&d->r, &width) || width > 4 ||
        !maniDracoBytes(&d->r, (size_t)count * width, &raw))
    {
        return maniDracoInvalid;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t x = 0;
        for (uint32_t k = 0; k < width; ++k)
        {
            x |= (uint32_t)raw[(size_t)i * width + k] << (8u * k);
        }
        v->portable[i] = x;
    }
    return maniDracoDone;
}

// The first position as a prediction's parent: decoded before v, of
// three portable components (Google's SetParentAttribute).
static bool Parent(const Decoder* d, const Value* v)
{
    const Value* p = d->position;
    return p != NULL && p != v && p->ready && p->pcomps == 3;
}

// The schemes the library reads: difference everywhere;
// over Edgebreaker, the parallelograms and texture coordinates with wrap,
// geometric normals with the octahedron, each with its transform.
static bool Accepts(const Decoder* d, const Value* v, const Group* g, uint8_t transform)
{
    bool normals = v->decoder == DecoderNormals;
    if (transform != (normals ? maniDracoOctahedron : maniDracoWrap))
    {
        return false;
    }
    switch (v->scheme)
    {
    case PredictionDifference:
        return true;
    case maniDracoParallelogram:
    case maniDracoMultiParallelogram:
    case maniDracoConstrained:
        return g != NULL && !normals;
    case maniDracoTexCoords:
        return g != NULL && !normals && v->pcomps == 2 && Parent(d, v);
    case maniDracoGeometricNormal:
        return g != NULL && normals && Parent(d, v);
    default:
        return false;
    }
}

static void Predictor(const Decoder* d, const Value* v, const Group* g, maniDracoPredictor* p)
{
    // Accepts takes a mesh prediction over Edgebreaker only.
    MANI_ASSERT(g != NULL);
    *p = (maniDracoPredictor){0};
    p->scheme = (uint8_t)v->scheme;
    p->components = v->pcomps;
    p->table = g->table;
    p->order = &g->order;
    p->transform = maniDracoTransformApply;
    p->context = &v->xf;
    p->pointCount = d->mesh->pointCount;
    if (Parent(d, v))
    {
        // A parent is decoded, over Edgebreaker, so it has its values and
        // its group's point map.
        MANI_ASSERT(d->position->portable != NULL && d->position->a->pointToValue != NULL);
        p->positions = d->position->portable;
        p->positionOfPoint = d->position->a->pointToValue;
        p->positionCount = d->position->entries;
    }
}

// The prediction's data, the transform's, then the prediction reverted.
static maniDracoStatus Revert(Decoder* d, Value* v, const Group* g, uint32_t entries)
{
    maniDracoPredictor p;
    bool mesh = v->scheme != PredictionDifference;
    if (mesh)
    {
        Predictor(d, v, g, &p);
        maniDracoStatus s = maniDracoPredictData(&d->r, d->arena, &p, entries * v->pcomps);
        if (s != maniDracoDone)
        {
            return s;
        }
    }
    if (!maniDracoTransformRead(&d->r, &v->xf) || (mesh && !maniDracoPredictFlips(&d->r, &p)))
    {
        return maniDracoInvalid;
    }
    if (!mesh)
    {
        maniDracoTransformDifference(&v->xf, v->portable, entries);
        return maniDracoDone;
    }
    p.octahedronMax = v->xf.octahedronMax;
    return maniDracoPredictRevert(&p, v->portable, entries) ? maniDracoDone : maniDracoInvalid;
}

// An integer-based attribute's portable values: its prediction, the
// values, signed unless the octahedron's, and the prediction reverted.
static maniDracoStatus Portable(Decoder* d, Value* v, const Group* g, uint32_t entries)
{
    uint8_t scheme = 0;
    uint8_t transform = 0;
    if (!maniDracoU8(&d->r, &scheme))
    {
        return maniDracoInvalid;
    }
    v->scheme = (int8_t)scheme;
    if (v->scheme != PredictionNone &&
        (!maniDracoU8(&d->r, &transform) || !Accepts(d, v, g, transform)))
    {
        return maniDracoInvalid;
    }
    v->xf.type = (int8_t)transform;
    v->xf.components = v->pcomps;
    uint32_t count = entries * v->pcomps;
    v->portable = maniArenaTake(d->arena, count, sizeof(uint32_t));
    if (v->portable == NULL)
    {
        return maniDracoCapacity;
    }
    maniDracoStatus s = Integers(d, v, count);
    bool positive = v->scheme != PredictionNone && v->xf.type == maniDracoOctahedron;
    for (uint32_t i = 0; s == maniDracoDone && !positive && i < count; ++i)
    {
        v->portable[i] = Unzigzag(v->portable[i]);
    }
    if (s == maniDracoDone && v->scheme != PredictionNone)
    {
        s = Revert(d, v, g, entries);
    }
    v->ready = s == maniDracoDone;
    return s;
}

// What the transforms need, after the portable values: a quantization's
// minimum, range and bits (1 to 30), or a normal's bits (2 to 30).
static bool TransformNeeds(maniDracoReader* r, Value* v)
{
    if (v->decoder == DecoderQuantization)
    {
        for (uint32_t c = 0; c < v->a->components; ++c)
        {
            uint32_t bits = 0;
            if (!maniDracoU32(r, &bits))
            {
                return false;
            }
            memcpy(&v->minimum[c], &bits, sizeof bits);
        }
        uint32_t range = 0;
        if (!maniDracoU32(r, &range) || !maniDracoU8(r, &v->bits))
        {
            return false;
        }
        memcpy(&v->range, &range, sizeof range);
        return v->bits >= 1 && v->bits <= 30;
    }
    if (v->decoder == DecoderNormals)
    {
        return maniDracoU8(r, &v->bits) && v->bits >= 2 && v->bits <= 30;
    }
    return true;
}

static void StoreFloat(uint8_t* p, float f)
{
    uint32_t bits = 0;
    memcpy(&bits, &f, sizeof bits);
    maniStoreU32(p, bits);
}

// A unit vector from quantized octahedral coordinates (Google's).
static void UnitVector(const Value* v, int32_t qs, int32_t qt, uint8_t* out)
{
    // TransformNeeds refused a normal's bits outside 2 to 30.
    MANI_ASSERT(v->bits >= 2 && v->bits <= 30);
    int32_t maxValue = (int32_t)((1u << v->bits) - 1u) - 1;
    float scale = 2.0f / (float)maxValue;
    float y = (float)qs * scale - 1.0f;
    float z = (float)qt * scale - 1.0f;
    float x = 1.0f - fabsf(y) - fabsf(z);
    float offset = -x < 0 ? 0 : -x;
    y += y < 0 ? offset : -offset;
    z += z < 0 ? offset : -offset;
    float norm = x * x + y * y + z * z;
    float r[3] = {0, 0, 0};
    if (!((double)norm < 1e-6))
    {
        float inv = 1.0f / sqrtf(norm);
        r[0] = x * inv;
        r[1] = y * inv;
        r[2] = z * inv;
    }
    for (uint32_t c = 0; c < 3; ++c)
    {
        StoreFloat(out + 4u * c, r[c]);
    }
}

// The portable values to the attribute's own: integers cut to its type,
// quantized values scaled back, normals from octahedral coordinates.
static void Original(Value* v, uint32_t entries)
{
    maniDracoAttribute* a = v->a;
    uint32_t size = maniDracoTypeSize(a->dataType);
    if (v->decoder == DecoderNormals)
    {
        // Takes gave the normals' decoder three binary32 components, two
        // portable.
        MANI_ASSERT(v->pcomps == 2 && a->components == 3 && size == 4);
        for (uint32_t i = 0; i < entries; ++i)
        {
            UnitVector(v, Signed(v->portable[2u * i]), Signed(v->portable[2u * i + 1u]),
                       a->values + (size_t)i * 12u);
        }
        return;
    }
    size_t n = a->components;
    if (v->decoder == DecoderQuantization)
    {
        // TransformNeeds refused a quantization's bits outside 1 to 30.
        MANI_ASSERT(v->bits >= 1 && v->bits <= 30);
        float delta = v->range / (float)(int32_t)((1u << v->bits) - 1u);
        for (size_t i = 0; i < entries; ++i)
        {
            for (size_t c = 0; c < n; ++c)
            {
                size_t k = i * n + c;
                StoreFloat(a->values + k * 4u,
                           (float)Signed(v->portable[k]) * delta + v->minimum[c]);
            }
        }
        return;
    }
    // 1, 2 or 4 bytes: integers of 32 bits at most (Takes).
    MANI_ASSERT(size == 1 || size == 2 || size == 4);
    for (size_t k = 0; k < (size_t)entries * n; ++k)
    {
        uint32_t x = v->portable[k];
        switch (size)
        {
        case 1:
            a->values[k] = (uint8_t)x;
            break;
        case 2:
            maniStoreU16(a->values + k * 2u, (uint16_t)x);
            break;
        default:
            maniStoreU32(a->values + k * 4u, x);
            break;
        }
    }
}

// An Edgebreaker decoder's order: its traversal, then each point's value.
static maniDracoStatus Sequence(Decoder* d, Group* g)
{
    maniDracoStatus s = maniDracoTraverse(d->arena, g->table, d->eb.cornerPoint, g->mapSize,
                                          g->traversal, &g->order);
    if (s != maniDracoDone)
    {
        return s;
    }
    g->pointToValue = maniArenaTake(d->arena, d->mesh->pointCount, sizeof(uint32_t));
    if (g->pointToValue == NULL)
    {
        return maniDracoCapacity;
    }
    return maniDracoPointValues(g->table, &g->order, d->eb.cornerPoint, d->mesh->pointCount,
                                g->pointToValue)
               ? maniDracoDone
               : maniDracoInvalid;
}

// One attribute's first pass: its values raw (the generic decoder) or
// their portable form.
static maniDracoStatus FirstPass(Decoder* d, Value* v, const Group* g, uint32_t entries)
{
    maniDracoAttribute* a = v->a;
    size_t bytes = (size_t)entries * a->components * maniDracoTypeSize(a->dataType);
    v->entries = entries;
    a->valueCount = entries;
    a->pointToValue = g != NULL ? g->pointToValue : NULL;
    a->values = maniArenaTake(d->arena, bytes, 1);
    if (a->values == NULL)
    {
        return maniDracoCapacity;
    }
    if (v->decoder != DecoderGeneric)
    {
        return Portable(d, v, g, entries);
    }
    const uint8_t* raw = NULL;
    if (!maniDracoBytes(&d->r, bytes, &raw))
    {
        return maniDracoInvalid;
    }
    if (bytes > 0)
    {
        memcpy(a->values, raw, bytes);
    }
    return maniDracoDone;
}

// One decoder's attributes, in Google's three passes: a value per point,
// or over Edgebreaker, per vertex its traversal reaches.
static maniDracoStatus DecodeValues(Decoder* d, Value* values, uint32_t count, Group* g)
{
    maniDracoStatus s = g != NULL ? Sequence(d, g) : maniDracoDone;
    uint32_t entries = g != NULL ? g->order.count : d->mesh->pointCount;
    // Every value count (points times at most 16 components) fits 32 bits.
    if (s == maniDracoDone && entries > UINT32_MAX / MANI_DRACO_MAX_COMPONENTS)
    {
        return maniDracoCapacity;
    }
    for (uint32_t i = 0; s == maniDracoDone && i < count; ++i)
    {
        s = FirstPass(d, &values[i], g, entries);
    }
    for (uint32_t i = 0; s == maniDracoDone && i < count; ++i)
    {
        s = TransformNeeds(&d->r, &values[i]) ? maniDracoDone : maniDracoInvalid;
    }
    for (uint32_t i = 0; s == maniDracoDone && i < count; ++i)
    {
        if (values[i].decoder != DecoderGeneric)
        {
            Original(&values[i], entries);
        }
    }
    return s;
}

// Edgebreaker's decoder headers: each one's data (negative the
// positions', an attribute data once each), its kind (0 a vertex attribute over the
// mesh's table, else a corner attribute over its data's, depth first)
// and its traversal.
static maniDracoStatus Groups(Decoder* d, uint32_t decoders)
{
    const maniDracoEdgebreaker* eb = &d->eb;
    bool used[129] = {false};
    d->groups = maniArenaTake(d->arena, decoders, sizeof(Group));
    if (d->groups == NULL)
    {
        return maniDracoCapacity;
    }
    for (uint32_t i = 0; i < decoders; ++i)
    {
        Group* g = &d->groups[i];
        uint8_t id = 0;
        uint8_t kind = 0;
        if (!maniDracoU8(&d->r, &id) || !maniDracoU8(&d->r, &kind) ||
            !maniDracoU8(&d->r, &g->traversal) || g->traversal > maniDracoPredictionDegree)
        {
            return maniDracoInvalid;
        }
        // Google's decoder reads any negative data id as the positions'.
        int32_t data = (int8_t)id < 0 ? -1 : (int8_t)id;
        if (data >= (int32_t)eb->dataCount || used[data + 1] ||
            (kind != 0 && (data < 0 || g->traversal != maniDracoDepthFirst)))
        {
            return maniDracoInvalid;
        }
        used[data + 1] = true;
        const maniDracoTable* own = data >= 0 ? &eb->data[data] : &eb->mesh;
        g->table = kind != 0 ? own : &eb->mesh;
        g->mapSize = own->vertices > eb->mesh.vertices ? own->vertices : eb->mesh.vertices;
    }
    return maniDracoDone;
}

// The first position attribute, every decoder's described.
static void FindPosition(Decoder* d, Value* const* values, const uint32_t* counts,
                         uint32_t decoders)
{
    for (uint32_t i = 0; i < decoders; ++i)
    {
        for (uint32_t j = 0; j < counts[i]; ++j)
        {
            if (d->position == NULL && values[i][j].a->type == maniDracoPosition)
            {
                d->position = &values[i][j];
            }
        }
    }
}

static maniDracoStatus Attributes(Decoder* d)
{
    uint8_t decoders = 0;
    if (!maniDracoU8(&d->r, &decoders))
    {
        return maniDracoInvalid;
    }
    maniDracoStatus s = d->method == Edgebreaker ? Groups(d, decoders) : maniDracoDone;
    Value* values[256];
    uint32_t counts[256];
    uint32_t total = 0;
    for (uint32_t i = 0; s == maniDracoDone && i < decoders; ++i)
    {
        s = Describes(d, &values[i], &counts[i]);
        total += s == maniDracoDone ? counts[i] : 0u;
    }
    if (s == maniDracoDone)
    {
        FindPosition(d, values, counts, decoders);
    }
    for (uint32_t i = 0; s == maniDracoDone && i < decoders; ++i)
    {
        s = DecodeValues(d, values[i], counts[i], d->groups != NULL ? &d->groups[i] : NULL);
    }
    if (s != maniDracoDone)
    {
        return s;
    }
    maniDracoMesh* m = d->mesh;
    m->attributes = maniArenaTake(d->arena, total, sizeof(maniDracoAttribute));
    if (m->attributes == NULL)
    {
        return maniDracoCapacity;
    }
    for (uint32_t i = 0; i < decoders; ++i)
    {
        for (uint32_t j = 0; j < counts[i]; ++j)
        {
            m->attributes[m->attributeCount++] = *values[i][j].a;
        }
    }
    return maniDracoDone;
}

maniDracoStatus maniDracoDecode(maniArena* arena, const uint8_t* in, size_t size,
                                uint32_t maxPoints, uint32_t maxFaces, maniDracoMesh* out)
{
    *out = (maniDracoMesh){0};
    Decoder d = {.r = {in, in + size},
                 .arena = arena,
                 .maxPoints = maxPoints,
                 .maxFaces = maxFaces,
                 .mesh = out};
    uint16_t flags = 0;
    maniDracoStatus s = Header(&d, &flags);
    if (s == maniDracoDone && (flags & METADATA_FLAG) != 0)
    {
        s = Metadata(&d);
    }
    if (s == maniDracoDone)
    {
        s = d.method == Sequential ? SequentialConnectivity(&d) : EdgebreakerConnectivity(&d);
    }
    if (s == maniDracoDone)
    {
        s = Attributes(&d);
    }
    return s;
}
