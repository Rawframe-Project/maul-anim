// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX blend shapes as morph targets, as ufbx reads them: a
// set a BlendShape deformer (its BlendShapeChannels the channels, their
// Shape geometries the targets at FullWeights / 100) or, before 7.2, a
// mesh holding Shape children (each a channel of one target at 1). A
// shape's Indexes, Vertices and Normals are offsets of control points;
// Blender's binary exporter writes per-vertex weights into FullWeights,
// folded into the offsets. A shape after another of the same target is
// left out (ufbx weighs it 0 at every channel weight), as is a channel
// left without shapes. Targets keep the clips' order and names, so
// weight tracks find them.

#include "arena.h"
#include "basis.h"
#include "builder.h"
#include "fbx_blend.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_options.h"
#include "fbx_value.h"
#include "fpenv.h"
#include "sort.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/morph.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

static bool IsBlendShape(const maniFbx* fbx, uint32_t object)
{
    return maniFbxDeformerIs(fbx, object, "BlendShape", 10);
}

// Whether an owner's child node is a shape ufbx reads: a Shape with
// Indexes and Vertices.
static bool IsShapeNode(const maniFbx* fbx, uint32_t node)
{
    return maniFbxNameIs(fbx, node, "Shape", 5) &&
           maniFbxFindChild(fbx, node, "Indexes", 7) != MANI_FBX_NONE &&
           maniFbxFindChild(fbx, node, "Vertices", 8) != MANI_FBX_NONE;
}

// The number of shapes of an owner's node.
static uint32_t ShapeChildren(const maniFbx* fbx, uint32_t owner)
{
    uint32_t n = 0;
    for (uint32_t k = fbx->nodes[fbx->objects[owner].node].firstChild; k != MANI_FBX_NONE;
         k = fbx->nodes[k].next)
    {
        n += IsShapeNode(fbx, k) ? 1 : 0;
    }
    return n;
}

// A shape owner with shapes; not a legacy skin, which shares its mesh
// model's node.
static bool IsOwner(const maniFbx* fbx, uint32_t object)
{
    return fbx->objects[object].role == maniFbxRoleNone && maniFbxIsShapeOwner(fbx, object) &&
           ShapeChildren(fbx, object) > 0;
}

// The morph-th set's object: BlendShapes first, then owners; or
// MANI_FBX_NONE.
static uint32_t SetObject(const maniFbx* fbx, uint32_t morph)
{
    uint32_t n = 0;
    for (int pass = 0; pass < 2; ++pass)
    {
        for (uint32_t o = 0; o < fbx->objectCount; ++o)
        {
            bool match = pass == 0 ? IsBlendShape(fbx, o) : IsOwner(fbx, o);
            if (match && n++ == morph)
            {
                return o;
            }
        }
    }
    return MANI_FBX_NONE;
}

uint32_t maniFbxMorphCount(const maniFbx* fbx)
{
    uint32_t n = 0;
    for (uint32_t o = 0; fbx != NULL && o < fbx->objectCount; ++o)
    {
        n += IsBlendShape(fbx, o) || IsOwner(fbx, o) ? 1 : 0;
    }
    return n;
}

uint32_t maniFbxMorphModel(const maniFbx* fbx, uint32_t morph)
{
    uint32_t object = fbx != NULL ? SetObject(fbx, morph) : MANI_FBX_NONE;
    uint32_t model = object == MANI_FBX_NONE
                         ? MANI_FBX_NONE
                         : (IsBlendShape(fbx, object) ? maniFbxDeformerModel(fbx, object)
                                                      : maniFbxShapeOwnerModel(fbx, object));
    return model != MANI_FBX_NONE ? model : UINT32_MAX;
}

// A BlendShape's channels, in connection order.
typedef struct Channels
{
    uint32_t* out;
    uint32_t count;
} Channels;

static void VisitChannel(const maniFbx* fbx, uint32_t child, void* context)
{
    Channels* c = context;
    if (maniFbxIsBlendChannel(fbx, child))
    {
        if (c->out != NULL)
        {
            c->out[c->count] = child;
        }
        ++c->count;
    }
}

// A BlendShape's channels into the arena.
static maniImportResult ListChannels(const maniFbx* fbx, maniArena* arena, uint32_t object,
                                     Channels* out)
{
    *out = (Channels){NULL, 0};
    maniFbxEachChild(fbx, object, VisitChannel, out);
    out->out = maniArenaTake(arena, out->count, sizeof(uint32_t));
    if (out->out == NULL && out->count > 0)
    {
        return Capacity();
    }
    out->count = 0;
    maniFbxEachChild(fbx, object, VisitChannel, out);
    return maniFbxSuccess();
}

// A channel's shapes into the arena, sorted by target.
static maniImportResult ReadShapes(const maniFbx* fbx, maniArena* arena, uint32_t channel,
                                   maniFbxBlendShape** out, uint32_t* countOut)
{
    *out = NULL;
    *countOut = 0;
    maniImportResult r = maniFbxReadBlendShapes(fbx, channel, NULL, countOut);
    *out = r.result == mani_success ? maniArenaTake(arena, *countOut, sizeof(maniFbxBlendShape))
                                    : NULL;
    if (r.result == mani_success && *out == NULL && *countOut > 0)
    {
        return Capacity();
    }
    return r.result == mani_success ? maniFbxReadBlendShapes(fbx, channel, *out, countOut) : r;
}

// Whether the k-th of sorted shapes is a target: the first of its
// target weight.
static bool Kept(const maniFbxBlendShape* shapes, uint32_t k)
{
    return k == 0 || shapes[k].target != shapes[k - 1].target;
}

static uint32_t KeptCount(const maniFbxBlendShape* shapes, uint32_t count)
{
    uint32_t n = 0;
    for (uint32_t k = 0; shapes != NULL && k < count; ++k)
    {
        n += Kept(shapes, k) ? 1 : 0;
    }
    return n;
}

// The targets a BlendShape holds.
static maniImportResult CountTargets(const maniFbx* fbx, maniArena* arena, uint32_t object,
                                     uint32_t* countOut)
{
    Channels list;
    maniImportResult r = ListChannels(fbx, arena, object, &list);
    *countOut = 0;
    for (uint32_t c = 0; r.result == mani_success && c < list.count; ++c)
    {
        maniFbxBlendShape* shapes = NULL;
        uint32_t count = 0;
        r = ReadShapes(fbx, arena, list.out[c], &shapes, &count);
        *countOut += r.result == mani_success ? KeptCount(shapes, count) : 0;
    }
    return r;
}

uint32_t maniFbxMorphTargetCount(const maniFbx* fbx, uint32_t morph)
{
    uint32_t object = fbx != NULL ? SetObject(fbx, morph) : MANI_FBX_NONE;
    if (object == MANI_FBX_NONE)
    {
        return 0;
    }
    if (!IsBlendShape(fbx, object))
    {
        return ShapeChildren(fbx, object);
    }
    maniArena arena = maniArenaMake(&fbx->allocator);
    uint32_t count = 0;
    maniImportResult r = CountTargets(fbx, &arena, object, &count);
    maniArenaRelease(&arena);
    return r.result == mani_success ? count : 0;
}

// The import's state.
typedef struct Morph
{
    const maniFbx* fbx;
    maniArena* arena;
    maniBasis positions;
    maniBasis directions;
    uint32_t object;
    uint32_t morph;
    uint32_t vertices;
    bool normals;
    // The channels and their targets, filled as read.
    maniMorphChannelSource* channels;
    uint32_t channelCount;
} Morph;

// A child node's values as binary64 into the arena (NULL and 0 when
// absent).
static maniImportResult Values(Morph* m, uint32_t node, const char* name, double** out,
                               uint32_t* countOut)
{
    const maniFbx* fbx = m->fbx;
    *out = NULL;
    *countOut = 0;
    uint32_t child = maniFbxFindChild(fbx, node, name, strlen(name));
    if (child == MANI_FBX_NONE)
    {
        return maniFbxSuccess();
    }
    maniImportResult r = maniFbxReadValues(fbx, child, NULL, countOut);
    *out = r.result == mani_success ? maniArenaTake(m->arena, *countOut, sizeof(double)) : NULL;
    if (r.result == mani_success && *out == NULL && *countOut > 0)
    {
        return Capacity();
    }
    return r.result == mani_success ? maniFbxReadValues(fbx, child, *out, countOut) : r;
}
// The control points the set deforms: the Vertices of the BlendShape's
// geometry (or model), or of the owner itself.
static maniImportResult ControlPoints(Morph* m)
{
    const maniFbx* fbx = m->fbx;
    uint32_t candidates[8];
    uint32_t n = 0;
    if (IsBlendShape(fbx, m->object))
    {
        const uint32_t* list = NULL;
        uint32_t k = maniFbxParentConnections(fbx, m->object, &list);
        for (uint32_t i = 0; i < k && n < 8; ++i)
        {
            const maniFbxConnection* c = &fbx->connections[list[i]];
            if (c->childProperty == MANI_FBX_NONE && c->parentProperty == MANI_FBX_NONE &&
                c->parent < fbx->objectCount)
            {
                candidates[n++] = c->parent;
            }
        }
    }
    else
    {
        candidates[n++] = m->object;
    }
    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t node = fbx->objects[candidates[i]].node;
        uint32_t child = maniFbxFindChild(fbx, node, "Vertices", 8);
        uint32_t floats = 0;
        if (child != MANI_FBX_NONE &&
            maniFbxReadValues(fbx, child, NULL, &floats).result == mani_success && floats > 0)
        {
            m->vertices = floats / 3;
            return floats % 3 == 0 ? maniFbxSuccess()
                                   : maniFbxFail(maniImportReasonProperty,
                                                 fbx->nodes[child].nameOffset, m->morph);
        }
    }
    return maniFbxFail(maniImportReasonProperty,
                       fbx->nodes[fbx->objects[m->object].node].nameOffset, m->morph);
}

// A whole index below the control points, or -1.
static int64_t Index(double value, uint32_t vertices)
{
    return value >= 0.0 && value < (double)vertices && value == (double)(int64_t)value
               ? (int64_t)value
               : -1;
}

// One shape node's offsets as a target: indices past the control points
// skipped, each offset times its per-vertex weight (weights NULL for
// none), in the output's basis.
static maniImportResult ReadShape(Morph* m, uint32_t node, const double* weights,
                                  maniMorphTargetSource* out)
{
    const maniFbx* fbx = m->fbx;
    double* index = NULL;
    double* position = NULL;
    double* normal = NULL;
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
    maniImportResult r = Values(m, node, "Indexes", &index, &a);
    r = r.result == mani_success ? Values(m, node, "Vertices", &position, &b) : r;
    r = r.result == mani_success ? Values(m, node, "Normals", &normal, &c) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (b != 3 * (uint64_t)a || (normal != NULL && c != b))
    {
        return maniFbxFail(maniImportReasonProperty, fbx->nodes[node].nameOffset, m->morph);
    }
    uint32_t* indices = maniArenaTake(m->arena, a + 1, sizeof(uint32_t));
    float* positions = maniArenaTake(m->arena, (size_t)a * 3 + 3, sizeof(float));
    float* normals = maniArenaTake(m->arena, (size_t)a * 3 + 3, sizeof(float));
    const float** deltas = (const float**)maniArenaTake(m->arena, 2, sizeof(void*));
    if (indices == NULL || positions == NULL || normals == NULL || deltas == NULL)
    {
        return Capacity();
    }
    uint32_t kept = 0;
    for (uint32_t i = 0; index != NULL && position != NULL && i < a; ++i)
    {
        int64_t v = Index(index[i], m->vertices);
        if (v < 0)
        {
            continue;
        }
        double w = weights != NULL ? weights[i] : 1.0;
        float* p = positions + 3 * (size_t)kept;
        float* n = normals + 3 * (size_t)kept;
        for (int k = 0; k < 3; ++k)
        {
            p[k] = (float)(position[3 * (size_t)i + k] * w);
            n[k] = normal != NULL ? (float)(normal[3 * (size_t)i + k] * w) : 0.0f;
        }
        maniBasisVector(&m->positions, p, true);
        maniBasisVector(&m->directions, n, true);
        indices[kept++] = (uint32_t)v;
    }
    deltas[0] = positions;
    deltas[1] = normal != NULL ? normals : NULL;
    m->normals = m->normals || normal != NULL;
    *out = (maniMorphTargetSource){1.0f, kept, indices, deltas, NULL, 0};
    return maniFbxSuccess();
}

// "#" and n in decimal into out (at least 11 bytes); its length.
static size_t Numbered(uint32_t n, char* out)
{
    char digits[10];
    size_t count = 0;
    do
    {
        digits[count++] = (char)('0' + n % 10);
        n /= 10;
    } while (n != 0);
    size_t length = 0;
    out[length++] = '#';
    while (count > 0)
    {
        out[length++] = digits[--count];
    }
    return length;
}

// An object's name into the arena; "#" and the set's channel index when
// it has none (room left for a target's suffix).
static const char* ObjectName(Morph* m, uint32_t object, uint32_t channel, uint32_t* lengthOut)
{
    size_t length = maniFbxObjectName(m->fbx, object, NULL, 0);
    char* name = maniArenaTake(m->arena, length + 24, 1);
    if (name == NULL)
    {
        return NULL;
    }
    length = maniFbxObjectName(m->fbx, object, (uint8_t*)name, length);
    length = length > 0 ? length : Numbered(channel, name);
    *lengthOut = (uint32_t)length;
    return name;
}

// Blender's per-vertex weights for a channel's first shape: its
// FullWeights / 100 where they are as many as the shape's offsets; else
// NULL. (ufbx's later shapes of a Blender channel share target 1, so are
// left out.)
static maniImportResult BlenderWeights(Morph* m, uint32_t channel, uint32_t shape,
                                       const double** out)
{
    *out = NULL;
    if (!maniFbxFromBlender(m->fbx))
    {
        return maniFbxSuccess();
    }
    double* full = NULL;
    double* index = NULL;
    uint32_t n = 0;
    uint32_t a = 0;
    maniImportResult r = Values(m, m->fbx->objects[channel].node, "FullWeights", &full, &n);
    r = r.result == mani_success ? Values(m, m->fbx->objects[shape].node, "Indexes", &index, &a)
                                 : r;
    if (r.result == mani_success && full != NULL && n == a)
    {
        for (uint32_t i = 0; i < n; ++i)
        {
            full[i] /= 100.0;
        }
        *out = full;
    }
    return r;
}

// A shape's name: its own, or for a shape without one its channel's,
// with '/' and k past the first (the clips' track rule).
static maniImportResult ShapeName(Morph* m, const maniMorphChannelSource* channel, uint32_t shape,
                                  uint32_t k, maniMorphTargetSource* target)
{
    size_t length = maniFbxObjectName(m->fbx, shape, NULL, 0);
    target->name = NULL;
    target->nameLength = 0;
    if (length == 0 && k == 0)
    {
        target->name = channel->name;
        target->nameLength = channel->nameLength;
        return maniFbxSuccess();
    }
    char* name = maniArenaTake(m->arena, length > 0 ? length : channel->nameLength + 11u, 1);
    if (name == NULL)
    {
        return Capacity();
    }
    if (length > 0)
    {
        length = maniFbxObjectName(m->fbx, shape, (uint8_t*)name, length);
    }
    else
    {
        memcpy(name, channel->name, channel->nameLength);
        char suffix[12];
        size_t n = Numbered(k, suffix);
        suffix[0] = '/';
        memcpy(name + channel->nameLength, suffix, n);
        length = channel->nameLength + n;
    }
    target->name = name;
    target->nameLength = (uint32_t)length;
    return maniFbxSuccess();
}

// A BlendShapeChannel's shapes as a channel's targets; a channel of none
// left out.
static maniImportResult ReadChannel(Morph* m, uint32_t channel, uint32_t c)
{
    const maniFbx* fbx = m->fbx;
    maniFbxBlendShape* shapes = NULL;
    uint32_t count = 0;
    maniImportResult r = ReadShapes(fbx, m->arena, channel, &shapes, &count);
    uint32_t kept = r.result == mani_success ? KeptCount(shapes, count) : 0;
    maniMorphChannelSource* out = &m->channels[m->channelCount];
    if (r.result != mani_success || kept == 0)
    {
        return r;
    }
    out->name = ObjectName(m, channel, c, &out->nameLength);
    maniMorphTargetSource* targets = maniArenaTake(m->arena, kept, sizeof(maniMorphTargetSource));
    if (out->name == NULL || targets == NULL)
    {
        return Capacity();
    }
    uint32_t t = 0;
    for (uint32_t k = 0; r.result == mani_success && shapes != NULL && k < count; ++k)
    {
        if (!Kept(shapes, k))
        {
            continue;
        }
        const double* weights = NULL;
        r = k == 0 ? BlenderWeights(m, channel, shapes[k].object, &weights) : r;
        r = r.result == mani_success
                ? ReadShape(m, fbx->objects[shapes[k].object].node, weights, &targets[t])
                : r;
        r = r.result == mani_success ? ShapeName(m, out, shapes[k].object, k, &targets[t]) : r;
        targets[t++].fullWeight = (float)shapes[k].target;
    }
    out->targetCount = kept;
    out->targets = targets;
    m->channelCount += r.result == mani_success ? 1 : 0;
    return r;
}

// An owner's shapes (before 7.2), each a channel of one target at 1,
// named by its first value.
static maniImportResult ReadOwner(Morph* m)
{
    const maniFbx* fbx = m->fbx;
    uint32_t c = 0;
    maniImportResult r = maniFbxSuccess();
    for (uint32_t k = fbx->nodes[fbx->objects[m->object].node].firstChild;
         r.result == mani_success && k != MANI_FBX_NONE; k = fbx->nodes[k].next)
    {
        if (!IsShapeNode(fbx, k))
        {
            continue;
        }
        maniMorphTargetSource* target = maniArenaTake(m->arena, 1, sizeof(maniMorphTargetSource));
        uint32_t property = maniFbxNodeProperty(fbx, k, 0);
        size_t length = maniFbxStringDecode(fbx, property, NULL, 0);
        char* name = maniArenaTake(m->arena, length + 12, 1);
        if (target == NULL || name == NULL)
        {
            return Capacity();
        }
        length = maniFbxStringDecode(fbx, property, (uint8_t*)name, length);
        r = ReadShape(m, k, NULL, target);
        length = length > 0 ? length : Numbered(c, name);
        target->name = name;
        target->nameLength = (uint32_t)length;
        m->channels[c++] = (maniMorphChannelSource){name, (uint32_t)length, 1, target};
    }
    m->channelCount = c;
    return r;
}

// Whether channels a and b have the same name.
static bool SameName(const maniMorphChannelSource* a, const maniMorphChannelSource* b)
{
    return a->nameLength == b->nameLength && memcmp(a->name, b->name, a->nameLength) == 0;
}

// A channel repeating an earlier one's name renamed with "#" and its
// index (its targets named already, as the clips' tracks name them).
static maniImportResult Unique(Morph* m)
{
    uint32_t n = m->channelCount;
    uint64_t* keys = maniArenaTake(m->arena, (size_t)n + 1, sizeof(uint64_t));
    uint32_t* order = maniArenaTake(m->arena, (size_t)n + 1, sizeof(uint32_t));
    uint32_t* scratch = maniArenaTake(m->arena, (size_t)n + 1, sizeof(uint32_t));
    bool* repeats = maniArenaTake(m->arena, (size_t)n + 1, sizeof(bool));
    if (keys == NULL || order == NULL || scratch == NULL || repeats == NULL)
    {
        return Capacity();
    }
    for (uint32_t c = 0; c < n; ++c)
    {
        keys[c] =
            maniHash64(MANI_HASH_INIT, m->channels[c].name, (int32_t)m->channels[c].nameLength);
        repeats[c] = false;
    }
    maniSortByKey(keys, n, order, scratch);
    // Sorted stably, so within a run of equal hashes in channel order.
    for (uint32_t i = 0; i < n; ++i)
    {
        for (uint32_t j = i + 1; j < n && keys[order[j]] == keys[order[i]]; ++j)
        {
            repeats[order[j]] =
                repeats[order[j]] || SameName(&m->channels[order[i]], &m->channels[order[j]]);
        }
    }
    for (uint32_t c = 0; c < n; ++c)
    {
        maniMorphChannelSource* ch = &m->channels[c];
        char* name = repeats[c] ? maniArenaTake(m->arena, (size_t)ch->nameLength + 11, 1) : NULL;
        if (repeats[c] && name == NULL)
        {
            return Capacity();
        }
        if (name != NULL)
        {
            memcpy(name, ch->name, ch->nameLength);
            ch->nameLength += (uint32_t)Numbered(c, name + ch->nameLength);
            ch->name = name;
        }
    }
    return maniFbxSuccess();
}

static maniImportResult BuildMorph(maniBuilder* builder, Morph* m)
{
    const maniFbx* fbx = m->fbx;
    maniImportResult r = ControlPoints(m);
    bool blend = IsBlendShape(fbx, m->object);
    Channels list = {NULL, 0};
    r = r.result == mani_success && blend ? ListChannels(fbx, m->arena, m->object, &list) : r;
    uint32_t channels = blend ? list.count : ShapeChildren(fbx, m->object);
    m->channels = r.result == mani_success ? maniArenaTake(m->arena, (size_t)channels + 1,
                                                           sizeof(maniMorphChannelSource))
                                           : NULL;
    if (r.result == mani_success && m->channels == NULL)
    {
        return Capacity();
    }
    for (uint32_t c = 0; blend && r.result == mani_success && c < list.count; ++c)
    {
        r = ReadChannel(m, list.out[c], c);
    }
    r = r.result == mani_success && !blend ? ReadOwner(m) : r;
    r = r.result == mani_success ? Unique(m) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    static const maniMorphAttributeSource kAttributes[2] = {{maniMorphPosition, 0},
                                                            {maniMorphNormal, 0}};
    maniMorphSource source = {m->channels, m->channelCount, m->vertices, kAttributes,
                              m->normals ? 2u : 1u};
    maniBuildResult b = maniBuildMorphTargets(builder, &source);
    if (b.result == mani_success || b.result == mani_errorCapacity)
    {
        return (maniImportResult){b.result, maniImportReasonNone, 0, 0};
    }
    return maniFbxFail(maniImportReasonBuild, fbx->nodes[fbx->objects[m->object].node].nameOffset,
                       (uint32_t)b.reason);
}

static MANI_NOINLINE maniImportResult FbxBuildMorphTargetsWork(maniBuilder* builder,
                                                               const maniFbx* fbx, uint32_t morph,
                                                               const maniFbxOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorBusy, maniImportReasonNone, 0, 0};
    }
    uint32_t object = fbx != NULL ? SetObject(fbx, morph) : MANI_FBX_NONE;
    if (!maniFbxUsable(builder, fbx, options, object != MANI_FBX_NONE) || fbx == NULL)
    {
        return Invalid();
    }
    maniArena arena = maniArenaMake(&fbx->allocator);
    Morph m = {.fbx = fbx, .arena = &arena, .object = object, .morph = morph};
    m.positions = maniFbxBasis(fbx, options);
    m.directions = m.positions;
    m.directions.unit = 1.0f;
    maniImportResult r = BuildMorph(builder, &m);
    maniArenaRelease(&arena);
    return r;
}

maniImportResult maniFbxBuildMorphTargets(maniBuilder* builder, const maniFbx* fbx, uint32_t morph,
                                          const maniFbxOptions* options)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxBuildMorphTargetsWork(builder, fbx, morph, options);
    maniFloatEnvLeave(env);
    return result;
}

// An owner's shapes' rest weights: the owner's numeric property of each
// shape's name, over 100.
static void OwnerWeights(const maniFbx* fbx, uint32_t owner, float* out)
{
    uint32_t c = 0;
    for (uint32_t k = fbx->nodes[fbx->objects[owner].node].firstChild; k != MANI_FBX_NONE;
         k = fbx->nodes[k].next)
    {
        if (!IsShapeNode(fbx, k))
        {
            continue;
        }
        uint8_t name[256];
        size_t length = maniFbxStringDecode(fbx, maniFbxNodeProperty(fbx, k, 0), name, sizeof name);
        uint32_t entry = length <= sizeof name
                             ? maniFbxFindProperty(fbx, owner, (const char*)name, length)
                             : MANI_FBX_NONE;
        double rest = 0.0;
        if (entry == MANI_FBX_NONE || !maniFbxEntryNumeric(fbx, entry) ||
            !maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, 0), &rest))
        {
            rest = 0.0;
        }
        out[c++] = (float)(rest / 100.0);
    }
}

// A BlendShape's targets' rest weights, as the clips weigh them.
static maniImportResult BlendRestWeights(const maniFbx* fbx, maniArena* arena, uint32_t object,
                                         float* out)
{
    Channels list;
    maniImportResult r = ListChannels(fbx, arena, object, &list);
    uint32_t at = 0;
    for (uint32_t c = 0; r.result == mani_success && c < list.count; ++c)
    {
        maniFbxBlendShape* shapes = NULL;
        uint32_t count = 0;
        r = ReadShapes(fbx, arena, list.out[c], &shapes, &count);
        double* effective = r.result == mani_success
                                ? maniArenaTake(arena, (size_t)count + 1, sizeof(double))
                                : NULL;
        if (r.result == mani_success && effective == NULL)
        {
            return Capacity();
        }
        if (r.result == mani_success)
        {
            maniFbxBlendWeights(shapes, count, maniFbxBlendRestPercent(fbx, list.out[c]) / 100.0,
                                effective);
        }
        for (uint32_t k = 0; r.result == mani_success && shapes != NULL && k < count; ++k)
        {
            if (Kept(shapes, k))
            {
                out[at++] = (float)effective[k];
            }
        }
    }
    return r;
}

// The set's target count written, and whether the weights' array holds
// it.
static bool Holds(uint32_t total, const float* weightsOut, uint32_t capacity, uint32_t* countOut)
{
    if (countOut != NULL)
    {
        *countOut = total;
    }
    return weightsOut != NULL && capacity >= total;
}

static MANI_NOINLINE maniImportResult FbxMorphWeightsWork(const maniFbx* fbx, uint32_t morph,
                                                          float* weightsOut, uint32_t capacity,
                                                          uint32_t* countOut)
{
    uint32_t object = fbx != NULL ? SetObject(fbx, morph) : MANI_FBX_NONE;
    if ((weightsOut == NULL && capacity > 0) || object == MANI_FBX_NONE)
    {
        return Invalid();
    }
    const maniImportResult tooSmall = {mani_errorCapacity, maniImportReasonNone, 0, 0};
    if (!IsBlendShape(fbx, object))
    {
        if (!Holds(ShapeChildren(fbx, object), weightsOut, capacity, countOut))
        {
            return tooSmall;
        }
        OwnerWeights(fbx, object, weightsOut);
        return maniFbxSuccess();
    }
    maniArena arena = maniArenaMake(&fbx->allocator);
    uint32_t total = 0;
    maniImportResult r = CountTargets(fbx, &arena, object, &total);
    r = r.result == mani_success && !Holds(total, weightsOut, capacity, countOut) ? tooSmall : r;
    r = r.result == mani_success ? BlendRestWeights(fbx, &arena, object, weightsOut) : r;
    maniArenaRelease(&arena);
    return r;
}

maniImportResult maniFbxMorphWeights(const maniFbx* fbx, uint32_t morph, float* weightsOut,
                                     uint32_t capacity, uint32_t* countOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxMorphWeightsWork(fbx, morph, weightsOut, capacity, countOut);
    maniFloatEnvLeave(env);
    return result;
}
