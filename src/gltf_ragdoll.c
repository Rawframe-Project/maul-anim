// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The OMI physics extensions as a ragdoll source: dynamic and
// kinematic bodies under the skeleton's bones, each with its one
// collider's shape and frame (rigid, in the bone's frame, the shape
// times the node's even scale), its mass; the joints read by
// gltf_ragdoll_joint.c; the root body's facing and up axes from the
// caller's. What a profile cannot hold is refused at its value. The
// arrays live in the document's ragdoll arena until the next read.

#include "gltf_ragdoll.h"

#include "arena.h"
#include "decompose.h"
#include "fpenv.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_names.h"
#include "gltf_property.h"
#include "gltf_rest.h"
#include "json.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <string.h>

// The larger and the smaller of two floats (no libm call).
static float Larger(float a, float b)
{
    return a > b ? a : b;
}

static float Smaller(float a, float b)
{
    return a < b ? a : b;
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

maniImportResult maniGltfRagdollFail(const maniGltfRagdollReader* r, maniImportReason reason,
                                     uint32_t value, uint32_t item)
{
    return maniImportFail(reason, maniGltfOffset(&r->gltf->g, value), item);
}

maniGltfRigid maniGltfRigidIn(maniGltfRigid a, maniGltfRigid b)
{
    maniQuat back = maniQuatConjugate(a.r);
    return (maniGltfRigid){maniQuatRotate(back, maniVec3Sub(b.t, a.t)),
                           maniQuatNormalize(maniQuatMul(back, b.r))};
}

maniImportResult maniGltfRagdollWorld(const maniGltfRagdollReader* r, uint32_t node,
                                      maniGltfRigid* frameOut, maniVec3* scaleOut)
{
    uint32_t depth = 0;
    for (uint32_t n = node; n != MANI_JSON_NONE; n = r->gltf->nodeParents[n])
    {
        r->chain[depth++] = n;
    }
    maniAffine world = maniGltfIdentityAffine();
    while (depth > 0)
    {
        maniTransform rest;
        maniImportResult result = maniGltfNodeRest(r->gltf, r->chain[--depth], r->options, &rest);
        if (result.result != mani_success)
        {
            return result;
        }
        maniAffine local = maniAffineFromTrs(rest.translation, rest.rotation, rest.scale);
        world = maniAffineMul(&world, &local);
    }
    maniTransform t = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    (void)maniDecompose(&world, &t);
    *frameOut = (maniGltfRigid){t.translation, maniQuatNormalize(t.rotation)};
    if (scaleOut != NULL)
    {
        *scaleOut = t.scale;
    }
    return maniImportSuccess();
}

maniImportResult maniGltfRagdollExtension(const maniGltfRagdollReader* r, uint32_t object,
                                          const char* name, uint32_t item, uint32_t* out)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t extensions = MANI_JSON_NONE;
    *out = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, object, "extensions", maniJsonObject, false, item, &extensions);
    if (result.result == mani_success && extensions != MANI_JSON_NONE)
    {
        result = maniGltfMember(g, extensions, name, maniJsonObject, false, item, out);
    }
    return result;
}

maniImportResult maniGltfRagdollNumber(const maniGltfRagdollReader* r, uint32_t object,
                                       const char* name, uint32_t item, float* out)
{
    uint32_t value = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonNumber, false, item, &value);
    if (result.result != mani_success || value == MANI_JSON_NONE)
    {
        return result;
    }
    uint64_t bits = 0;
    bool fits = maniJsonBinary(&r->gltf->json, value, maniBinary32, &bits);
    uint32_t word = (uint32_t)bits;
    float f = 0.0f;
    memcpy(&f, &word, sizeof f);
    if (!fits || !(fabsf(f) < HUGE_VALF))
    {
        return maniGltfRagdollFail(r, maniImportReasonProperty, value, item);
    }
    *out = f;
    return result;
}

// The bone of a node or of its nearest ancestor that is one.
static uint32_t HoldingBone(const maniGltfRagdollReader* r, uint32_t node, uint32_t* boneNodeOut)
{
    for (uint32_t n = node; n != MANI_JSON_NONE; n = r->gltf->nodeParents[n])
    {
        uint32_t bone = maniGltfNodeBone(r->gltf, r->skeleton, n);
        if (bone != MANI_NO_BONE)
        {
            *boneNodeOut = n;
            return bone;
        }
    }
    return MANI_NO_BONE;
}

// `centerOfMass` absent or 0, `gravityFactor` absent or 1, `mass` above 0.
static maniImportResult ReadMotion(const maniGltfRagdollReader* r, uint32_t motion, uint32_t item,
                                   float* massOut)
{
    const maniJson* json = &r->gltf->json;
    uint32_t center = MANI_JSON_NONE;
    float gravity = 1.0f;
    *massOut = 1.0f;
    maniImportResult result = maniGltfRagdollNumber(r, motion, "mass", item, massOut);
    if (result.result == mani_success && !(*massOut > 0.0f))
    {
        uint32_t at = maniJsonFind(json, motion, "mass", 4);
        return maniGltfRagdollFail(r, maniImportReasonProperty, at, item);
    }
    if (result.result == mani_success)
    {
        result = maniGltfMember(&r->gltf->g, motion, "centerOfMass", maniJsonArray, false, item,
                                &center);
    }
    if (result.result == mani_success && center != MANI_JSON_NONE)
    {
        uint32_t element = center + 1;
        bool zero = json->values[center].count == 3;
        for (uint32_t k = 0; k < json->values[center].count && zero; ++k)
        {
            uint64_t bits = 0;
            zero = json->values[element].kind == maniJsonNumber &&
                   maniJsonBinary(json, element, maniBinary32, &bits) && (bits << 1) == 0;
            element = json->values[element].next;
        }
        if (!zero)
        {
            return maniGltfRagdollFail(r, maniImportReasonUnsupported, center, item);
        }
    }
    if (result.result == mani_success)
    {
        result = maniGltfRagdollNumber(r, motion, "gravityFactor", item, &gravity);
    }
    if (result.result == mani_success && gravity != 1.0f)
    {
        uint32_t at = maniJsonFind(json, motion, "gravityFactor", 13);
        return maniGltfRagdollFail(r, maniImportReasonUnsupported, at, item);
    }
    return result;
}

// A body on node's motion when a bone holds it: dynamic or kinematic;
// static refused.
static maniImportResult ReadBody(maniGltfRagdollReader* r, uint32_t node, uint32_t motion)
{
    const maniJson* json = &r->gltf->json;
    uint32_t type = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, motion, "type", maniJsonString, true, node, &type);
    if (result.result != mani_success)
    {
        return result;
    }
    bool moves = maniJsonStringEquals(json, type, "dynamic", 7) ||
                 maniJsonStringEquals(json, type, "kinematic", 9);
    bool still = maniJsonStringEquals(json, type, "static", 6);
    uint32_t boneNode = MANI_JSON_NONE;
    uint32_t bone = HoldingBone(r, node, &boneNode);
    if (!moves && !still)
    {
        return maniGltfRagdollFail(r, maniImportReasonProperty, type, node);
    }
    if (bone == MANI_NO_BONE)
    {
        r->nodeBody[node] = MANI_GLTF_NOT_OURS;
        return result;
    }
    if (still)
    {
        return maniGltfRagdollFail(r, maniImportReasonUnsupported, type, node);
    }
    maniRagdollBodySource* b = &r->bodies[r->bodyCount];
    *b = (maniRagdollBodySource){0};
    b->bone = bone;
    b->shape = UINT32_MAX;
    b->name = maniGltfNodeName(r->gltf, node, &b->nameLength);
    result = ReadMotion(r, motion, node, &b->mass);
    r->bodyNodes[r->bodyCount] = node;
    r->boneNodes[r->bodyCount] = boneNode;
    r->nodeBody[node] = r->bodyCount++;
    return result;
}

// Three positive numbers of an array member, their default when absent.
static maniImportResult Size3(const maniGltfRagdollReader* r, uint32_t object, uint32_t item,
                              float out[3])
{
    const maniJson* json = &r->gltf->json;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, "size", maniJsonArray, false, item, &array);
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    bool ok = json->values[array].count == 3;
    uint32_t element = array + 1;
    for (uint32_t k = 0; k < 3 && ok; ++k)
    {
        uint64_t bits = 0;
        ok = json->values[element].kind == maniJsonNumber &&
             maniJsonBinary(json, element, maniBinary32, &bits);
        uint32_t word = (uint32_t)bits;
        memcpy(&out[k], &word, sizeof out[k]);
        ok = ok && out[k] > 0.0f && out[k] < HUGE_VALF;
        element = json->values[element].next;
    }
    return ok ? result : maniGltfRagdollFail(r, maniImportReasonProperty, array, item);
}

// A positive number member, its default when absent.
static maniImportResult Positive(const maniGltfRagdollReader* r, uint32_t object, const char* name,
                                 uint32_t item, float* out)
{
    maniImportResult result = maniGltfRagdollNumber(r, object, name, item, out);
    if (result.result == mani_success && !(*out > 0.0f))
    {
        uint32_t at = maniJsonFind(&r->gltf->json, object, name, strlen(name));
        return maniGltfRagdollFail(r, maniImportReasonProperty, at, item);
    }
    return result;
}

// A capsule's details: its radii equal (a tapered capsule refused) and
// its height not negative.
static maniImportResult Capsule(const maniGltfRagdollReader* r, uint32_t details, uint32_t item,
                                float* radius, float* height)
{
    const maniJson* json = &r->gltf->json;
    float top = 0.5f;
    maniImportResult result = Positive(r, details, "radiusBottom", item, radius);
    if (result.result == mani_success)
    {
        result = Positive(r, details, "radiusTop", item, &top);
    }
    if (result.result == mani_success)
    {
        result = maniGltfRagdollNumber(r, details, "height", item, height);
    }
    if (result.result == mani_success && !(*height >= 0.0f))
    {
        uint32_t at = maniJsonFind(json, details, "height", 6);
        return maniGltfRagdollFail(r, maniImportReasonProperty, at, item);
    }
    if (result.result == mani_success && top != *radius)
    {
        // A tapered capsule: at its top radius, or its bottom one when the
        // top is the default.
        uint32_t at = maniJsonFind(json, details, "radiusTop", 9);
        at = at != MANI_JSON_NONE ? at : maniJsonFind(json, details, "radiusBottom", 12);
        return maniGltfRagdollFail(r, maniImportReasonUnsupported, at, item);
    }
    return result;
}

// A shape's sizes in a profile's terms (unscaled): a box's half extents,
// a sphere's radius, a capsule's radius and caps' distance; OMI's
// defaults when its details are absent.
static maniImportResult ReadShape(const maniGltfRagdollReader* r, uint32_t shape, uint32_t item,
                                  maniRagdollBodySource* b)
{
    const maniJson* json = &r->gltf->json;
    const maniGltfJson* g = &r->gltf->g;
    static const char* const kTypes[6] = {"box",      "sphere", "capsule",
                                          "cylinder", "convex", "trimesh"};
    uint32_t type = MANI_JSON_NONE;
    maniImportResult result = maniGltfMember(g, shape, "type", maniJsonString, true, item, &type);
    if (result.result != mani_success)
    {
        return result;
    }
    uint32_t kind = 6;
    for (uint32_t k = 0; k < 6; ++k)
    {
        kind = maniJsonStringEquals(json, type, kTypes[k], strlen(kTypes[k])) ? k : kind;
    }
    if (kind >= 3)
    {
        maniImportReason reason = kind < 6 ? maniImportReasonUnsupported : maniImportReasonProperty;
        return maniGltfRagdollFail(r, reason, type, item);
    }
    uint32_t details = MANI_JSON_NONE;
    result = maniGltfMember(g, shape, kTypes[kind], maniJsonObject, false, item, &details);
    float size[3] = {1.0f, 1.0f, 1.0f};
    float radius = 0.5f;
    float height = 1.0f;
    if (result.result == mani_success && details != MANI_JSON_NONE)
    {
        result = kind == 0   ? Size3(r, details, item, size)
                 : kind == 1 ? Positive(r, details, "radius", item, &radius)
                             : Capsule(r, details, item, &radius, &height);
    }
    static const maniRagdollShape kShapes[3] = {maniRagdollBox, maniRagdollSphere,
                                                maniRagdollCapsule};
    b->shape = kShapes[kind];
    b->size = kind == 0   ? (maniVec3){0.5f * size[0], 0.5f * size[1], 0.5f * size[2]}
              : kind == 1 ? (maniVec3){radius, 0.0f, 0.0f}
                          : (maniVec3){radius, height, 0.0f};
    return result;
}

// The same scale on the three axes within a thousandth, positive.
static bool Even(maniVec3 s)
{
    float most = Larger(s.x, Larger(s.y, s.z));
    float least = Smaller(s.x, Smaller(s.y, s.z));
    return least > 0.0f && most - least <= 1e-3f * most;
}

// A collider on a node below one of the skeleton's bodies: its shape
// and frame become the body's; a second refused.
static maniImportResult ReadCollider(maniGltfRagdollReader* r, uint32_t node, uint32_t collider,
                                     uint32_t shapes)
{
    const maniJson* json = &r->gltf->json;
    uint32_t owner = r->nodeBody[node];
    if (owner >= MANI_GLTF_NOT_OURS)
    {
        return maniImportSuccess();
    }
    maniRagdollBodySource* b = &r->bodies[owner];
    if (b->shape != UINT32_MAX)
    {
        return maniGltfRagdollFail(r, maniImportReasonUnsupported, collider, node);
    }
    uint32_t count = shapes != MANI_JSON_NONE ? json->values[shapes].count : 0;
    uint32_t index = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfIndex(&r->gltf->g, collider, "shape", true, count, node, &index);
    uint32_t shape = shapes + 1;
    for (uint32_t k = 0; result.result == mani_success && k < index; ++k)
    {
        shape = json->values[shape].next;
    }
    if (result.result == mani_success && json->values[shape].kind != maniJsonObject)
    {
        return maniGltfRagdollFail(r, maniImportReasonProperty, shape, node);
    }
    if (result.result == mani_success)
    {
        result = ReadShape(r, shape, node, b);
    }
    maniGltfRigid world;
    maniGltfRigid bone;
    maniVec3 scale;
    if (result.result == mani_success)
    {
        result = maniGltfRagdollWorld(r, node, &world, &scale);
    }
    if (result.result == mani_success)
    {
        result = maniGltfRagdollWorld(r, r->boneNodes[owner], &bone, NULL);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    if (!Even(scale))
    {
        return maniGltfRagdollFail(r, maniImportReasonUnsupported, collider, node);
    }
    float k = (scale.x + scale.y + scale.z) / 3.0f;
    b->size = maniVec3Scale(b->size, k);
    r->bodyWorld[owner] = world;
    maniGltfRigid frame = maniGltfRigidIn(bone, world);
    b->frameTranslation = frame.t;
    b->frameRotation = frame.r;
    return result;
}

// Every node's body: its own motion's, or its nearest ancestor's.
static void Owners(maniGltfRagdollReader* r, const uint32_t* own)
{
    for (uint32_t node = 0; node < r->gltf->nodeCount; ++node)
    {
        uint32_t n = node;
        while (n != MANI_JSON_NONE && own[n] == UINT32_MAX)
        {
            n = r->gltf->nodeParents[n];
        }
        r->nodeBody[node] = n != MANI_JSON_NONE ? own[n] : UINT32_MAX;
    }
}

// Each node's `OMI_physics_body` member name (motion or collider) read
// by read.
static maniImportResult EachNode(maniGltfRagdollReader* r, const char* name, uint32_t shapes)
{
    maniImportResult result = maniImportSuccess();
    for (uint32_t node = 0; node < r->gltf->nodeCount && result.result == mani_success; ++node)
    {
        uint32_t body = MANI_JSON_NONE;
        uint32_t member = MANI_JSON_NONE;
        result =
            maniGltfRagdollExtension(r, r->gltf->nodeValues[node], "OMI_physics_body", node, &body);
        if (result.result == mani_success && body != MANI_JSON_NONE)
        {
            result = maniGltfMember(&r->gltf->g, body, name, maniJsonObject, false, node, &member);
        }
        if (result.result != mani_success || member == MANI_JSON_NONE)
        {
            continue;
        }
        result = shapes == UINT32_MAX ? ReadBody(r, node, member)
                                      : ReadCollider(r, node, member, shapes);
    }
    return result;
}

// The root body's model rotation: its bone's rest in the model, its
// frame after.
static maniQuat RootModel(const maniGltfRagdollReader* r, uint32_t root)
{
    maniAffine model = maniGltfIdentityAffine();
    for (uint32_t bone = r->bodies[root].bone; bone != MANI_NO_BONE;
         bone = maniSkeleton_GetParent(r->skeleton, bone))
    {
        maniTransform t = maniSkeleton_GetRest(r->skeleton, bone);
        maniAffine local = maniAffineFromTrs(t.translation, t.rotation, t.scale);
        model = maniAffineMul(&local, &model);
    }
    maniTransform t = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    (void)maniDecompose(&model, &t);
    return maniQuatNormalize(maniQuatMul(t.rotation, r->bodies[root].frameRotation));
}

// The body none of whose bone's ancestors has one (the builder refuses
// two).
static uint32_t Root(const maniGltfRagdollReader* r)
{
    for (uint32_t i = 0; i < r->bodyCount; ++i)
    {
        bool below = false;
        for (uint32_t bone = maniSkeleton_GetParent(r->skeleton, r->bodies[i].bone);
             bone != MANI_NO_BONE && !below; bone = maniSkeleton_GetParent(r->skeleton, bone))
        {
            for (uint32_t j = 0; j < r->bodyCount && !below; ++j)
            {
                below = r->bodies[j].bone == bone;
            }
        }
        if (!below)
        {
            return i;
        }
    }
    return 0;
}

// The document-level `name` extension's array member `array`, or
// MANI_JSON_NONE.
static maniImportResult DocumentArray(const maniGltfRagdollReader* r, const char* name,
                                      const char* array, uint32_t* out)
{
    uint32_t extension = MANI_JSON_NONE;
    *out = MANI_JSON_NONE;
    maniImportResult result = maniGltfRagdollExtension(r, 0, name, 0, &extension);
    if (result.result == mani_success && extension != MANI_JSON_NONE)
    {
        result = maniGltfMember(&r->gltf->g, extension, array, maniJsonArray, false, 0, out);
    }
    return result;
}

static maniImportResult Read(maniGltfRagdollReader* r, uint32_t* own)
{
    uint32_t shapes = MANI_JSON_NONE;
    maniImportResult result = DocumentArray(r, "OMI_physics_shape", "shapes", &shapes);
    if (result.result == mani_success)
    {
        result = DocumentArray(r, "OMI_physics_joint", "physicsJoints", &r->settings);
    }
    r->settingsCount = r->settings != MANI_JSON_NONE ? r->gltf->json.values[r->settings].count : 0;
    if (result.result == mani_success)
    {
        result = EachNode(r, "motion", UINT32_MAX);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    memcpy(own, r->nodeBody, (size_t)r->gltf->nodeCount * sizeof *own);
    Owners(r, own);
    result = EachNode(r, "collider", shapes);
    for (uint32_t i = 0; i < r->bodyCount && result.result == mani_success; ++i)
    {
        if (r->bodies[i].shape == UINT32_MAX)
        {
            uint32_t body = MANI_JSON_NONE;
            (void)maniGltfRagdollExtension(r, r->gltf->nodeValues[r->bodyNodes[i]],
                                           "OMI_physics_body", r->bodyNodes[i], &body);
            return maniGltfRagdollFail(r, maniImportReasonUnsupported, body, r->bodyNodes[i]);
        }
    }
    return result.result == mani_success ? maniGltfReadRagdollJoints(r) : result;
}

static bool AxesValid(maniVec3 facing, maniVec3 up)
{
    float f = maniVec3Dot(facing, facing);
    float u = maniVec3Dot(up, up);
    return f >= 0.998f && f <= 1.002f && u >= 0.998f && u <= 1.002f &&
           fabsf(maniVec3Dot(facing, up)) <= 1e-3f;
}

// An axis in the output's basis, turned into the root body's frame.
static maniVec3 InRoot(const maniGltfRagdollReader* r, maniQuat root, maniVec3 axis)
{
    float v[3] = {axis.x, axis.y, axis.z};
    maniGltfBasisVector(r->options, v, true);
    return maniQuatRotate(maniQuatConjugate(root), (maniVec3){v[0], v[1], v[2]});
}

static MANI_NOINLINE maniImportResult GltfReadRagdollWork(maniGltf* gltf,
                                                          const maniSkeleton* skeleton,
                                                          const maniGltfOptions* options,
                                                          maniVec3 facing, maniVec3 up,
                                                          maniRagdollSource* sourceOut)
{
    if (gltf == NULL || skeleton == NULL || options == NULL || sourceOut == NULL ||
        !maniGltfOptionsValid(options) || !AxesValid(facing, up))
    {
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0};
    }
    *sourceOut = (maniRagdollSource){0};
    maniArenaRelease(&gltf->ragdoll);
    gltf->ragdoll = maniArenaMake(&gltf->allocator);
    uint32_t n = gltf->nodeCount;
    maniGltfRagdollReader r = {.gltf = gltf, .skeleton = skeleton, .options = options};
    r.bodies = maniArenaTake(&gltf->ragdoll, n, sizeof *r.bodies);
    r.bodyNodes = maniArenaTake(&gltf->ragdoll, n, sizeof(uint32_t));
    r.boneNodes = maniArenaTake(&gltf->ragdoll, n, sizeof(uint32_t));
    r.bodyWorld = maniArenaTake(&gltf->ragdoll, n, sizeof *r.bodyWorld);
    r.nodeBody = maniArenaTake(&gltf->ragdoll, n, sizeof(uint32_t));
    r.chain = maniArenaTake(&gltf->ragdoll, n, sizeof(uint32_t));
    uint32_t* own = maniArenaTake(&gltf->ragdoll, n, sizeof(uint32_t));
    if (n > 0 && (r.bodies == NULL || r.bodyNodes == NULL || r.boneNodes == NULL ||
                  r.bodyWorld == NULL || r.nodeBody == NULL || r.chain == NULL || own == NULL))
    {
        return Capacity();
    }
    for (uint32_t k = 0; k < n; ++k)
    {
        r.nodeBody[k] = UINT32_MAX;
    }
    maniImportResult result = Read(&r, own);
    if (result.result != mani_success || r.bodyCount == 0)
    {
        sourceOut->skeleton = result.result == mani_success ? skeleton : NULL;
        return result;
    }
    maniQuat root = RootModel(&r, Root(&r));
    *sourceOut = (maniRagdollSource){skeleton, r.bodies, r.bodyCount, InRoot(&r, root, facing),
                                     InRoot(&r, root, up)};
    return result;
}

maniImportResult maniGltf_ReadRagdoll(maniGltf* gltf, const maniSkeleton* skeleton,
                                      const maniGltfOptions* options, maniVec3 facing, maniVec3 up,
                                      maniRagdollSource* sourceOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = GltfReadRagdollWork(gltf, skeleton, options, facing, up, sourceOut);
    maniFloatEnvLeave(env);
    return result;
}
