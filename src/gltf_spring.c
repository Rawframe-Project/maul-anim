// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// `VRMC_springBone` 1.0 as spring bone sources (mani-0011): colliders,
// collider groups and springs read with the schema's defaults and
// ranges, the specification's rules on chains and centers checked,
// nodes bound to bones by name, vectors moved into the output's basis.
// The sources live in the document's spring arena until the next read.

#include "arena.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_names.h"
#include "gltf_property.h"
#include "json.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <string.h>

typedef struct Reader
{
    maniGltf* gltf;
    const maniSkeleton* skeleton;
    const maniGltfOptions* options;
    maniSpringSetSource* set;
} Reader;

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Fail(const Reader* r, maniImportReason reason, uint32_t value,
                             uint32_t item)
{
    return maniImportFail(reason, maniGltfOffset(&r->gltf->g, value), item);
}

// A number member in [minimum, maximum], its default when absent.
static maniImportResult Number(const Reader* r, uint32_t object, const char* name, float minimum,
                               float maximum, uint32_t item, float* out)
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
    if (!fits || !(f >= minimum) || !(f <= maximum))
    {
        return Fail(r, maniImportReasonProperty, value, item);
    }
    *out = f;
    return result;
}

// Three numbers, their default when absent, moved into the output's
// basis as a translation is.
static maniImportResult Vector(const Reader* r, uint32_t object, const char* name, uint32_t item,
                               maniVec3* out)
{
    const maniJson* json = &r->gltf->json;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonArray, false, item, &array);
    if (result.result != mani_success)
    {
        return result;
    }
    float v[3] = {out->x, out->y, out->z};
    if (array != MANI_JSON_NONE)
    {
        if (json->values[array].count != 3)
        {
            return Fail(r, maniImportReasonProperty, array, item);
        }
        uint32_t element = array + 1;
        for (uint32_t k = 0; k < 3; ++k)
        {
            uint64_t bits = 0;
            if (!maniJsonBinary(json, element, maniBinary32, &bits))
            {
                return Fail(r, maniImportReasonProperty, element, item);
            }
            uint32_t word = (uint32_t)bits;
            memcpy(&v[k], &word, sizeof word);
            element = json->values[element].next;
        }
    }
    maniGltfBasisVector(r->options, v, true);
    *out = (maniVec3){v[0], v[1], v[2]};
    return result;
}

// A node member as a bone of the skeleton; *nodeOut gets the node.
static maniImportResult Bone(const Reader* r, uint32_t object, const char* name, bool required,
                             uint32_t item, uint32_t* nodeOut, uint32_t* boneOut)
{
    maniImportResult result =
        maniGltfIndex(&r->gltf->g, object, name, required, r->gltf->nodeCount, item, nodeOut);
    *boneOut = MANI_NO_BONE;
    if (result.result != mani_success || *nodeOut == MANI_JSON_NONE)
    {
        return result;
    }
    *boneOut = maniGltfNodeBone(r->gltf, r->skeleton, *nodeOut);
    if (*boneOut == MANI_NO_BONE)
    {
        uint32_t at = maniJsonFind(&r->gltf->json, object, name, strlen(name));
        return Fail(r, maniImportReasonBone, at, item);
    }
    return result;
}

// An optional name, decoded into the arena.
static maniImportResult Name(Reader* r, uint32_t object, uint32_t item, const char** name,
                             uint32_t* length)
{
    uint32_t value = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, "name", maniJsonString, false, item, &value);
    *name = NULL;
    *length = 0;
    if (result.result != mani_success || value == MANI_JSON_NONE)
    {
        return result;
    }
    size_t raw = r->gltf->json.values[value].length;
    uint8_t* text = maniArenaTake(&r->gltf->springs, raw + 1, 1);
    if (text == NULL)
    {
        return Capacity();
    }
    *length = (uint32_t)maniJsonStringDecode(&r->gltf->json, value, text, raw);
    *name = (const char*)text;
    return result;
}

// An array member's elements as indices below count, into the arena.
static maniImportResult Indices(Reader* r, uint32_t object, const char* name, bool required,
                                uint32_t count, uint32_t item, const uint32_t** out,
                                uint32_t* countOut)
{
    const maniJson* json = &r->gltf->json;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonArray, required, item, &array);
    *out = NULL;
    *countOut = 0;
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t n = json->values[array].count;
    uint32_t* indices = maniArenaTake(&r->gltf->springs, n, sizeof(uint32_t));
    if (indices == NULL)
    {
        return Capacity();
    }
    if (n == 0)
    {
        return Fail(r, maniImportReasonProperty, array, item);
    }
    uint32_t element = array + 1;
    for (uint32_t k = 0; k < n; ++k)
    {
        int64_t index = -1;
        if (!maniJsonInteger(json, element, 0, (int64_t)count - 1, &index))
        {
            bool whole = maniJsonInteger(json, element, 0, INT64_MAX, &index);
            return Fail(r, whole ? maniImportReasonIndex : maniImportReasonProperty, element, item);
        }
        indices[k] = (uint32_t)index;
        element = json->values[element].next;
    }
    *out = indices;
    *countOut = n;
    return result;
}

static maniImportResult ReadShape(const Reader* r, uint32_t collider, uint32_t item,
                                  maniColliderSource* c)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t shape = MANI_JSON_NONE;
    uint32_t sphere = MANI_JSON_NONE;
    uint32_t capsule = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, collider, "shape", maniJsonObject, true, item, &shape);
    if (result.result == mani_success)
    {
        result = maniGltfMember(g, shape, "sphere", maniJsonObject, false, item, &sphere);
    }
    if (result.result == mani_success)
    {
        result = maniGltfMember(g, shape, "capsule", maniJsonObject, false, item, &capsule);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    if ((sphere == MANI_JSON_NONE) == (capsule == MANI_JSON_NONE))
    {
        return Fail(r, maniImportReasonProperty, shape, item);
    }
    uint32_t body = sphere != MANI_JSON_NONE ? sphere : capsule;
    c->shape = sphere != MANI_JSON_NONE ? maniColliderSphere : maniColliderCapsule;
    result = Vector(r, body, "offset", item, &c->offset);
    if (result.result == mani_success)
    {
        result = Number(r, body, "radius", 0.0f, 3.4e38f, item, &c->radius);
    }
    if (result.result == mani_success && c->shape == maniColliderCapsule)
    {
        result = Vector(r, body, "tail", item, &c->tail);
    }
    return result;
}

static maniImportResult ReadColliders(Reader* r, uint32_t extension)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, extension, "colliders", maniJsonArray, false, 0, &array);
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t n = r->gltf->json.values[array].count;
    maniColliderSource* colliders = maniArenaTake(&r->gltf->springs, n, sizeof(maniColliderSource));
    if (colliders == NULL)
    {
        return Capacity();
    }
    uint32_t value = array + 1;
    for (uint32_t i = 0; result.result == mani_success && i < n; ++i)
    {
        uint32_t node = MANI_JSON_NONE;
        if (r->gltf->json.values[value].kind != maniJsonObject)
        {
            return Fail(r, maniImportReasonProperty, value, i);
        }
        result = Bone(r, value, "node", true, i, &node, &colliders[i].bone);
        if (result.result == mani_success)
        {
            result = ReadShape(r, value, i, &colliders[i]);
        }
        value = r->gltf->json.values[value].next;
    }
    r->set->colliders = colliders;
    r->set->colliderCount = n;
    return result;
}

static maniImportResult ReadGroups(Reader* r, uint32_t extension)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, extension, "colliderGroups", maniJsonArray, false, 0, &array);
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t n = r->gltf->json.values[array].count;
    maniColliderGroupSource* groups =
        maniArenaTake(&r->gltf->springs, n, sizeof(maniColliderGroupSource));
    if (groups == NULL)
    {
        return Capacity();
    }
    uint32_t value = array + 1;
    for (uint32_t i = 0; result.result == mani_success && i < n; ++i)
    {
        if (r->gltf->json.values[value].kind != maniJsonObject)
        {
            return Fail(r, maniImportReasonProperty, value, i);
        }
        result = Name(r, value, i, &groups[i].name, &groups[i].nameLength);
        if (result.result == mani_success)
        {
            result = Indices(r, value, "colliders", true, r->set->colliderCount, i,
                             &groups[i].colliders, &groups[i].colliderCount);
        }
        value = r->gltf->json.values[value].next;
    }
    r->set->colliderGroups = groups;
    r->set->colliderGroupCount = n;
    return result;
}

static bool IsAncestor(const maniGltf* gltf, uint32_t ancestor, uint32_t node)
{
    for (uint32_t v = gltf->nodeParents[node]; v != MANI_JSON_NONE; v = gltf->nodeParents[v])
    {
        if (v == ancestor)
        {
            return true;
        }
    }
    return false;
}

static maniImportResult ReadJoint(const Reader* r, uint32_t value, uint32_t item, uint32_t* node,
                                  maniSpringJointSource* j)
{
    *j = (maniSpringJointSource){.stiffness = 1.0f, .gravityDirection = {0, -1, 0}, .drag = 0.5f};
    if (r->gltf->json.values[value].kind != maniJsonObject)
    {
        return Fail(r, maniImportReasonProperty, value, item);
    }
    maniImportResult result = Bone(r, value, "node", true, item, node, &j->bone);
    if (result.result == mani_success)
    {
        result = Number(r, value, "hitRadius", 0.0f, 3.4e38f, item, &j->hitRadius);
    }
    if (result.result == mani_success)
    {
        result = Number(r, value, "stiffness", 0.0f, 3.4e38f, item, &j->stiffness);
    }
    if (result.result == mani_success)
    {
        result = Number(r, value, "gravityPower", 0.0f, 3.4e38f, item, &j->gravityPower);
    }
    if (result.result == mani_success)
    {
        result = Vector(r, value, "gravityDir", item, &j->gravityDirection);
    }
    if (result.result == mani_success)
    {
        result = Number(r, value, "dragForce", 0.0f, 1.0f, item, &j->drag);
    }
    return result;
}

// A spring's joints: each an ancestor of the next, none in an earlier
// spring (owner marks each joint node with its spring + 1).
static maniImportResult ReadJoints(Reader* r, uint32_t spring, uint32_t item, uint32_t* owner,
                                   maniSpringSource* s, uint32_t* firstNode)
{
    const maniJson* json = &r->gltf->json;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, spring, "joints", maniJsonArray, true, item, &array);
    if (result.result != mani_success)
    {
        return result;
    }
    uint32_t n = json->values[array].count;
    maniSpringJointSource* joints =
        maniArenaTake(&r->gltf->springs, n, sizeof(maniSpringJointSource));
    if (joints == NULL)
    {
        return Capacity();
    }
    if (n == 0)
    {
        return Fail(r, maniImportReasonProperty, array, item);
    }
    uint32_t value = array + 1;
    uint32_t previous = MANI_JSON_NONE;
    for (uint32_t k = 0; result.result == mani_success && k < n; ++k)
    {
        uint32_t node = MANI_JSON_NONE;
        result = ReadJoint(r, value, item, &node, &joints[k]);
        if (result.result != mani_success)
        {
            return result;
        }
        if (owner[node] != 0)
        {
            return Fail(r, maniImportReasonProperty, value, item);
        }
        owner[node] = item + 1;
        if (previous != MANI_JSON_NONE && !IsAncestor(r->gltf, previous, node))
        {
            return Fail(r, maniImportReasonHierarchy, value, item);
        }
        *firstNode = k == 0 ? node : *firstNode;
        previous = node;
        value = json->values[value].next;
    }
    s->joints = joints;
    s->jointCount = n;
    return result;
}

static maniImportResult ReadSpring(Reader* r, uint32_t spring, uint32_t item, uint32_t* owner,
                                   maniSpringSource* s)
{
    if (r->gltf->json.values[spring].kind != maniJsonObject)
    {
        return Fail(r, maniImportReasonProperty, spring, item);
    }
    uint32_t first = MANI_JSON_NONE;
    uint32_t center = MANI_JSON_NONE;
    maniImportResult result = Name(r, spring, item, &s->name, &s->nameLength);
    if (result.result == mani_success)
    {
        result = ReadJoints(r, spring, item, owner, s, &first);
    }
    if (result.result == mani_success)
    {
        result = Indices(r, spring, "colliderGroups", false, r->set->colliderGroupCount, item,
                         &s->colliderGroups, &s->colliderGroupCount);
    }
    if (result.result == mani_success)
    {
        result = Bone(r, spring, "center", false, item, &center, &s->center);
    }
    if (result.result == mani_success && center != MANI_JSON_NONE && center != first &&
        !IsAncestor(r->gltf, center, first))
    {
        uint32_t at = maniJsonFind(&r->gltf->json, spring, "center", 6);
        return Fail(r, maniImportReasonHierarchy, at, item);
    }
    return result;
}

static maniImportResult ReadSprings(Reader* r, uint32_t extension)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, extension, "springs", maniJsonArray, false, 0, &array);
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t n = r->gltf->json.values[array].count;
    maniSpringSource* springs = maniArenaTake(&r->gltf->springs, n, sizeof(maniSpringSource));
    uint32_t* owner = maniArenaTake(&r->gltf->springs, r->gltf->nodeCount, sizeof(uint32_t));
    if (springs == NULL || owner == NULL)
    {
        return Capacity();
    }
    uint32_t value = array + 1;
    for (uint32_t i = 0; result.result == mani_success && i < n; ++i)
    {
        result = ReadSpring(r, value, i, owner, &springs[i]);
        value = r->gltf->json.values[value].next;
    }
    r->set->springs = springs;
    r->set->springCount = n;
    return result;
}

static maniImportResult Read(Reader* r)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t extensions = MANI_JSON_NONE;
    uint32_t extension = MANI_JSON_NONE;
    uint32_t version = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, 0, "extensions", maniJsonObject, false, 0, &extensions);
    if (result.result == mani_success && extensions != MANI_JSON_NONE)
    {
        result =
            maniGltfMember(g, extensions, "VRMC_springBone", maniJsonObject, false, 0, &extension);
    }
    if (result.result != mani_success || extension == MANI_JSON_NONE)
    {
        return result;
    }
    result = maniGltfMember(g, extension, "specVersion", maniJsonString, true, 0, &version);
    if (result.result != mani_success)
    {
        return result;
    }
    if (!maniJsonStringEquals(&r->gltf->json, version, "1.0", 3))
    {
        return Fail(r, maniImportReasonVersion, version, 0);
    }
    result = ReadColliders(r, extension);
    if (result.result == mani_success)
    {
        result = ReadGroups(r, extension);
    }
    if (result.result == mani_success)
    {
        result = ReadSprings(r, extension);
    }
    return result;
}

maniImportResult maniGltfReadSprings(maniGltf* gltf, const maniSkeleton* skeleton,
                                     const maniGltfOptions* options, maniSpringSetSource* setOut)
{
    if (gltf == NULL || skeleton == NULL || options == NULL || setOut == NULL ||
        !maniGltfOptionsValid(options))
    {
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    maniArenaRelease(&gltf->springs);
    gltf->springs = maniArenaMake(&gltf->allocator);
    maniSpringSetSource set = {.skeleton = skeleton};
    Reader r = {gltf, skeleton, options, &set};
    maniImportResult result = Read(&r);
    *setOut = result.result == mani_success ? set : (maniSpringSetSource){0};
    return result;
}
