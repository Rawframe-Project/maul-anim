// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// `VRMC_springBone` 1.0 as spring bone sources: colliders,
// collider groups and springs read with the schema's defaults and
// ranges, the specification's rules on chains and centers checked,
// nodes bound to bones by name, vectors moved into the output's basis.
// The sources live in the document's spring arena until the next read.

#include "arena.h"
#include "fpenv.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_names.h"
#include "gltf_property.h"
#include "json.h"
#include "spring_frame.h"
#include "vecmath.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
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

// A number member that must be present, in [minimum, maximum].
static maniImportResult RequiredNumber(const Reader* r, uint32_t object, const char* name,
                                       float minimum, float maximum, uint32_t item, float* out)
{
    uint32_t value = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonNumber, true, item, &value);
    return result.result == mani_success ? Number(r, object, name, minimum, maximum, item, out)
                                         : result;
}

// Within a thousandth of length 1, as the spring builder takes it.
static bool Unit(float lengthSquared)
{
    return lengthSquared >= 0.998f && lengthSquared <= 1.002f;
}

// A unit quaternion member (x, y, z, w), its default when absent; not
// moved into the output's basis.
static maniImportResult Quat(const Reader* r, uint32_t object, const char* name, uint32_t item,
                             maniQuat* out)
{
    const maniJson* json = &r->gltf->json;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonArray, false, item, &array);
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    if (json->values[array].count != 4)
    {
        return Fail(r, maniImportReasonProperty, array, item);
    }
    float q[4];
    uint32_t element = array + 1;
    for (uint32_t k = 0; k < 4; ++k)
    {
        uint64_t bits = 0;
        if (!maniJsonBinary(json, element, maniBinary32, &bits))
        {
            return Fail(r, maniImportReasonProperty, element, item);
        }
        uint32_t word = (uint32_t)bits;
        memcpy(&q[k], &word, sizeof word);
        element = json->values[element].next;
    }
    maniQuat v = {q[0], q[1], q[2], q[3]};
    if (!Unit(maniQuatDot(v, v)))
    {
        return Fail(r, maniImportReasonProperty, array, item);
    }
    *out = v;
    return result;
}

// An extension in an object's extensions: *out its object, or
// MANI_JSON_NONE when absent or of a version other than 1.0 (ignored,
// as a reader without the extension would); a version not a string is
// refused.
static maniImportResult Extension(const Reader* r, uint32_t object, const char* name, uint32_t item,
                                  uint32_t* out)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t extensions = MANI_JSON_NONE;
    uint32_t extension = MANI_JSON_NONE;
    uint32_t version = MANI_JSON_NONE;
    *out = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(g, object, "extensions", maniJsonObject, false, item, &extensions);
    if (result.result == mani_success && extensions != MANI_JSON_NONE)
    {
        result = maniGltfMember(g, extensions, name, maniJsonObject, false, item, &extension);
    }
    if (result.result != mani_success || extension == MANI_JSON_NONE)
    {
        return result;
    }
    result = maniGltfMember(g, extension, "specVersion", maniJsonString, true, item, &version);
    if (result.result == mani_success && maniJsonStringEquals(&r->gltf->json, version, "1.0", 3))
    {
        *out = extension;
    }
    return result;
}

// The one member of object named in names (count of them), its index in
// *kindOut and its value in *valueOut; none or more than one refused.
static maniImportResult OneOf(const Reader* r, uint32_t object, const char* const* names,
                              uint32_t count, uint32_t item, uint32_t* kindOut, uint32_t* valueOut)
{
    uint32_t found = 0;
    for (uint32_t k = 0; k < count; ++k)
    {
        uint32_t value = MANI_JSON_NONE;
        maniImportResult result =
            maniGltfMember(&r->gltf->g, object, names[k], maniJsonObject, false, item, &value);
        if (result.result != mani_success)
        {
            return result;
        }
        if (value != MANI_JSON_NONE)
        {
            *kindOut = k;
            *valueOut = value;
            ++found;
        }
    }
    return found == 1 ? maniImportSuccess() : Fail(r, maniImportReasonProperty, object, item);
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

// A collider's VRMC_springBone_extended_collider shape in place of its
// fallback: a sphere or capsule, outside or inside, or a plane (its
// unit normal in tail).
static maniImportResult ReadExtendedShape(const Reader* r, uint32_t collider, uint32_t item,
                                          maniColliderSource* c)
{
    static const char* const kShapes[3] = {"sphere", "capsule", "plane"};
    uint32_t extension = MANI_JSON_NONE;
    uint32_t shape = MANI_JSON_NONE;
    maniImportResult result =
        Extension(r, collider, "VRMC_springBone_extended_collider", item, &extension);
    if (result.result == mani_success && extension != MANI_JSON_NONE)
    {
        result =
            maniGltfMember(&r->gltf->g, extension, "shape", maniJsonObject, false, item, &shape);
    }
    if (result.result != mani_success || shape == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t kind = 0;
    uint32_t body = MANI_JSON_NONE;
    result = OneOf(r, shape, kShapes, 3, item, &kind, &body);
    if (result.result != mani_success)
    {
        return result;
    }
    bool plane = kind == maniColliderPlane;
    *c = (maniColliderSource){
        .bone = c->bone, .shape = kind, .tail = {0.0f, 0.0f, plane ? 1.0f : 0.0f}};
    result = Vector(r, body, "offset", item, &c->offset);
    if (result.result == mani_success && !plane)
    {
        bool inside = false;
        result = Number(r, body, "radius", 0.0f, 3.4e38f, item, &c->radius);
        if (result.result == mani_success)
        {
            result = maniGltfBool(&r->gltf->g, body, "inside", item, &inside);
        }
        c->inside = inside ? 1u : 0u;
    }
    if (result.result == mani_success && kind != maniColliderSphere)
    {
        result = Vector(r, body, plane ? "normal" : "tail", item, &c->tail);
    }
    if (result.result == mani_success && plane && !Unit(maniVec3Dot(c->tail, c->tail)))
    {
        uint32_t at = maniJsonFind(&r->gltf->json, body, "normal", 6);
        return Fail(r, maniImportReasonProperty, at, item);
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
        if (result.result == mani_success)
        {
            result = ReadExtendedShape(r, value, i, &colliders[i]);
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

// A joint's VRMC_springBone_limit: a cone, hinge or spherical limit, its
// angles as written (the builder clamps them) and its rotation.
static maniImportResult ReadLimit(const Reader* r, uint32_t joint, uint32_t item,
                                  maniSpringJointSource* j)
{
    static const char* const kLimits[3] = {"cone", "hinge", "spherical"};
    uint32_t extension = MANI_JSON_NONE;
    uint32_t limit = MANI_JSON_NONE;
    maniImportResult result = Extension(r, joint, "VRMC_springBone_limit", item, &extension);
    if (result.result == mani_success && extension != MANI_JSON_NONE)
    {
        result =
            maniGltfMember(&r->gltf->g, extension, "limit", maniJsonObject, true, item, &limit);
    }
    if (result.result != mani_success || limit == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t kind = 0;
    uint32_t body = MANI_JSON_NONE;
    result = OneOf(r, limit, kLimits, 3, item, &kind, &body);
    if (result.result != mani_success)
    {
        return result;
    }
    j->limit = kind + 1;
    bool spherical = j->limit == maniSpringLimitSpherical;
    result =
        RequiredNumber(r, body, spherical ? "pitch" : "angle", 0.0f, 3.4e38f, item, &j->limitAngle);
    if (result.result == mani_success && spherical)
    {
        result = RequiredNumber(r, body, "yaw", 0.0f, 3.4e38f, item, &j->limitYaw);
    }
    if (result.result == mani_success)
    {
        result = Quat(r, body, "rotation", item, &j->limitRotation);
    }
    return result;
}

static maniImportResult ReadJoint(const Reader* r, uint32_t value, uint32_t item, bool last,
                                  uint32_t* node, maniSpringJointSource* j)
{
    *j = (maniSpringJointSource){.stiffness = 1.0f,
                                 .gravityDirection = {0, -1, 0},
                                 .drag = 0.5f,
                                 .limitRotation = {0, 0, 0, 1}};
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
    // The last joint is only a tail: its limit is ignored, unread.
    if (result.result == mani_success && !last)
    {
        result = ReadLimit(r, value, item, j);
    }
    return result;
}

// Whether the options change no axis.
static bool BasisKept(const maniGltfOptions* options)
{
    return options->axes[0] == maniAxisX && options->axes[1] == maniAxisY &&
           options->axes[2] == maniAxisZ;
}

static maniVec3 BasisAxis(const maniGltfOptions* options, float x, float y, float z)
{
    float v[3] = {x, y, z};
    maniGltfBasisVector(options, v, true);
    return (maniVec3){v[0], v[1], v[2]};
}

// The rotation of a matrix of columns c0, c1, c2, a rotation whose
// entries are 0 or ±1.
static maniQuat FromColumns(maniVec3 c0, maniVec3 c1, maniVec3 c2)
{
    float trace = c0.x + c1.y + c2.z;
    if (trace > 0.0f)
    {
        float s = 2.0f * sqrtf(trace + 1.0f);
        return (maniQuat){(c1.z - c2.y) / s, (c2.x - c0.z) / s, (c0.y - c1.x) / s, 0.25f * s};
    }
    if (c0.x > c1.y && c0.x > c2.z)
    {
        float s = 2.0f * sqrtf(1.0f + c0.x - c1.y - c2.z);
        return (maniQuat){0.25f * s, (c1.x + c0.y) / s, (c2.x + c0.z) / s, (c1.z - c2.y) / s};
    }
    if (c1.y > c2.z)
    {
        float s = 2.0f * sqrtf(1.0f + c1.y - c0.x - c2.z);
        return (maniQuat){(c1.x + c0.y) / s, 0.25f * s, (c2.y + c1.z) / s, (c2.x - c0.z) / s};
    }
    float s = 2.0f * sqrtf(1.0f + c2.z - c0.x - c1.y);
    return (maniQuat){(c2.x + c0.z) / s, (c2.y + c1.z) / s, 0.25f * s, (c0.y - c1.x) / s};
}

// A limit's rotation R from glTF's basis B to the output's: the frame
// the builder makes, the turn from +y to the output's rest direction a'
// times R', is B F M with F = D(a) R glTF's frame and M the mirror of
// the limit's x when det B < 0, so R' = D'(a')* Arc(B y, a') (B R B*) (B M).
static maniQuat LimitRotation(const Reader* r, uint32_t head, uint32_t tail, maniQuat rotation)
{
    const maniGltfOptions* o = r->options;
    if (BasisKept(o))
    {
        return rotation;
    }
    maniVec3 bx = BasisAxis(o, 1.0f, 0.0f, 0.0f);
    maniVec3 by = BasisAxis(o, 0.0f, 1.0f, 0.0f);
    maniVec3 bz = BasisAxis(o, 0.0f, 0.0f, 1.0f);
    bool mirror = maniVec3Dot(maniVec3Cross(bx, by), bz) < 0.0f;
    maniVec3 a = maniSpringRestDirection(r->skeleton, head, tail);
    maniVec3 opposite = maniVec3Scale(by, -1.0f);
    // B D(a) B* turns B y onto a' as D(a) turns y onto a; half a turn
    // about B x for a' opposite B y, as D's about x.
    maniQuat arc = a.x == opposite.x && a.y == opposite.y && a.z == opposite.z
                       ? (maniQuat){bx.x, bx.y, bx.z, 0.0f}
                       : maniQuatBetween(by, a);
    float q[4] = {rotation.x, rotation.y, rotation.z, rotation.w};
    maniGltfBasisQuat(o, q);
    maniQuat turned = {q[0], q[1], q[2], q[3]};
    maniQuat k = FromColumns(mirror ? maniVec3Scale(bx, -1.0f) : bx, by, bz);
    maniQuat frame = maniQuatMul(maniQuatMul(arc, turned), k);
    return maniQuatNormalize(maniQuatMul(maniQuatConjugate(maniSpringTurnFromY(a)), frame));
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
        result = ReadJoint(r, value, item, k + 1 == n, &node, &joints[k]);
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
    for (uint32_t k = 0; k + 1 < n; ++k)
    {
        if (joints[k].limit != maniSpringLimitNone)
        {
            joints[k].limitRotation =
                LimitRotation(r, joints[k].bone, joints[k + 1].bone, joints[k].limitRotation);
        }
    }
    s->joints = joints;
    s->jointCount = n;
    return result;
}

static maniImportResult ReadSpring(Reader* r, uint32_t spring, uint32_t item, uint32_t* owner,
                                   maniSpringSource* s, uint32_t* centerOut)
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
    *centerOut = center;
    if (result.result == mani_success && center != MANI_JSON_NONE && center != first &&
        !IsAncestor(r->gltf, center, first))
    {
        uint32_t at = maniJsonFind(&r->gltf->json, spring, "center", 6);
        return Fail(r, maniImportReasonHierarchy, at, item);
    }
    return result;
}

// No center is another chain's joint or below one, once every chain's
// joints are known.
static maniImportResult CheckCenters(const Reader* r, uint32_t array, const uint32_t* owner,
                                     const uint32_t* centers)
{
    const maniGltf* gltf = r->gltf;
    uint32_t spring = array + 1;
    for (uint32_t i = 0; i < gltf->json.values[array].count; ++i)
    {
        for (uint32_t v = centers[i]; v != MANI_JSON_NONE; v = gltf->nodeParents[v])
        {
            if (owner[v] != 0 && owner[v] != i + 1)
            {
                uint32_t at = maniJsonFind(&gltf->json, spring, "center", 6);
                return Fail(r, maniImportReasonHierarchy, at, i);
            }
        }
        spring = gltf->json.values[spring].next;
    }
    return maniImportSuccess();
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
    uint32_t* centers = maniArenaTake(&r->gltf->springs, n, sizeof(uint32_t));
    if (centers == NULL)
    {
        return Capacity();
    }
    uint32_t value = array + 1;
    for (uint32_t i = 0; result.result == mani_success && i < n; ++i)
    {
        result = ReadSpring(r, value, i, owner, &springs[i], &centers[i]);
        value = r->gltf->json.values[value].next;
    }
    if (result.result == mani_success)
    {
        result = CheckCenters(r, array, owner, centers);
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

static MANI_NOINLINE maniImportResult GltfReadSpringsWork(maniGltf* gltf,
                                                          const maniSkeleton* skeleton,
                                                          const maniGltfOptions* options,
                                                          maniSpringSetSource* setOut)
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

maniImportResult maniGltfReadSprings(maniGltf* gltf, const maniSkeleton* skeleton,
                                     const maniGltfOptions* options, maniSpringSetSource* setOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = GltfReadSpringsWork(gltf, skeleton, options, setOut);
    maniFloatEnvLeave(env);
    return result;
}
