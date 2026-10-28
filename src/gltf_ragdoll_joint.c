// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// OMI joints as a ragdoll's joints: each joint node joins a
// body to its nearest ancestor body, its frame the child side's node's
// in the child body, the two sides together at rest; every linear axis
// fixed, angular limits as twist (x) and symmetric swings (y, z);
// angular force-mode drives equal on the three axes, without targets,
// as the drive. What a profile cannot hold is refused at its value.

#include "arena.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "gltf_ragdoll.h"
#include "json.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define AXES 3u
#define FREE HUGE_VALF

// The larger and the smaller of two floats (no libm call).
static float Larger(float a, float b)
{
    return a > b ? a : b;
}

static float Smaller(float a, float b)
{
    return a < b ? a : b;
}

static maniImportResult Fail(const maniGltfRagdollReader* r, maniImportReason reason,
                             uint32_t value, uint32_t item)
{
    return maniGltfRagdollFail(r, reason, value, item);
}

// The body of the nearest bone above body's that has one, or UINT32_MAX.
static uint32_t ParentBody(const maniGltfRagdollReader* r, uint32_t body)
{
    for (uint32_t bone = maniSkeletonParent(r->skeleton, r->bodies[body].bone);
         bone != MANI_NO_BONE; bone = maniSkeletonParent(r->skeleton, bone))
    {
        for (uint32_t j = 0; j < r->bodyCount; ++j)
        {
            if (r->bodies[j].bone == bone)
            {
                return j;
            }
        }
    }
    return UINT32_MAX;
}

// An array member's elements as axes 0 to 2 into *maskOut (bit per
// axis); absent gives 0.
static maniImportResult Axes(const maniGltfRagdollReader* r, uint32_t object, const char* name,
                             uint32_t item, uint32_t* maskOut)
{
    const maniJson* json = &r->gltf->json;
    uint32_t array = MANI_JSON_NONE;
    *maskOut = 0;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonArray, false, item, &array);
    if (result.result != mani_success || array == MANI_JSON_NONE)
    {
        return result;
    }
    uint32_t element = array + 1;
    for (uint32_t k = 0; k < json->values[array].count; ++k)
    {
        int64_t axis = -1;
        if (!maniJsonInteger(json, element, 0, AXES - 1, &axis))
        {
            return Fail(r, maniImportReasonProperty, element, item);
        }
        *maskOut |= 1u << axis;
        element = json->values[element].next;
    }
    return result;
}

typedef struct Limits
{
    float low[AXES];
    float high[AXES];
    uint32_t angular;
    uint32_t linear;
} Limits;

// One limit: linear axes fixed at 0, angular ones' ranges kept (an axis
// limited twice refused); a soft limit refused.
static maniImportResult ReadLimit(const maniGltfRagdollReader* r, uint32_t limit, uint32_t item,
                                  Limits* l)
{
    const maniJson* json = &r->gltf->json;
    uint32_t linear = 0;
    uint32_t angular = 0;
    float low = -FREE;
    float high = FREE;
    float stiffness = -1.0f;
    maniImportResult result = Axes(r, limit, "linearAxes", item, &linear);
    if (result.result == mani_success)
    {
        result = Axes(r, limit, "angularAxes", item, &angular);
    }
    if (result.result == mani_success && linear == 0 && angular == 0)
    {
        return Fail(r, maniImportReasonProperty, limit, item);
    }
    if (result.result == mani_success)
    {
        result = maniGltfRagdollNumber(r, limit, "min", item, &low);
    }
    if (result.result == mani_success)
    {
        result = maniGltfRagdollNumber(r, limit, "max", item, &high);
    }
    if (result.result == mani_success)
    {
        result = maniGltfRagdollNumber(r, limit, "stiffness", item, &stiffness);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    if (low > high || (angular & l->angular) != 0)
    {
        return Fail(r, maniImportReasonProperty, limit, item);
    }
    if (stiffness >= 0.0f)
    {
        uint32_t at = maniJsonFind(json, limit, "stiffness", 9);
        return Fail(r, maniImportReasonUnsupported, at, item);
    }
    if (linear != 0 && (low != 0.0f || high != 0.0f))
    {
        return Fail(r, maniImportReasonUnsupported, limit, item);
    }
    l->linear |= linear;
    l->angular |= angular;
    for (uint32_t a = 0; a < AXES; ++a)
    {
        if ((angular & (1u << a)) != 0)
        {
            l->low[a] = low;
            l->high[a] = high;
        }
    }
    return result;
}

// The settings' limits as the joint's twist and swings.
static maniImportResult ReadLimits(const maniGltfRagdollReader* r, uint32_t settings, uint32_t item,
                                   maniRagdollJointSource* j)
{
    const maniJson* json = &r->gltf->json;
    Limits l = {{-FREE, -FREE, -FREE}, {FREE, FREE, FREE}, 0, 0};
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, settings, "limits", maniJsonArray, false, item, &array);
    uint32_t count = array != MANI_JSON_NONE ? json->values[array].count : 0;
    uint32_t element = array + 1;
    for (uint32_t k = 0; k < count && result.result == mani_success; ++k)
    {
        result = json->values[element].kind == maniJsonObject
                     ? ReadLimit(r, element, item, &l)
                     : Fail(r, maniImportReasonProperty, element, item);
        element = json->values[element].next;
    }
    if (result.result != mani_success)
    {
        return result;
    }
    if (l.linear != 7u)
    {
        // A joint whose sides may part: not a ragdoll's.
        return Fail(r, maniImportReasonUnsupported, settings, item);
    }
    for (uint32_t a = 1; a < AXES; ++a)
    {
        float tolerance = 1e-6f * Larger(1.0f, fabsf(l.high[a]));
        if ((l.angular & (1u << a)) != 0 && fabsf(l.low[a] + l.high[a]) > tolerance)
        {
            return Fail(r, maniImportReasonUnsupported, array, item);
        }
    }
    j->twistMin = Larger(l.low[0], -PI_F);
    j->twistMax = Smaller(l.high[0], PI_F);
    j->twistMin = Smaller(j->twistMin, j->twistMax);
    j->swingY = Smaller(l.high[1], PI_F);
    j->swingZ = Smaller(l.high[2], PI_F);
    return result;
}

typedef struct Drive
{
    float stiffness;
    float damping;
    float maxTorque;
} Drive;

// A string member among names (count of them): its index in *kindOut,
// count when none matches.
static maniImportResult Word(const maniGltfRagdollReader* r, uint32_t object, const char* name,
                             const char* const* names, uint32_t count, uint32_t item,
                             uint32_t* kindOut, uint32_t* valueOut)
{
    maniImportResult result =
        maniGltfMember(&r->gltf->g, object, name, maniJsonString, true, item, valueOut);
    *kindOut = count;
    for (uint32_t k = 0; k < count && result.result == mani_success; ++k)
    {
        if (maniJsonStringEquals(&r->gltf->json, *valueOut, names[k], strlen(names[k])))
        {
            *kindOut = k;
        }
    }
    return result;
}

// A target member absent or 0.
static maniImportResult NoTarget(const maniGltfRagdollReader* r, uint32_t drive, const char* name,
                                 uint32_t item)
{
    float target = 0.0f;
    maniImportResult result = maniGltfRagdollNumber(r, drive, name, item, &target);
    if (result.result == mani_success && target != 0.0f)
    {
        uint32_t at = maniJsonFind(&r->gltf->json, drive, name, strlen(name));
        return Fail(r, maniImportReasonUnsupported, at, item);
    }
    return result;
}

// One drive: angular, force mode, no target; its axis's entry in drives.
static maniImportResult ReadDrive(const maniGltfRagdollReader* r, uint32_t drive, uint32_t item,
                                  Drive* drives, uint32_t* mask)
{
    static const char* const kTypes[2] = {"angular", "linear"};
    static const char* const kModes[2] = {"force", "acceleration"};
    uint32_t type = 0;
    uint32_t mode = 0;
    uint32_t at = MANI_JSON_NONE;
    int64_t axis = 0;
    Drive d = {0.0f, 0.0f, -1.0f};
    maniImportResult result = Word(r, drive, "type", kTypes, 2, item, &type, &at);
    if (result.result == mani_success && type != 0)
    {
        return Fail(r, type == 1 ? maniImportReasonUnsupported : maniImportReasonProperty, at,
                    item);
    }
    result = Word(r, drive, "mode", kModes, 2, item, &mode, &at);
    if (result.result == mani_success && mode != 0)
    {
        return Fail(r, mode == 1 ? maniImportReasonUnsupported : maniImportReasonProperty, at,
                    item);
    }
    if (result.result == mani_success)
    {
        result = maniGltfInteger(&r->gltf->g, drive, "axis", true, 0, AXES - 1, item, &axis);
    }
    if (result.result == mani_success && (*mask & (1u << axis)) != 0)
    {
        return Fail(r, maniImportReasonProperty, drive, item);
    }
    static const char* const kNumbers[3] = {"stiffness", "damping", "maxForce"};
    float* values[3] = {&d.stiffness, &d.damping, &d.maxTorque};
    for (uint32_t k = 0; k < 3 && result.result == mani_success; ++k)
    {
        result = maniGltfRagdollNumber(r, drive, kNumbers[k], item, values[k]);
        if (result.result == mani_success && k < 2 && !(*values[k] >= 0.0f))
        {
            at = maniJsonFind(&r->gltf->json, drive, kNumbers[k], strlen(kNumbers[k]));
            return Fail(r, maniImportReasonProperty, at, item);
        }
    }
    if (result.result == mani_success)
    {
        result = NoTarget(r, drive, "positionTarget", item);
    }
    if (result.result == mani_success)
    {
        result = NoTarget(r, drive, "velocityTarget", item);
    }
    // Unbounded (absent or negative): the largest finite float.
    d.maxTorque = d.maxTorque < 0.0f ? FLT_MAX : d.maxTorque;
    drives[axis] = d;
    *mask |= 1u << axis;
    return result;
}

// The settings' drives as the joint's one drive: none, or the same on
// the three angular axes.
static maniImportResult ReadDrives(const maniGltfRagdollReader* r, uint32_t settings, uint32_t item,
                                   maniRagdollJointSource* j)
{
    const maniJson* json = &r->gltf->json;
    Drive drives[AXES] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    uint32_t mask = 0;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfMember(&r->gltf->g, settings, "drives", maniJsonArray, false, item, &array);
    uint32_t count = array != MANI_JSON_NONE ? json->values[array].count : 0;
    uint32_t element = array + 1;
    for (uint32_t k = 0; k < count && result.result == mani_success; ++k)
    {
        result = json->values[element].kind == maniJsonObject
                     ? ReadDrive(r, element, item, drives, &mask)
                     : Fail(r, maniImportReasonProperty, element, item);
        element = json->values[element].next;
    }
    if (result.result != mani_success || mask == 0)
    {
        return result;
    }
    bool same = mask == 7u;
    for (uint32_t a = 1; a < AXES && same; ++a)
    {
        same = drives[a].stiffness == drives[0].stiffness &&
               drives[a].damping == drives[0].damping && drives[a].maxTorque == drives[0].maxTorque;
    }
    if (!same)
    {
        return Fail(r, maniImportReasonUnsupported, array, item);
    }
    bool none = drives[0].stiffness == 0.0f && drives[0].damping == 0.0f;
    j->driveStiffness = drives[0].stiffness;
    j->driveDamping = drives[0].damping;
    j->driveMaxTorque = none ? 0.0f : drives[0].maxTorque;
    return result;
}

// The two sides' frames together at rest: within a thousandth of a
// meter, and of a radian (the turn between them's half-angle sine).
static bool Together(maniGltfRigid a, maniGltfRigid b)
{
    maniVec3 d = maniVec3Sub(a.t, b.t);
    maniQuat turn = maniQuatMul(maniQuatConjugate(a.r), b.r);
    float sine = sqrtf(turn.x * turn.x + turn.y * turn.y + turn.z * turn.z);
    return maniVec3Dot(d, d) <= 1e-6f && sine <= 5e-4f;
}

// A joint node: its two bodies, the child's joint from the settings.
static maniImportResult ReadJoint(maniGltfRagdollReader* r, uint32_t node, uint32_t joint,
                                  uint8_t* jointed)
{
    const maniGltfJson* g = &r->gltf->g;
    uint32_t settings = MANI_JSON_NONE;
    uint32_t connected = MANI_JSON_NONE;
    maniImportResult result =
        maniGltfIndex(g, joint, "joint", true, r->settingsCount, node, &settings);
    if (result.result == mani_success)
    {
        result =
            maniGltfIndex(g, joint, "connectedNode", true, r->gltf->nodeCount, node, &connected);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    uint32_t a = r->nodeBody[node];
    uint32_t b = r->nodeBody[connected];
    if (a == UINT32_MAX || b == UINT32_MAX)
    {
        return Fail(r, maniImportReasonProperty, joint, node);
    }
    if (a == MANI_GLTF_NOT_OURS && b == MANI_GLTF_NOT_OURS)
    {
        return result;
    }
    bool aChild = a != MANI_GLTF_NOT_OURS && ParentBody(r, a) == b;
    bool bChild = b != MANI_GLTF_NOT_OURS && ParentBody(r, b) == a;
    if ((!aChild && !bChild) || jointed[aChild ? a : b] != 0)
    {
        return Fail(r, maniImportReasonUnsupported, joint, node);
    }
    uint32_t child = aChild ? a : b;
    maniGltfRigid side;
    maniGltfRigid other;
    result = maniGltfRagdollWorld(r, aChild ? node : connected, &side, NULL);
    if (result.result == mani_success)
    {
        result = maniGltfRagdollWorld(r, aChild ? connected : node, &other, NULL);
    }
    if (result.result == mani_success && !Together(side, other))
    {
        return Fail(r, maniImportReasonUnsupported, joint, node);
    }
    const maniJson* json = &r->gltf->json;
    uint32_t object = r->settings + 1;
    for (uint32_t k = 0; k < settings; ++k)
    {
        object = json->values[object].next;
    }
    if (result.result == mani_success && json->values[object].kind != maniJsonObject)
    {
        return Fail(r, maniImportReasonProperty, object, node);
    }
    maniRagdollJointSource* j = &r->bodies[child].joint;
    maniGltfRigid frame = maniGltfRigidIn(r->bodyWorld[child], side);
    j->translation = frame.t;
    j->rotation = frame.r;
    if (result.result == mani_success)
    {
        result = ReadLimits(r, object, node, j);
    }
    if (result.result == mani_success)
    {
        result = ReadDrives(r, object, node, j);
    }
    jointed[child] = 1;
    return result;
}

maniImportResult maniGltfReadRagdollJoints(maniGltfRagdollReader* r)
{
    uint8_t* jointed = maniArenaTake(&r->gltf->ragdoll, r->bodyCount + 1u, 1);
    if (jointed == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    memset(jointed, 0, r->bodyCount + 1u);
    maniImportResult result = maniImportSuccess();
    for (uint32_t node = 0; node < r->gltf->nodeCount && result.result == mani_success; ++node)
    {
        uint32_t joint = MANI_JSON_NONE;
        result = maniGltfRagdollExtension(r, r->gltf->nodeValues[node], "OMI_physics_joint", node,
                                          &joint);
        if (result.result == mani_success && joint != MANI_JSON_NONE)
        {
            result = ReadJoint(r, node, joint, jointed);
        }
    }
    for (uint32_t i = 0; i < r->bodyCount && result.result == mani_success; ++i)
    {
        if (jointed[i] == 0 && ParentBody(r, i) != UINT32_MAX)
        {
            uint32_t body = MANI_JSON_NONE;
            (void)maniGltfRagdollExtension(r, r->gltf->nodeValues[r->bodyNodes[i]],
                                           "OMI_physics_body", r->bodyNodes[i], &body);
            return Fail(r, maniImportReasonUnsupported, body, r->bodyNodes[i]);
        }
    }
    return result;
}
