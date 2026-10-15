// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX files before 6.0 (mani-0017), as ufbx reads them. Every top-level
// Model is an object keyed by its name; a model's Children list names
// its children; a mesh's Link children are the clusters of one skin per
// mesh, each linking the model it names. The objects get synthetic ids
// for the skin and its clusters, which nothing names. A model has no
// property entries: the last channel driving one of its Lcl properties
// in file order (any take's, or the model's own) gives its rest value.

#include "fbx_legacy.h"

#include "allocator.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_take.h"
#include "fbx_value.h"

#include <stdbool.h>
#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Refuse(const maniFbx* fbx, uint32_t node)
{
    return maniFbxFail(maniImportReasonProperty, fbx->nodes[node].nameOffset, 0);
}

static bool Named(const maniFbx* fbx, uint32_t node)
{
    const uint8_t* bytes = NULL;
    uint32_t length = 0;
    return maniFbxString(fbx, maniFbxNodeProperty(fbx, node, 0), &bytes, &length);
}

// Whether a model's links are read: a mesh's (with Vertices and
// PolygonVertexIndex), as ufbx reads them.
static bool Mesh(const maniFbx* fbx, uint32_t node)
{
    return maniFbxFindChild(fbx, node, "Vertices", 8) != MANI_FBX_NONE &&
           maniFbxFindChild(fbx, node, "PolygonVertexIndex", 18) != MANI_FBX_NONE;
}

// A synthetic id for a skin or a cluster, unlike any name's hash but by
// a 64-bit collision.
static int64_t SyntheticId(uint32_t node, uint32_t role)
{
    return INT64_MIN + (int64_t)node * 2 + (int64_t)(role - maniFbxRoleLegacySkin);
}

static void Put(maniFbx* fbx, uint32_t at, uint32_t node, uint32_t role)
{
    if (fbx->objects != NULL)
    {
        fbx->objects[at] =
            (maniFbxObject){SyntheticId(node, role), node, MANI_FBX_NONE, MANI_FBX_NONE, role};
    }
}

// A mesh model's skin and its clusters, one per Link, after the model;
// counted (objects NULL) or listed. A Link without its name is refused.
static maniImportResult Links(maniFbx* fbx, uint32_t model, uint32_t* count)
{
    bool skin = false;
    for (uint32_t k = Mesh(fbx, model) ? fbx->nodes[model].firstChild : MANI_FBX_NONE;
         k != MANI_FBX_NONE; k = fbx->nodes[k].next)
    {
        if (!maniFbxNameIs(fbx, k, "Link", 4))
        {
            continue;
        }
        if (!Named(fbx, k))
        {
            return Refuse(fbx, k);
        }
        if (!skin)
        {
            Put(fbx, (*count)++, model, maniFbxRoleLegacySkin);
            skin = true;
        }
        Put(fbx, (*count)++, k, maniFbxRoleLegacyCluster);
    }
    return maniFbxSuccess();
}

// The objects: counted (objects NULL) or listed. A Model without its
// name is refused, as ufbx refuses the file.
static maniImportResult EachObject(maniFbx* fbx, uint32_t* count)
{
    *count = 0;
    maniImportResult r = maniFbxSuccess();
    for (uint32_t m = fbx->nodes[0].firstChild; r.result == mani_success && m != MANI_FBX_NONE;
         m = fbx->nodes[m].next)
    {
        if (!maniFbxNameIs(fbx, m, "Model", 5))
        {
            continue;
        }
        if (!Named(fbx, m))
        {
            return Refuse(fbx, m);
        }
        bool ok = false;
        if (fbx->objects != NULL)
        {
            fbx->objects[*count] =
                (maniFbxObject){maniFbxNameKey(fbx, maniFbxNodeProperty(fbx, m, 0), &ok), m,
                                MANI_FBX_NONE, MANI_FBX_NONE, maniFbxRoleNone};
        }
        ++*count;
        r = Links(fbx, m, count);
    }
    return r;
}

static maniImportResult Objects(maniFbx* fbx)
{
    uint32_t count = 0;
    maniImportResult r = EachObject(fbx, &count);
    if (r.result != mani_success)
    {
        return r;
    }
    fbx->objects = maniAllocate(&fbx->allocator, (size_t)count + 1, sizeof(maniFbxObject), 8);
    if (fbx->objects == NULL)
    {
        return Capacity();
    }
    fbx->objectCapacity = count;
    r = EachObject(fbx, &fbx->objectCount);
    return r.result == mani_success ? maniFbxIndexObjects(fbx) : r;
}

// One connection: counted, and written once the list exists (the second
// pass makes the same ones).
static void Connect(maniFbx* fbx, uint32_t child, uint32_t parent)
{
    if (fbx->connections != NULL)
    {
        fbx->connections[fbx->connectionCount] =
            (maniFbxConnection){child, parent, MANI_FBX_NONE, MANI_FBX_NONE};
    }
    ++fbx->connectionCount;
}

// A model's children by its Children list (names of no object skipped).
static void Children(maniFbx* fbx, uint32_t model)
{
    uint32_t list = maniFbxFindChild(fbx, fbx->objects[model].node, "Children", 8);
    uint32_t n = list != MANI_FBX_NONE ? fbx->nodes[list].propertyCount : 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        uint32_t child = maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, list, k));
        if (child < fbx->objectCount)
        {
            Connect(fbx, child, model);
        }
    }
}

// The connections: children to models, the skin to its mesh, each
// cluster to the skin and the linked model to the cluster; counted
// (connections NULL) or written.
static void EachConnection(maniFbx* fbx)
{
    fbx->connectionCount = 0;
    uint32_t skin = MANI_FBX_NONE;
    for (uint32_t o = 0; o < fbx->objectCount; ++o)
    {
        const maniFbxObject* object = &fbx->objects[o];
        if (object->role == maniFbxRoleNone)
        {
            Children(fbx, o);
            continue;
        }
        if (object->role == maniFbxRoleLegacySkin)
        {
            // Its mesh is the model listed just before it.
            skin = o;
            Connect(fbx, o, o - 1);
            continue;
        }
        Connect(fbx, o, skin);
        uint32_t linked = maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, object->node, 0));
        if (linked < fbx->objectCount)
        {
            Connect(fbx, linked, o);
        }
    }
}

static maniImportResult Connections(maniFbx* fbx)
{
    EachConnection(fbx);
    uint32_t count = fbx->connectionCount;
    fbx->connections =
        maniAllocate(&fbx->allocator, (size_t)count + 1, sizeof(maniFbxConnection), 8);
    fbx->connectionCapacity = count;
    if (fbx->connections == NULL)
    {
        fbx->connectionCount = 0;
        return Capacity();
    }
    EachConnection(fbx);
    return maniFbxIndexConnections(fbx);
}

// A channel's rest value, when it is one of a Transform's T, R and S or
// names an Lcl property.
static void RestOf(maniFbx* fbx, uint32_t object, uint32_t channel, uint32_t which)
{
    uint32_t components[3];
    uint32_t n = maniFbxChannelComponents(fbx, channel, components);
    if (n == 0)
    {
        return;
    }
    double* rest = fbx->legacyRests[object][which];
    for (uint32_t k = 0; k < 3; ++k)
    {
        rest[k] = k < n ? maniFbxChannelDefault(fbx, components[k]) : 0.0;
    }
    fbx->legacyRestMask[object] |= (uint8_t)(1u << which);
}

static const char* const kShort[3] = {"T", "R", "S"};
static const char* const kLong[3] = {"Lcl Translation", "Lcl Rotation", "Lcl Scaling"};

// The rests a model node's channels give: Transform's T, R and S, and
// the Lcl properties by name.
static void ModelRests(maniFbx* fbx, uint32_t node)
{
    uint32_t object = maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, node, 0));
    if (object >= fbx->objectCount)
    {
        return;
    }
    for (uint32_t c = fbx->nodes[node].firstChild; c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        uint32_t name = maniFbxNodeProperty(fbx, c, 0);
        if (!maniFbxNameIs(fbx, c, "Channel", 7))
        {
            continue;
        }
        bool transform = maniFbxStringEquals(fbx, name, "Transform", 9);
        for (uint32_t t = transform ? fbx->nodes[c].firstChild : MANI_FBX_NONE; t != MANI_FBX_NONE;
             t = fbx->nodes[t].next)
        {
            for (uint32_t k = 0; k < 3 && maniFbxNameIs(fbx, t, "Channel", 7); ++k)
            {
                if (maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, t, 0), kShort[k], 1))
                {
                    RestOf(fbx, object, t, k);
                }
            }
        }
        for (uint32_t k = 0; k < 3; ++k)
        {
            if (maniFbxStringEquals(fbx, name, kLong[k], strlen(kLong[k])))
            {
                RestOf(fbx, object, c, k);
            }
        }
    }
}

// The rests in file order: each Model's own channels, each take's.
static maniImportResult Rests(maniFbx* fbx)
{
    size_t n = (size_t)fbx->objectCount + 1;
    fbx->legacyRests = maniAllocate(&fbx->allocator, n, sizeof *fbx->legacyRests, 8);
    fbx->legacyRestMask = maniAllocate(&fbx->allocator, n, 1, 8);
    if (fbx->legacyRests == NULL || fbx->legacyRestMask == NULL)
    {
        return Capacity();
    }
    memset(fbx->legacyRestMask, 0, n);
    for (uint32_t top = fbx->nodes[0].firstChild; top != MANI_FBX_NONE; top = fbx->nodes[top].next)
    {
        if (maniFbxNameIs(fbx, top, "Model", 5))
        {
            ModelRests(fbx, top);
            continue;
        }
        bool takes = maniFbxNameIs(fbx, top, "Takes", 5);
        for (uint32_t take = takes ? fbx->nodes[top].firstChild : MANI_FBX_NONE;
             take != MANI_FBX_NONE; take = fbx->nodes[take].next)
        {
            for (uint32_t m = maniFbxNameIs(fbx, take, "Take", 4) ? fbx->nodes[take].firstChild
                                                                  : MANI_FBX_NONE;
                 m != MANI_FBX_NONE; m = fbx->nodes[m].next)
            {
                if (maniFbxNameIs(fbx, m, "Model", 5))
                {
                    ModelRests(fbx, m);
                }
            }
        }
    }
    return maniFbxSuccess();
}

maniImportResult maniFbxReadLegacy(maniFbx* fbx)
{
    maniImportResult r = Objects(fbx);
    r = r.result == mani_success ? Connections(fbx) : r;
    uint32_t settings = maniFbxFindChild(fbx, 0, "Settings", 8);
    r = r.result == mani_success
            ? maniFbxReadSettingsWith(fbx, maniFbxLegacyFrameRate(fbx, settings))
            : r;
    return r.result == mani_success ? Rests(fbx) : r;
}

void maniFbxReleaseLegacy(maniFbx* fbx)
{
    size_t n = (size_t)fbx->objectCount + 1;
    maniRelease(&fbx->allocator, fbx->legacyRests, n, sizeof *fbx->legacyRests, 8);
    maniRelease(&fbx->allocator, fbx->legacyRestMask, n, 1, 8);
    fbx->legacyRests = NULL;
    fbx->legacyRestMask = NULL;
}
