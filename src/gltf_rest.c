// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Node rests and placement. A node gives a matrix or
// TRS, never both; a matrix must be affine and decompose into a
// TRS that composes back within 1e-4 of its largest column, which rules
// out the shear glTF forbids. Everything is read in glTF's basis, then
// moved into the output's.

#include "gltf_rest.h"

#include "allocator.h"
#include "decompose.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"
#include "vecmath.h"

#include <math.h>
#include <string.h>

// An array member of exactly `count` numbers as binary32, or absent.
static maniImportResult ReadNumbers(const maniGltf* gltf, uint32_t object, const char* name,
                                    uint32_t count, uint32_t item, float* out, bool* presentOut)
{
    const maniGltfJson* g = &gltf->g;
    uint32_t array = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, object, name, maniJsonArray, false, item, &array);
    *presentOut = array != MANI_JSON_NONE;
    if (r.result != mani_success || array == MANI_JSON_NONE)
    {
        return r;
    }
    if (gltf->json.values[array].count != count)
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, array), item);
    }
    uint32_t element = array + 1;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint64_t bits = 0;
        if (!maniJsonBinary(&gltf->json, element, maniBinary32, &bits))
        {
            return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, element), item);
        }
        uint32_t word = (uint32_t)bits;
        memcpy(&out[i], &word, sizeof word);
        element = gltf->json.values[element].next;
    }
    return maniImportSuccess();
}

// The 3x4 part of a column-major 4x4, in glTF's basis.
static maniAffine FromColumns(const float m[16])
{
    maniAffine a;
    for (uint32_t c = 0; c < 4; ++c)
    {
        for (uint32_t r = 0; r < 3; ++r)
        {
            a.m[c * 3 + r] = m[c * 4 + r];
        }
    }
    return a;
}

static bool Affine(const float m[16])
{
    return fabsf(m[3]) <= 1e-5f && fabsf(m[7]) <= 1e-5f && fabsf(m[11]) <= 1e-5f &&
           fabsf(m[15] - 1.0f) <= 1e-5f;
}

maniImportResult maniGltfMatrixAffine(const maniGltf* gltf, const float m[16],
                                      const maniGltfOptions* options, uint64_t at, uint32_t item,
                                      maniAffine* affineOut)
{
    (void)gltf;
    if (!Affine(m))
    {
        return maniImportFail(maniImportReasonMatrix, at, item);
    }
    *affineOut = maniGltfBasisAffine(options, FromColumns(m));
    return maniImportSuccess();
}

// A node matrix as TRS: affine, decomposed, and composing back.
static maniImportResult DecomposeNode(const float m[16], uint64_t at, uint32_t node,
                                      maniTransform* out)
{
    maniAffine a = FromColumns(m);
    if (!Affine(m) || !maniDecomposeExact(&a, out))
    {
        return maniImportFail(maniImportReasonMatrix, at, node);
    }
    return maniImportSuccess();
}

maniImportResult maniGltfNodeRest(const maniGltf* gltf, uint32_t node,
                                  const maniGltfOptions* options, maniTransform* restOut)
{
    uint32_t value = gltf->nodeValues[node];
    float m[16] = {0};
    float t[3] = {0, 0, 0};
    float q[4] = {0, 0, 0, 1};
    float s[3] = {1, 1, 1};
    bool hasMatrix = false;
    bool hasT = false;
    bool hasR = false;
    bool hasS = false;
    maniImportResult r = ReadNumbers(gltf, value, "matrix", 16, node, m, &hasMatrix);
    if (r.result == mani_success)
    {
        r = ReadNumbers(gltf, value, "translation", 3, node, t, &hasT);
    }
    if (r.result == mani_success)
    {
        r = ReadNumbers(gltf, value, "rotation", 4, node, q, &hasR);
    }
    if (r.result == mani_success)
    {
        r = ReadNumbers(gltf, value, "scale", 3, node, s, &hasS);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    maniTransform rest = {{t[0], t[1], t[2]}, {q[0], q[1], q[2], q[3]}, {s[0], s[1], s[2]}};
    if (hasMatrix)
    {
        uint32_t at = maniJsonFind(&gltf->json, value, "matrix", 6);
        if (hasT || hasR || hasS)
        {
            return maniImportFail(maniImportReasonProperty, maniGltfOffset(&gltf->g, at), node);
        }
        r = DecomposeNode(m, maniGltfOffset(&gltf->g, at), node, &rest);
    }
    else if (q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f && q[3] == 0.0f)
    {
        uint32_t at = maniJsonFind(&gltf->json, value, "rotation", 8);
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(&gltf->g, at), node);
    }
    rest.rotation = maniQuatNormalize(rest.rotation);
    *restOut = maniGltfBasisTransform(options, rest);
    return r;
}

maniImportResult maniGltfPlacement(const maniGltf* gltf, uint32_t root,
                                   const maniGltfOptions* options, maniAffine* placementOut)
{
    uint32_t depth = gltf->hierarchy.depths[root];
    uint32_t* path = maniAllocate(&gltf->allocator, (size_t)depth + 1, sizeof(uint32_t), 8);
    if (path == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    uint32_t v = gltf->nodeParents[root];
    for (uint32_t k = depth; k-- > 0; v = gltf->nodeParents[v])
    {
        path[k] = v;
    }
    maniAffine p = maniGltfIdentityAffine();
    maniImportResult r = maniImportSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < depth; ++k)
    {
        maniTransform local;
        r = maniGltfNodeRest(gltf, path[k], options, &local);
        maniAffine a = maniAffineFromTrs(local.translation, local.rotation, local.scale);
        p = maniAffineMul(&p, &a);
    }
    maniRelease(&gltf->allocator, path, (size_t)depth + 1, sizeof(uint32_t), 8);
    *placementOut = p;
    return r;
}

maniImportResult maniGltfRootRest(const maniGltf* gltf, uint32_t root,
                                  const maniGltfOptions* options, maniTransform* restOut)
{
    maniImportResult r = maniGltfNodeRest(gltf, root, options, restOut);
    if (r.result != mani_success || options->foldPlacement == 0)
    {
        return r;
    }
    maniAffine p;
    r = maniGltfPlacement(gltf, root, options, &p);
    if (r.result != mani_success)
    {
        return r;
    }
    maniAffine local = maniAffineFromTrs(restOut->translation, restOut->rotation, restOut->scale);
    maniAffine folded = maniAffineMul(&p, &local);
    if (!maniDecomposeExact(&folded, restOut))
    {
        return maniImportFail(maniImportReasonMatrix,
                              maniGltfOffset(&gltf->g, gltf->nodeValues[root]), root);
    }
    return r;
}
