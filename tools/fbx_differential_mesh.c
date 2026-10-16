// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ufbx differential's mesh data (mani-0018, mani-0021): each skin's
// influences against ufbx's skin vertices (ufbx's weights normalized as
// maul-anim's are, a joint's repeated weights summed), its blend weights,
// and its control points skinned at rest by the skin's method against
// ufbx_get_skin_vertex_matrix; each set of morph targets against ufbx's
// blend deformer: its targets (a shape after an equal target left out),
// their names, full weights and position offsets, and the rest weights.
// A mismatch is reported once a skin or set, at its first vertex.

#include "fbx_differential.h"
#include "ufbx.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/morph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The joints the clusters stand for: a bone's cluster the next joint,
// UINT32_MAX for a cluster without one.
static uint32_t* ClusterJoints(const ufbx_skin_deformer* skin)
{
    uint32_t* joints = malloc((skin->clusters.count + 1) * sizeof(uint32_t));
    uint32_t next = 0;
    for (size_t i = 0; joints != NULL && i < skin->clusters.count; ++i)
    {
        joints[i] = skin->clusters.data[i]->bone_node != NULL ? next++ : UINT32_MAX;
    }
    return joints;
}

// A vertex's weight a joint, ours and ufbx's (normalized); false where
// they differ.
static bool SameWeights(const maniInfluences* ours, const ufbx_skin_deformer* theirs,
                        const uint32_t* clusterJoints, uint32_t v, double* a, double* b,
                        uint32_t joints)
{
    memset(a, 0, joints * sizeof(double));
    memset(b, 0, joints * sizeof(double));
    for (uint32_t slot = 0; slot < maniInfluencesPerVertex(ours); ++slot)
    {
        float w = 0.0f;
        uint32_t j = maniInfluence(ours, v, slot, &w);
        a[j < joints ? j : 0] += (double)w;
    }
    ufbx_skin_vertex vertex = theirs->vertices.data[v];
    double total = 0.0;
    for (uint32_t i = 0; i < vertex.num_weights; ++i)
    {
        ufbx_skin_weight w = theirs->weights.data[vertex.weight_begin + i];
        uint32_t j = clusterJoints[w.cluster_index];
        if (j < joints)
        {
            b[j] += w.weight;
            total += w.weight;
        }
    }
    bool same = true;
    for (uint32_t j = 0; same && j < joints; ++j)
    {
        same = fabs(a[j] - (total > 0.0 ? b[j] / total : 0.0)) <= 1e-6;
    }
    double dq = (double)maniInfluencesDualQuaternionWeight(ours, v);
    return same && fabs(dq - vertex.dq_weight) <= 1e-6;
}

// Whether a refusal agrees with ufbx's mesh: one without control points
// (ufbx's empty mesh, nothing to skin or morph) refused as such.
static bool EmptyAgrees(maniImportResult r, const ufbx_mesh* mesh)
{
    return mesh->num_vertices == 0 && r.result == mani_errorFormat &&
           r.reason == maniImportReasonProperty;
}

static bool OpenInfluences(Context* c, uint32_t k, const maniSkin* skin, const ufbx_mesh* mesh,
                           Blob* blob, maniInfluences* out)
{
    maniImportResult r = maniFbxBuildInfluences(c->builder, c->fbx, k, skin);
    if (EmptyAgrees(r, mesh))
    {
        return false;
    }
    if (r.result != mani_success || !Copy(c->builder, blob) ||
        maniOpenInfluences(blob->data, blob->size, out).result != mani_success)
    {
        Mismatch(c, "influence", "skin %u not built (result %d, reason %d, item %u)", k,
                 (int)r.result, (int)r.reason, r.item);
        return false;
    }
    return true;
}

// Runs the skin's job over the points.
static bool Skin(const maniInfluences* influences, maniSkinningMethod method,
                 const maniAffine* palette, const float* dualQuaternions, uint32_t joints,
                 const float* in, float* out)
{
    maniSkinStream stream = {in, out, 12, 12, maniStreamPoint, 0};
    maniSkinJob job = {influences, palette, dualQuaternions,
                       joints,     0,       maniInfluencesVertexCount(influences),
                       1,          &stream};
    maniResult r = method == maniSkinningDualQuaternion ? maniSkinDualQuaternion(&job)
                   : method == maniSkinningBlended      ? maniSkinBlended(&job)
                                                        : maniSkinLinear(&job);
    return r == mani_success;
}

// A skinned point's tolerance scale: its distance from its bones' origins
// (at least 1), as a bone's world is held within 1e-4 of its axes'
// lengths, so a point's error grows with its lever arm.
static double Lever(const ufbx_skin_deformer* skin, uint32_t v, ufbx_vec3 point)
{
    double lever = 1.0;
    ufbx_skin_vertex vertex = skin->vertices.data[v];
    for (uint32_t i = 0; i < vertex.num_weights; ++i)
    {
        const ufbx_skin_cluster* cluster =
            skin->clusters.data[skin->weights.data[vertex.weight_begin + i].cluster_index];
        const ufbx_node* bone = cluster->bone_node;
        if (bone != NULL)
        {
            ufbx_vec3 origin = bone->node_to_world.cols[3];
            double dx = point.x - origin.x;
            double dy = point.y - origin.y;
            double dz = point.z - origin.z;
            lever = fmax(lever, sqrt(dx * dx + dy * dy + dz * dz));
        }
    }
    return lever;
}

// Our skinned control points at a pose (in the skeleton's placement)
// against ufbx's skin, in the scene evaluated at the same time; the first vertex that differs,
// UINT32_MAX for none, or the vertex count when nothing could be skinned.
static uint32_t CompareSkinned(const Context* c, uint32_t k, const maniSkin* skin,
                               const maniSkeleton* skeleton, const maniPose* pose,
                               const maniAffine* placement, const maniInfluences* influences,
                               const ufbx_skin_deformer* theirs, const ufbx_mesh* mesh)
{
    uint32_t joints = maniSkinJointCount(skin);
    uint32_t vertices = maniInfluencesVertexCount(influences);
    maniAffine* palette = malloc(((size_t)joints + 1) * sizeof(maniAffine));
    float* dq = malloc(((size_t)joints + 1) * 20 * sizeof(float));
    float* in = malloc(((size_t)vertices + 1) * 3 * sizeof(float));
    float* out = malloc(((size_t)vertices + 1) * 3 * sizeof(float));
    bool ready = pose != NULL && palette != NULL && dq != NULL && in != NULL && out != NULL &&
                 maniComputePalette(pose, skeleton, skin, palette, joints, NULL) == mani_success &&
                 maniPaletteDualQuaternions(palette, joints, dq) == mani_success;
    for (uint32_t v = 0; ready && v < vertices; ++v)
    {
        for (int i = 0; i < 3; ++i)
        {
            in[3 * v + i] = (float)mesh->vertices.data[v].v[i];
        }
    }
    ready = ready && Skin(influences, maniFbxSkinMethod(c->fbx, k), palette, dq, joints, in, out);
    double p[12];
    FromAffine(placement, p);
    uint32_t first = ready ? UINT32_MAX : vertices;
    for (uint32_t v = 0; ready && first == UINT32_MAX && v < vertices; ++v)
    {
        // A vertex without weight is left as it is by both.
        if (theirs->vertices.data[v].num_weights == 0)
        {
            continue;
        }
        ufbx_matrix m = ufbx_get_skin_vertex_matrix(theirs, v, NULL);
        ufbx_vec3 expected = ufbx_transform_position(&m, mesh->vertices.data[v]);
        const float* o = out + 3 * (size_t)v;
        double lever = Lever(theirs, v, expected);
        for (int i = 0; i < 3; ++i)
        {
            double ours =
                p[i] * (double)o[0] + p[3 + i] * (double)o[1] + p[6 + i] * (double)o[2] + p[9 + i];
            first = fabs(ours - expected.v[i]) <= 1e-4 * lever ? first : v;
        }
    }
    free(palette);
    free(dq);
    free(in);
    free(out);
    return first;
}

void CompareInfluences(Context* c, uint32_t k, uint32_t s, const maniSkin* skin,
                       const maniSkeleton* skeleton)
{
    const ufbx_node* node = NULL;
    ufbx_skin_deformer* theirs = TheirSkin(c, k, &node);
    if (theirs == NULL || node == NULL || node->mesh == NULL)
    {
        return;
    }
    Blob blob = {NULL, NULL, 0};
    maniInfluences ours;
    if (!OpenInfluences(c, k, skin, node->mesh, &blob, &ours))
    {
        Release(&blob);
        return;
    }
    uint32_t vertices = maniInfluencesVertexCount(&ours);
    uint32_t joints = maniSkinJointCount(skin);
    uint32_t* clusterJoints = ClusterJoints(theirs);
    double* a = malloc(((size_t)joints + 1) * sizeof(double));
    double* b = malloc(((size_t)joints + 1) * sizeof(double));
    uint32_t v = 0;
    bool counted = vertices == theirs->vertices.count && vertices == node->mesh->num_vertices;
    while (counted && clusterJoints != NULL && a != NULL && b != NULL && v < vertices &&
           SameWeights(&ours, theirs, clusterJoints, v, a, b, joints))
    {
        ++v;
    }
    if (!counted)
    {
        Mismatch(c, "influence", "skin %u: %u vertices, ufbx %zu", k, vertices,
                 theirs->vertices.count);
    }
    else if (v < vertices)
    {
        Mismatch(c, "influence", "skin %u vertex %u: weights or blend weight differ", k, v);
    }
    else
    {
        void* raw = NULL;
        maniPose* pose = NewPose(skeleton, &raw);
        maniFbxOptions options = maniDefaultFbxOptions();
        maniAffine placement;
        uint32_t differs =
            Placement(c, s, &options, &placement).result == mani_success
                ? CompareSkinned(c, k, skin, skeleton, pose, &placement, &ours, theirs, node->mesh)
                : 0;
        free(raw);
        if (differs != UINT32_MAX)
        {
            Mismatch(c, "skinning", "skin %u (method %u) vertex %u", k,
                     maniFbxSkinMethod(c->fbx, k), differs);
        }
        c->skinsCompared += differs == UINT32_MAX ? 1 : 0;
    }
    free(clusterJoints);
    free(a);
    free(b);
    Release(&blob);
}

// Skin k built on the clip's skeleton, and its influences.
static bool OpenClipSkin(Context* c, uint32_t k, Blob* skinBlob, maniSkin* skin, Blob* blob,
                         maniInfluences* influences)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    return maniFbxBuildSkin(c->builder, c->fbx, k, &c->skeleton, &options).result == mani_success &&
           Copy(c->builder, skinBlob) &&
           maniOpenSkin(skinBlob->data, skinBlob->size, skin).result == mani_success &&
           maniFbxBuildInfluences(c->builder, c->fbx, k, skin).result == mani_success &&
           Copy(c->builder, blob) &&
           maniOpenInfluences(blob->data, blob->size, influences).result == mani_success;
}

void CompareSkinsAt(Context* c, uint32_t clip, const maniPose* pose, const ufbx_scene* evaluated,
                    double time)
{
    for (uint32_t k = 0; !c->byModel && k < maniFbxSkinCount(c->fbx); ++k)
    {
        const ufbx_node* node = NULL;
        ufbx_skin_deformer* theirs =
            maniFbxSkinSkeleton(c->fbx, k) == 0 ? TheirSkin(c, k, &node) : NULL;
        if (theirs == NULL || node == NULL || node->mesh == NULL || node->mesh->num_vertices == 0)
        {
            continue;
        }
        const ufbx_skin_deformer* posed =
            (const ufbx_skin_deformer*)evaluated->elements.data[theirs->element.element_id];
        Blob skinBlob = {NULL, NULL, 0};
        Blob blob = {NULL, NULL, 0};
        maniSkin skin;
        maniInfluences influences;
        bool opened = OpenClipSkin(c, k, &skinBlob, &skin, &blob, &influences);
        uint32_t differs = opened ? CompareSkinned(c, k, &skin, &c->skeleton, pose, &c->placement,
                                                   &influences, posed, node->mesh)
                                  : 0;
        maniSkinningMethod method = maniFbxSkinMethod(c->fbx, k);
        c->posedCompared += differs == UINT32_MAX ? 1 : 0;
        c->posedDualQuaternion += differs == UINT32_MAX && method != maniSkinningLinear ? 1 : 0;
        if (differs != UINT32_MAX)
        {
            Mismatch(c, method == maniSkinningLinear ? "skinning" : "skinning-dq",
                     "clip %u t=%g skin %u (method %u) vertex %u%s", clip, time, k, method, differs,
                     opened ? "" : " (not built)");
        }
    }
}

// The ufbx mesh node our set m deforms, and the set's blend deformer: the
// mesh's of the same rank among our sets on it.
static ufbx_blend_deformer* TheirBlend(const Context* c, uint32_t m, const ufbx_node** nodeOut)
{
    uint32_t length = 0;
    uint32_t model = maniFbxMorphModel(c->fbx, m);
    const char* name = model != UINT32_MAX ? maniFbxModelBoneName(c->fbx, model, &length) : NULL;
    const ufbx_node* node = name != NULL ? ufbx_find_node_len(c->scene, name, length) : NULL;
    *nodeOut = node;
    uint32_t rank = 0;
    for (uint32_t j = 0; j < m; ++j)
    {
        rank += maniFbxMorphModel(c->fbx, j) == model ? 1 : 0;
    }
    return node != NULL && node->mesh != NULL && rank < node->mesh->blend_deformers.count
               ? node->mesh->blend_deformers.data[rank]
               : NULL;
}

// Whether keyframe k of a channel is a target of ours.
static bool KeptKey(const ufbx_blend_channel* channel, size_t k)
{
    return k == 0 ||
           channel->keyframes.data[k].target_weight != channel->keyframes.data[k - 1].target_weight;
}

// Our target t against a keyframe: name, full weight and offsets;
// positions are scratch for 2 times the vertices.
static bool SameTarget(const maniMorphTargets* ours, uint32_t t, const ufbx_blend_channel* channel,
                       size_t k, float* weights, float* positions, ufbx_vec3* expected)
{
    uint32_t vertices = maniMorphVertexCount(ours);
    uint32_t length = 0;
    const char* name = maniMorphTargetName(ours, t, &length);
    const ufbx_blend_keyframe* key = &channel->keyframes.data[k];
    if (name == NULL || !ShapeNamed(channel, k, name, length) ||
        fabs((double)maniMorphTargetFullWeight(ours, t) - key->target_weight) > 1e-6)
    {
        return false;
    }
    memset(weights, 0, maniMorphTargetCount(ours) * sizeof(float));
    weights[t] = 1.0f;
    memset(positions, 0, (size_t)vertices * 3 * sizeof(float));
    memset(expected, 0, (size_t)vertices * sizeof(ufbx_vec3));
    maniMorphStream stream = {positions, positions, 12, 12, maniMorphPosition, 0, 3, 0};
    maniMorphJob job = {ours, weights, 0, vertices, 1, 0, &stream};
    ufbx_add_blend_shape_vertex_offsets(key->shape, expected, vertices, 1.0);
    bool same = maniApplyMorphTargets(&job) == mani_success;
    for (uint32_t i = 0; same && i < 3 * vertices; ++i)
    {
        same = Near((double)positions[i], expected[i / 3].v[i % 3]);
    }
    return same;
}

// Our targets against the blend deformer's channels; the first target
// that differs, or UINT32_MAX.
static uint32_t CompareTargets(const maniMorphTargets* ours, const ufbx_blend_deformer* blend,
                               const float* rest)
{
    uint32_t count = maniMorphTargetCount(ours);
    uint32_t vertices = maniMorphVertexCount(ours);
    float* weights = malloc(((size_t)count + 1) * sizeof(float));
    float* positions = malloc(((size_t)vertices + 1) * 3 * sizeof(float));
    ufbx_vec3* expected = malloc(((size_t)vertices + 1) * sizeof(ufbx_vec3));
    uint32_t t = 0;
    bool same = weights != NULL && positions != NULL && expected != NULL;
    for (size_t ch = 0; same && ch < blend->channels.count; ++ch)
    {
        const ufbx_blend_channel* channel = blend->channels.data[ch];
        for (size_t k = 0; same && k < channel->keyframes.count; ++k)
        {
            if (!KeptKey(channel, k))
            {
                continue;
            }
            same = t < count && SameTarget(ours, t, channel, k, weights, positions, expected) &&
                   Near((double)rest[t], channel->keyframes.data[k].effective_weight);
            t += same ? 1 : 0;
        }
    }
    free(weights);
    free(positions);
    free(expected);
    return same && t == count ? UINT32_MAX : t;
}

static void CompareMorph(Context* c, uint32_t m)
{
    const ufbx_node* node = NULL;
    ufbx_blend_deformer* theirs = TheirBlend(c, m, &node);
    maniFbxOptions options = maniDefaultFbxOptions();
    Blob blob = {NULL, NULL, 0};
    maniMorphTargets ours;
    uint32_t count = maniFbxMorphTargetCount(c->fbx, m);
    float* rest = malloc(((size_t)count + 1) * sizeof(float));
    maniImportResult r = maniFbxBuildMorphTargets(c->builder, c->fbx, m, &options);
    bool built = r.result == mani_success && Copy(c->builder, &blob) &&
                 maniOpenMorphTargets(blob.data, blob.size, &ours).result == mani_success &&
                 rest != NULL && maniFbxMorphWeights(c->fbx, m, rest, count).result == mani_success;
    if (theirs == NULL)
    {
        Mismatch(c, "morph", "set %u has no ufbx blend deformer", m);
    }
    else if (EmptyAgrees(r, node->mesh))
    {
        // Nothing to morph on either side.
    }
    else if (!built)
    {
        Mismatch(c, "morph", "set %u not built (result %d, reason %d, item %u)", m, (int)r.result,
                 (int)r.reason, r.item);
    }
    else if (maniMorphVertexCount(&ours) != node->mesh->num_vertices)
    {
        Mismatch(c, "morph", "set %u: %u vertices, ufbx %zu", m, maniMorphVertexCount(&ours),
                 node->mesh->num_vertices);
    }
    else
    {
        uint32_t t = CompareTargets(&ours, theirs, rest);
        if (t != UINT32_MAX)
        {
            Mismatch(c, "morph", "set %u target %u of %u differs", m, t,
                     maniMorphTargetCount(&ours));
        }
        c->morphsCompared += t == UINT32_MAX ? 1 : 0;
    }
    free(rest);
    Release(&blob);
}

void CompareMorphs(Context* c)
{
    for (uint32_t m = 0; m < maniFbxMorphCount(c->fbx); ++m)
    {
        CompareMorph(c, m);
    }
}
