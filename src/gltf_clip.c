// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One glTF animation as a clip of one skeleton (mani-0010): channels
// bound by bone name, keys moved into the output's basis, morph weights
// and pointer channels split into float tracks, the placement above the
// root composed in on request, and the whole built by the clip builder,
// whose refusals come back naming the channel.

#include "arena.h"
#include "builder.h"
#include "clip_build.h"
#include "clip_file.h"
#include "gltf_animation.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_layout.h"
#include "gltf_names.h"
#include "gltf_node.h"
#include "gltf_property.h"
#include "gltf_rest.h"
#include "json.h"
#include "vecmath.h"

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <string.h>

// A channel the clip holds, or one above the root it composes in.
typedef struct Bound
{
    maniGltfChannel c;
    uint32_t index;
    // The bone of a TRS channel, or MANI_NO_BONE.
    uint32_t bone;
    bool placement;
    float* times;
    float* values;
} Bound;

typedef struct Build
{
    maniArena arena;
    const maniGltf* gltf;
    const maniSkeleton* skeleton;
    const maniGltfOptions* options;
    uint32_t animation;
    Bound* bound;
    uint32_t boundCount;
    // The node whose name is the skeleton's root bone's.
    uint32_t rootNode;
    float duration;
    float rate;
    maniBoneTrackSource* tracks;
    uint32_t* trackChannel;
    uint32_t trackCount;
    maniFloatTrackSource* floats;
    uint32_t* floatChannel;
    uint32_t floatCount;
} Build;

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static uint32_t NodeBone(const Build* b, uint32_t node)
{
    return maniGltfNodeBone(b->gltf, b->skeleton, node);
}

// The first node named as the skeleton's root bone.
static uint32_t RootNode(const Build* b)
{
    uint32_t length = 0;
    const char* root = maniSkeletonBoneName(b->skeleton, 0, &length);
    for (uint32_t v = 0; v < b->gltf->nodeCount; ++v)
    {
        uint32_t n = 0;
        const char* name = maniGltfNodeName(b->gltf, v, &n);
        if (n == length && memcmp(name, root, n) == 0)
        {
            return v;
        }
    }
    return MANI_JSON_NONE;
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

// A node carrying a skin whose joints are all bones of the skeleton.
static bool SkinsInto(const Build* b, uint32_t node)
{
    int64_t skin = -1;
    uint32_t value = maniJsonFind(&b->gltf->json, b->gltf->nodeValues[node], "skin", 4);
    if (b->gltf->skinCount == 0 ||
        !maniJsonInteger(&b->gltf->json, value, 0, (int64_t)b->gltf->skinCount - 1, &skin))
    {
        return false;
    }
    uint32_t count = 0;
    uint32_t element = maniGltfSkinJoints(b->gltf, (uint32_t)skin, &count) + 1;
    for (uint32_t j = 0; j < count; ++j)
    {
        if (NodeBone(b, maniGltfJointNode(b->gltf, element)) == MANI_NO_BONE)
        {
            return false;
        }
        element = b->gltf->json.values[element].next;
    }
    return true;
}

// Whether the clip holds a channel, or composes it in as placement.
static void Classify(const Build* b, Bound* x)
{
    x->bone = MANI_NO_BONE;
    x->placement = false;
    switch (x->c.target)
    {
    case maniGltfTargetTranslation:
    case maniGltfTargetRotation:
    case maniGltfTargetScale:
        x->bone = NodeBone(b, x->c.node);
        x->placement = x->bone == MANI_NO_BONE && b->options->foldPlacement != 0 &&
                       b->rootNode != MANI_JSON_NONE && IsAncestor(b->gltf, x->c.node, b->rootNode);
        return;
    default:
        return;
    }
}

static bool Kept(const Build* b, const Bound* x)
{
    switch (x->c.target)
    {
    case maniGltfTargetTranslation:
    case maniGltfTargetRotation:
    case maniGltfTargetScale:
        return x->bone != MANI_NO_BONE || x->placement;
    case maniGltfTargetWeights:
        return NodeBone(b, x->c.node) != MANI_NO_BONE || SkinsInto(b, x->c.node);
    case maniGltfTargetPointer:
        return true;
    default:
        return false;
    }
}

// The values of a TRS channel, keys and tangents alike, in the output's
// basis.
static void ToBasis(const Build* b, Bound* x)
{
    size_t parts = x->c.interpolation == maniInterpolationCubicSpline ? 3 : 1;
    size_t vectors = parts * x->c.keys;
    for (size_t i = 0; i < vectors; ++i)
    {
        if (x->c.target == maniGltfTargetRotation)
        {
            maniGltfBasisQuat(b->options, x->values + 4 * i);
        }
        else
        {
            maniGltfBasisVector(b->options, x->values + 3 * i,
                                x->c.target == maniGltfTargetTranslation);
        }
    }
}

static size_t OutputFloats(const maniGltf* gltf, const maniGltfChannel* c)
{
    const maniGltfAccessor* a = &gltf->accessors[c->output];
    return (size_t)a->count * a->rows * a->columns;
}

// Every channel's keys (for the duration), and the kept ones' values.
static maniImportResult ReadChannels(Build* b)
{
    uint32_t count = 0;
    uint32_t channel = maniGltfAnimationChannels(b->gltf, b->animation, &count) + 1;
    b->bound = maniArenaTake(&b->arena, count, sizeof(Bound));
    if (b->bound == NULL)
    {
        return Capacity();
    }
    for (uint32_t i = 0; i < count; ++i, channel = b->gltf->json.values[channel].next)
    {
        Bound x = {.index = i};
        maniImportResult r = maniGltfReadChannel(b->gltf, b->animation, channel, i, &x.c);
        if (r.result != mani_success || x.c.target == maniGltfTargetIgnored)
        {
            if (r.result != mani_success)
            {
                return r;
            }
            continue;
        }
        x.times = maniArenaTake(&b->arena, x.c.keys, sizeof(float));
        r = x.times != NULL ? maniGltfReadFloats(b->gltf, x.c.input, x.times) : Capacity();
        if (r.result != mani_success)
        {
            return r;
        }
        float last = x.times[x.c.keys - 1];
        b->duration = last > b->duration ? last : b->duration;
        Classify(b, &x);
        if (!Kept(b, &x))
        {
            continue;
        }
        x.values = maniArenaTake(&b->arena, OutputFloats(b->gltf, &x.c), sizeof(float));
        r = x.values != NULL ? maniGltfReadFloats(b->gltf, x.c.output, x.values) : Capacity();
        if (r.result != mani_success)
        {
            return r;
        }
        if (x.c.target <= maniGltfTargetScale)
        {
            ToBasis(b, &x);
        }
        b->bound[b->boundCount++] = x;
    }
    return maniImportSuccess();
}

static maniChannelSource Source(const Bound* x)
{
    return (maniChannelSource){x->times, x->values, x->c.keys, x->c.interpolation};
}

// One track per bone with a channel; the track's first channel for
// refusals.
static maniImportResult BoneTracks(Build* b)
{
    uint32_t bones = maniSkeletonBoneCount(b->skeleton);
    uint32_t* trackOf = maniArenaTake(&b->arena, bones, sizeof(uint32_t));
    // One spare: folding may add a root track no channel made.
    b->tracks = maniArenaTake(&b->arena, (size_t)b->boundCount + 1, sizeof(maniBoneTrackSource));
    b->trackChannel = maniArenaTake(&b->arena, (size_t)b->boundCount + 1, sizeof(uint32_t));
    if (trackOf == NULL || b->tracks == NULL || b->trackChannel == NULL)
    {
        return Capacity();
    }
    for (uint32_t i = 0; i < b->boundCount; ++i)
    {
        const Bound* x = &b->bound[i];
        if (x->bone == MANI_NO_BONE)
        {
            continue;
        }
        if (trackOf[x->bone] == 0)
        {
            b->tracks[b->trackCount].bone = x->bone;
            b->trackChannel[b->trackCount] = x->index;
            trackOf[x->bone] = ++b->trackCount;
        }
        maniBoneTrackSource* t = &b->tracks[trackOf[x->bone] - 1];
        maniChannelSource* slot = x->c.target == maniGltfTargetTranslation ? &t->translation
                                  : x->c.target == maniGltfTargetRotation  ? &t->rotation
                                                                           : &t->scale;
        *slot = Source(x);
    }
    return maniImportSuccess();
}

// A node's local transform at a time: its channels among the bound ones,
// its rest elsewhere.
static maniImportResult LocalAt(const Build* b, uint32_t node, double time, maniTransform* out)
{
    maniImportResult r = maniGltfNodeRest(b->gltf, node, b->options, out);
    for (uint32_t i = 0; r.result == mani_success && i < b->boundCount; ++i)
    {
        const Bound* x = &b->bound[i];
        if (x->c.node != node || x->c.target > maniGltfTargetScale)
        {
            continue;
        }
        maniChannelSource source = Source(x);
        double v[4];
        maniEvaluateChannel(&source, x->c.target == maniGltfTargetRotation ? 4 : 3, time, v);
        if (x->c.target == maniGltfTargetTranslation)
        {
            out->translation = (maniVec3){(float)v[0], (float)v[1], (float)v[2]};
        }
        else if (x->c.target == maniGltfTargetRotation)
        {
            out->rotation = (maniQuat){(float)v[0], (float)v[1], (float)v[2], (float)v[3]};
        }
        else
        {
            out->scale = (maniVec3){(float)v[0], (float)v[1], (float)v[2]};
        }
    }
    return r;
}

static maniAffine AffineOf(const maniTransform* t)
{
    return maniAffineFromTrs(t->translation, t->rotation, t->scale);
}

// The channel a folded root track answers for: the root's own, or the
// first above it.
static uint32_t FoldChannel(const Build* b)
{
    for (uint32_t i = 0; i < b->boundCount; ++i)
    {
        if (b->bound[i].bone == 0 || b->bound[i].placement)
        {
            return b->bound[i].index;
        }
    }
    return 0;
}

// The root's track replaced by the placement composed in, sampled on the
// clip's grid with linear keys the clip reproduces at its samples.
static maniImportResult FoldRoot(Build* b, uint32_t samples)
{
    uint32_t depth = b->gltf->nodeDepths[b->rootNode];
    uint32_t* path = maniArenaTake(&b->arena, (size_t)depth + 1, sizeof(uint32_t));
    float* times = maniArenaTake(&b->arena, samples, sizeof(float));
    float* t = maniArenaTake(&b->arena, (size_t)samples * 3, sizeof(float));
    float* q = maniArenaTake(&b->arena, (size_t)samples * 4, sizeof(float));
    float* s = maniArenaTake(&b->arena, (size_t)samples * 3, sizeof(float));
    if (path == NULL || times == NULL || t == NULL || q == NULL || s == NULL)
    {
        return Capacity();
    }
    uint32_t v = b->gltf->nodeParents[b->rootNode];
    for (uint32_t k = depth; k-- > 0; v = b->gltf->nodeParents[v])
    {
        path[k] = v;
    }
    maniImportResult r = maniImportSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < samples; ++k)
    {
        double time = maniClipSampleTime(k, samples, b->rate, b->duration);
        maniAffine composed = maniGltfIdentityAffine();
        maniTransform local;
        for (uint32_t i = 0; r.result == mani_success && i <= depth; ++i)
        {
            r = LocalAt(b, i < depth ? path[i] : b->rootNode, time, &local);
            maniAffine a = AffineOf(&local);
            composed = maniAffineMul(&composed, &a);
        }
        maniTransform d;
        if (r.result == mani_success && maniDecomposeAffine(&composed, &d) != mani_success)
        {
            r = maniImportFail(maniImportReasonData, 0, FoldChannel(b));
        }
        times[k] = (float)time;
        memcpy(t + 3 * (size_t)k, &d.translation, sizeof d.translation);
        memcpy(q + 4 * (size_t)k, &d.rotation, sizeof d.rotation);
        memcpy(s + 3 * (size_t)k, &d.scale, sizeof d.scale);
    }
    maniBoneTrackSource* root = NULL;
    for (uint32_t i = 0; i < b->trackCount; ++i)
    {
        root = b->tracks[i].bone == 0 ? &b->tracks[i] : root;
    }
    if (root == NULL)
    {
        root = &b->tracks[b->trackCount];
        b->trackChannel[b->trackCount++] = FoldChannel(b);
        root->bone = 0;
    }
    root->translation = (maniChannelSource){times, t, samples, maniInterpolationLinear};
    root->rotation = (maniChannelSource){times, q, samples, maniInterpolationLinear};
    root->scale = (maniChannelSource){times, s, samples, maniInterpolationLinear};
    return r;
}

// Whether folding changes the root: a channel above it, or the root
// animated under a placement that is not the identity.
static bool NeedsFold(const Build* b, const maniAffine* placement)
{
    if (b->options->foldPlacement == 0 || b->rootNode == MANI_JSON_NONE)
    {
        return false;
    }
    bool rootAnimated = false;
    for (uint32_t i = 0; i < b->boundCount; ++i)
    {
        if (b->bound[i].placement)
        {
            return true;
        }
        rootAnimated = rootAnimated || b->bound[i].bone == 0;
    }
    maniAffine identity = maniGltfIdentityAffine();
    bool isIdentity = true;
    for (uint32_t i = 0; i < 12; ++i)
    {
        isIdentity = isIdentity && placement->m[i] == identity.m[i];
    }
    return rootAnimated && !isIdentity;
}

// Decimal digits of n after a slash: "/12".
static size_t SlashNumber(char* out, uint32_t n)
{
    char digits[10];
    size_t count = 0;
    do
    {
        digits[count++] = (char)('0' + n % 10);
        n /= 10;
    } while (n != 0);
    out[0] = '/';
    for (size_t i = 0; i < count; ++i)
    {
        out[1 + i] = digits[count - 1 - i];
    }
    return count + 1;
}

// A morph target's name from the mesh's extras.targetNames, if it has one
// for every target; NULL otherwise.
static uint32_t TargetNames(const maniGltf* gltf, uint32_t node, uint32_t targets)
{
    int64_t mesh = -1;
    uint32_t index = maniJsonFind(&gltf->json, gltf->nodeValues[node], "mesh", 4);
    if (!maniJsonInteger(&gltf->json, index, 0, (int64_t)gltf->meshCount - 1, &mesh))
    {
        return MANI_JSON_NONE;
    }
    uint32_t extras = maniJsonFind(&gltf->json, gltf->meshValues[mesh], "extras", 6);
    uint32_t names = maniJsonFind(&gltf->json, extras, "targetNames", 11);
    if (names == MANI_JSON_NONE || gltf->json.values[names].kind != maniJsonArray ||
        gltf->json.values[names].count != targets)
    {
        return MANI_JSON_NONE;
    }
    uint32_t element = names + 1;
    for (uint32_t i = 0; i < targets; ++i)
    {
        if (gltf->json.values[element].kind != maniJsonString ||
            gltf->json.values[element].length == 0)
        {
            return MANI_JSON_NONE;
        }
        element = gltf->json.values[element].next;
    }
    return names;
}

// Float track k of a channel of `width` components: its keys, as the
// clip builder takes them (in-tangent, value, out-tangent for a cubic).
static float* Component(Build* b, const Bound* x, uint32_t k, uint32_t width)
{
    bool cubic = x->c.interpolation == maniInterpolationCubicSpline;
    size_t parts = cubic ? 3 : 1;
    float* out = maniArenaTake(&b->arena, parts * x->c.keys, sizeof(float));
    for (size_t key = 0; out != NULL && key < x->c.keys; ++key)
    {
        for (size_t p = 0; p < parts; ++p)
        {
            out[key * parts + p] = x->values[(key * parts + p) * width + k];
        }
    }
    return out;
}

// One float track: name and keys.
static maniImportResult AddFloat(Build* b, const Bound* x, const char* name, size_t length,
                                 float* values)
{
    if (values == NULL)
    {
        return Capacity();
    }
    b->floats[b->floatCount] = (maniFloatTrackSource){
        name, (uint32_t)length, 0, {x->times, values, x->c.keys, x->c.interpolation}};
    b->floatChannel[b->floatCount++] = x->index;
    return maniImportSuccess();
}

static maniImportResult WeightTracks(Build* b, const Bound* x)
{
    uint32_t nodeLength = 0;
    const char* node = maniGltfNodeName(b->gltf, x->c.node, &nodeLength);
    uint32_t names = TargetNames(b->gltf, x->c.node, x->c.width);
    uint32_t element = names != MANI_JSON_NONE ? names + 1 : MANI_JSON_NONE;
    maniImportResult r = maniImportSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < x->c.width; ++k)
    {
        size_t target = element != MANI_JSON_NONE ? b->gltf->json.values[element].length : 0;
        char* name = maniArenaTake(&b->arena, nodeLength + target + 12, 1);
        if (name == NULL)
        {
            return Capacity();
        }
        memcpy(name, node, nodeLength);
        size_t length = nodeLength;
        if (element != MANI_JSON_NONE)
        {
            name[length++] = '/';
            length +=
                maniJsonStringDecode(&b->gltf->json, element, (uint8_t*)name + length, target);
            element = b->gltf->json.values[element].next;
        }
        else
        {
            length += SlashNumber(name + length, k);
        }
        r = AddFloat(b, x, name, length, Component(b, x, k, x->c.width));
    }
    return r;
}

static maniImportResult PointerTracks(Build* b, const Bound* x)
{
    size_t raw = b->gltf->json.values[x->c.pointer].length;
    maniImportResult r = maniImportSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < x->c.width; ++k)
    {
        char* name = maniArenaTake(&b->arena, raw + 12, 1);
        if (name == NULL)
        {
            return Capacity();
        }
        size_t length = maniJsonStringDecode(&b->gltf->json, x->c.pointer, (uint8_t*)name, raw);
        length += x->c.width > 1 ? SlashNumber(name + length, k) : 0;
        r = AddFloat(b, x, name, length, Component(b, x, k, x->c.width));
    }
    return r;
}

static maniImportResult FloatTracks(Build* b)
{
    size_t total = 0;
    for (uint32_t i = 0; i < b->boundCount; ++i)
    {
        bool isFloat = b->bound[i].c.target >= maniGltfTargetWeights;
        total += isFloat ? b->bound[i].c.width : 0;
    }
    b->floats = maniArenaTake(&b->arena, total, sizeof(maniFloatTrackSource));
    b->floatChannel = maniArenaTake(&b->arena, total, sizeof(uint32_t));
    if (b->floats == NULL || b->floatChannel == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniImportSuccess();
    for (uint32_t i = 0; r.result == mani_success && i < b->boundCount; ++i)
    {
        const Bound* x = &b->bound[i];
        r = x->c.target == maniGltfTargetWeights   ? WeightTracks(b, x)
            : x->c.target == maniGltfTargetPointer ? PointerTracks(b, x)
                                                   : r;
    }
    return r;
}

// The clip builder's refusal as the import's: keys and values as Data on
// the channel, repeated float names as Name, the rest as Build.
static maniImportResult FromBuild(const Build* b, maniBuildResult r)
{
    if (r.result == mani_success || r.result == mani_errorCapacity)
    {
        return (maniImportResult){r.result, maniImportReasonNone, 0, 0, 0};
    }
    uint32_t list = r.reason & 0xF00u;
    uint32_t reason = r.reason & 0xFFu;
    bool floats = list == maniBuildReasonFloatTrack;
    bool onTrack = (list == 0 || floats) && reason != maniBuildReasonTiming &&
                   reason != maniBuildReasonKind && reason != maniBuildReasonCount;
    uint32_t channel = floats && r.item < b->floatCount    ? b->floatChannel[r.item]
                       : !floats && r.item < b->trackCount ? b->trackChannel[r.item]
                                                           : 0;
    if (!onTrack)
    {
        return maniImportFail(maniImportReasonBuild, 0, r.reason);
    }
    bool name = floats && reason == maniBuildReasonTrack;
    uint32_t channels = 0;
    uint32_t value = maniGltfAnimationChannels(b->gltf, b->animation, &channels) + 1;
    for (uint32_t i = 0; i < channel; ++i)
    {
        value = b->gltf->json.values[value].next;
    }
    return maniImportFail(name ? maniImportReasonName : maniImportReasonData,
                          maniGltfOffset(&b->gltf->g, value), channel);
}

static maniImportResult Run(maniBuilder* builder, Build* b, maniLoopPolicy loop, uint32_t* boundOut)
{
    maniImportResult r = ReadChannels(b);
    if (r.result == mani_success)
    {
        r = BoneTracks(b);
    }
    maniAffine placement = maniGltfIdentityAffine();
    if (r.result == mani_success && b->rootNode != MANI_JSON_NONE)
    {
        r = maniGltfPlacement(b->gltf, b->rootNode, b->options, &placement);
    }
    uint32_t samples = 0;
    if (r.result == mani_success && NeedsFold(b, &placement))
    {
        if (!maniClipSampleCountFor(b->duration, b->rate, &samples))
        {
            return maniImportFail(maniImportReasonBuild, 0, maniBuildReasonTiming);
        }
        r = FoldRoot(b, samples);
    }
    if (r.result == mani_success)
    {
        r = FloatTracks(b);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (boundOut != NULL)
    {
        *boundOut = b->boundCount;
    }
    maniClipSource source = {.skeleton = b->skeleton,
                             .boneTracks = b->tracks,
                             .floatTracks = b->floats,
                             .boneTrackCount = b->trackCount,
                             .floatTrackCount = b->floatCount,
                             .loop = loop,
                             .sampleRate = b->rate,
                             .duration = b->duration};
    return FromBuild(b, maniBuildClip(builder, &source));
}

maniImportResult maniGltfBuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t animation,
                                   const maniSkeleton* skeleton, const maniGltfOptions* options,
                                   float sampleRate, maniLoopPolicy loop, uint32_t* boundOut)
{
    if (builder == NULL)
    {
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    if (gltf == NULL || skeleton == NULL || options == NULL || !maniGltfOptionsValid(options) ||
        animation >= gltf->animationCount)
    {
        ++builder->misuses;
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    if (boundOut != NULL)
    {
        *boundOut = 0;
    }
    Build b = {.arena = maniArenaMake(&gltf->allocator),
               .gltf = gltf,
               .skeleton = skeleton,
               .options = options,
               .animation = animation,
               .rate = sampleRate == 0.0f ? 30.0f : sampleRate};
    b.rootNode = RootNode(&b);
    maniImportResult r = Run(builder, &b, loop, boundOut);
    maniArenaRelease(&b.arena);
    return r;
}
