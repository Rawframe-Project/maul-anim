// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a glTF document: JSON alone or in a GLB container,
// the asset version, the required extensions, then buffers, buffer
// views and accessors; and the host's external buffers.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "fpenv.h"
#include "gltf_animation.h"
#include "gltf_buffer.h"
#include "gltf_compression.h"
#include "gltf_document.h"
#include "gltf_layout.h"
#include "gltf_names.h"
#include "gltf_node.h"
#include "gltf_property.h"
#include "host_code.h"
#include "json.h"

#include "maul-anim/import.h"

#include <string.h>

#define MANI_GLB_MAGIC 0x46546C67u
#define MANI_GLB_JSON  0x4E4F534Au
#define MANI_GLB_BIN   0x004E4942u

// Extensions a file may require that the importer reads, or that change
// nothing it reads (meshes, materials, textures, lights).
static const char* const kKnownExtensions[] = {
    "KHR_animation_pointer",
    "KHR_mesh_quantization",
    "KHR_draco_mesh_compression",
    "EXT_meshopt_compression",
    "KHR_meshopt_compression",
    "KHR_texture_basisu",
    "KHR_texture_transform",
    "EXT_texture_webp",
    "EXT_texture_avif",
    "KHR_lights_punctual",
    "KHR_materials_variants",
    "KHR_materials_unlit",
    "KHR_materials_emissive_strength",
    "KHR_materials_clearcoat",
    "KHR_materials_ior",
    "KHR_materials_iridescence",
    "KHR_materials_sheen",
    "KHR_materials_specular",
    "KHR_materials_transmission",
    "KHR_materials_volume",
    "KHR_materials_anisotropy",
    "KHR_materials_dispersion",
    "KHR_materials_pbrSpecularGlossiness",
};

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

// GLB's header and chunks: the JSON chunk first, a BIN chunk second if
// any, chunks after them skipped; every chunk 4-aligned and inside the
// length the header gives.
static maniImportResult ReadGlb(maniGltf* gltf, const uint8_t* bytes, size_t size,
                                const uint8_t** jsonOut, size_t* jsonSizeOut)
{
    if (size < 20 || maniLoadU32(bytes + 4) != 2)
    {
        return maniImportFail(maniImportReasonContainer, size < 20 ? size : 4, 0);
    }
    uint64_t length = maniLoadU32(bytes + 8);
    if (length < 20 || length > size || length % 4 != 0)
    {
        return maniImportFail(maniImportReasonContainer, 8, 0);
    }
    uint64_t at = 12;
    uint32_t chunk = 0;
    while (at < length)
    {
        uint64_t chunkLength = length - at >= 8 ? maniLoadU32(bytes + at) : UINT64_MAX;
        if (chunkLength == UINT64_MAX || chunkLength % 4 != 0 || chunkLength > length - at - 8)
        {
            return maniImportFail(maniImportReasonContainer, at, chunk);
        }
        uint32_t type = maniLoadU32(bytes + at + 4);
        bool json = type == MANI_GLB_JSON;
        bool bin = type == MANI_GLB_BIN;
        if ((chunk == 0) != json || (bin && chunk != 1))
        {
            return maniImportFail(maniImportReasonContainer, at + 4, chunk);
        }
        if (json)
        {
            *jsonOut = bytes + at + 8;
            *jsonSizeOut = (size_t)chunkLength;
        }
        if (bin)
        {
            gltf->bin = bytes + at + 8;
            gltf->binSize = chunkLength;
        }
        at += 8 + chunkLength;
        ++chunk;
    }
    gltf->isGlb = true;
    return maniImportSuccess();
}

// "major.minor" into its two numbers; false for anything else.
static bool ParseVersion(const maniJson* json, uint32_t value, uint32_t* major, uint32_t* minor)
{
    uint8_t text[24];
    if (json->values[value].length >= sizeof text)
    {
        return false;
    }
    size_t n = maniJsonStringDecode(json, value, text, sizeof text);
    uint32_t parts[2] = {0, 0};
    uint32_t part = 0;
    size_t digits = 0;
    for (size_t i = 0; i < n; ++i)
    {
        if (text[i] == '.' && part == 0 && digits > 0)
        {
            part = 1;
            digits = 0;
        }
        else if (text[i] >= '0' && text[i] <= '9' && digits < 6)
        {
            parts[part] = parts[part] * 10 + (uint32_t)(text[i] - '0');
            ++digits;
        }
        else
        {
            return false;
        }
    }
    *major = parts[0];
    *minor = parts[1];
    return part == 1 && digits > 0;
}

static maniImportResult ReadAsset(const maniGltf* gltf)
{
    const maniGltfJson* g = &gltf->g;
    uint32_t asset = MANI_JSON_NONE;
    uint32_t version = MANI_JSON_NONE;
    uint32_t minimum = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, 0, "asset", maniJsonObject, true, 0, &asset);
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, asset, "version", maniJsonString, true, 0, &version);
    }
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, asset, "minVersion", maniJsonString, false, 0, &minimum);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t major = 0;
    uint32_t minor = 0;
    if (!ParseVersion(&gltf->json, version, &major, &minor) || major != 2)
    {
        return maniImportFail(maniImportReasonVersion, maniGltfOffset(g, version), 0);
    }
    if (minimum != MANI_JSON_NONE &&
        (!ParseVersion(&gltf->json, minimum, &major, &minor) || major != 2 || minor != 0))
    {
        return maniImportFail(maniImportReasonVersion, maniGltfOffset(g, minimum), 0);
    }
    return maniImportSuccess();
}

static maniImportResult ReadExtensions(const maniGltf* gltf)
{
    const maniGltfJson* g = &gltf->g;
    uint32_t array = MANI_JSON_NONE;
    uint32_t count = 0;
    maniImportResult r = maniGltfArray(g, "extensionsRequired", maniJsonString, &array, &count);
    uint32_t name = array + 1;
    for (uint32_t i = 0; r.result == mani_success && i < count; ++i)
    {
        bool known = false;
        for (size_t k = 0; k < sizeof kKnownExtensions / sizeof kKnownExtensions[0]; ++k)
        {
            known = known || maniJsonStringEquals(&gltf->json, name, kKnownExtensions[k],
                                                  strlen(kKnownExtensions[k]));
        }
        if (!known)
        {
            return maniImportFail(maniImportReasonExtension, maniGltfOffset(g, name), i);
        }
        name = gltf->json.values[name].next;
    }
    return r;
}

static maniImportResult Open(maniGltf* gltf, const uint8_t* bytes, size_t size)
{
    const uint8_t* text = bytes;
    size_t textSize = size;
    maniImportResult r;
    if (size >= 4 && maniLoadU32(bytes) == MANI_GLB_MAGIC)
    {
        r = ReadGlb(gltf, bytes, size, &text, &textSize);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    gltf->g = (maniGltfJson){&gltf->json, (uint64_t)(text - bytes)};
    maniJsonResult j = maniReadJson(&gltf->allocator, text, textSize, &gltf->json);
    if (j.result != mani_success)
    {
        return j.result == mani_errorCapacity
                   ? Capacity()
                   : maniImportFail(maniImportReasonJson, gltf->g.base + j.offset, j.reason);
    }
    if (gltf->json.values[0].kind != maniJsonObject)
    {
        return maniImportFail(maniImportReasonProperty, gltf->g.base, 0);
    }
    r = ReadAsset(gltf);
    if (r.result == mani_success)
    {
        r = ReadExtensions(gltf);
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadBuffers(gltf);
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadLayout(gltf);
    }
    if (r.result == mani_success)
    {
        r = maniGltfFinishCompression(gltf);
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadNodes(gltf);
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadNames(gltf);
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadAnimations(gltf);
    }
    return r;
}

static MANI_NOINLINE maniImportResult OpenGltfWork(maniBuilder* builder, const void* bytes,
                                                   size_t size, maniGltf** gltfOut)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorState, maniImportReasonNone, 0, 0, 0};
    }
    if (gltfOut != NULL)
    {
        *gltfOut = NULL;
    }
    if (builder == NULL)
    {
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    if (bytes == NULL || gltfOut == NULL)
    {
        maniBuilderMisuse(builder);
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    maniGltf* gltf = maniAllocate(&builder->allocator, 1, sizeof(maniGltf), 8);
    if (gltf == NULL)
    {
        return Capacity();
    }
    memset(gltf, 0, sizeof *gltf);
    gltf->allocator = builder->allocator;
    maniImportResult r = Open(gltf, bytes, size);
    if (r.result != mani_success)
    {
        maniCloseGltf(gltf);
        return r;
    }
    *gltfOut = gltf;
    return r;
}

maniImportResult maniOpenGltf(maniBuilder* builder, const void* bytes, size_t size,
                              maniGltf** gltfOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = OpenGltfWork(builder, bytes, size, gltfOut);
    maniFloatEnvLeave(env);
    return result;
}

void maniCloseGltf(maniGltf* gltf)
{
    // A close from the builder's allocator, which the document shares, is
    // refused and counted on the builder.
    if (gltf == NULL || maniHostAllocatorCalledBack(&gltf->allocator))
    {
        return;
    }
    maniAllocator allocator = gltf->allocator;
    maniArenaRelease(&gltf->springs);
    maniArenaRelease(&gltf->ragdoll);
    maniGltfReleaseAnimations(gltf);
    maniGltfReleaseNames(gltf);
    maniGltfReleaseNodes(gltf);
    maniGltfReleaseBuffers(gltf);
    maniGltfReleaseDecoded(gltf);
    maniRelease(&allocator, gltf->views, gltf->viewCount, sizeof(maniGltfView), 8);
    maniRelease(&allocator, gltf->accessors, gltf->accessorCount, sizeof(maniGltfAccessor), 8);
    maniReleaseJson(&allocator, &gltf->json);
    maniRelease(&allocator, gltf, 1, sizeof(maniGltf), 8);
}

uint32_t maniGltfBufferCount(const maniGltf* gltf)
{
    return gltf->bufferCount;
}

const char* maniGltfBufferUri(const maniGltf* gltf, uint32_t buffer, uint32_t* lengthOut)
{
    bool external = buffer < gltf->bufferCount && gltf->buffers[buffer].external;
    if (lengthOut != NULL)
    {
        *lengthOut = external ? gltf->buffers[buffer].uriLength : 0;
    }
    if (!external)
    {
        return NULL;
    }
    return gltf->buffers[buffer].uri != NULL ? gltf->buffers[buffer].uri : "";
}

static MANI_NOINLINE maniImportResult GltfSupplyBufferWork(maniGltf* gltf, uint32_t buffer,
                                                           const void* bytes, size_t size)
{
    if (gltf == NULL || bytes == NULL || buffer >= gltf->bufferCount ||
        !gltf->buffers[buffer].external)
    {
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    if (size < gltf->buffers[buffer].byteLength)
    {
        return maniImportFail(maniImportReasonBuffer, size, buffer);
    }
    gltf->buffers[buffer].data = bytes;
    maniImportResult r = maniGltfDecodeBuffer(gltf, buffer);
    if (r.result != mani_success)
    {
        gltf->buffers[buffer].data = NULL;
    }
    return r;
}

maniImportResult maniGltfSupplyBuffer(maniGltf* gltf, uint32_t buffer, const void* bytes,
                                      size_t size)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = GltfSupplyBufferWork(gltf, buffer, bytes, size);
    maniFloatEnvLeave(env);
    return result;
}

uint32_t maniGltfSkeletonCount(const maniGltf* gltf)
{
    return gltf->skeletonCount;
}

uint32_t maniGltfSkinCount(const maniGltf* gltf)
{
    return gltf->skinCount;
}

uint32_t maniGltfSkinSkeleton(const maniGltf* gltf, uint32_t skin)
{
    return skin < gltf->skinCount ? gltf->skinSkeletons[skin] : UINT32_MAX;
}

uint32_t maniGltfNodeCount(const maniGltf* gltf)
{
    return gltf->nodeCount;
}

uint32_t maniGltfAnimationCount(const maniGltf* gltf)
{
    return gltf->animationCount;
}

const char* maniGltfAnimationName(const maniGltf* gltf, uint32_t animation, uint32_t* lengthOut)
{
    const maniBoneNames* names = &gltf->animationNames;
    bool inRange = animation < gltf->animationCount;
    if (lengthOut != NULL)
    {
        *lengthOut = inRange ? names->starts[animation + 1] - names->starts[animation] : 0;
    }
    return inRange ? names->text + names->starts[animation] : NULL;
}
