// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF 2.0 import (mani-0008), offline: a document opened from JSON or
// GLB bytes the caller keeps alive, its external buffers supplied by the
// host, then skeletons, skins and clips built from it.

#ifndef MAUL_ANIM_IMPORT_H
#define MAUL_ANIM_IMPORT_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // An opened glTF document.
    typedef struct maniGltf maniGltf;

    // An opened FBX document: its tree, read from the caller's bytes.
    typedef struct maniFbx maniFbx;

    // Why an import refused its input.
    typedef uint32_t maniImportReason;
    enum
    {
        maniImportReasonNone = 0,
        // The JSON is not RFC 8259's (item: the reader's own reason).
        maniImportReasonJson = 1,
        // The GLB container: header, length, chunks.
        maniImportReasonContainer = 2,
        // `asset.version` is not 2.x, or `minVersion` is past 2.0.
        maniImportReasonVersion = 3,
        // An extension in `extensionsRequired` the importer does not read.
        maniImportReasonExtension = 4,
        // A property missing, of the wrong kind, or out of its range.
        maniImportReasonProperty = 5,
        // An index past the array it refers to.
        maniImportReasonIndex = 6,
        // A buffer's data: its URI, its encoding, or too few bytes.
        maniImportReasonBuffer = 7,
        // An accessor's or buffer view's layout: out of bounds, misaligned,
        // a stride where none may be.
        maniImportReasonLayout = 8,
        // Data the specification forbids: NaN or infinite floats, sparse
        // indices not increasing.
        maniImportReasonData = 9,
        // An external buffer the host has not supplied.
        maniImportReasonPending = 10,
        // The node hierarchy: a node with two parents, a cycle, or a
        // skin's joints with no common root.
        maniImportReasonHierarchy = 11,
        // Bone names that collide even after renaming.
        maniImportReasonName = 12,
        // A node matrix that is projective or does not decompose (shear).
        maniImportReasonMatrix = 13,
        // A joint or animated node missing from the skeleton given.
        maniImportReasonBone = 14,
        // The builder refused what the document gave (item: the build's
        // reason).
        maniImportReasonBuild = 15,
        // An FBX file in the ASCII form that is not its syntax.
        maniImportReasonSyntax = 16,
    };

    // An axis of glTF's space, or its negative.
    typedef uint32_t maniAxis;
    enum
    {
        maniAxisX = 0,
        maniAxisY = 1,
        maniAxisZ = 2,
        maniAxisNegativeX = 3,
        maniAxisNegativeY = 4,
        maniAxisNegativeZ = 5,
    };

    // How a document's skeletons, skins and clips are built.
    typedef struct maniGltfOptions
    {
        // The basis change: the output's x, y and z axes as glTF axes,
        // each axis once (a signed permutation); X, Y, Z for none. For
        // Z up from glTF's Y up: X, NegativeZ, Y.
        maniAxis axes[3];
        // Fold the nodes above a skeleton's root into the root's rest
        // and keys, in place of reporting them as its placement.
        uint32_t foldPlacement;
    } maniGltfOptions;

    // No basis change, placement reported.
    static inline maniGltfOptions maniDefaultGltfOptions(void)
    {
        maniGltfOptions options = {{maniAxisX, maniAxisY, maniAxisZ}, 0};
        return options;
    }

    // An import's outcome: the status; for a refusal, the reason, the
    // offset of the first byte found wrong, and the item (an index into
    // the array the reason concerns). The offset is into the bytes given
    // to maniOpenGltf, JSON or GLB, except for maniImportReasonData and
    // for too few bytes supplied, where it is into the buffer's bytes.
    typedef struct maniImportResult
    {
        maniResult result;
        maniImportReason reason;
        uint64_t offset;
        uint32_t item;
        uint32_t reserved;
    } maniImportResult;

    /// Opens a glTF document from JSON or GLB bytes (told apart by GLB's
    /// magic), checking the container, the asset version, required
    /// extensions, buffers, buffer views and accessors. `data:` buffers
    /// are decoded into the document; external buffers wait for
    /// maniGltfSupplyBuffer.
    ///
    /// @param builder  A builder; the document uses its allocator and
    ///                 must be closed before the builder is destroyed.
    /// @param bytes    The file; kept alive and unchanged while the
    ///                 document is open.
    /// @param size     Its size in bytes.
    /// @param gltfOut  Receives the document.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorFormat` with a reason, an offset and an item;
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniOpenGltf(maniBuilder* builder, const void* bytes,
                                                          size_t size, maniGltf** gltfOut);

    /// Closes a document and frees its memory. NULL is allowed.
    ///
    /// @param gltf  The document.
    /// @par Thread safety
    /// Safe from any thread; the document is used by one thread at a time.
    MANI_API void maniCloseGltf(maniGltf* gltf);

    /// Returns the number of buffers.
    ///
    /// @param gltf  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfBufferCount(const maniGltf* gltf);

    /// Returns an external buffer's URI, as the JSON gives it after its
    /// escapes (a URI reference relative to the document; percent
    /// escapes are left to the host, which resolves it), or NULL for a
    /// buffer the document holds (GLB or `data:`) or past the count.
    ///
    /// @param gltf       The document.
    /// @param buffer     The buffer.
    /// @param lengthOut  Receives the URI's length in bytes, 0 for NULL.
    ///                   May be NULL.
    /// @return The URI, UTF-8 and not terminated, or NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGltfBufferUri(const maniGltf* gltf, uint32_t buffer,
                                           uint32_t* lengthOut);

    /// Supplies an external buffer's bytes, which the caller keeps alive
    /// and unchanged while the document is open. They must hold the
    /// buffer's `byteLength`; bytes past it are ignored.
    ///
    /// @param gltf    The document.
    /// @param buffer  An external buffer.
    /// @param bytes   Its bytes.
    /// @param size    Their size.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a buffer that is not external; `mani_errorFormat` with
    /// maniImportReasonBuffer for too few bytes.
    /// @par Thread safety
    /// Safe from any thread; the document is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniGltfSupplyBuffer(maniGltf* gltf, uint32_t buffer,
                                                                  const void* bytes, size_t size);

    /// Returns the number of skeletons the document's skins make: skins
    /// sharing any node of their joints' hierarchy share one, numbered by
    /// their lowest skin.
    ///
    /// @param gltf  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfSkeletonCount(const maniGltf* gltf);

    /// Returns the number of skins.
    ///
    /// @param gltf  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfSkinCount(const maniGltf* gltf);

    /// Returns the skeleton a skin is part of.
    ///
    /// @param gltf  The document.
    /// @param skin  The skin.
    /// @return The skeleton, or UINT32_MAX for a skin past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfSkinSkeleton(const maniGltf* gltf, uint32_t skin);

    /// Returns the number of nodes.
    ///
    /// @param gltf  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfNodeCount(const maniGltf* gltf);

    /// Builds a skeleton of the document into the builder: its skins'
    /// joints, their common root and the nodes between, named by their
    /// nodes' names (renamed `name#node` where missing or repeated).
    ///
    /// @param builder   The builder.
    /// @param gltf      The document.
    /// @param skeleton  The skeleton, below maniGltfSkeletonCount.
    /// @param options   The options.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton past the count or options out of range;
    /// `mani_errorFormat` with a reason; `mani_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniGltfBuildSkeleton(maniBuilder* builder,
                                                                   const maniGltf* gltf,
                                                                   uint32_t skeleton,
                                                                   const maniGltfOptions* options);

    /// Builds a node's subtree as a skeleton, for rigid hierarchies.
    ///
    /// @param builder  The builder.
    /// @param gltf     The document.
    /// @param node     The root node.
    /// @param options  The options.
    /// @return As maniGltfBuildSkeleton.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniGltfBuildNodeSkeleton(
        maniBuilder* builder, const maniGltf* gltf, uint32_t node, const maniGltfOptions* options);

    /// Returns a skeleton's placement: the product of the local
    /// transforms of the nodes above its root, the basis change applied;
    /// identity when the options fold it into the root.
    ///
    /// @param gltf          The document.
    /// @param skeleton      The skeleton.
    /// @param options       The options.
    /// @param placementOut  Receives the placement.
    /// @return `mani_success`; `mani_errorInvalid` as
    /// maniGltfBuildSkeleton; `mani_errorFormat` for a node above the
    /// root whose matrix is refused.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniImportResult
    maniGltfSkeletonPlacement(const maniGltf* gltf, uint32_t skeleton,
                              const maniGltfOptions* options, maniAffine* placementOut);

    /// Builds a skin into the builder against an opened skeleton: each
    /// joint found by its bone name, the inverse binds read (identity
    /// when the skin has none) and the basis change applied.
    ///
    /// @param builder   The builder.
    /// @param gltf      The document.
    /// @param skin      The skin.
    /// @param skeleton  An opened skeleton holding the skin's joints.
    /// @param options   The options the skeleton was built with.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skin past the count or options out of range; `mani_errorFormat`
    /// with a reason; `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniGltfBuildSkin(maniBuilder* builder,
                                                               const maniGltf* gltf, uint32_t skin,
                                                               const maniSkeleton* skeleton,
                                                               const maniGltfOptions* options);

    /// Returns the bone name a node has in every skeleton, skin and clip
    /// built from the document (mani-0009): its name, or, when it has
    /// none or an earlier node has the same, that name followed by `#`
    /// and the node index.
    ///
    /// @param gltf       The document.
    /// @param node       The node.
    /// @param lengthOut  Receives the length in bytes, 0 past the count.
    ///                   May be NULL.
    /// @return The name, UTF-8 and not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGltfNodeBoneName(const maniGltf* gltf, uint32_t node,
                                              uint32_t* lengthOut);

    /// Returns the number of nodes whose bone name is not their name as
    /// given: unnamed, or named as an earlier node. Their names follow
    /// the node index, so a host binding by name across exports can warn
    /// or keep its own map.
    ///
    /// @param gltf  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfRenamedCount(const maniGltf* gltf);

    /// Returns a renamed node, in node order.
    ///
    /// @param gltf   The document.
    /// @param index  Below maniGltfRenamedCount.
    /// @return The node, or UINT32_MAX past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfRenamedNode(const maniGltf* gltf, uint32_t index);

    /// Returns the number of animations.
    ///
    /// @param gltf  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfAnimationCount(const maniGltf* gltf);

    /// Returns an animation's name, UTF-8 and not terminated; empty for an
    /// animation without one.
    ///
    /// @param gltf       The document.
    /// @param animation  The animation.
    /// @param lengthOut  Receives the length in bytes, 0 past the count.
    ///                   May be NULL.
    /// @return The name, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGltfAnimationName(const maniGltf* gltf, uint32_t animation,
                                               uint32_t* lengthOut);

    /// Builds one animation as a clip of an opened skeleton (mani-0010):
    /// translation, rotation and scale channels on its bones (found by
    /// name), morph weights of its bones and of meshes it skins, and
    /// `KHR_animation_pointer` channels to anything but nodes, each as
    /// float tracks; the basis change applied and, with placement folded,
    /// the nodes above the root composed in. The duration is the
    /// animation's latest key over every channel.
    ///
    /// @param builder     The builder.
    /// @param gltf        The document, its buffers supplied.
    /// @param animation   The animation.
    /// @param skeleton    An opened skeleton.
    /// @param options     The options the skeleton was built with.
    /// @param sampleRate  Samples per second; 0 for 30.
    /// @param loop        The clip's loop policy.
    /// @param boundOut    Receives how many of the animation's channels the
    ///                    clip holds. May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, an
    /// animation past the count or options out of range;
    /// `mani_errorFormat` with a reason (Data with the channel as item for
    /// keys the clip builder refuses); `mani_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult
    maniGltfBuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t animation,
                      const maniSkeleton* skeleton, const maniGltfOptions* options,
                      float sampleRate, maniLoopPolicy loop, uint32_t* boundOut);

    /// Reads the document's `VRMC_springBone` extension (version 1.0) as
    /// spring bone sources on an opened skeleton (mani-0011): colliders,
    /// collider groups and springs, nodes found by bone name, offsets,
    /// tails and gravity directions in the output's basis. A document
    /// without the extension gives an empty set. The arrays are the
    /// document's until the next call or maniCloseGltf.
    ///
    /// @param gltf      The document.
    /// @param skeleton  An opened skeleton holding the springs' nodes.
    /// @param options   The options the skeleton was built with.
    /// @param setOut    Receives the sources.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// options out of range; `mani_errorFormat` with a reason;
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the document is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniGltfReadSprings(maniGltf* gltf,
                                                                 const maniSkeleton* skeleton,
                                                                 const maniGltfOptions* options,
                                                                 maniSpringSetSource* setOut);

    /// Opens an FBX document (mani-0012) from the bytes of a file in the
    /// binary form, versions 3000 to 7700, either byte order. Every record
    /// and length is checked against the bytes; no array is decoded until
    /// asked for.
    ///
    /// @param builder  The builder whose allocator the document uses; it
    ///                 must be closed before the builder is destroyed.
    /// @param bytes    The file; kept alive and unchanged while the
    ///                 document is open.
    /// @param size     Its size in bytes.
    /// @param fbxOut   Receives the document.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorFormat` with a reason (Container, Version, Syntax) and
    /// the offset of the first byte found wrong; `mani_errorCapacity`
    /// when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniOpenFbx(maniBuilder* builder, const void* bytes,
                                                         size_t size, maniFbx** fbxOut);

    /// Closes an FBX document and frees its memory. NULL is allowed.
    ///
    /// @param fbx  The document.
    /// @par Thread safety
    /// Safe from any thread; the document is used by one thread at a time.
    MANI_API void maniCloseFbx(maniFbx* fbx);

    /// Returns the document's version (7400 for FBX 7.4), 0 for NULL.
    ///
    /// @param fbx  The document.
    /// @return The version.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxVersion(const maniFbx* fbx);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_IMPORT_H
