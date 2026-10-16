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
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // An opened glTF document.
    typedef struct maniGltf maniGltf;

    // One clip's settings for maniGltfBuildClip and maniFbxBuildClip
    // (mani-0022), beside the document's options.
    typedef struct maniImportClipOptions
    {
        // Samples per second; 0 for the format's default.
        float sampleRate;
        maniLoopPolicy loop;
        // maniAdditiveNone, or the basis a difference clip is taken from.
        maniAdditiveBasis additive;
        uint32_t reserved;
        // The root motion to take out, or NULL: a bone of the opened
        // skeleton, axes in the output's basis.
        const maniRootMotionSource* rootMotion;
    } maniImportClipOptions;

    // The format's rate, clamped, not additive, no root motion.
    static inline maniImportClipOptions maniDefaultImportClipOptions(void)
    {
        maniImportClipOptions options = {0.0f, maniLoopClamp, maniAdditiveNone, 0, NULL};
        return options;
    }

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
        // A vertex's joints and weights the format forbids: a weight below
        // 0, a joint with two non-zero weights or past the skin, glTF's
        // normalized weights not summing to 255 or 65535 (offset: the
        // weights' accessor or array; item: the vertex).
        maniImportReasonInfluence = 17,
    };

    // An axis, or its negative.
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
    /// animation's latest key over every channel. The clip's root motion,
    /// if any, is taken out as maniBuildClip takes it.
    ///
    /// @param builder     The builder.
    /// @param gltf        The document, its buffers supplied.
    /// @param animation   The animation.
    /// @param skeleton    An opened skeleton.
    /// @param options     The options the skeleton was built with.
    /// @param clipOptions The clip's settings; sample rate 0 for 30.
    /// @param boundOut    Receives how many of the animation's channels the
    ///                    clip holds. May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, an
    /// animation past the count, options out of range, or a clip setting
    /// out of range (a rate not finite or negative, a loop, basis or
    /// root motion source field out of range, a root motion bone past the
    /// skeleton); `mani_errorFormat` with a reason (Data with the channel
    /// as item for keys the clip builder refuses; Build with the build
    /// reason as item for anything else it refuses, such as a root motion
    /// bone the animation leaves without a track); `mani_errorCapacity`
    /// when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult
    maniGltfBuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t animation,
                      const maniSkeleton* skeleton, const maniGltfOptions* options,
                      const maniImportClipOptions* clipOptions, uint32_t* boundOut);

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

    /// Returns the number of meshes in the document.
    ///
    /// @param gltf  The document.
    /// @return The mesh count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfMeshCount(const maniGltf* gltf);

    /// Returns the number of primitives a mesh has.
    ///
    /// @param gltf  The document.
    /// @param mesh  The mesh.
    /// @return The primitive count; 0 for a mesh past the count or one
    /// whose primitives are not an array.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfPrimitiveCount(const maniGltf* gltf, uint32_t mesh);

    /// Returns the number of morph targets a mesh primitive has.
    ///
    /// @param gltf       The document.
    /// @param mesh       The mesh.
    /// @param primitive  The primitive.
    /// @return The target count; 0 for none or past a count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfPrimitiveTargetCount(const maniGltf* gltf, uint32_t mesh,
                                                   uint32_t primitive);

    /// Returns the mesh a node instances.
    ///
    /// @param gltf  The document.
    /// @param node  The node.
    /// @return The mesh, or UINT32_MAX for none or a node past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfNodeMesh(const maniGltf* gltf, uint32_t node);

    /// Returns the skin that deforms a node's mesh.
    ///
    /// @param gltf  The document.
    /// @param node  The node.
    /// @return The skin, or UINT32_MAX for none or a node past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniGltfNodeSkin(const maniGltf* gltf, uint32_t node);

    /// Reads the morph weights a node's mesh starts with: the node's
    /// `weights`, else its mesh's, else zeros.
    ///
    /// @param gltf        The document.
    /// @param node        The node.
    /// @param weightsOut  Receives count weights.
    /// @param count       The mesh's morph target count.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// node past the count or a count other than its mesh's targets;
    /// `mani_errorFormat` (Property) for weights not an array of that many
    /// numbers.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniImportResult maniGltfMorphWeights(const maniGltf* gltf,
                                                                  uint32_t node, float* weightsOut,
                                                                  uint32_t count);

    /// Builds a primitive's influences into the builder, bound to an
    /// opened skin built from the glTF skin that deforms it (its joints
    /// in the skin's order): every `JOINTS_n` and `WEIGHTS_n` set read,
    /// zero weights dropped, then sorted and renormalized as
    /// maniBuildInfluences does.
    ///
    /// @param builder    The builder.
    /// @param gltf       The document.
    /// @param mesh       The mesh.
    /// @param primitive  The primitive.
    /// @param skin       The opened skin.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a mesh or primitive past its count; `mani_errorFormat` with a
    /// reason: Property (no `JOINTS_0`, sets not paired, an accessor of
    /// the wrong type or count), Extension (Draco-compressed),
    /// Influence, Data, Pending, Build; `mani_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniGltfBuildInfluences(maniBuilder* builder,
                                                                     const maniGltf* gltf,
                                                                     uint32_t mesh,
                                                                     uint32_t primitive,
                                                                     const maniSkin* skin);

    /// Builds a primitive's morph targets into the builder: a channel a
    /// target, at full weight 1, named by the mesh's `extras.targetNames`
    /// when it names every target and by the target's index otherwise
    /// (weight tracks are the node's bone name, a slash and that name);
    /// positions, normals, tangents, texture coordinate and color sets
    /// read (other semantics left), positions, normals and tangents in
    /// the output's basis, VEC3 colors with alpha 0; a target keeps the
    /// vertices it moves.
    ///
    /// @param builder    The builder.
    /// @param gltf       The document.
    /// @param mesh       The mesh.
    /// @param primitive  The primitive.
    /// @param options    The options the skeleton was built with.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// mesh or primitive past its count, a primitive without targets or
    /// options out of range; `mani_errorFormat` with a reason: Property
    /// (a semantic the primitive lacks, an accessor of the wrong type or
    /// count, more than 32 attributes), Index, Extension (Draco), Data,
    /// Pending, Build; `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult
    maniGltfBuildMorphTargets(maniBuilder* builder, const maniGltf* gltf, uint32_t mesh,
                              uint32_t primitive, const maniGltfOptions* options);

    /// Opens an FBX document (mani-0012) from the bytes of a file, binary
    /// (told apart by its magic, either byte order) or ASCII, versions 3000
    /// to 7700. Every record, token and length is checked against the
    /// bytes; no array is decoded and no number converted until asked
    /// for.
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

    /// Returns the number of models (the objects of kind Model, in the
    /// order of the file's `Objects`), 0 for NULL.
    ///
    /// @param fbx  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxModelCount(const maniFbx* fbx);

    /// Returns the bone name a model has in every skeleton, skin and clip
    /// built from the document (mani-0014): its name without its class,
    /// or, when it has none or an earlier model has the same, that name
    /// followed by `#` and the model index.
    ///
    /// @param fbx        The document.
    /// @param model      The model.
    /// @param lengthOut  Receives the length in bytes, 0 past the count.
    ///                   May be NULL.
    /// @return The name, UTF-8 and not terminated, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniFbxModelBoneName(const maniFbx* fbx, uint32_t model,
                                              uint32_t* lengthOut);

    /// Returns the number of models whose bone names differ from their
    /// names, 0 for NULL.
    ///
    /// @param fbx  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxRenamedCount(const maniFbx* fbx);

    /// Returns a renamed model, in model order.
    ///
    /// @param fbx    The document.
    /// @param index  Below maniFbxRenamedCount.
    /// @return The model, or UINT32_MAX past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxRenamedModel(const maniFbx* fbx, uint32_t index);

    /// Returns the number of skeletons (mani-0014): skins' linked models
    /// and limb chains, joined where they share a model, numbered by
    /// their lowest skin, then chain; 0 for NULL.
    ///
    /// @param fbx  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxSkeletonCount(const maniFbx* fbx);

    /// Returns the number of skins (deformers of subclass Skin, in object
    /// order), 0 for NULL.
    ///
    /// @param fbx  The document.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxSkinCount(const maniFbx* fbx);

    /// Returns the skeleton a skin is part of.
    ///
    /// @param fbx   The document.
    /// @param skin  The skin.
    /// @return The skeleton, or UINT32_MAX for a skin past the count or
    /// one linking no model.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxSkinSkeleton(const maniFbx* fbx, uint32_t skin);

    /// Returns the model whose mesh a skin deforms: the skin's geometry's
    /// model (its own parent in 6.x files).
    ///
    /// @param fbx   The document.
    /// @param skin  The skin.
    /// @return The model, or UINT32_MAX for a skin past the count or one
    /// with no mesh model.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxSkinModel(const maniFbx* fbx, uint32_t skin);

    // An FBX scene's settings (GlobalSettings, mani-0013).
    typedef struct maniFbxSettings
    {
        // The file's right, up and front axes.
        maniAxis axes[3];
        uint32_t timeMode;
        // Metres a unit, now and as first authored.
        double unitMeters;
        double originalUnitMeters;
        // The frame rate, rateNumerator / rateDenominator frames a
        // second (30000 / 1001 for NTSC); a denominator of 0 for a custom
        // rate, customRate.
        uint32_t rateNumerator;
        uint32_t rateDenominator;
        double customRate;
        // The time span in FBX ticks, ticksPerSecond a second
        // (46186158000, or 141120000 where the header defines the newer
        // unit).
        int64_t spanStart;
        int64_t spanStop;
        int64_t ticksPerSecond;
    } maniFbxSettings;

    /// Reads the scene's settings, FBX's defaults where the file is
    /// silent: X right, Y up, Z front, a centimetre a unit, 30 frames a
    /// second.
    ///
    /// @param fbx          The document.
    /// @param settingsOut  Receives the settings.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniFbxReadSettings(const maniFbx* fbx,
                                                           maniFbxSettings* settingsOut);

    // How an FBX document's skeletons, skins and clips are built
    // (mani-0014).
    typedef struct maniFbxOptions
    {
        // With convertAxes 1, the output's right, up and front axes, each
        // axis once: the file's axes (maniFbxSettings) are turned to
        // them, exactly, mirroring when handedness differs.
        maniAxis axes[3];
        uint32_t convertAxes;
        // Fold the models above a skeleton's root into the root's rest
        // and keys, in place of reporting them as its placement.
        uint32_t foldPlacement;
        // The output's unit in metres (1 for metres), translations
        // scaled to it; 0 keeps the file's unit.
        double unitMeters;
        // A helper bone under every parent of a model that does not
        // inherit its parent's scale normally, not only where exact
        // compensation needs one: for clips from other files.
        uint32_t inheritHelpers;
        uint32_t reserved;
    } maniFbxOptions;

    // The file's axes and units, placement reported, helpers only where
    // needed.
    static inline maniFbxOptions maniDefaultFbxOptions(void)
    {
        maniFbxOptions options = {{maniAxisX, maniAxisY, maniAxisZ}, 0, 0, 0.0, 0, 0};
        return options;
    }

    /// Builds a skeleton of the document into the builder (mani-0014):
    /// its skins' linked models and limb chains, the models between and
    /// their common root, named by their bone names, with helper bones
    /// (`name#unscaled`) where FBX's scale inheritance needs them.
    ///
    /// @param builder   The builder.
    /// @param fbx       The document.
    /// @param skeleton  The skeleton, below maniFbxSkeletonCount.
    /// @param options   The options.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton past the count or options out of range;
    /// `mani_errorFormat` with a reason; `mani_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniFbxBuildSkeleton(maniBuilder* builder,
                                                                  const maniFbx* fbx,
                                                                  uint32_t skeleton,
                                                                  const maniFbxOptions* options);

    /// Builds a model's subtree as a skeleton, for rigid hierarchies.
    ///
    /// @param builder  The builder.
    /// @param fbx      The document.
    /// @param model    The root model, below maniFbxModelCount.
    /// @param options  The options.
    /// @return As maniFbxBuildSkeleton.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniFbxBuildModelSkeleton(
        maniBuilder* builder, const maniFbx* fbx, uint32_t model, const maniFbxOptions* options);

    /// Returns where a skeleton's root hangs: the world of the model
    /// above it (its unscaled world when the root does not inherit its
    /// scale normally), the options' axes and unit applied; identity when
    /// the root is at the top or the options fold the placement.
    ///
    /// @param fbx           The document.
    /// @param skeleton      The skeleton.
    /// @param options       The options.
    /// @param placementOut  Receives the placement.
    /// @return `mani_success`; `mani_errorInvalid` as
    /// maniFbxBuildSkeleton; `mani_errorFormat` for a model above the
    /// root whose properties are refused.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniImportResult maniFbxSkeletonPlacement(const maniFbx* fbx,
                                                                      uint32_t skeleton,
                                                                      const maniFbxOptions* options,
                                                                      maniAffine* placementOut);

    /// Builds a skin into the builder against an opened skeleton
    /// (mani-0015): each cluster linking a model a joint, found by the
    /// model's bone name, its inverse bind the mesh relative to the bone
    /// at bind (the cluster's matrices, a bind pose or the rest, as ufbx
    /// reads them) times the mesh's geometric transform, the options'
    /// axes and unit applied. Vertex weights are the mesh's to carry.
    ///
    /// @param builder   The builder.
    /// @param fbx       The document.
    /// @param skin      The skin, below maniFbxSkinCount.
    /// @param skeleton  An opened skeleton holding the skin's joints.
    /// @param options   The options the skeleton was built with.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skin past the count or options out of range; `mani_errorFormat`
    /// with a reason (Bone for a joint the skeleton lacks, Matrix for a
    /// bind that is not affine or not invertible, Property for an
    /// additive cluster); `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniFbxBuildSkin(maniBuilder* builder,
                                                              const maniFbx* fbx, uint32_t skin,
                                                              const maniSkeleton* skeleton,
                                                              const maniFbxOptions* options);

    /// Returns how a skin blends its joints, as ufbx evaluates it:
    /// dual quaternion for SkinningType DualQuaternion, blended for Blend
    /// or a skin that gives per-vertex blend weights, linear otherwise
    /// (Rigid among them).
    ///
    /// @param fbx   The document.
    /// @param skin  The skin.
    /// @return The method; linear for a skin past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniSkinningMethod maniFbxSkinMethod(const maniFbx* fbx, uint32_t skin);

    /// Builds a skin's influences into the builder, a vertex a control
    /// point of its mesh, bound to the opened skin maniFbxBuildSkin built
    /// from it (a cluster's index its joint): every cluster's Indexes and
    /// Weights, indices past the control points skipped, a vertex given
    /// twice by one cluster summed, then sorted and renormalized as
    /// maniBuildInfluences does; blend weights (BlendIndexes and
    /// BlendWeights, 1 for a dual quaternion skin, 0 else) unless the
    /// method is linear.
    ///
    /// @param builder  The builder.
    /// @param fbx      The document.
    /// @param skin     The skin.
    /// @param built    The opened skin.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a skin past the count; `mani_errorFormat` with a reason: Property
    /// (no control points, Indexes and Weights of different counts),
    /// Influence (a weight below 0, a blend weight outside 0 to 1), Data,
    /// Build; `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniFbxBuildInfluences(maniBuilder* builder,
                                                                    const maniFbx* fbx,
                                                                    uint32_t skin,
                                                                    const maniSkin* built);

    /// Returns how many sets of morph targets the document has: its
    /// BlendShape deformers, in file order, then (before 7.2) the meshes
    /// holding Shape children.
    ///
    /// @param fbx  The document; NULL gives 0.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxMorphCount(const maniFbx* fbx);

    /// Returns the mesh model a set of morph targets deforms; its bone
    /// name begins the clips' weight tracks for it (`<mesh>/<target>`).
    ///
    /// @param fbx    The document.
    /// @param morph  The set.
    /// @return The model, or UINT32_MAX for none or a set past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxMorphModel(const maniFbx* fbx, uint32_t morph);

    /// Returns the number of targets a set of morph targets has: its
    /// channels' shapes, but a shape after another of the same full
    /// weight (whose weight track is always 0).
    ///
    /// @param fbx    The document.
    /// @param morph  The set.
    /// @return The count; 0 past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxMorphTargetCount(const maniFbx* fbx, uint32_t morph);

    /// Builds a set of morph targets into the builder, as ufbx reads
    /// them: a channel a BlendShapeChannel (a Shape child before 7.2),
    /// its shapes the targets at FullWeights / 100 (as the clips order
    /// them), a vertex a control point; positions and normals as offsets
    /// in the options' axes (positions in its unit too), indices past the
    /// control points skipped, Blender's per-vertex weights folded in; a
    /// shape after another of the same full weight left out, as is a
    /// channel without shapes. Channels are named as their objects (one
    /// repeating an earlier one's name followed by '#' and its index); a
    /// target as its shape, else as its channel with '/' and its index
    /// past the first, as weight tracks are.
    ///
    /// @param builder  The builder.
    /// @param fbx      The document.
    /// @param morph    The set.
    /// @param options  The options the skeleton was built with.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// set past the count or options out of range; `mani_errorFormat`
    /// with a reason: Property (no control points, a shape's arrays of
    /// mismatched sizes), Data, Build (a full weight of 0, or a name not
    /// UTF-8; the item the builder's reason);
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult maniFbxBuildMorphTargets(
        maniBuilder* builder, const maniFbx* fbx, uint32_t morph, const maniFbxOptions* options);

    /// Reads the target weights a set of morph targets has at rest: each
    /// channel's DeformPercent walked through its shapes, as the clips'
    /// weight tracks are.
    ///
    /// @param fbx         The document.
    /// @param morph       The set.
    /// @param weightsOut  Receives count weights.
    /// @param count       The set's target count.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// set past the count or another count; `mani_errorFormat` (Data) for
    /// a FullWeights value not finite; `mani_errorCapacity` when the
    /// allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniImportResult maniFbxMorphWeights(const maniFbx* fbx, uint32_t morph,
                                                                 float* weightsOut, uint32_t count);

    /// Returns how many clips the document has: its animation stacks
    /// (takes), in file order (mani-0016).
    ///
    /// @param fbx  The document; NULL gives 0.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniFbxClipCount(const maniFbx* fbx);

    /// Returns a clip's name: its stack's, without the class, UTF-8 and
    /// not terminated.
    ///
    /// @param fbx        The document.
    /// @param clip       The clip.
    /// @param lengthOut  Receives the length in bytes, 0 past the count.
    ///                   May be NULL.
    /// @return The name, or NULL past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniFbxClipName(const maniFbx* fbx, uint32_t clip, uint32_t* lengthOut);

    /// Builds a clip into the builder for an opened skeleton (mani-0016):
    /// the stack's layers and curves evaluated as ufbx evaluates them, at
    /// every sample of the clip's grid from the stack's start (its local,
    /// reference or take times, else its keys), whole ticks apart. Each
    /// bone of the skeleton named as a model (or a model's helper,
    /// `name#unscaled`) takes its local as a skeleton built from the file
    /// formed its rest, the options' axes and unit applied and, with
    /// placement folded, the placement composed into the root; a channel
    /// that never leaves the skeleton's rest is left out. Other numbers a
    /// bone's model animates become float tracks named `bone/property`
    /// (`/0` to `/2` added for vectors). The clip's root motion, if any,
    /// is taken out as maniBuildClip takes it (its bone's track then keeps
    /// translation and rotation).
    ///
    /// @param builder     The builder.
    /// @param fbx         The document.
    /// @param clip        The clip, below maniFbxClipCount.
    /// @param skeleton    An opened skeleton.
    /// @param options     The options the skeleton was built with.
    /// @param clipOptions The clip's settings; sample rate 0 for the
    ///                    scene's frame rate up to 120, else 30.
    /// @param boundOut    Receives how many of the stack's channel bindings
    ///                    drive models the clip reads. May be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// clip past the count, options out of range, or a clip setting out
    /// of range (as maniGltfBuildClip); `mani_errorFormat` with a reason (Property with the clip
    /// as item for a stack's curves or channels refused, or with the model
    /// for an animated integer input out of range; Matrix with the model
    /// for a local that does not decompose at a sample; Name with the
    /// model for a repeated float track name; Build with the build reason
    /// as item for a timing, basis or root motion the builder refuses);
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniImportResult
    maniFbxBuildClip(maniBuilder* builder, const maniFbx* fbx, uint32_t clip,
                     const maniSkeleton* skeleton, const maniFbxOptions* options,
                     const maniImportClipOptions* clipOptions, uint32_t* boundOut);

    /// Returns where a model skeleton's root hangs, as
    /// maniFbxSkeletonPlacement.
    ///
    /// @param fbx           The document.
    /// @param model         The root model.
    /// @param options       The options.
    /// @param placementOut  Receives the placement.
    /// @return As maniFbxSkeletonPlacement.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniImportResult maniFbxModelPlacement(const maniFbx* fbx,
                                                                   uint32_t model,
                                                                   const maniFbxOptions* options,
                                                                   maniAffine* placementOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_IMPORT_H
