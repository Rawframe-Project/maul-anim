# Maul Anim API reference

Generated from the public headers by `tools/gen_api.py`. The headers
are the source of truth; this file mirrors them.

## `base.h`

The base of the Maul Anim API: the library version, the export and attribute macros, and the result codes every fallible function returns.

```c
maniVersion maniGetVersion(void);
```
Returns the version of the library that was linked, which may differ from the MANI_VERSION macros a program was compiled with.  @return The library version. @par Thread safety Safe from any thread.

```c
uint64_t maniHash64(uint64_t seed, const void* data, int32_t byteCount);
```
Hashes bytes into a 64-bit value, the hash the library's determinism checks are built on and a host can use to compare poses between machines: eight bytes a round, read in the host's byte order, xored in, multiplied by an odd constant and folded, the leftover bytes one at a time. Its constants are frozen.  @param seed       MANI_HASH_INIT, or a hash to continue. @param data       The bytes. May be NULL when byteCount is 0. @param byteCount  The number of bytes, at least 0. @return The hash. @par Thread safety Safe from any thread.

```c
const char* maniResultName(maniResult result);
```
Returns the name of a result code, for diagnostics.  @param result  Any value; an unknown one is named as such. @return A static, NUL-terminated string such as "mani_errorCapacity". @par Thread safety Safe from any thread.

## `build.h`

The offline side: a builder turns source data into the library's files (mani-0003), which it keeps until the next build for a copy into caller memory. A builder allocates through the allocator of its def and is built only when MAUL_ANIM_OFFLINE is on.

```c
static inline maniBuilderDef maniDefaultBuilderDef(void);
```
Returns a builder def with every setting at its default.  @return The def, with a valid cookie. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniCreateBuilder(const maniBuilderDef* def, maniBuilder** builderOut);
```
Makes a builder from a def.  @param def        The def. @param builderOut Receives the builder, or NULL on failure. @return `mani_success`; `mani_errorVersion` for a def built against another major or minor version; `mani_errorInvalid` for a NULL argument, a def without its cookie or an allocator with one function; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread.

```c
void maniDestroyBuilder(maniBuilder* builder);
```
Destroys a builder and the file it holds.  @param builder  The builder, or NULL. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
uint32_t maniBuilderMisuses(const maniBuilder* builder);
```
Returns how many calls on the builder were refused for invalid arguments (conventions section 8).  @param builder  The builder. @return The count. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildSkeleton(maniBuilder* builder, const maniSkeletonSource* source);
```
Builds a skeleton file: checks every bone, orders the bones depth-first (children and roots in the source's order), normalizes rotations, and keeps the file in place of any built before.  @param builder  The builder. @param source   The bones. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse) or with the reason and source bone for a refused source; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildSkin(maniBuilder* builder, const maniSkinSource* source);
```
Builds a skin file bound to the source's skeleton, and keeps it in place of any file built before.  @param builder  The builder. @param source   The skin. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and joint for a refused source (no joints or too many, a bone past the skeleton's, a matrix not finite); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniBuildResult maniBuildClip(maniBuilder* builder, const maniClipSource* source);
```
Builds a full-precision clip file: evaluates every channel at each sample time as glTF defines its interpolation (in binary64, rotations normalized and kept in one hemisphere from sample to sample), and keeps the file in place of any built before.  @param builder  The builder. @param source   The clip. @return `mani_success`; `mani_errorInvalid` for a NULL argument (a misuse), or with the reason and track for a refused source; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniMeasureClip(maniBuilder* builder, const maniClipSource* source, const maniClip* clip, maniClipReport* reportOut);
```
Measures a clip against the source it was built from, at every sample time and halfway between samples, through the clip's own sampling, against the source evaluated in binary64. Rotations are compared by the angle between them.  @param builder    A builder, for its allocator. @param source     The clip's source, as maniBuildClip accepts it. @param clip       The clip opened from what the build made. @param reportOut  Receives the report. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a source the build refuses, or a clip not of the source's skeleton or missing one of its float tracks; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniCopyBuilt(maniBuilder* builder, void* bufferOut, size_t capacity, size_t* sizeOut);
```
Copies the last built file into caller memory.  @param builder   The builder. @param bufferOut Receives the bytes: at least capacity bytes; may be NULL when capacity is 0. Open the copy from a 16-byte aligned address. @param capacity  The buffer's size in bytes. @param sizeOut   Receives the file's size in bytes. May be NULL. @return `mani_success`; `mani_errorCapacity` when the buffer is smaller than the file, with the size still written; `mani_errorInvalid` for a NULL builder, a NULL buffer with a positive capacity, or a builder holding no file. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniCopySkeletonOrder(maniBuilder* builder, uint32_t* boneOut, uint32_t capacity, uint32_t* countOut);
```
Copies where each source bone of the last built skeleton went: the skeleton's index of source bone i at boneOut[i].  @param builder   The builder. @param boneOut   Receives up to capacity indices; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the source's bone count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the count, with the count still written and the array filled as far as it goes; `mani_errorInvalid` for a NULL builder, a NULL array with a positive capacity, or a builder holding no skeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

## `clip.h`

Clips (mani-0006): a skeleton's motion as samples on one uniform grid, read in place from a clip file and bound to its skeleton by identity. Sampling costs the same at any time, in any order.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenClip(const void* bytes, size_t size, maniClip* clipOut);
```
Opens a clip file in place: checks the container, the header, every track and every sample (finite, rotations unit) and the identity.  @param bytes    The file, 16-byte aligned. @param size     Its size in bytes; bytes past the file are ignored. @param clipOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
float maniClipDuration(const maniClip* clip);
```
Returns the clip's duration in seconds, at least 0.  @param clip  An opened clip. @return The duration. @par Thread safety Safe from any thread.

```c
float maniClipSampleRate(const maniClip* clip);
```
Returns the number of samples per second.  @param clip  An opened clip. @return The rate. @par Thread safety Safe from any thread.

```c
uint32_t maniClipSampleCount(const maniClip* clip);
```
Returns the number of samples, at least 1: one at each multiple of the sample period, the last at the duration.  @param clip  An opened clip. @return The sample count. @par Thread safety Safe from any thread.

```c
maniLoopPolicy maniClipLoopPolicy(const maniClip* clip);
```
Returns the clip's loop policy.  @param clip  An opened clip. @return maniLoopClamp or maniLoopRepeat. @par Thread safety Safe from any thread.

```c
uint64_t maniClipSkeletonIdentity(const maniClip* clip);
```
Returns the identity of the skeleton the clip is bound to.  @param clip  An opened clip. @return The skeleton's identity. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClip(const maniClip* clip, const maniSkeleton* skeleton, float time, maniPose* pose);
```
Samples the clip at a time into a pose: every bone's local transform, the clip's channels where it animates them and the skeleton's rest elsewhere; the model is marked stale. The time is clamped to [0, duration] (NaN is 0). Between two samples, translation and scale interpolate linearly and rotations by normalized linear interpolation.  @param clip      An opened clip. @param skeleton  The clip's skeleton. @param time      The time in seconds. @param pose      A pose of the skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a skeleton that is not the clip's or the pose's. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
uint32_t maniClipFloatTrackCount(const maniClip* clip);
```
Returns the number of float tracks (morph weights, curves).  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniClipFloatTrackName(const maniClip* clip, uint32_t track, uint32_t* lengthOut);
```
Returns a float track's name, UTF-8 and not terminated.  @param clip       An opened clip. @param track      The track, below the count. @param lengthOut  Receives the length in bytes, 0 for a track past the count. May be NULL. @return The name's bytes, or NULL for a track past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniClipFindFloatTrack(const maniClip* clip, const char* name, size_t length);
```
Finds a float track by name.  @param clip    An opened clip. @param name    The name's UTF-8 bytes; may be NULL when length is 0. @param length  The name's length in bytes. @return The track, or MANI_NO_TRACK. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniSampleClipFloats(const maniClip* clip, float time, float* values, uint32_t capacity, uint32_t* countOut);
```
Samples every float track at a time, clamped as maniSampleClip clamps it: a linear track interpolates its two samples, a step track holds the sample at or before the time.  @param clip      An opened clip. @param time      The time in seconds. @param values    Receives one value per track, in track order; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the track count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the track count, with the count written and nothing else; `mani_errorInvalid` for a NULL clip or a NULL array with a positive capacity. @par Thread safety Safe from any thread.

```c
uint32_t maniClipDiscreteTrackCount(const maniClip* clip);
```
Returns the number of discrete tracks (whole numbers held between keys).  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniClipDiscreteTrackName(const maniClip* clip, uint32_t track, uint32_t* lengthOut);
```
Returns a discrete track's name, as maniClipFloatTrackName does.  @param clip       An opened clip. @param track      The track. @param lengthOut  Receives the length in bytes. May be NULL. @return The name's bytes, or NULL for a track past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniClipFindDiscreteTrack(const maniClip* clip, const char* name, size_t length);
```
Finds a discrete track by name.  @param clip    An opened clip. @param name    The name's UTF-8 bytes; may be NULL when length is 0. @param length  The name's length in bytes. @return The track, or MANI_NO_TRACK. @par Thread safety Safe from any thread.

```c
int32_t maniClipDiscreteValue(const maniClip* clip, uint32_t track, float time);
```
Returns a discrete track's value at a time: its last key at or before the time, or its first key before that (NaN is 0).  @param clip   An opened clip. @param track  The track. @param time   The time in seconds. @return The value, or 0 for a track past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniClipEventCount(const maniClip* clip);
```
Returns the number of events.  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
maniClipEvent maniClipEventAt(const maniClip* clip, uint32_t index);
```
Returns an event; events are in increasing time.  @param clip   An opened clip. @param index  The event. @return The event, or a zeroed one for an index past the count. @par Thread safety Safe from any thread.

```c
void maniClipEventRange(const maniClip* clip, float from, float to, uint32_t* firstOut, uint32_t* countOut);
```
Finds the events a playhead passes going from one time to another: forward (from below to) those with from <= time < to, backward (from above to) those with to < time <= from; none when the two are equal or either is NaN. They are consecutive indices.  @param clip      An opened clip. @param from      Where the playhead was. @param to        Where it is. @param firstOut  Receives the first event's index. @param countOut  Receives how many. @par Thread safety Safe from any thread.

```c
uint32_t maniClipMarkerCount(const maniClip* clip);
```
Returns the number of sync markers.  @param clip  An opened clip. @return The count. @par Thread safety Safe from any thread.

```c
maniClipEvent maniClipMarkerAt(const maniClip* clip, uint32_t index);
```
Returns a sync marker; markers are in increasing time.  @param clip   An opened clip. @param index  The marker. @return The marker, or a zeroed one for an index past the count. @par Thread safety Safe from any thread.

## `import.h`

glTF 2.0 import (mani-0008), offline: a document opened from JSON or GLB bytes the caller keeps alive, its external buffers supplied by the host, then skeletons, skins and clips built from it.

```c
static inline maniGltfOptions maniDefaultGltfOptions(void);
```

```c
MANI_NODISCARD MANI_API maniImportResult maniOpenGltf(maniBuilder* builder, const void* bytes, size_t size, maniGltf** gltfOut);
```
Opens a glTF document from JSON or GLB bytes (told apart by GLB's magic), checking the container, the asset version, required extensions, buffers, buffer views and accessors. `data:` buffers are decoded into the document; external buffers wait for maniGltfSupplyBuffer.  @param builder  A builder; the document uses its allocator and must be closed before the builder is destroyed. @param bytes    The file; kept alive and unchanged while the document is open. @param size     Its size in bytes. @param gltfOut  Receives the document. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with a reason, an offset and an item; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
void maniCloseGltf(maniGltf* gltf);
```
Closes a document and frees its memory. NULL is allowed.  @param gltf  The document. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniGltfBufferCount(const maniGltf* gltf);
```
Returns the number of buffers.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniGltfBufferUri(const maniGltf* gltf, uint32_t buffer, uint32_t* lengthOut);
```
Returns an external buffer's URI, as the JSON gives it after its escapes (a URI reference relative to the document; percent escapes are left to the host, which resolves it), or NULL for a buffer the document holds (GLB or `data:`) or past the count.  @param gltf       The document. @param buffer     The buffer. @param lengthOut  Receives the URI's length in bytes, 0 for NULL. May be NULL. @return The URI, UTF-8 and not terminated, or NULL. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfSupplyBuffer(maniGltf* gltf, uint32_t buffer, const void* bytes, size_t size);
```
Supplies an external buffer's bytes, which the caller keeps alive and unchanged while the document is open. They must hold the buffer's `byteLength`; bytes past it are ignored.  @param gltf    The document. @param buffer  An external buffer. @param bytes   Its bytes. @param size    Their size. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a buffer that is not external; `mani_errorFormat` with maniImportReasonBuffer for too few bytes. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniGltfSkeletonCount(const maniGltf* gltf);
```
Returns the number of skeletons the document's skins make: skins sharing any node of their joints' hierarchy share one, numbered by their lowest skin.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfSkinCount(const maniGltf* gltf);
```
Returns the number of skins.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfSkinSkeleton(const maniGltf* gltf, uint32_t skin);
```
Returns the skeleton a skin is part of.  @param gltf  The document. @param skin  The skin. @return The skeleton, or UINT32_MAX for a skin past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfNodeCount(const maniGltf* gltf);
```
Returns the number of nodes.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildSkeleton(maniBuilder* builder, const maniGltf* gltf, uint32_t skeleton, const maniGltfOptions* options);
```
Builds a skeleton of the document into the builder: its skins' joints, their common root and the nodes between, named by their nodes' names (renamed `name#node` where missing or repeated).  @param builder   The builder. @param gltf      The document. @param skeleton  The skeleton, below maniGltfSkeletonCount. @param options   The options. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton past the count or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildNodeSkeleton( maniBuilder* builder, const maniGltf* gltf, uint32_t node, const maniGltfOptions* options);
```
Builds a node's subtree as a skeleton, for rigid hierarchies.  @param builder  The builder. @param gltf     The document. @param node     The root node. @param options  The options. @return As maniGltfBuildSkeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfSkeletonPlacement(const maniGltf* gltf, uint32_t skeleton, const maniGltfOptions* options, maniAffine* placementOut);
```
Returns a skeleton's placement: the product of the local transforms of the nodes above its root, the basis change applied; identity when the options fold it into the root.  @param gltf          The document. @param skeleton      The skeleton. @param options       The options. @param placementOut  Receives the placement. @return `mani_success`; `mani_errorInvalid` as maniGltfBuildSkeleton; `mani_errorFormat` for a node above the root whose matrix is refused. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildSkin(maniBuilder* builder, const maniGltf* gltf, uint32_t skin, const maniSkeleton* skeleton, const maniGltfOptions* options);
```
Builds a skin into the builder against an opened skeleton: each joint found by its bone name, the inverse binds read (identity when the skin has none) and the basis change applied.  @param builder   The builder. @param gltf      The document. @param skin      The skin. @param skeleton  An opened skeleton holding the skin's joints. @param options   The options the skeleton was built with. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skin past the count or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
const char* maniGltfNodeBoneName(const maniGltf* gltf, uint32_t node, uint32_t* lengthOut);
```
Returns the bone name a node has in every skeleton, skin and clip built from the document (mani-0009): its name, or, when it has none or an earlier node has the same, that name followed by `#` and the node index.  @param gltf       The document. @param node       The node. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, UTF-8 and not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfRenamedCount(const maniGltf* gltf);
```
Returns the number of nodes whose bone name is not their name as given: unnamed, or named as an earlier node. Their names follow the node index, so a host binding by name across exports can warn or keep its own map.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfRenamedNode(const maniGltf* gltf, uint32_t index);
```
Returns a renamed node, in node order.  @param gltf   The document. @param index  Below maniGltfRenamedCount. @return The node, or UINT32_MAX past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniGltfAnimationCount(const maniGltf* gltf);
```
Returns the number of animations.  @param gltf  The document. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniGltfAnimationName(const maniGltf* gltf, uint32_t animation, uint32_t* lengthOut);
```
Returns an animation's name, UTF-8 and not terminated; empty for an animation without one.  @param gltf       The document. @param animation  The animation. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, or NULL past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfBuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t animation, const maniSkeleton* skeleton, const maniGltfOptions* options, float sampleRate, maniLoopPolicy loop, uint32_t* boundOut);
```
Builds one animation as a clip of an opened skeleton (mani-0010): translation, rotation and scale channels on its bones (found by name), morph weights of its bones and of meshes it skins, and `KHR_animation_pointer` channels to anything but nodes, each as float tracks; the basis change applied and, with placement folded, the nodes above the root composed in. The duration is the animation's latest key over every channel.  @param builder     The builder. @param gltf        The document, its buffers supplied. @param animation   The animation. @param skeleton    An opened skeleton. @param options     The options the skeleton was built with. @param sampleRate  Samples per second; 0 for 30. @param loop        The clip's loop policy. @param boundOut    Receives how many of the animation's channels the clip holds. May be NULL. @return `mani_success`; `mani_errorInvalid` for a NULL argument, an animation past the count or options out of range; `mani_errorFormat` with a reason (Data with the channel as item for keys the clip builder refuses); `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniGltfReadSprings(maniGltf* gltf, const maniSkeleton* skeleton, const maniGltfOptions* options, maniSpringSetSource* setOut);
```
Reads the document's `VRMC_springBone` extension (version 1.0) as spring bone sources on an opened skeleton (mani-0011): colliders, collider groups and springs, nodes found by bone name, offsets, tails and gravity directions in the output's basis. A document without the extension gives an empty set. The arrays are the document's until the next call or maniCloseGltf.  @param gltf      The document. @param skeleton  An opened skeleton holding the springs' nodes. @param options   The options the skeleton was built with. @param setOut    Receives the sources. @return `mani_success`; `mani_errorInvalid` for a NULL argument or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniOpenFbx(maniBuilder* builder, const void* bytes, size_t size, maniFbx** fbxOut);
```
Opens an FBX document (mani-0012) from the bytes of a file, binary (told apart by its magic, either byte order) or ASCII, versions 3000 to 7700. Every record, token and length is checked against the bytes; no array is decoded and no number converted until asked for.  @param builder  The builder whose allocator the document uses; it must be closed before the builder is destroyed. @param bytes    The file; kept alive and unchanged while the document is open. @param size     Its size in bytes. @param fbxOut   Receives the document. @return `mani_success`; `mani_errorInvalid` for a NULL argument; `mani_errorFormat` with a reason (Container, Version, Syntax) and the offset of the first byte found wrong; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
void maniCloseFbx(maniFbx* fbx);
```
Closes an FBX document and frees its memory. NULL is allowed.  @param fbx  The document. @par Thread safety Safe from any thread; the document is used by one thread at a time.

```c
uint32_t maniFbxVersion(const maniFbx* fbx);
```
Returns the document's version (7400 for FBX 7.4), 0 for NULL.  @param fbx  The document. @return The version. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxModelCount(const maniFbx* fbx);
```
Returns the number of models (the objects of kind Model, in the order of the file's `Objects`), 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
const char* maniFbxModelBoneName(const maniFbx* fbx, uint32_t model, uint32_t* lengthOut);
```
Returns the bone name a model has in every skeleton, skin and clip built from the document (mani-0014): its name without its class, or, when it has none or an earlier model has the same, that name followed by `#` and the model index.  @param fbx        The document. @param model      The model. @param lengthOut  Receives the length in bytes, 0 past the count. May be NULL. @return The name, UTF-8 and not terminated, or NULL past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxRenamedCount(const maniFbx* fbx);
```
Returns the number of models whose bone names differ from their names, 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxRenamedModel(const maniFbx* fbx, uint32_t index);
```
Returns a renamed model, in model order.  @param fbx    The document. @param index  Below maniFbxRenamedCount. @return The model, or UINT32_MAX past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkeletonCount(const maniFbx* fbx);
```
Returns the number of skeletons (mani-0014): skins' linked models and limb chains, joined where they share a model, numbered by their lowest skin, then chain; 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkinCount(const maniFbx* fbx);
```
Returns the number of skins (deformers of subclass Skin, in object order), 0 for NULL.  @param fbx  The document. @return The count. @par Thread safety Safe from any thread.

```c
uint32_t maniFbxSkinSkeleton(const maniFbx* fbx, uint32_t skin);
```
Returns the skeleton a skin is part of.  @param fbx   The document. @param skin  The skin. @return The skeleton, or UINT32_MAX for a skin past the count or one linking no model. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniFbxReadSettings(const maniFbx* fbx, maniFbxSettings* settingsOut);
```
Reads the scene's settings, FBX's defaults where the file is silent: X right, Y up, Z front, a centimetre a unit, 30 frames a second.  @param fbx          The document. @param settingsOut  Receives the settings. @return `mani_success`; `mani_errorInvalid` for a NULL argument. @par Thread safety Safe from any thread.

```c
static inline maniFbxOptions maniDefaultFbxOptions(void);
```

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildSkeleton(maniBuilder* builder, const maniFbx* fbx, uint32_t skeleton, const maniFbxOptions* options);
```
Builds a skeleton of the document into the builder (mani-0014): its skins' linked models and limb chains, the models between and their common root, named by their bone names, with helper bones (`name#unscaled`) where FBX's scale inheritance needs them.  @param builder   The builder. @param fbx       The document. @param skeleton  The skeleton, below maniFbxSkeletonCount. @param options   The options. @return `mani_success`; `mani_errorInvalid` for a NULL argument, a skeleton past the count or options out of range; `mani_errorFormat` with a reason; `mani_errorCapacity` when the allocator fails. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxBuildModelSkeleton( maniBuilder* builder, const maniFbx* fbx, uint32_t model, const maniFbxOptions* options);
```
Builds a model's subtree as a skeleton, for rigid hierarchies.  @param builder  The builder. @param fbx      The document. @param model    The root model, below maniFbxModelCount. @param options  The options. @return As maniFbxBuildSkeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxSkeletonPlacement(const maniFbx* fbx, uint32_t skeleton, const maniFbxOptions* options, maniAffine* placementOut);
```
Returns where a skeleton's root hangs: the world of the model above it (its unscaled world when the root does not inherit its scale normally), the options' axes and unit applied; identity when the root is at the top or the options fold the placement.  @param fbx           The document. @param skeleton      The skeleton. @param options       The options. @param placementOut  Receives the placement. @return `mani_success`; `mani_errorInvalid` as maniFbxBuildSkeleton; `mani_errorFormat` for a model above the root whose properties are refused. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniImportResult maniFbxModelPlacement(const maniFbx* fbx, uint32_t model, const maniFbxOptions* options, maniAffine* placementOut);
```
Returns where a model skeleton's root hangs, as maniFbxSkeletonPlacement.  @param fbx           The document. @param model         The root model. @param options       The options. @param placementOut  Receives the placement. @return As maniFbxSkeletonPlacement. @par Thread safety Safe from any thread.

## `pose.h`

Poses (mani-0004): a skeleton's local transforms and the model matrices composed from them, in memory the caller owns. A pose holds no pointers: copying its bytes copies the pose, and it may move between threads. A pose is used by one thread at a time.

```c
size_t maniPoseSize(const maniSkeleton* skeleton);
```
Returns the bytes a pose of the skeleton needs.  @param skeleton  An opened skeleton. @return The size in bytes. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniInitPose(const maniSkeleton* skeleton, void* memory, size_t size, maniPose** poseOut);
```
Makes a pose of the skeleton in caller memory, every bone at its rest, the model matrices composed.  @param skeleton  An opened skeleton. @param memory    At least maniPoseSize bytes, MANI_POSE_ALIGNMENT aligned; the pose lives in it. @param size      The memory's size in bytes. @param poseOut   Receives the pose. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned memory; `mani_errorCapacity` when size is too small. @par Thread safety Safe from any thread.

```c
uint32_t maniPoseBoneCount(const maniPose* pose);
```
Returns the number of bones of the pose's skeleton.  @param pose  The pose. @return The bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
maniTransform maniPoseLocal(const maniPose* pose, uint32_t bone);
```
Returns a bone's local transform.  @param pose  The pose. @param bone  The bone. @return The transform, or the identity for a bone past the count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
void maniPoseSetLocal(maniPose* pose, uint32_t bone, maniTransform transform);
```
Sets a bone's local transform and marks the model matrices stale. A bone past the count is ignored.  @param pose       The pose. @param bone       The bone. @param transform  The transform; its rotation is used as given and should be unit. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseReadLocals(const maniPose* pose, uint32_t first, uint32_t count, maniTransform* transforms);
```
Reads the local transforms of bones first to first + count - 1.  @param pose        The pose. @param first       The first bone. @param count       How many bones; the range must lie within the bone count. @param transforms  Receives count transforms. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a range past the bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseWriteLocals(maniPose* pose, uint32_t first, uint32_t count, const maniTransform* transforms);
```
Writes the local transforms of bones first to first + count - 1 and marks the model matrices stale.  @param pose        The pose. @param first       The first bone. @param count       How many bones; the range must lie within the bone count. @param transforms  The count transforms. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a range past the bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseSetRest(maniPose* pose, const maniSkeleton* skeleton);
```
Sets every bone to the skeleton's rest and composes the model.  @param pose      The pose. @param skeleton  The pose's skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a skeleton of another identity. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniComputeModel(maniPose* pose, const maniSkeleton* skeleton);
```
Composes every bone's model matrix: its parent's model matrix times its local transform's matrix (scale, then rotation, then translation), a root's being its local's.  @param pose      The pose. @param skeleton  The pose's skeleton. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a skeleton of another identity. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
bool maniPoseModelCurrent(const maniPose* pose);
```
Returns whether the model matrices match the local transforms: true after maniComputeModel until a local is set.  @param pose  The pose. @return True when current. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
maniAffine maniPoseModel(const maniPose* pose, uint32_t bone);
```
Returns a bone's model matrix as last composed.  @param pose  The pose. @param bone  The bone. @return The matrix, or the identity for a bone past the count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniPoseReadModels(const maniPose* pose, uint32_t first, uint32_t count, maniAffine* matrices);
```
Reads the model matrices of bones first to first + count - 1, as last composed.  @param pose      The pose. @param first     The first bone. @param count     How many bones; the range must lie within the bone count. @param matrices  Receives count matrices. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a range past the bone count. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniDecomposeAffine(const maniAffine* matrix, maniTransform* transformOut);
```
Splits an affine matrix into translation, rotation and scale: the fourth column, the rotation of the polar decomposition (the rotation nearest the matrix, exact up to rounding when the matrix has no shear) as a quaternion with w >= 0, and the scale along the rotated axes. A mirror becomes a negative x scale. A matrix with a zero or flat axis still splits, its rotation built from the axes that are left.  @param matrix        The matrix. @param transformOut  Receives the transform. @return `mani_success`; `mani_errorInvalid` for a NULL argument or a component that is not finite. @par Thread safety Safe from any thread.

```c
maniTransform maniPoseModelTransform(const maniPose* pose, uint32_t bone);
```
Returns a bone's model matrix, as last composed, split as maniDecomposeAffine splits it.  @param pose  The pose. @param bone  The bone. @return The transform, or the identity for a bone past the count or a matrix that is not finite. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

## `skeleton.h`

Skeletons (mani-0003): bones in depth-first order, each with a parent, a rest transform and a unique name, read in place from a skeleton file's bytes. Opening a file checks all of it once; the view then reads it without checks and allocates nothing.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenSkeleton(const void* bytes, size_t size, maniSkeleton* skeletonOut);
```
Opens a skeleton file in place: checks the container and every section (counts, depth-first order, rests finite with unit rotations, names valid UTF-8, unique and matching their hashes, the identity) and fills the view.  @param bytes       The file, 16-byte aligned. @param size        Its size in bytes; bytes past the file are ignored. @param skeletonOut Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonBoneCount(const maniSkeleton* skeleton);
```
Returns the number of bones, 1 to MANI_MAX_BONES.  @param skeleton  An opened skeleton. @return The bone count. @par Thread safety Safe from any thread.

```c
uint64_t maniSkeletonIdentity(const maniSkeleton* skeleton);
```
Returns the skeleton's identity: a hash of its bones' order, parents, rests and names, which files built against it record.  @param skeleton  An opened skeleton. @return The identity. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonParent(const maniSkeleton* skeleton, uint32_t bone);
```
Returns a bone's parent, always a lower index.  @param skeleton  An opened skeleton. @param bone      The bone. @return The parent, or MANI_NO_BONE for a root or a bone past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonSubtreeEnd(const maniSkeleton* skeleton, uint32_t bone);
```
Returns the end of a bone's subtree: the bone and its descendants are the indices from bone up to, not including, the end.  @param skeleton  An opened skeleton. @param bone      The bone. @return The end, or bone itself for a bone past the count (an empty range). @par Thread safety Safe from any thread.

```c
maniTransform maniSkeletonRest(const maniSkeleton* skeleton, uint32_t bone);
```
Returns a bone's rest transform, relative to its parent.  @param skeleton  An opened skeleton. @param bone      The bone. @return The rest, or the identity transform for a bone past the count. @par Thread safety Safe from any thread.

```c
const char* maniSkeletonBoneName(const maniSkeleton* skeleton, uint32_t bone, uint32_t* lengthOut);
```
Returns a bone's name, UTF-8 and not terminated.  @param skeleton  An opened skeleton. @param bone      The bone. @param lengthOut Receives the name's length in bytes, 0 for a bone past the count. May be NULL. @return The name's bytes, inside the file, or NULL for a bone past the count. @par Thread safety Safe from any thread.

```c
uint32_t maniSkeletonFindBone(const maniSkeleton* skeleton, const char* name, size_t length);
```
Finds a bone by its name.  @param skeleton  An opened skeleton. @param name      The name's UTF-8 bytes; may be NULL when length is 0. @param length    The name's length in bytes. @return The bone, or MANI_NO_BONE when no bone has that name. @par Thread safety Safe from any thread.

## `skin.h`

Skins (mani-0005): the bones a mesh is bound to, in the mesh's order, each with its inverse bind matrix, read in place from a skin file and bound to one skeleton by its identity; and the palettes a renderer skins with.

```c
MANI_NODISCARD MANI_API maniFileResult maniOpenSkin(const void* bytes, size_t size, maniSkin* skinOut);
```
Opens a skin file in place: checks the container, the sizes, every joint against the skeleton's bone count the file records, and every inverse bind finite.  @param bytes    The file, 16-byte aligned. @param size     Its size in bytes; bytes past the file are ignored. @param skinOut  Receives the view on success. @return `mani_success`; `mani_errorInvalid` for a NULL argument or misaligned bytes; `mani_errorFormat` with the offset of the first byte found wrong; `mani_errorVersion` for a newer format. @par Thread safety Safe from any thread.

```c
uint32_t maniSkinJointCount(const maniSkin* skin);
```
Returns the number of joints, at least 1.  @param skin  An opened skin. @return The joint count. @par Thread safety Safe from any thread.

```c
uint64_t maniSkinSkeletonIdentity(const maniSkin* skin);
```
Returns the identity of the skeleton the skin is bound to.  @param skin  An opened skin. @return The skeleton's identity. @par Thread safety Safe from any thread.

```c
uint32_t maniSkinJoint(const maniSkin* skin, uint32_t joint);
```
Returns the skeleton bone of a joint.  @param skin   An opened skin. @param joint  The joint. @return The bone, or MANI_NO_BONE for a joint past the count. @par Thread safety Safe from any thread.

```c
maniAffine maniSkinInverseBind(const maniSkin* skin, uint32_t joint);
```
Returns a joint's inverse bind matrix.  @param skin   An opened skin. @param joint  The joint. @return The matrix, or the identity for a joint past the count. @par Thread safety Safe from any thread.

```c
MANI_NODISCARD MANI_API maniResult maniComputePalette(const maniPose* pose, const maniSkeleton* skeleton, const maniSkin* skin, maniAffine* palette, uint32_t capacity, uint32_t* countOut);
```
Computes a skin's palette: entry j is the model matrix of joint j's bone times the joint's inverse bind matrix.  @param pose      A pose of the skeleton with its model composed. @param skeleton  The skeleton the skin is bound to. @param skin      The skin. @param palette   Receives the entries; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the joint count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the joint count, with the count written and nothing else; `mani_errorInvalid` for a NULL argument, a skeleton that is not the skin's or the pose's, or a pose whose model is stale. @par Thread safety Safe from any thread; the pose is used by one thread at a time.

---

98 functions across 7 headers.
