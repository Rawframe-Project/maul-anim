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
MANI_NODISCARD MANI_API maniResult maniCopyBuilt(maniBuilder* builder, void* bufferOut, size_t capacity, size_t* sizeOut);
```
Copies the last built file into caller memory.  @param builder   The builder. @param bufferOut Receives the bytes: at least capacity bytes; may be NULL when capacity is 0. Open the copy from a 16-byte aligned address. @param capacity  The buffer's size in bytes. @param sizeOut   Receives the file's size in bytes. May be NULL. @return `mani_success`; `mani_errorCapacity` when the buffer is smaller than the file, with the size still written; `mani_errorInvalid` for a NULL builder, a NULL buffer with a positive capacity, or a builder holding no file. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

```c
MANI_NODISCARD MANI_API maniResult maniCopySkeletonOrder(maniBuilder* builder, uint32_t* boneOut, uint32_t capacity, uint32_t* countOut);
```
Copies where each source bone of the last built skeleton went: the skeleton's index of source bone i at boneOut[i].  @param builder   The builder. @param boneOut   Receives up to capacity indices; may be NULL when capacity is 0. @param capacity  The array's length. @param countOut  Receives the source's bone count. May be NULL. @return `mani_success`; `mani_errorCapacity` when the array is shorter than the count, with the count still written and the array filled as far as it goes; `mani_errorInvalid` for a NULL builder, a NULL array with a positive capacity, or a builder holding no skeleton. @par Thread safety Safe from any thread; the builder is used by one thread at a time.

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

37 functions across 5 headers.
