# Importing

The offline side (built with `MAUL_ANIM_OFFLINE`, on by default) imports glTF 2.0 and FBX files into the library's files: skeletons, skins, influences, morph targets, clips, spring sets and ragdoll profiles. A program usually imports in its asset pipeline, compresses the clips, and ships the files.

## Contents

- [glTF](#gltf)
- [FBX](#fbx)
- [Refusals](#refusals)
- [A memory budget for untrusted files](#a-memory-budget-for-untrusted-files)
- [Compression](#compression)
- [As text](#as-text)

## glTF

A document opens from JSON or GLB bytes the program keeps alive; external buffers are supplied by the program (`maniGltf_GetBufferUri`, `maniGltf_SupplyBuffer`), since the library opens no files. Meshopt-compressed files, as gltfpack writes them, and Draco-compressed meshes are read.

```c
maniGltf* gltf = NULL;
maniImportResult read = maniOpenGltf(builder, bytes, size, &gltf);   // bytes kept alive
maniGltfOptions options = maniDefaultGltfOptions();
maniImportResult skeletonImported = maniBuildGltfSkeleton(builder, gltf, 0, &options);
size_t skeletonSize = 0;
maniResult skeletonCopied = maniBuilder_CopyBuilt(builder, skeletonFile, skeletonCapacity, &skeletonSize);
maniSkeleton skeleton;
maniFileResult skeletonOpened = maniOpenSkeleton(skeletonFile, skeletonSize, &skeleton);

maniImportClipOptions clipOptions = maniDefaultImportClipOptions();   // 30 samples a second
maniImportResult clipImported =
    maniBuildGltfClip(builder, gltf, 0, &skeleton, &options, &clipOptions, NULL);
size_t clipSize = 0;
maniResult clipCopied = maniBuilder_CopyBuilt(builder, clipFile, clipCapacity, &clipSize);
maniClip clip;
maniFileResult clipOpened = maniOpenClip(clipFile, clipSize, &clip);
maniCloseGltf(gltf);
```

glTF has no skeleton object: skins whose joints share nodes form one skeleton, numbered by their lowest skin (`maniGltf_GetSkeletonCount`). A node's bone name is its name, or its name followed by `#` and its index when it has none or repeats an earlier one; `maniGltf_GetRenamedNode` lists those, so a program binding by name across exports can warn. The options choose the target axes (a signed permutation, for a Z-up engine) and whether the transform above the root is folded into it or reported (`maniGltf_GetSkeletonPlacement`).

Clip options (`maniImportClipOptions`) set the sample rate, loop policy, additive basis and root motion source per clip. Morph weights become float tracks named node `/` target; `KHR_animation_pointer` channels to other properties become float tracks named by the pointer.

## FBX

`maniOpenFbx` reads binary and ASCII FBX files from version 3000 to 7700, and the same calls build skeletons, skins, influences, morph targets and clips (`maniBuildFbxSkeleton`, `maniBuildFbxClip` and the rest). FBX's pivots, pre- and post-rotations and scale inheritance modes are evaluated as ufbx evaluates them and land on the library's one transform rule; where a file's scale inheritance cannot be expressed that way, a helper bone named `<parent>#unscaled` is added. Each animation stack is a clip; blend shape weights become float tracks named `mesh/shape`.

## Refusals

A refused file says why: the result's reason (`maniImportReasonJson`, `...Layout`, `...Data`, `...Unsupported` and the rest), the byte offset in the file or buffer, and an item. Values out of range, broken layouts and features the importer cannot represent faithfully (a sheared rest, an additive FBX cluster) are refused rather than approximated.

## A memory budget for untrusted files

An importer never takes more memory than a file's declared counts justify: compressed data may expand at most 1,024 times, and a Draco stream is bounded by its primitive's accessors. The declared counts, though, are the file's own. An accessor with no buffer view stands for zeros and costs nothing in the file, at most 16,777,216 elements each, and one primitive may declare many, so a two-kilobyte file can ask for gigabytes.

The builder's allocator is where a program sets its limit: the importer asks it for every block with its size, and a block it refuses fails the call with `mani_errorCapacity` and keeps nothing. The C library's allocator is not a limit everywhere: Linux overcommits by default, so `malloc` may grant a block larger than the machine has and the process dies when the block is first touched. A program that imports files it does not trust gives its builder a budget:

```c
typedef struct Budget
{
    size_t used;
    size_t limit;
} Budget;

static void* BudgetAlloc(size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    if (alignment > alignof(max_align_t) || size > budget->limit - budget->used)
    {
        return NULL;
    }
    void* memory = malloc(size);
    budget->used += memory != NULL ? size : 0;
    return memory;
}

static void BudgetFree(void* memory, size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    (void)alignment;
    if (memory != NULL)
    {
        budget->used -= size;
        free(memory);
    }
}
```

```c
Budget budget = {0, (size_t)256 << 20};   // 256 MiB for one file's import
maniBuilderDef guarded = maniDefaultBuilderDef();
guarded.allocator = (maniAllocator){BudgetAlloc, BudgetFree, &budget};
maniBuilder* careful = NULL;
maniResult created = maniCreateBuilder(&guarded, &careful);
```

The allocator's functions must not call the builder they serve; such a call is refused and counted (`maniBuilder_GetMisuse`).

## Compression

A full-precision clip compresses within an error the program chooses, measured on a shell of points around each bone:

```c
maniCompressOptions tolerance = maniDefaultCompressOptions();   // 0.1 mm at 3 cm
maniCompressReport report;
maniResult compressed = maniCompressClip(builder, &clip, &skeleton, &tolerance, &report);
size_t smallSize = 0;
maniResult smallCopied = maniBuilder_CopyBuilt(builder, smallFile, smallCapacity, &smallSize);
maniClip small;
maniFileResult smallOpened = maniOpenClip(smallFile, smallSize, &small);
// report.shellError is the largest error found at any sample.
```

A compressed clip opens and plays through the same calls as a full one, with the same cursors. Precision and shell distance can be set per bone, for a character whose fingers matter less than its hands.

Keep the full clip or its source: a library that changes the clip format refuses older clips with `mani_errorVersion`, and they are compressed again from it.

## As text

Graphs and the retargeting and mirror maps have a JSON form, read and written (`maniBuildGraphJson`, `maniWriteRetargetMapJson`, `maniBuildRetargetMapJson`, `maniWriteMirrorMapJson`, `maniBuildMirrorMapJson`), so a tool can edit them as text and a repository can review them as diffs.
