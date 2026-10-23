# FBX import

FBX has no published specification and its SDK is closed. The binary encoding is described in the Blender Foundation's public notes, and ufbx, an open reader, shows what files in the wild hold. The importer follows ufbx's reading wherever FBX leaves a choice, so ufbx is its reference: a file both read gives the same skeletons, skins, clips and weights. It produces the same files the glTF importer does, through the same builders.

## One tree from either form

`maniOpenFbx` ([`src/fbx.c`](../../src/fbx.c), the tree in [`src/fbx_document.h`](../../src/fbx_document.h)) reads either form into one tree: a flat array of nodes (name, a run of properties, first child, next sibling) and one of properties (type, count, where the payload lies). Anything without the binary magic is read as text.

- **Binary** ([`src/fbx_binary.c`](../../src/fbx_binary.c)): versions 3000 to 7700, 13-byte record headers before 7500 and 25 from it, both byte orders. Every record's end must lie within its parent's and match where its contents end; each child list ends in a null record; at most 64 levels. A record whose end is 0 holds no children, as ufbx reads what Maya 2019 writes.
- **ASCII** ([`src/fbx_ascii.c`](../../src/fbx_ascii.c)): the same tree from text, numbers kept as text and converted exactly when read ([gltf-import.md](gltf-import.md#json-and-numbers)), infinities and NaNs as ufbx spells them (and MSVC's `1.#QNAN`, which ufbx misses) refused where read.

Values are decoded on request, so arrays the import never needs cost nothing. A zlib array is inflated by the family's inflate ([`src/inflate.c`](../../src/inflate.c)) into exactly its decoded size; a size past 1,032 times its compressed bytes (DEFLATE's most) is refused before anything is sized by it. Strings are made valid UTF-8 as ufbx makes them, so names and comparisons of names match ufbx's.

## Objects and settings

Objects are found by id through a sorted index, connections indexed from both ends in file order, and a property an object lacks comes from the `Definitions` template of its kind as ufbx chooses it ([`src/fbx_object.c`](../../src/fbx_object.c)). Version 6 files name objects instead of numbering them and are read into the same graph; files before 6.0 are read too, their top-level models and takes turned into the same objects, connections and stacks. `GlobalSettings` gives the axes, the unit and the frame rate as an exact rational (NTSC rates over 1001), so resampled NTSC animation lands on whole frames.

## Transforms

An FBX model's local transform is a chain of pivots, offsets and pre- and post-rotations around its translation, rotation and scale, with six Euler orders and three ways to inherit a parent's scale. The runtime has one rule: a bone's model matrix is its parent's times its local TRS ([clips-and-compression.md](clips-and-compression.md#transforms-and-skeletons)).

The chain `T Roff Rp Rpre R Rpost^-1 Rp^-1 Soff Sp S Sp^-1` is exactly one TRS (every factor but the rotations and S is a translation), evaluated in binary64 as ufbx evaluates it ([`src/fbx_transform.c`](../../src/fbx_transform.c)). Worlds follow ufbx's evaluation of each inherit type; a bone's local is its parent bone's world inverted times its own, decomposed. Where that local does not decompose exactly (a non-uniform scale under a child that does not inherit it normally), a helper bone `<parent>#unscaled` holding the inverse of the inherited scale is added, at rest or at any sample of a stack: so FBX's whole transform model lands on the one rule exactly, and helper bones appear only in files whose scale inheritance needs them.

Skeletons are skins' linked models and limb chains with the models between, merged when they share a model; placement, target axes and units are reported or applied, as for glTF.

## Skins and blend shapes

Joints are a skin's clusters in connection order; inverse binds are the cluster's `Transform` times the mesh's geometric transform, with ufbx's fallbacks (`TransformLink`, a bind pose, the world at rest) where exporters omit them. Additive clusters are refused rather than bound wrongly. Vertex weights are read per control point as ufbx reads them, into the same influence format as glTF's ([skinning.md](skinning.md)).

Blend shape channels become one float track per shape holding its effective weight at every sample, as ufbx computes it from the channel's percentage and the shapes' full weights, so in-betweens resolve at import and the runtime has one weighting model.

## Clips

Each `AnimationStack` is a clip, built for one opened skeleton ([`src/fbx_clip.c`](../../src/fbx_clip.c)). Curves are decoded and evaluated as ufbx does: Bézier segments solved for time by Newton's method, constant and linear segments, extrapolation (constant, along the tangent, repeated plain, relative or mirrored) in whole ticks. Layers combine in connection order with their weights and blend modes, rotations through quaternions. Every animated property feeds the locals and worlds at each sample of the clip's grid (the scene's frame rate up to 120 a second, else 30, in whole ticks so keys on frames land on samples), and each bone's local is formed exactly as its rest was, helper bones included. Other animated numbers become float tracks named `bone/property`, as glTF's pointer tracks are.

## The ufbx differential

`tools/fbx_differential.c`, built only with `MAUL_ANIM_DIFFERENTIAL` against ufbx at a pinned commit and never linked into the library, reads every file of ufbx's corpus (690 files from many tools, versions 2000 to 8000, many made to break readers) both ways and compares whether each opens, the frame rate, every skeleton's rest worlds, every skin's joints and inverse binds, and every clip's bone worlds and float tracks at five sample times, within 1e-4 of their size. Known deviations are listed with their reasons in `tools/fbx-deviations.txt`; anything else that disagrees is a defect. The corpus stays outside the repository and the job runs on a schedule.

The fixtures under `test/data/fbx` are scenes written for this library, kept as ASCII and converted to every binary form (both header sizes, deflated arrays, big-endian) by `tools/fbx_binary.py`, which shares no code with the library. Their expected values are ufbx's, written by the differential into `test/fbx_fixture_expect.h`; the scheduled job writes the header again and holds it against the committed one.

## Traps

- A choice ufbx makes and the importer does not follow must be listed in `tools/fbx-deviations.txt` with its reason, or the differential fails.
- Helper bones depend on the animation as well as the rest: a file whose clips need one the skeleton lacks refuses that clip (`maniImportReasonMatrix`) rather than playing it wrong.
- Big-endian FBX files are read; big-endian hosts are not supported.
