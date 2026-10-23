# glTF import

The glTF 2.0 importer turns a document (JSON or GLB) into the library's files: skeletons, skins, influences, morph targets, clips, spring sets and ragdoll profiles. It runs offline with a builder's allocator, does no file IO, takes no callbacks, and treats every byte as hostile. This page explains its reading choices; the user side is in [importing.md](../user/importing.md).

## JSON and numbers

The reader ([`src/json.c`](../../src/json.c)) follows RFC 8259 strictly (no comments, no trailing commas, no leading zeros, UTF-8 throughout, surrogates paired), skips a leading byte order mark as glTF allows, and takes an object's last member of a repeated name, as glTF asks. It runs twice over the text, first checking and counting, then filling one flat array of values allocated once; it is iterative and nests at most 128 deep. Strings and numbers stay spans of the text, decoded when read.

Numbers convert when read, to what the property wants, with integer arithmetic only ([`src/decimal.c`](../../src/decimal.c)): `strtof` depends on the locale's decimal point, has rounded wrongly in older C libraries, and a double rounded again to float differs from the correctly rounded float in rare halfway cases. The conversion is Eisel and Lemire's method first (the first 19 significant digits times a 128-bit power of five from a generated table, `tools/gen_decimal_powers.py`), and the multiprecision decimal method for what it leaves open, rounding once at the target width. A value past the format's largest is refused, never infinite. Both methods agree on 1.86 million decimals and every midpoint between adjacent values; the tests hold them to an exact rational oracle (`tools/gen_number_cases.py`).

## The document

`maniOpenGltf` ([`src/gltf.c`](../../src/gltf.c), with [`src/gltf_layout.c`](../../src/gltf_layout.c) for views and accessors) tells GLB from JSON by GLB's magic and checks the container, `asset.version` (2.x) and `extensionsRequired` (only extensions the importer reads or that change nothing it reads). It checks every buffer view and accessor against the buffers' declared lengths before any data is read: offsets, strides, alignment, matrix column padding, sparse indices. Data errors (a NaN, indices out of order) are found where data is read, with the offset into the buffer. Accessor `min` and `max` are never trusted.

Buffers: the GLB's BIN chunk, `data:` URIs decoded once, or external buffers whose URI the host resolves (`maniGltfBufferUri`) and supplies (`maniGltfSupplyBuffer`). The library never opens a file.

One deliberate leniency: a view of other than vertex data (inverse binds, animation samplers, sparse parts) may give a stride equal to its element size, read as none. The specification forbids a stride there and glTF-Validator reports it, but the specification's read formula gives it a meaning, the bytes read are those no stride reads, and exporters (Sketchfab's among them) write it.

## Compression

`EXT_meshopt_compression` and `KHR_meshopt_compression` are decoded by the library's own decoder from the specifications ([`src/gltf_meshopt.c`](../../src/gltf_meshopt.c)): every mode and filter, stricter than meshoptimizer's decoder where the specification calls a stream invalid. Filters are the specification's binary32 formulas in a fixed order, so the bits are the same everywhere. A view may not claim more than 1,024 times its stream, the most a valid stream decodes into, so no file makes the document allocate more.

`KHR_draco_mesh_compression` primitives are decoded by the library's own Draco decoder (bitstream 2.2, sequential and Edgebreaker, every prediction Google's encoder writes; [`src/gltf_draco.c`](../../src/gltf_draco.c) and its modules), with Google's decoder as the reference where the specification is silent. Before the decoder sizes anything, the stream's counts are bounded by the primitive's accessors, so a stream makes the importer take no more than its primitive declares.

## Skeletons and skins

glTF has no skeleton object: a skin lists joint nodes. A skin's bones are its joints' deepest common ancestor and every node on the paths to each joint, so a skeleton has one root and nodes between joints are bones without palette entries. Skins whose bones share a node form one skeleton (a body and a head over one rig). The hierarchy is checked at open (no node with two parents, no cycle), and depths and common ancestors come from walks that stop at known nodes and a binary lifting table, so a hostile document cannot make a walk quadratic.

Names are fixed per document at open: a node's name, or the name followed by `#` and its index when it has none or repeats an earlier one (`maniGltfNodeBoneName`, `maniGltfRenamedNode`). A node has one name in every skeleton, skin and clip built from the document, so skins and clips find their bones by name whatever bone set a skeleton has.

Rests are a node's TRS or its matrix decomposed, refused when the matrix is projective or sheared (glTF forbids shear). The transform above the root is the placement, reported (`maniGltfSkeletonPlacement`) or folded into the root on request; a folded non-uniform scale above a rotated root would shear, and is refused rather than dropped. A basis change is a signed permutation of axes applied exactly as `B G B^-1`.

## Animations

`maniGltfBuildClip` builds one animation for one opened skeleton: TRS channels of its bones, morph weights of its meshes as float tracks named node `/` target, and `KHR_animation_pointer` channels to other properties as float tracks named by the pointer. Every sampler and channel is checked at open. With the placement folded, animation above the root is composed into the root track at each sample, and a sample that would shear refuses the clip rather than change the motion unannounced. The duration is the latest key of every channel, so one animation's clips for several skeletons share it.

Clips are built through the clip builder, which evaluates glTF's interpolations exactly as its Appendix C defines them ([clips-and-compression.md](clips-and-compression.md#clips-on-one-grid)).

## Spring bones and ragdolls

`maniGltfReadSprings` reads `VRMC_springBone` 1.0 with its extended collider and limit extensions onto an opened skeleton, every value checked against the schema's ranges, offsets and directions moved into the output basis. VRM 0.x's `secondaryAnimation` is another format and is not read.

`maniGltfReadRagdoll` reads the three `OMI_physics_*` extensions: a body sits on its node's nearest bone, its shape is its one collider's, a joint joins a body to its nearest ancestor body. What does not map onto the profile (static bodies on bones, meshes as shapes, linear joint axes, asymmetric swings) is refused with `maniImportReasonUnsupported` at its value; inertia, triggers and materials are ignored, as engines derive or own them.

## Memory budget

No count a file declares is capped beyond the bounds above: an accessor without a view (zeros) costs its declared size, at most 2^24 elements each. A host reading files it does not trust gives the builder an allocator with a budget; a block it refuses fails the call with `mani_errorCapacity` and nothing is kept. The C library's allocator is no ceiling on Linux, which overcommits, so `test_guide_import` runs a hostile file under a memory limit in CI.
