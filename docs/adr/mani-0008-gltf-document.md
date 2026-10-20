# mani-0008. The glTF document: containers, buffers and accessors

Status: Accepted

## Context

A glTF asset is JSON alone or in a GLB container, with its binary data
in the GLB's BIN chunk, in `data:` URIs or in files beside it. The
library does no file IO and takes no callbacks, and every byte is
hostile until checked. Skeletons, skins and clips read their numbers
through accessors, whose layout rules (offsets, strides, alignment,
matrix column padding, normalized integers, sparse substitution) the
specification fixes exactly.

## Decision

- **Opening** (`maniOpenGltf`, with a builder's allocator) tells GLB
  from JSON by GLB's magic. GLB: version 2, a length within the bytes
  given (bytes past it ignored), every chunk 4-aligned and inside the
  length, the JSON chunk first, a BIN chunk second if present, other
  chunks after them skipped as the specification requires.
  `asset.version` must be 2.x and `minVersion`, if given, 2.0.
  `extensionsRequired` may name only extensions the importer reads or
  that change nothing it reads (meshes, materials, textures, lights);
  any other is refused with its name's offset.
- **Buffers:** the first buffer without a URI in a GLB is the BIN
  chunk, which must hold its `byteLength`. A `data:` URI (scheme and
  media type compared without case; the media type
  `application/octet-stream` or `application/gltf-buffer`, as the
  specification requires) is decoded once into the document, base64
  with its padding or percent-encoded. Any other URI, or none, makes an
  external buffer: `maniGltfBufferUri` gives the URI with its JSON
  escapes decoded and its percent escapes left for the host, which
  resolves it and passes the bytes to `maniGltfSupplyBuffer`. A
  resource may be longer than `byteLength`; only that much is read. A
  buffer marked a meshopt `fallback` is never read and not external,
  nor is one without a URI (not the BIN chunk) that only compressed
  views use: a placeholder for their decoded bytes.
- **Buffer views and accessors** are checked at open against the
  buffers' declared lengths: offsets, lengths and strides (4 to 252,
  multiples of 4), every accessor's span inside its view, offsets
  aligned to the component size, matrix columns of 1- and 2-byte
  components padded to 4 bytes, `normalized` refused on 32-bit types,
  sparse indices of unsigned types from views without a stride. A view
  of other than vertex data (sparse parts, inverse binds, animation
  samplers) may give a stride equal to its element size, read as none.
  This is a deliberate leniency, departing from the specification
  (which forbids a stride there) and from glTF-Validator (which reports
  `BUFFER_VIEW_INVALID_BYTE_STRIDE`,
  `SKIN_IBM_ACCESSOR_WITH_BYTESTRIDE` and
  `ANIMATION_SAMPLER_ACCESSOR_WITH_BYTESTRIDE` as errors): the
  specification's read formula gives such a stride its meaning, the
  bytes read are those no stride reads, the view's length is checked
  as for no stride, and exporters write it (Sketchfab's among them).
  Any other stride there is refused.
  Accessors read into binary32 with the specification's conversions
  for normalized integers, sparse replacements applied; indices not
  increasing, NaN and infinite floats refused where read, with the
  offset into the buffer. An accessor without a view has at most 2^24
  elements, since its zeros cost the file nothing.
- **Meshopt compression:** `EXT_meshopt_compression` and its Khronos
  successor `KHR_meshopt_compression` are read on buffer views, every
  mode (attributes of version 0, and of version 1 under KHR; triangles;
  indices) and filter (octahedral, quaternion, exponential, and KHR's
  color) by the library's own decoder, from the specifications. A
  view's object is checked at open by their rules: its stream inside
  its buffer and not in a fallback, the view's length its count times
  its stride, the stride and filter its mode allows, both extensions on
  one view or buffer refused, a fallback used only by views its own
  extension compresses, and a view no longer than 1,024 times its
  stream (the most a valid stream decodes into, so no file makes the
  document allocate more than that). Each stream is decoded when its
  buffer's bytes are present, at open or when supplied, into bytes the
  document owns that stand for the view; a stream the specification
  calls invalid (stricter than meshoptimizer's decoder: a FIFO entry
  never written, a malformed code table, a LEB128 value past 5 bytes)
  is refused as Data at its offset in its buffer, a supplied buffer
  then left unsupplied. Filters are the specification's binary32
  formulas in a fixed order, the same bits on every platform; those
  bits hold under the build's `-ffp-contract=off` and without fast math
  (configuration stops on fast math in the flags), and a build that
  changes either may round a filtered value otherwise.
- **Refusals** (`maniImportResult`): the reason, the first byte found
  wrong (in the bytes given, or in a buffer's bytes for data), and the
  item. A string value's offset is its first byte after the quote.

## Consequences

The host does all IO and owns every byte; the document keeps only what
it decoded and a flat array of the JSON's values. Every layout error is
found before any data is read, and any data error where it is read.
Accessor `min` and `max` are never trusted. A compressed view costs the
document its decoded size whether or not it is read, the size an
uncompressed file would hold; decoding on first read would spare that
memory but make every read path mutable. No count a file declares is
capped otherwise: an accessor without a view (zeros, or sparse values
over zeros) costs its declared size as a stored one does. The host's
allocator, asked for every block with its size, is the import's memory
budget; a block it refuses fails the call with `mani_errorCapacity`,
nothing kept.
