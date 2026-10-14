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
  resource may be longer than `byteLength`; only that much is read.
- **Buffer views and accessors** are checked at open against the
  buffers' declared lengths: offsets, lengths and strides (4 to 252,
  multiples of 4), every accessor's span inside its view, offsets
  aligned to the component size, matrix columns of 1- and 2-byte
  components padded to 4 bytes, `normalized` refused on 32-bit types,
  sparse indices of unsigned types from views without a stride.
  Accessors read into binary32 with the specification's conversions
  for normalized integers, sparse replacements applied; indices not
  increasing, NaN and infinite floats refused where read, with the
  offset into the buffer. An accessor without a view has at most 2^24
  elements, since its zeros cost the file nothing.
- **Refusals** (`maniImportResult`): the reason, the first byte found
  wrong (in the bytes given, or in a buffer's bytes for data), and the
  item. A string value's offset is its first byte after the quote.

## Consequences

The host does all IO and owns every byte; the document keeps only what
it decoded and a flat array of the JSON's values. Every layout error is
found before any data is read, and any data error where it is read.
Accessor `min` and `max` are never trusted. Extensions that compress or
relocate buffer data (`EXT_meshopt_compression`) are refused until the
importer reads them.
