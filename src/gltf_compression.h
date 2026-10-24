// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF's meshopt buffer compression: `EXT_meshopt_compression`
// and `KHR_meshopt_compression` on buffer views and buffers. A view's
// stream is checked at open and decoded when its buffer's bytes are
// present, into bytes the document owns that then stand for the view.

#ifndef MAUL_ANIM_SRC_GLTF_COMPRESSION_H
#define MAUL_ANIM_SRC_GLTF_COMPRESSION_H

#include "gltf_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

// Reads a buffer's `fallback` mark, if any, into its `fallback`.
maniImportResult maniGltfReadFallback(maniGltf* gltf, uint32_t value, uint32_t buffer);

// Reads a buffer view's extension object, if any, into the view (whose
// own members are read): every rule of the specification, a stream's
// range inside its buffer, a fallback buffer used only by views its
// extension compresses. `source` stays MANI_JSON_NONE without one.
maniImportResult maniGltfReadCompression(maniGltf* gltf, uint32_t value, uint32_t view);

// After every view is read: makes a buffer without a URI that only
// compressed views use a placeholder (neither external nor read), and
// decodes the streams whose buffers the document holds.
maniImportResult maniGltfFinishCompression(maniGltf* gltf);

// Decodes every stream in the buffer, whose bytes are now present; on a
// refusal (Data at the stream's offset in the buffer, item the view) the
// views it decoded are released again.
maniImportResult maniGltfDecodeBuffer(maniGltf* gltf, uint32_t buffer);

// Frees the decoded bytes.
void maniGltfReleaseDecoded(maniGltf* gltf);

#endif // MAUL_ANIM_SRC_GLTF_COMPRESSION_H
