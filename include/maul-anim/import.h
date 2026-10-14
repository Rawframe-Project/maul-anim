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

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // An opened glTF document.
    typedef struct maniGltf maniGltf;

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
    };

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

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_IMPORT_H
