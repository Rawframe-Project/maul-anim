// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The one file container every kind of file uses (mani-0002): a 64-byte
// little-endian header, a table of sections, and the sections, each
// 16-byte aligned and addressed by its offset from the file's start, so a
// file is read in place. Opening checks the header, the table and the
// payload's hash once; the kinds check their own sections after it.
//
// The header:
//    0  "MANI"
//    4  kind (u16)                 6  format version (u16)
//    8  header size, 64 (u32)     12  section count (u32)
//   16  writer's library version: major, minor, patch (u16 each), 0 (u16)
//   24  file size (u64)           32  payload hash (u64): maniHash64 from
//                                     MANI_HASH_INIT over bytes 64 to the end
//   40  identity (u64)            48  zero (16 bytes)
// The table, from byte 64, holds per section: tag (u32), zero (u32),
// offset (u64), size (u64). Sections follow the table in table order,
// without overlap, tags strictly increasing.

#ifndef MAUL_ANIM_SRC_CONTAINER_H
#define MAUL_ANIM_SRC_CONTAINER_H

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MANI_CONTAINER_HEADER_SIZE 64u
#define MANI_CONTAINER_ENTRY_SIZE  24u
#define MANI_CONTAINER_ALIGNMENT   16u
// A named limit: no kind needs more sections than this.
#define MANI_CONTAINER_MAX_SECTIONS 64u

// The kinds of file.
enum
{
    maniKindSkeleton = 1,
    maniKindClip = 2,
    maniKindGraph = 3,
    maniKindRetargetMap = 4,
};

// An opened container: a view over the caller's bytes.
typedef struct maniContainer
{
    const uint8_t* bytes;
    uint64_t size;
    uint64_t identity;
    uint32_t sectionCount;
    uint16_t kind;
    uint16_t formatVersion;
} maniContainer;

// One section a writer lays out: its tag, bytes and size.
typedef struct maniSectionSource
{
    uint32_t tag;
    const void* data;
    uint64_t size;
} maniSectionSource;

// Opens bytes as a container of the given kind whose format version is at
// most maxVersion: checks the alignment of bytes (16), the header, the
// table, every section's place and the payload hash. Returns
// mani_errorInvalid for a NULL or misaligned pointer, mani_errorFormat with
// the offset of the first wrong byte, or mani_errorVersion for a newer
// format version.
maniFileResult maniOpenContainer(const void* bytes, size_t size, uint16_t kind, uint16_t maxVersion,
                                 maniContainer* containerOut);

// The section with the given tag: its bytes, size and offset in the file.
// False when the container has none.
bool maniContainerSection(const maniContainer* container, uint32_t tag, const uint8_t** dataOut,
                          uint64_t* sizeOut, uint64_t* offsetOut);

// The size of the container holding these sections, in the order given
// (tags strictly increasing). False when it does not fit in 64 bits or
// there are more than MANI_CONTAINER_MAX_SECTIONS.
bool maniContainerSize(const maniSectionSource* sections, uint32_t count, uint64_t* sizeOut);

// Writes the container into out, which holds at least maniContainerSize's
// bytes and is zeroed by the call; the payload hash is computed here.
void maniWriteContainer(uint8_t* out, uint16_t kind, uint16_t formatVersion, uint64_t identity,
                        const maniSectionSource* sections, uint32_t count);

#endif // MAUL_ANIM_SRC_CONTAINER_H
