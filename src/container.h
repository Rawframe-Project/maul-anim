// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The one file container every kind of file uses: a 48-byte
// little-endian header, a table of sections, and the sections, each
// 16-byte aligned and addressed by its offset from the file's start, so a
// file is read in place. Opening checks the header, the table and the
// payload's hash once; the kinds check their own sections after it.
//
// The header:
//    0  "MANI"
//    4  kind (u16)                 6  format version (u16)
//    8  header size, 48 (u32)     12  section count (u32)
//   16  writer's library version: major, minor, patch (u16 each), 0 (u16)
//   24  file size (u64)           32  payload hash (u64): maniHash64 from
//                                     MANI_HASH_INIT over bytes 48 to the end
//   40  identity (u64)
// The zero u16 at 22 aligns the file size: five u16 fields after the
// magic and two u32 leave the u64 fields two bytes short whatever their
// order. The table, from byte 48, holds per section: tag (u32), zero
// (u32, aligning the offset), offset (u64), size (u64). Sections follow
// the table in table order, without overlap, tags strictly increasing.

#ifndef MAUL_ANIM_SRC_CONTAINER_H
#define MAUL_ANIM_SRC_CONTAINER_H

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MANI_CONTAINER_HEADER_SIZE 48u
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
    maniKindSkin = 5,
    maniKindInfluences = 6,
    maniKindMorphTargets = 7,
    maniKindMotionDatabase = 8,
    maniKindLearnedModel = 9,
    maniKindLearnedCheckpoint = 10,
    maniKindSpringSet = 11,
    maniKindRagdoll = 12,
    maniKindMirrorMap = 13,
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

// maniHash64 over any number of bytes, continuing seed: the same value as
// one call would give.
uint64_t maniHashBytes(uint64_t seed, const uint8_t* data, uint64_t size);

// Opens bytes as a container of the given kind and format version: checks
// the alignment of bytes (16), the header, the table, every section's
// place and the payload hash. Returns mani_errorInvalid for a NULL or
// misaligned pointer, mani_errorFormat with the offset of the first wrong
// byte, or mani_errorVersion for any other format version.
maniFileResult maniOpenContainer(const void* bytes, size_t size, uint16_t kind, uint16_t version,
                                 maniContainer* containerOut);

// An opener's checks of a file's bytes, filling the view at out.
typedef maniFileResult (*maniContainerOpener)(const void* bytes, size_t size, void* out);

// The opener's result, a refusal with mani_errorFormat given the tag of
// the section its offset lies in: 0 in the header, the table or the
// padding between sections. Any other result is returned as it was.
// Reads only the header and the table, within size; a refusal of the
// table itself lies in the table, so a table it scans is one the
// container checked. The worker a public opener calls inside the
// library's floating-point environment.
maniFileResult maniContainerOpenTagged(const void* bytes, size_t size, void* out,
                                       maniContainerOpener opener);

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
