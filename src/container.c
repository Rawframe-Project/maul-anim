// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Every check reports the offset of the field it found wrong, so a
// corrupt file says where; nothing past the header is read before the
// header's sizes are known to fit.

#include "container.h"

#include "bytes.h"
#include "invariant.h"

#include "maul-anim/base.h"

#include <stdckdint.h>
#include <string.h>

static const uint8_t kMagic[4] = {'M', 'A', 'N', 'I'};

uint64_t maniHashBytes(uint64_t seed, const uint8_t* data, uint64_t size)
{
    // maniHash64 takes counts below 2^31 a call; pieces of 2^30 bytes, a
    // multiple of eight, continue it exactly.
    uint64_t hash = seed;
    for (uint64_t done = 0; done < size;)
    {
        uint64_t piece = size - done < 0x40000000u ? size - done : 0x40000000u;
        hash = maniHash64(hash, data + done, (int32_t)piece);
        done += piece;
    }
    return hash;
}

// The hash of bytes 64 to the end.
static uint64_t HashPayload(const uint8_t* b, uint64_t fileSize)
{
    return maniHashBytes(MANI_HASH_INIT, b + MANI_CONTAINER_HEADER_SIZE,
                         fileSize - MANI_CONTAINER_HEADER_SIZE);
}

static maniFileResult Fail(maniResult result, uint64_t offset)
{
    return (maniFileResult){result, offset};
}

static maniFileResult CheckHeader(const uint8_t* b, size_t size, uint16_t kind, uint16_t maxVersion)
{
    if (size < MANI_CONTAINER_HEADER_SIZE)
    {
        return Fail(mani_errorFormat, size);
    }
    if (memcmp(b, kMagic, sizeof kMagic) != 0)
    {
        return Fail(mani_errorFormat, 0);
    }
    if (maniLoadU16(b + 4) != kind)
    {
        return Fail(mani_errorFormat, 4);
    }
    uint16_t version = maniLoadU16(b + 6);
    if (version == 0)
    {
        return Fail(mani_errorFormat, 6);
    }
    if (version > maxVersion)
    {
        return Fail(mani_errorVersion, 6);
    }
    if (maniLoadU32(b + 8) != MANI_CONTAINER_HEADER_SIZE)
    {
        return Fail(mani_errorFormat, 8);
    }
    if (maniLoadU32(b + 12) > MANI_CONTAINER_MAX_SECTIONS)
    {
        return Fail(mani_errorFormat, 12);
    }
    if (maniLoadU16(b + 22) != 0)
    {
        return Fail(mani_errorFormat, 22);
    }
    uint64_t fileSize = maniLoadU64(b + 24);
    if (fileSize < MANI_CONTAINER_HEADER_SIZE || fileSize > size)
    {
        return Fail(mani_errorFormat, 24);
    }
    for (uint32_t i = 48; i < MANI_CONTAINER_HEADER_SIZE; ++i)
    {
        if (b[i] != 0)
        {
            return Fail(mani_errorFormat, i);
        }
    }
    return Fail(mani_success, 0);
}

// The table's entries: zero where zero is due, aligned offsets past the
// table, in order without overlap, inside the file, tags increasing.
static maniFileResult CheckTable(const uint8_t* b, uint64_t fileSize, uint32_t count)
{
    uint64_t tableEnd = MANI_CONTAINER_HEADER_SIZE + (uint64_t)count * MANI_CONTAINER_ENTRY_SIZE;
    if (tableEnd > fileSize)
    {
        return Fail(mani_errorFormat, 12);
    }
    uint64_t next = tableEnd;
    uint32_t lastTag = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint64_t at = MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        uint32_t tag = maniLoadU32(b + at);
        if (i > 0 && tag <= lastTag)
        {
            return Fail(mani_errorFormat, at);
        }
        lastTag = tag;
        if (maniLoadU32(b + at + 4) != 0)
        {
            return Fail(mani_errorFormat, at + 4);
        }
        uint64_t offset = maniLoadU64(b + at + 8);
        uint64_t sectionSize = maniLoadU64(b + at + 16);
        if (offset % MANI_CONTAINER_ALIGNMENT != 0 || offset < next || offset > fileSize)
        {
            return Fail(mani_errorFormat, at + 8);
        }
        if (sectionSize > fileSize - offset)
        {
            return Fail(mani_errorFormat, at + 16);
        }
        next = offset + sectionSize;
    }
    return Fail(mani_success, 0);
}

MANI_COLD maniFileResult maniOpenContainer(const void* bytes, size_t size, uint16_t kind,
                                           uint16_t maxVersion, maniContainer* containerOut)
{
    if (bytes == NULL || containerOut == NULL || (uintptr_t)bytes % MANI_CONTAINER_ALIGNMENT != 0)
    {
        return Fail(mani_errorInvalid, 0);
    }
    const uint8_t* b = (const uint8_t*)bytes;
    maniFileResult r = CheckHeader(b, size, kind, maxVersion);
    if (r.result != mani_success)
    {
        return r;
    }
    uint64_t fileSize = maniLoadU64(b + 24);
    uint32_t count = maniLoadU32(b + 12);
    r = CheckTable(b, fileSize, count);
    if (r.result != mani_success)
    {
        return r;
    }
    if (HashPayload(b, fileSize) != maniLoadU64(b + 32))
    {
        return Fail(mani_errorFormat, 32);
    }
    *containerOut = (maniContainer){
        .bytes = b,
        .size = fileSize,
        .identity = maniLoadU64(b + 40),
        .sectionCount = count,
        .kind = kind,
        .formatVersion = maniLoadU16(b + 6),
    };
    return Fail(mani_success, 0);
}

bool maniContainerSection(const maniContainer* container, uint32_t tag, const uint8_t** dataOut,
                          uint64_t* sizeOut, uint64_t* offsetOut)
{
    for (uint32_t i = 0; i < container->sectionCount; ++i)
    {
        const uint8_t* entry =
            container->bytes + MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        if (maniLoadU32(entry) == tag)
        {
            uint64_t offset = maniLoadU64(entry + 8);
            *dataOut = container->bytes + offset;
            *sizeOut = maniLoadU64(entry + 16);
            *offsetOut = offset;
            return true;
        }
    }
    return false;
}

static uint64_t AlignUp(uint64_t value)
{
    return (value + MANI_CONTAINER_ALIGNMENT - 1) / MANI_CONTAINER_ALIGNMENT *
           MANI_CONTAINER_ALIGNMENT;
}

bool maniContainerSize(const maniSectionSource* sections, uint32_t count, uint64_t* sizeOut)
{
    if (count > MANI_CONTAINER_MAX_SECTIONS)
    {
        return false;
    }
    uint64_t size =
        AlignUp(MANI_CONTAINER_HEADER_SIZE + (uint64_t)count * MANI_CONTAINER_ENTRY_SIZE);
    for (uint32_t i = 0; i < count; ++i)
    {
        uint64_t end = 0;
        if (ckd_add(&end, size, sections[i].size) || end > UINT64_MAX - MANI_CONTAINER_ALIGNMENT)
        {
            return false;
        }
        size = i + 1 < count ? AlignUp(end) : end;
    }
    *sizeOut = size;
    return true;
}

void maniWriteContainer(uint8_t* out, uint16_t kind, uint16_t formatVersion, uint64_t identity,
                        const maniSectionSource* sections, uint32_t count)
{
    uint64_t fileSize = 0;
    bool sized = maniContainerSize(sections, count, &fileSize);
    // Writers size the file with maniContainerSize before they allocate it.
    MANI_ASSERT(sized);
    (void)sized;
    for (uint32_t i = 1; i < count; ++i)
    {
        MANI_ASSERT(sections[i - 1].tag < sections[i].tag);
    }
    memset(out, 0, (size_t)fileSize);
    memcpy(out, kMagic, sizeof kMagic);
    maniStoreU16(out + 4, kind);
    maniStoreU16(out + 6, formatVersion);
    maniStoreU32(out + 8, MANI_CONTAINER_HEADER_SIZE);
    maniStoreU32(out + 12, count);
    maniStoreU16(out + 16, MANI_VERSION_MAJOR);
    maniStoreU16(out + 18, MANI_VERSION_MINOR);
    maniStoreU16(out + 20, MANI_VERSION_PATCH);
    maniStoreU64(out + 24, fileSize);
    maniStoreU64(out + 40, identity);
    uint64_t offset =
        AlignUp(MANI_CONTAINER_HEADER_SIZE + (uint64_t)count * MANI_CONTAINER_ENTRY_SIZE);
    for (uint32_t i = 0; i < count; ++i)
    {
        uint8_t* entry = out + MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        maniStoreU32(entry, sections[i].tag);
        maniStoreU64(entry + 8, offset);
        maniStoreU64(entry + 16, sections[i].size);
        if (sections[i].size > 0)
        {
            memcpy(out + offset, sections[i].data, (size_t)sections[i].size);
        }
        offset = AlignUp(offset + sections[i].size);
    }
    maniStoreU64(out + 32, HashPayload(out, fileSize));
}
