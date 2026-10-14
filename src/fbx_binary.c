// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The binary FBX container (mani-0012), from the Blender Foundation's
// description of it: a 27-byte header, then records of a name, typed
// properties and nested records, each list ended by an all-zero record.
// Read in two passes, the first counting and checking every length, the
// second filling the tree, both iterative with an explicit stack.

#include "fbx_binary.h"

#include "allocator.h"
#include "bytes.h"
#include "fbx_document.h"

#include <string.h>

static const uint8_t kMagic[MANI_FBX_MAGIC_SIZE] = "Kaydara FBX Binary  ";

bool maniFbxIsBinary(const uint8_t* bytes, size_t size)
{
    return size >= MANI_FBX_MAGIC_SIZE && memcmp(bytes, kMagic, MANI_FBX_MAGIC_SIZE) == 0;
}

static uint64_t LoadBig(const uint8_t* p, uint32_t bytes)
{
    uint64_t v = 0;
    for (uint32_t i = 0; i < bytes; ++i)
    {
        v = v << 8 | p[i];
    }
    return v;
}

uint32_t maniFbxLoad32(const maniFbx* fbx, uint64_t at)
{
    const uint8_t* p = fbx->bytes + at;
    return fbx->bigEndian ? (uint32_t)LoadBig(p, 4) : maniLoadU32(p);
}

uint64_t maniFbxLoad64(const maniFbx* fbx, uint64_t at)
{
    const uint8_t* p = fbx->bytes + at;
    return fbx->bigEndian ? LoadBig(p, 8) : maniLoadU64(p);
}

typedef struct Reader
{
    maniFbx* fbx;
    // The second pass writes what the first counted.
    bool fill;
    uint32_t nodes;
    uint32_t properties;
    uint32_t depth;
    // At each depth: the node whose children are being read, its last
    // child so far, and where it must end.
    uint32_t open[MANI_FBX_MAX_DEPTH + 1];
    uint32_t last[MANI_FBX_MAX_DEPTH + 1];
    uint64_t end[MANI_FBX_MAX_DEPTH + 1];
} Reader;

static maniImportResult Bad(uint64_t at)
{
    return maniFbxFail(maniImportReasonContainer, at, 0);
}

// Whether `length` bytes from `at` lie before `limit`.
static bool Fits(uint64_t at, uint64_t length, uint64_t limit)
{
    return at <= limit && length <= limit - at;
}

static uint32_t ArrayElementSize(uint8_t type)
{
    switch (type)
    {
    case 'b':
    case 'c':
        return 1;
    case 'i':
    case 'f':
        return 4;
    case 'l':
    case 'd':
        return 8;
    default:
        return 0;
    }
}

static uint32_t ScalarSize(uint8_t type)
{
    switch (type)
    {
    case 'C':
    case 'B':
    case 'Z':
        return 1;
    case 'Y':
        return 2;
    case 'I':
    case 'F':
        return 4;
    case 'L':
    case 'D':
        return 8;
    default:
        return 0;
    }
}

static void Emit(Reader* r, maniFbxProperty p)
{
    if (r->fill)
    {
        r->fbx->properties[r->properties] = p;
    }
    ++r->properties;
}

// An array's 12-byte header and its bytes, at `at` after the type.
static maniImportResult ReadArray(Reader* r, uint8_t type, uint64_t* at, uint64_t limit)
{
    const maniFbx* fbx = r->fbx;
    uint64_t p = *at;
    if (!Fits(p, 12, limit))
    {
        return Bad(p);
    }
    uint32_t count = maniFbxLoad32(fbx, p);
    uint32_t encoding = maniFbxLoad32(fbx, p + 4);
    uint64_t encoded = maniFbxLoad32(fbx, p + 8);
    if (encoding > 1)
    {
        return Bad(p + 4);
    }
    if (encoding == 0 && encoded != (uint64_t)count * ArrayElementSize(type))
    {
        return Bad(p + 8);
    }
    if (!Fits(p + 12, encoded, limit))
    {
        return Bad(p + 8);
    }
    Emit(r, (maniFbxProperty){p + 12, encoded, count, type, (uint8_t)encoding, {0, 0}});
    *at = p + 12 + encoded;
    return maniFbxSuccess();
}

// One property at *at, its type letter first, before `limit`.
static maniImportResult ReadProperty(Reader* r, uint64_t* at, uint64_t limit)
{
    uint64_t p = *at;
    if (p >= limit)
    {
        return Bad(p);
    }
    uint8_t type = r->fbx->bytes[p];
    uint32_t scalar = ScalarSize(type);
    if (scalar != 0)
    {
        if (!Fits(p + 1, scalar, limit))
        {
            return Bad(p);
        }
        Emit(r, (maniFbxProperty){p + 1, 0, 1, type, 0, {0, 0}});
        *at = p + 1 + scalar;
        return maniFbxSuccess();
    }
    if (type == 'S' || type == 'R')
    {
        uint64_t length = Fits(p + 1, 4, limit) ? maniFbxLoad32(r->fbx, p + 1) : UINT64_MAX;
        if (length == UINT64_MAX || !Fits(p + 5, length, limit))
        {
            return Bad(p);
        }
        Emit(r, (maniFbxProperty){p + 5, 0, (uint32_t)length, type, 0, {0, 0}});
        *at = p + 5 + length;
        return maniFbxSuccess();
    }
    if (ArrayElementSize(type) != 0)
    {
        *at = p + 1;
        return ReadArray(r, type, at, limit);
    }
    return Bad(p);
}

typedef struct Header
{
    uint64_t end;
    uint64_t properties;
    uint64_t listLength;
    uint32_t nameLength;
} Header;

static Header ReadHeader(const maniFbx* fbx, uint64_t at)
{
    if (fbx->version >= 7500)
    {
        return (Header){maniFbxLoad64(fbx, at), maniFbxLoad64(fbx, at + 8),
                        maniFbxLoad64(fbx, at + 16), fbx->bytes[at + 24]};
    }
    return (Header){maniFbxLoad32(fbx, at), maniFbxLoad32(fbx, at + 4), maniFbxLoad32(fbx, at + 8),
                    fbx->bytes[at + 12]};
}

// Links node n as the last child at the current depth.
static void Link(Reader* r, uint32_t n, uint64_t nameAt, uint32_t nameLength, uint32_t first,
                 uint32_t count)
{
    if (r->fill)
    {
        maniFbxNode* nodes = r->fbx->nodes;
        nodes[n] = (maniFbxNode){nameAt, nameLength, first, count, MANI_FBX_NONE, MANI_FBX_NONE, 0};
        uint32_t last = r->last[r->depth];
        if (last == MANI_FBX_NONE)
        {
            nodes[r->open[r->depth]].firstChild = n;
        }
        else
        {
            nodes[last].next = n;
        }
    }
    r->last[r->depth] = n;
}

// A record with a name at *at: its properties, then its children (if
// any bytes remain before its end) as a new depth.
static maniImportResult ReadNode(Reader* r, Header h, uint64_t* at, uint64_t headerSize)
{
    uint64_t start = *at;
    uint64_t names = start + headerSize;
    if (h.end <= names || h.end > r->fbx->size || h.end > r->end[r->depth] ||
        !Fits(names, h.nameLength, h.end) || !Fits(names + h.nameLength, h.listLength, h.end) ||
        h.properties > h.listLength || r->nodes == MANI_FBX_NONE - 1)
    {
        return Bad(start);
    }
    uint32_t n = r->nodes++;
    uint32_t first = r->properties;
    uint64_t p = names + h.nameLength;
    uint64_t valuesEnd = p + h.listLength;
    maniImportResult result = maniFbxSuccess();
    for (uint64_t i = 0; result.result == mani_success && i < h.properties; ++i)
    {
        result = ReadProperty(r, &p, valuesEnd);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    if (p != valuesEnd)
    {
        return Bad(p);
    }
    Link(r, n, names, h.nameLength, first, (uint32_t)h.properties);
    *at = valuesEnd;
    if (valuesEnd == h.end)
    {
        return result;
    }
    // At most MANI_FBX_MAX_DEPTH levels of records: a record at the
    // last level may hold none.
    if (r->depth + 1 >= MANI_FBX_MAX_DEPTH)
    {
        return Bad(start);
    }
    ++r->depth;
    r->open[r->depth] = n;
    r->last[r->depth] = MANI_FBX_NONE;
    r->end[r->depth] = h.end;
    return result;
}

static bool IsNull(Header h)
{
    return h.end == 0 && h.properties == 0 && h.listLength == 0 && h.nameLength == 0;
}

// Every record from the header on; the top-level list ends the read.
static maniImportResult ReadRecords(Reader* r)
{
    maniFbx* fbx = r->fbx;
    uint64_t headerSize = fbx->version >= 7500 ? 25 : 13;
    uint64_t at = MANI_FBX_HEADER_SIZE;
    r->nodes = 1;
    r->properties = 0;
    r->depth = 0;
    r->open[0] = 0;
    r->last[0] = MANI_FBX_NONE;
    r->end[0] = fbx->size;
    if (r->fill)
    {
        fbx->nodes[0] = (maniFbxNode){0, 0, 0, 0, MANI_FBX_NONE, MANI_FBX_NONE, 0};
    }
    for (;;)
    {
        if (!Fits(at, headerSize, fbx->size))
        {
            return Bad(at);
        }
        Header h = ReadHeader(fbx, at);
        if (!IsNull(h))
        {
            maniImportResult result = ReadNode(r, h, &at, headerSize);
            if (result.result != mani_success)
            {
                return result;
            }
            continue;
        }
        at += headerSize;
        if (r->depth == 0)
        {
            return maniFbxSuccess();
        }
        // The list must end where its node does.
        if (at != r->end[r->depth])
        {
            return Bad(at - headerSize);
        }
        --r->depth;
    }
}

static maniImportResult ReadHeaderBytes(maniFbx* fbx)
{
    const uint8_t* b = fbx->bytes;
    if (fbx->size < MANI_FBX_HEADER_SIZE || b[21] != 0x1A || b[22] > 1)
    {
        return Bad(fbx->size < MANI_FBX_HEADER_SIZE ? fbx->size : 21);
    }
    fbx->bigEndian = b[22] == 1;
    fbx->version = maniFbxLoad32(fbx, 23);
    if (fbx->version < 3000 || fbx->version > 7700)
    {
        return maniFbxFail(maniImportReasonVersion, 23, 0);
    }
    return maniFbxSuccess();
}

maniImportResult maniFbxReadBinary(maniFbx* fbx)
{
    maniImportResult result = ReadHeaderBytes(fbx);
    if (result.result != mani_success)
    {
        return result;
    }
    Reader r = {.fbx = fbx};
    result = ReadRecords(&r);
    if (result.result != mani_success)
    {
        return result;
    }
    // The counts are set first: closing releases by them.
    fbx->nodeCount = r.nodes;
    fbx->propertyCount = r.properties;
    fbx->nodes = maniAllocate(&fbx->allocator, r.nodes, sizeof(maniFbxNode), 8);
    fbx->properties = maniAllocate(&fbx->allocator, r.properties + 1, sizeof(maniFbxProperty), 8);
    if (fbx->nodes == NULL || fbx->properties == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    r.fill = true;
    return ReadRecords(&r);
}
