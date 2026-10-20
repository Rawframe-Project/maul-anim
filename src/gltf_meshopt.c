// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The decoders follow KHR_meshopt_compression's Appendix A, which holds
// EXT_meshopt_compression's streams as its version 0, read byte by byte
// against the stream's bounds: attribute blocks end where the tail's
// padding starts, triangle and index data where their tails start, and
// a stream that ends early or leaves bytes is invalid. The FIFOs count
// their writes, so reading an entry never written is invalid; the code
// table, the channel modes and LEB128 values are checked as the
// specification says. The filters follow Appendix B's formulas in
// binary32 in their order; out-of-range results of invalid input are
// clamped, never converted out of range.

#include "gltf_meshopt.h"

#include "bytes.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define GROUP     16u
#define MAX_BLOCK 256u

// Bounds a stream is read within.
typedef struct Reader
{
    const uint8_t* at;
    const uint8_t* end;
} Reader;

// A group's deltas at 1, 2 or 4 bits from p with its spare bytes from
// extra, which has a whole group's bytes ahead of it: each spare byte is
// read and taken only for a sentinel, as meshoptimizer's scalar path.
static inline const uint8_t* Unpack(const uint8_t* p, const uint8_t* extra, int bits, uint8_t* out)
{
    uint32_t b = (uint32_t)bits;
    uint32_t sentinel = (1u << b) - 1u;
    uint32_t per = 8u / b;
    for (uint32_t i = 0; i < GROUP / per; ++i)
    {
        uint32_t byte = p[i];
        for (uint32_t k = 0; k < per; ++k)
        {
            // 1-bit deltas from the least significant bit, wider ones from
            // the most.
            uint32_t v = b == 1u ? (byte >> k) & 1u : (byte >> (8u - b * (k + 1u))) & sentinel;
            uint32_t taken = v == sentinel ? 1u : 0u;
            out[i * per + k] = taken != 0 ? *extra : (uint8_t)v;
            extra += taken;
        }
    }
    return extra;
}

// One group of 16 byte deltas at bits each (0, 1, 2, 4 or 8), a delta of
// all ones taking the next byte after the packed ones.
static bool Group(Reader* r, int bits, uint8_t* out)
{
    if (bits == 0)
    {
        memset(out, 0, GROUP);
        return true;
    }
    size_t packed = (size_t)bits * GROUP / 8u;
    if ((size_t)(r->end - r->at) < packed)
    {
        return false;
    }
    const uint8_t* p = r->at;
    const uint8_t* extra = p + packed;
    if (bits == 8)
    {
        memcpy(out, p, GROUP);
        r->at = extra;
        return true;
    }
    if ((size_t)(r->end - extra) >= GROUP)
    {
        // A constant width each, so each unpacks with constant shifts.
        r->at = bits == 1   ? Unpack(p, extra, 1, out)
                : bits == 2 ? Unpack(p, extra, 2, out)
                            : Unpack(p, extra, 4, out);
        return true;
    }
    uint32_t sentinel = (1u << bits) - 1u;
    uint32_t per = 8u / (uint32_t)bits;
    for (uint32_t k = 0; k < GROUP; ++k)
    {
        // 1-bit deltas from the least significant bit, wider ones from the
        // most.
        uint32_t shift = bits == 1 ? k % 8u : 8u - (uint32_t)bits * (k % per + 1u);
        uint32_t v = ((uint32_t)p[k / per] >> shift) & sentinel;
        if (v == sentinel)
        {
            if (extra >= r->end)
            {
                return false;
            }
            v = *extra++;
        }
        out[k] = (uint8_t)v;
    }
    r->at = extra;
    return true;
}

// A byte position's data block: 2 header bits a group choosing from the
// bit lengths, then the groups.
static bool Bytes(Reader* r, uint32_t groups, const int* lengths, uint8_t* out)
{
    size_t header = (groups + 3u) / 4u;
    if ((size_t)(r->end - r->at) < header)
    {
        return false;
    }
    const uint8_t* h = r->at;
    r->at += header;
    for (uint32_t g = 0; g < groups; ++g)
    {
        int k = (h[g / 4u] >> ((g % 4u) * 2u)) & 3;
        if (!Group(r, lengths[k], out + (size_t)g * GROUP))
        {
            return false;
        }
    }
    return true;
}

static uint32_t Unzigzag(uint32_t v, uint32_t mask)
{
    return ((v & 1u) != 0 ? ~(v >> 1) : (v >> 1)) & mask;
}

// A channel's 4 byte positions turned from deltas into values, written
// for each width: byte deltas, 2-byte deltas or 4-byte XOR deltas rotated
// back.
static void Channel(const uint8_t (*deltas)[MAX_BLOCK], uint32_t n, uint32_t stride, uint32_t k,
                    uint8_t mode, uint8_t* last, uint8_t* out)
{
    uint8_t* o = out + k;
    if ((mode & 15u) == 0)
    {
        for (uint32_t h = 0; h < 4u; ++h)
        {
            const uint8_t* d = deltas[h];
            uint8_t p = last[k + h];
            for (uint32_t i = 0; i < n; ++i)
            {
                uint8_t z = d[i];
                p = (uint8_t)(p + (uint8_t)((z >> 1) ^ (0u - (z & 1u))));
                o[(size_t)i * stride + h] = p;
            }
        }
    }
    else if ((mode & 15u) == 1)
    {
        for (uint32_t h = 0; h < 4u; h += 2u)
        {
            const uint8_t* lo = deltas[h];
            const uint8_t* hi = deltas[h + 1u];
            uint16_t p = (uint16_t)(last[k + h] | (last[k + h + 1u] << 8));
            for (uint32_t i = 0; i < n; ++i)
            {
                uint32_t z = lo[i] | ((uint32_t)hi[i] << 8);
                p = (uint16_t)(p + (uint16_t)((z >> 1) ^ (0u - (z & 1u))));
                maniStoreU16(o + (size_t)i * stride + h, p);
            }
        }
    }
    else
    {
        uint32_t r = (32u - (mode >> 4)) & 31u;
        uint32_t p = maniLoadU32(last + k);
        for (uint32_t i = 0; i < n; ++i)
        {
            uint32_t z = deltas[0][i] | ((uint32_t)deltas[1][i] << 8) |
                         ((uint32_t)deltas[2][i] << 16) | ((uint32_t)deltas[3][i] << 24);
            p ^= (z << r) | (z >> ((32u - r) & 31u));
            maniStoreU32(o + (size_t)i * stride, p);
        }
    }
    memcpy(last + k, out + (size_t)(n - 1u) * stride + k, 4u);
}

// One attribute block of n elements into out.
static bool Block(Reader* r, uint32_t n, uint32_t stride, int version, const uint8_t* modes,
                  uint8_t* last, uint8_t* out)
{
    static const int kV0[4] = {0, 2, 4, 8};
    static const int kV1[2][4] = {{0, 1, 2, 4}, {1, 2, 4, 8}};
    uint8_t deltas[4][MAX_BLOCK];
    uint32_t groups = (n + GROUP - 1u) / GROUP;
    size_t control = version == 0 ? 0u : stride / 4u;
    if ((size_t)(r->end - r->at) < control)
    {
        return false;
    }
    const uint8_t* controls = r->at;
    r->at += control;
    for (uint32_t k = 0; k < stride; k += 4u)
    {
        uint8_t c = version == 0 ? 0u : controls[k / 4u];
        for (uint32_t j = 0; j < 4u; ++j)
        {
            uint32_t mode = ((uint32_t)c >> (j * 2u)) & 3u;
            bool ok = true;
            if (mode == 3u)
            {
                ok = (size_t)(r->end - r->at) >= n;
                if (ok)
                {
                    memcpy(deltas[j], r->at, n);
                    r->at += n;
                }
            }
            else if (mode == 2u)
            {
                memset(deltas[j], 0, n);
            }
            else
            {
                ok = Bytes(r, groups, version == 0 ? kV0 : kV1[mode], deltas[j]);
            }
            if (!ok)
            {
                return false;
            }
        }
        Channel((const uint8_t (*)[MAX_BLOCK])deltas, n, stride, k,
                version == 0 ? 0u : modes[k / 4u], last, out);
    }
    return true;
}

static bool Attributes(uint8_t* out, uint32_t count, uint32_t stride, bool v1, const uint8_t* in,
                       size_t size)
{
    int version = in[0] == 0xA0u ? 0 : (in[0] == 0xA1u && v1 ? 1 : -1);
    if (version < 0)
    {
        return false;
    }
    size_t tail = stride + (version == 0 ? 0u : stride / 4u);
    size_t least = version == 0 ? 32u : 24u;
    size_t pad = tail > least ? tail : least;
    if (size < 1u + pad)
    {
        return false;
    }
    const uint8_t* modes = in + size - tail + stride;
    for (uint32_t k = 0; version == 1 && k < stride / 4u; ++k)
    {
        uint8_t m = modes[k];
        if ((m & 15u) > 2u || ((m & 15u) < 2u && (m >> 4) != 0))
        {
            return false;
        }
    }
    uint8_t last[256];
    memcpy(last, in + size - tail, stride);
    uint32_t most = (8192u / stride) & ~(GROUP - 1u);
    most = most < MAX_BLOCK ? most : MAX_BLOCK;
    Reader r = {in + 1, in + size - pad};
    for (uint32_t done = 0; done < count;)
    {
        uint32_t n = count - done < most ? count - done : most;
        if (!Block(&r, n, stride, version, modes, last, out + (size_t)done * stride))
        {
            return false;
        }
        done += n;
    }
    return r.at == r.end;
}

// An unsigned LEB128 value of 1 to 5 bytes, bits past 32 dropped. Its
// first byte within the bounds, the rest read from the stream's tail
// after them if need be (4 bytes at least in both readers), the value
// then refused if it ended past them.
static inline bool Leb(Reader* r, uint32_t* out)
{
    const uint8_t* p = r->at;
    if (p >= r->end)
    {
        return false;
    }
    uint32_t v = *p++;
    if (v >= 128u)
    {
        v &= 127u;
        uint32_t k = 1;
        for (; k < 5u; ++k)
        {
            uint8_t b = *p++;
            v |= (uint32_t)(b & 127u) << (7u * k);
            if (b < 128u)
            {
                break;
            }
        }
        if (k == 5u || p > r->end)
        {
            return false;
        }
    }
    r->at = p;
    *out = v;
    return true;
}

static inline void Put(uint8_t* out, uint32_t stride, size_t i, uint32_t v)
{
    if (stride == 2u)
    {
        maniStoreU16(out + 2 * i, (uint16_t)v);
    }
    else
    {
        maniStoreU32(out + 4 * i, v);
    }
}

// The triangle decoder's state: the FIFOs with how many entries each has
// had written (64 bits, never wrapping), next and last. Entry k of 16
// (0 the most recent) has been written when k is below the count.
typedef struct Triangles
{
    uint32_t edges[16][2];
    uint32_t vertices[16];
    uint64_t edgeAt;
    uint64_t vertexAt;
    uint32_t next;
    uint32_t last;
} Triangles;

static void PushEdge(Triangles* t, uint32_t a, uint32_t b)
{
    t->edges[t->edgeAt & 15u][0] = a;
    t->edges[t->edgeAt & 15u][1] = b;
    t->edgeAt += 1u;
}

static void PushVertex(Triangles* t, uint32_t v)
{
    t->vertices[t->vertexAt & 15u] = v;
    t->vertexAt += 1u;
}

// Vertex FIFO entry k (0 the most recent).
static bool Vertex(const Triangles* t, uint32_t k, uint32_t* out)
{
    if (k >= t->vertexAt)
    {
        return false;
    }
    *out = t->vertices[(t->vertexAt - 1u - k) & 15u];
    return true;
}

// An index as a zigzag delta from last.
static inline bool Index(Reader* r, Triangles* t, uint32_t* out)
{
    uint32_t v = 0;
    if (!Leb(r, &v))
    {
        return false;
    }
    t->last += Unzigzag(v, 0xFFFFFFFFu);
    *out = t->last;
    return true;
}

// A code 0xXY with X below 0xf: an edge from the FIFO and a third index.
// Y below 0xd, the common codes, takes no branch on the data: the vertex
// FIFO's next slot is written each time and kept only for a new index,
// that slot being entry 15, which no code reads (13 at most).
static inline bool EdgeCode(Reader* r, Triangles* t, uint8_t code, uint32_t* tri)
{
    uint32_t x = code >> 4;
    uint32_t y = code & 15u;
    if (x >= t->edgeAt)
    {
        return false;
    }
    uint32_t a = t->edges[(t->edgeAt - 1u - x) & 15u][0];
    uint32_t b = t->edges[(t->edgeAt - 1u - x) & 15u][1];
    uint32_t c = 0;
    if (y < 0xDu)
    {
        if (y != 0u && y >= t->vertexAt)
        {
            return false;
        }
        uint32_t fresh = y == 0u ? 1u : 0u;
        uint32_t old = t->vertices[(t->vertexAt - 1u - y) & 15u];
        c = fresh != 0u ? t->next : old;
        t->next += fresh;
        t->vertices[t->vertexAt & 15u] = c;
        t->vertexAt += fresh;
    }
    else
    {
        if (y < 0xFu)
        {
            c = y == 0xDu ? t->last - 1u : t->last + 1u;
            t->last = c;
        }
        else if (!Index(r, t, &c))
        {
            return false;
        }
        PushVertex(t, c);
    }
    PushEdge(t, c, b);
    PushEdge(t, a, c);
    tri[0] = a;
    tri[1] = b;
    tri[2] = c;
    return true;
}

// An index of a code 0xfY or 0xfe/0xff by its nibble: 0 the next new
// index, 0xf (explicit codes only) an encoded one, else the vertex FIFO.
static bool Third(Reader* r, Triangles* t, uint32_t nibble, uint32_t* out)
{
    if (nibble == 0u)
    {
        *out = t->next++;
        return true;
    }
    return nibble == 0xFu ? Index(r, t, out) : Vertex(t, nibble - 1u, out);
}

// A code 0xfY (Y below 0xe, through the table) or 0xfe/0xff (a byte of
// data, 0 restarting next).
static bool FreeCode(Reader* r, Triangles* t, uint8_t code, const uint8_t* table, uint32_t* tri)
{
    uint32_t zw = 0;
    if (code < 0xFEu)
    {
        zw = table[code & 15u];
    }
    else
    {
        if (r->at >= r->end)
        {
            return false;
        }
        zw = *r->at++;
        t->next = zw == 0u ? 0u : t->next;
    }
    uint32_t z = zw >> 4;
    uint32_t w = zw & 15u;
    uint32_t a = 0;
    bool ok = code == 0xFFu ? Index(r, t, &a) : (a = t->next++, true);
    // The FIFO is read before any of the three is pushed.
    ok = ok && Third(r, t, z, &tri[1]) && Third(r, t, w, &tri[2]);
    if (!ok)
    {
        return false;
    }
    tri[0] = a;
    PushEdge(t, tri[1], a);
    PushEdge(t, tri[2], tri[1]);
    PushEdge(t, a, tri[2]);
    PushVertex(t, a);
    if (z == 0u || z == 0xFu)
    {
        PushVertex(t, tri[1]);
    }
    if (w == 0u || w == 0xFu)
    {
        PushVertex(t, tri[2]);
    }
    return true;
}

static inline bool TriangleList(uint8_t* out, uint32_t count, uint32_t stride, const uint8_t* in,
                                size_t size)
{
    size_t codes = count / 3u;
    if (in[0] != 0xE1u || size < 1u + codes + 16u)
    {
        return false;
    }
    const uint8_t* table = in + size - 16u;
    bool ok = table[14] == 0u && table[15] == 0u;
    for (uint32_t k = 0; k < 16u && ok; ++k)
    {
        ok = (table[k] >> 4) != 0xFu && (table[k] & 15u) != 0xFu;
    }
    Triangles t = {.edgeAt = 0};
    Reader r = {in + 1u + codes, table};
    for (size_t i = 0; i < codes && ok; ++i)
    {
        uint8_t code = in[1u + i];
        uint32_t tri[3] = {0, 0, 0};
        ok = code < 0xF0u ? EdgeCode(&r, &t, code, tri) : FreeCode(&r, &t, code, table, tri);
        if (ok)
        {
            Put(out, stride, 3u * i, tri[0]);
            Put(out, stride, 3u * i + 1u, tri[1]);
            Put(out, stride, 3u * i + 2u, tri[2]);
        }
    }
    return ok && r.at == r.end;
}

static inline bool IndexList(uint8_t* out, uint32_t count, uint32_t stride, const uint8_t* in,
                             size_t size)
{
    if (in[0] != 0xD1u || size < 1u + 4u)
    {
        return false;
    }
    Reader r = {in + 1, in + size - 4u};
    uint32_t last[2] = {0, 0};
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t v = 0;
        if (!Leb(&r, &v))
        {
            return false;
        }
        uint32_t base = v & 1u;
        last[base] += Unzigzag(v >> 1, 0xFFFFFFFFu);
        Put(out, stride, i, last[base]);
    }
    return r.at == r.end;
}

bool maniMeshoptDecode(uint8_t* out, uint32_t count, uint32_t stride, maniMeshoptMode mode, bool v1,
                       const uint8_t* in, size_t size)
{
    if (size < 1u)
    {
        return false;
    }
    switch (mode)
    {
    case maniMeshoptAttributes:
        return Attributes(out, count, stride, v1, in, size);
    // Each index width a copy of its own.
    case maniMeshoptTriangles:
        return stride == 2u ? TriangleList(out, count, 2u, in, size)
                            : TriangleList(out, count, 4u, in, size);
    default:
        return stride == 2u ? IndexList(out, count, 2u, in, size)
                            : IndexList(out, count, 4u, in, size);
    }
}

// round(v) (halfway away from zero) as an integer within low and high,
// 0 for NaN: invalid input gives an unspecified value, never an
// out-of-range conversion. The bounds are integers, so clamping first
// gives the same value; within them (65,535 at most) the conversion
// truncates exactly and v less its truncation, f, is exact.
static int32_t Rounded(float v, float low, float high)
{
    if (v >= low && v <= high)
    {
        // f + f is exact and below 2 in magnitude: it truncates to the
        // step away from zero exactly when f is half a unit or more.
        int32_t t = (int32_t)v;
        float f = v - (float)t;
        return t + (int32_t)(f + f);
    }
    return v > high ? (int32_t)high : (v < low ? (int32_t)low : 0);
}

// Rounded within -most and most, its range tested once.
static int32_t RoundedSigned(float v, float most)
{
    if (fabsf(v) <= most)
    {
        int32_t t = (int32_t)v;
        float f = v - (float)t;
        return t + (int32_t)(f + f);
    }
    return v > most ? (int32_t)most : (v < -most ? -(int32_t)most : 0);
}

static int32_t Signed(const uint8_t* p, uint32_t bytes)
{
    return bytes == 1u ? (int32_t)(int8_t)p[0] : (int32_t)(int16_t)maniLoadU16(p);
}

static uint32_t Unsigned(const uint8_t* p, uint32_t bytes)
{
    return bytes == 1u ? p[0] : maniLoadU16(p);
}

static void Store(uint8_t* p, uint32_t bytes, int32_t v)
{
    if (bytes == 1u)
    {
        p[0] = (uint8_t)v;
    }
    else
    {
        maniStoreU16(p, (uint16_t)v);
    }
}

static inline void Octahedral(uint8_t* e, uint32_t bytes)
{
    float most = bytes == 1u ? 127.0f : 32767.0f;
    float one = (float)Signed(e + 2u * bytes, bytes);
    float x = (float)Signed(e, bytes) / one;
    float y = (float)Signed(e + bytes, bytes) / one;
    float z = 1.0f - fabsf(x) - fabsf(y);
    float t = z < 0.0f ? z : 0.0f;
    x -= copysignf(t, x);
    y -= copysignf(t, y);
    float len = sqrtf(x * x + y * y + z * z);
    x /= len;
    y /= len;
    z /= len;
    Store(e, bytes, RoundedSigned(x * most, most));
    Store(e + bytes, bytes, RoundedSigned(y * most, most));
    Store(e + 2u * bytes, bytes, RoundedSigned(z * most, most));
}

static int32_t Load16(const uint8_t* p)
{
    return (int32_t)(int16_t)maniLoadU16(p);
}

static void Store16(uint8_t* p, float v)
{
    maniStoreU16(p, (uint16_t)RoundedSigned(v * 32767.0f, 32767.0f));
}

static void Quaternion(uint8_t* e)
{
    // 1 / sqrt(2) rounded to binary32.
    const float range = 0x1.6a09e6p-1f;
    int32_t s = Load16(e + 6);
    float one = (float)(s | 3);
    float x = (float)Load16(e) / one * range;
    float y = (float)Load16(e + 2) / one * range;
    float z = (float)Load16(e + 4) / one * range;
    float ww = 1.0f - x * x - y * y - z * z;
    float w = sqrtf(ww > 0.0f ? ww : 0.0f);
    // The three after the largest component, then it in its place.
    uint32_t most = (uint32_t)s & 3u;
    Store16(e + 2u * ((most + 1u) & 3u), x);
    Store16(e + 2u * ((most + 2u) & 3u), y);
    Store16(e + 2u * ((most + 3u) & 3u), z);
    Store16(e + 2u * most, w);
}

static void Exponential(uint8_t* e)
{
    uint32_t v = maniLoadU32(e);
    int32_t ex = (int32_t)((v >> 24) ^ 0x80u) - 0x80;
    int32_t m = (int32_t)((v & 0xFFFFFFu) ^ 0x800000u) - 0x800000;
    // m times 2^ex: from its bits when 2^ex is normal, which rounds the
    // product once as ldexpf does.
    float f = 0.0f;
    if (ex >= -126)
    {
        uint32_t power = (uint32_t)(ex + 127) << 23;
        float scale = 0.0f;
        memcpy(&scale, &power, sizeof scale);
        f = (float)m * scale;
    }
    else
    {
        f = ldexpf((float)m, ex);
    }
    uint32_t bits = 0;
    memcpy(&bits, &f, sizeof bits);
    maniStoreU32(e, bits);
}

static inline void Color(uint8_t* e, uint32_t bytes)
{
    float most = bytes == 1u ? 255.0f : 65535.0f;
    uint32_t alpha = Unsigned(e + 3u * bytes, bytes);
    uint32_t msb = 0;
    while (msb < 16u && (alpha >> (msb + 1u)) != 0u)
    {
        msb += 1u;
    }
    int32_t as = alpha == 0u ? 0 : (int32_t)((1u << (msb + 1u)) - 1u);
    int32_t y = (int32_t)Unsigned(e, bytes);
    int32_t co = Signed(e + bytes, bytes);
    int32_t cg = Signed(e + 2u * bytes, bytes);
    int32_t a = (int32_t)alpha & (as >> 1);
    a = (a << 1) | (a & 1);
    float ss = most / (float)as;
    const int32_t values[4] = {y + co - cg, y + cg, y - co - cg, a};
    for (uint32_t k = 0; k < 4u; ++k)
    {
        Store(e + k * bytes, bytes, Rounded((float)values[k] * ss, 0.0f, most));
    }
}

void maniMeshoptApplyFilter(uint8_t* data, uint32_t count, uint32_t stride,
                            maniMeshoptFilter filter)
{
    // A loop a filter, each with its widths constant.
    for (uint32_t i = 0; filter == maniMeshoptOctahedral && i < count; ++i)
    {
        uint8_t* e = data + (size_t)i * stride;
        if (stride == 4u)
        {
            Octahedral(e, 1u);
        }
        else
        {
            Octahedral(e, 2u);
        }
    }
    for (uint32_t i = 0; filter == maniMeshoptQuaternion && i < count; ++i)
    {
        Quaternion(data + (size_t)i * stride);
    }
    for (size_t k = 0; filter == maniMeshoptExponential && k < (size_t)count * stride; k += 4u)
    {
        Exponential(data + k);
    }
    for (uint32_t i = 0; filter == maniMeshoptColor && i < count; ++i)
    {
        uint8_t* e = data + (size_t)i * stride;
        if (stride == 4u)
        {
            Color(e, 1u);
        }
        else
        {
            Color(e, 2u);
        }
    }
}
