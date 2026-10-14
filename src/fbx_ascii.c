// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ASCII FBX form (mani-0012) read into the same tree as the binary
// one: `Name: value, value { children }`, `;` comments, quoted strings
// with FBX's `&quot;` entities, bare words, numbers kept as text and
// converted exactly when read, and a `*N { a: ... }` group folded into
// one array property of N numbers. Two passes, as the binary reader
// does, with a one-token lookahead and an explicit stack.

#include "fbx_ascii.h"

#include "allocator.h"
#include "decimal.h"
#include "fbx_document.h"

#include <stdbool.h>
#include <string.h>

typedef enum TokenKind
{
    TokenEnd,
    TokenName,
    TokenString,
    TokenNumber,
    // A token that starts as a number but is none (old writers' "1.#INF").
    TokenNotNumber,
    TokenWord,
    TokenStar,
    TokenComma,
    TokenOpen,
    TokenClose,
    TokenBad,
} TokenKind;

typedef struct Token
{
    TokenKind kind;
    // The token's text (a string's or a name's without quotes or colon).
    uint64_t start;
    uint64_t length;
    // Its first byte in the file, where a refusal points.
    uint64_t at;
} Token;

typedef struct Reader
{
    maniFbx* fbx;
    bool fill;
    uint64_t at;
    bool held;
    Token next;
    uint32_t nodes;
    uint32_t properties;
    uint32_t depth;
    uint32_t open[MANI_FBX_MAX_DEPTH + 1];
    uint32_t last[MANI_FBX_MAX_DEPTH + 1];
    // The first comment's version, if it gave one.
    uint32_t version;
    bool firstComment;
} Reader;

static bool WellFormed(const uint8_t* p, uint64_t length);

// The text's sign handled for the decimal conversion, which reads JSON's
// numbers: a leading '+' dropped.
static const char* NumberText(const maniFbx* fbx, uint64_t* at, uint64_t* length)
{
    if (*length > 1 && fbx->bytes[*at] == '+')
    {
        ++*at;
        --*length;
    }
    return (const char*)fbx->bytes + *at;
}

bool maniFbxTextDouble(const maniFbx* fbx, uint64_t at, uint64_t length, double* out)
{
    if (length == 0 || !WellFormed(fbx->bytes + at, length))
    {
        return false;
    }
    const char* text = NumberText(fbx, &at, &length);
    uint64_t bits = 0;
    if (!maniDecimalToBinary(text, (size_t)length, maniBinary64, &bits))
    {
        return false;
    }
    memcpy(out, &bits, sizeof *out);
    return true;
}

bool maniFbxTextInteger(const maniFbx* fbx, uint64_t at, uint64_t length, int64_t minimum,
                        int64_t maximum, int64_t* out)
{
    if (length == 0 || !WellFormed(fbx->bytes + at, length))
    {
        return false;
    }
    const char* text = NumberText(fbx, &at, &length);
    return maniDecimalToInteger(text, (size_t)length, minimum, maximum, out);
}

static maniImportResult Bad(uint64_t at)
{
    return maniFbxFail(maniImportReasonSyntax, at, 0);
}

static bool IsSpace(uint8_t c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

static bool IsDigit(uint8_t c)
{
    return c >= '0' && c <= '9';
}

static bool IsWordStart(uint8_t c)
{
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_';
}

static bool IsWord(uint8_t c)
{
    return IsWordStart(c) || IsDigit(c) || c == '-' || c == '(' || c == ')';
}

// "; FBX 7.4.0 project file": 7400, or 0.
static uint32_t CommentVersion(const uint8_t* p, size_t length)
{
    static const char kPrefix[] = "; FBX ";
    size_t n = sizeof kPrefix - 1;
    if (length < n + 5 || memcmp(p, kPrefix, n) != 0 || !IsDigit(p[n]) || p[n + 1] != '.' ||
        !IsDigit(p[n + 2]) || p[n + 3] != '.' || !IsDigit(p[n + 4]))
    {
        return 0;
    }
    return (uint32_t)(p[n] - '0') * 1000 + (uint32_t)(p[n + 2] - '0') * 100;
}

static void SkipSpace(Reader* r)
{
    const maniFbx* fbx = r->fbx;
    for (;;)
    {
        while (r->at < fbx->size && IsSpace(fbx->bytes[r->at]))
        {
            ++r->at;
        }
        if (r->at >= fbx->size || fbx->bytes[r->at] != ';')
        {
            return;
        }
        uint64_t start = r->at;
        while (r->at < fbx->size && fbx->bytes[r->at] != '\n')
        {
            ++r->at;
        }
        if (!r->firstComment)
        {
            r->firstComment = true;
            r->version = CommentVersion(fbx->bytes + start, (size_t)(r->at - start));
        }
    }
}

// A number: [+-]? (D+ (. D*)? | . D+) ([eE] [+-]? D+)?, as the decimal
// conversion reads it once a '+' is dropped.
static bool WellFormed(const uint8_t* p, uint64_t length)
{
    uint64_t i = p[0] == '+' || p[0] == '-' ? 1 : 0;
    uint64_t digits = 0;
    while (i < length && IsDigit(p[i]))
    {
        ++i;
        ++digits;
    }
    if (i < length && p[i] == '.')
    {
        ++i;
        while (i < length && IsDigit(p[i]))
        {
            ++i;
            ++digits;
        }
    }
    if (digits == 0)
    {
        return false;
    }
    if (i < length && (p[i] == 'e' || p[i] == 'E'))
    {
        ++i;
        i += i < length && (p[i] == '+' || p[i] == '-') ? 1 : 0;
        uint64_t exponent = i;
        while (i < length && IsDigit(p[i]))
        {
            ++i;
        }
        if (i == exponent)
        {
            return false;
        }
    }
    return i == length;
}

static Token NumberToken(Reader* r)
{
    const uint8_t* b = r->fbx->bytes;
    uint64_t start = r->at;
    while (r->at < r->fbx->size && (IsDigit(b[r->at]) || b[r->at] == '+' || b[r->at] == '-' ||
                                    b[r->at] == '.' || b[r->at] == 'e' || b[r->at] == 'E'))
    {
        ++r->at;
    }
    // Old writers' non-finite numbers: "1.#INF", "-1.#IND", "1.#QNAN".
    while (r->at < r->fbx->size && (IsWord(b[r->at]) || b[r->at] == '#'))
    {
        ++r->at;
    }
    uint64_t length = r->at - start;
    return (Token){WellFormed(b + start, length) ? TokenNumber : TokenNotNumber, start, length,
                   start};
}

// A quoted string, or a name when a colon follows its closing quote.
static Token StringToken(Reader* r)
{
    const maniFbx* fbx = r->fbx;
    uint64_t start = ++r->at;
    while (r->at < fbx->size && fbx->bytes[r->at] != '"')
    {
        ++r->at;
    }
    if (r->at >= fbx->size)
    {
        return (Token){TokenBad, start - 1, 0, start - 1};
    }
    Token t = {TokenString, start, r->at - start, start - 1};
    ++r->at;
    if (r->at < fbx->size && fbx->bytes[r->at] == ':')
    {
        ++r->at;
        t.kind = TokenName;
    }
    return t;
}

// A bare word, or a name when a colon follows it (spaces allowed).
static Token WordToken(Reader* r)
{
    const maniFbx* fbx = r->fbx;
    uint64_t start = r->at;
    while (r->at < fbx->size && IsWord(fbx->bytes[r->at]))
    {
        ++r->at;
    }
    Token t = {TokenWord, start, r->at - start, start};
    uint64_t end = r->at;
    while (r->at < fbx->size && (fbx->bytes[r->at] == ' ' || fbx->bytes[r->at] == '\t'))
    {
        ++r->at;
    }
    if (r->at < fbx->size && fbx->bytes[r->at] == ':')
    {
        ++r->at;
        t.kind = TokenName;
    }
    else
    {
        r->at = end;
    }
    return t;
}

static Token Read(Reader* r)
{
    SkipSpace(r);
    const maniFbx* fbx = r->fbx;
    if (r->at >= fbx->size)
    {
        return (Token){TokenEnd, r->at, 0, r->at};
    }
    uint8_t c = fbx->bytes[r->at];
    if (IsWordStart(c))
    {
        return WordToken(r);
    }
    if (IsDigit(c) || c == '-' || c == '+' || c == '.')
    {
        return NumberToken(r);
    }
    if (c == '"')
    {
        return StringToken(r);
    }
    static const char kSingles[] = "*,{}";
    static const TokenKind kKinds[] = {TokenStar, TokenComma, TokenOpen, TokenClose};
    const char* single = c != 0 ? strchr(kSingles, c) : NULL;
    Token t = {single != NULL ? kKinds[single - kSingles] : TokenBad, r->at, 1, r->at};
    ++r->at;
    return t;
}

static Token Next(Reader* r)
{
    if (r->held)
    {
        r->held = false;
        return r->next;
    }
    return Read(r);
}

static void Hold(Reader* r, Token t)
{
    r->held = true;
    r->next = t;
}

static void Emit(Reader* r, uint8_t type, uint64_t offset, uint64_t size, uint32_t count)
{
    if (r->fill)
    {
        r->fbx->properties[r->properties] = (maniFbxProperty){offset, size, count, type, 0, {0, 0}};
    }
    ++r->properties;
}

// `*N { a: v, ... }` after the star: one array property of N numbers.
static maniImportResult ReadArray(Reader* r, uint64_t star)
{
    Token count = Next(r);
    int64_t n = 0;
    if (count.kind != TokenNumber ||
        !maniFbxTextInteger(r->fbx, count.start, count.length, 0, UINT32_MAX, &n) ||
        Next(r).kind != TokenOpen)
    {
        return Bad(star);
    }
    Token a = Next(r);
    if (a.kind != TokenName || a.length != 1 || r->fbx->bytes[a.start] != 'a')
    {
        // An empty array may hold no `a:` at all.
        if (a.kind == TokenClose && n == 0)
        {
            Emit(r, 'A', a.start, 0, 0);
            return maniFbxSuccess();
        }
        return Bad(a.at);
    }
    uint64_t first = r->at;
    uint64_t values = 0;
    Token t = Next(r);
    while (t.kind == TokenNumber || t.kind == TokenNotNumber)
    {
        ++values;
        t = Next(r);
        if (t.kind != TokenComma)
        {
            break;
        }
        t = Next(r);
    }
    if (t.kind != TokenClose || values != (uint64_t)n)
    {
        return Bad(t.kind != TokenClose ? t.at : star);
    }
    Emit(r, 'A', first, t.start - first, (uint32_t)n);
    return maniFbxSuccess();
}

static bool IsValue(TokenKind kind)
{
    return kind == TokenString || kind == TokenNumber || kind == TokenNotNumber ||
           kind == TokenWord;
}

static uint8_t PropertyType(TokenKind kind)
{
    return kind == TokenString ? 'S' : kind == TokenWord ? 'W' : kind == TokenNumber ? 'N' : 'Q';
}

// A node's values after its name; the token after them is held.
static maniImportResult ReadValues(Reader* r)
{
    Token t = Next(r);
    t = t.kind == TokenComma ? Next(r) : t;
    while (IsValue(t.kind) || t.kind == TokenStar)
    {
        if (t.kind == TokenStar)
        {
            maniImportResult result = ReadArray(r, t.start);
            if (result.result != mani_success)
            {
                return result;
            }
        }
        else if (t.length > UINT32_MAX)
        {
            return Bad(t.at);
        }
        else
        {
            bool text = t.kind == TokenString || t.kind == TokenWord;
            Emit(r, PropertyType(t.kind), t.start, t.length, text ? (uint32_t)t.length : 1);
        }
        t = Next(r);
        if (t.kind != TokenComma)
        {
            break;
        }
        t = Next(r);
        if (!IsValue(t.kind) && t.kind != TokenStar)
        {
            return Bad(t.at);
        }
    }
    Hold(r, t);
    return maniFbxSuccess();
}

static void Link(Reader* r, uint32_t n, Token name, uint32_t first)
{
    if (r->fill)
    {
        maniFbxNode* nodes = r->fbx->nodes;
        nodes[n] = (maniFbxNode){name.start,    (uint32_t)name.length, first, r->properties - first,
                                 MANI_FBX_NONE, MANI_FBX_NONE,         0};
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

static maniImportResult ReadNode(Reader* r, Token name)
{
    if (name.length > UINT32_MAX || r->nodes == MANI_FBX_NONE - 1)
    {
        return Bad(name.at);
    }
    uint32_t n = r->nodes++;
    uint32_t first = r->properties;
    maniImportResult result = ReadValues(r);
    if (result.result != mani_success)
    {
        return result;
    }
    Link(r, n, name, first);
    Token t = Next(r);
    if (t.kind != TokenOpen)
    {
        Hold(r, t);
        return result;
    }
    if (r->depth + 1 >= MANI_FBX_MAX_DEPTH)
    {
        return Bad(t.at);
    }
    ++r->depth;
    r->open[r->depth] = n;
    r->last[r->depth] = MANI_FBX_NONE;
    return result;
}

static maniImportResult ReadAll(Reader* r)
{
    r->at = 0;
    r->held = false;
    r->nodes = 1;
    r->properties = 0;
    r->depth = 0;
    r->open[0] = 0;
    r->last[0] = MANI_FBX_NONE;
    r->firstComment = false;
    r->version = 0;
    if (r->fill)
    {
        r->fbx->nodes[0] = (maniFbxNode){0, 0, 0, 0, MANI_FBX_NONE, MANI_FBX_NONE, 0};
    }
    for (;;)
    {
        Token t = Next(r);
        if (t.kind == TokenEnd)
        {
            return r->depth == 0 ? maniFbxSuccess() : Bad(t.at);
        }
        if (t.kind == TokenClose && r->depth > 0)
        {
            --r->depth;
            continue;
        }
        if (t.kind != TokenName)
        {
            return Bad(t.at);
        }
        maniImportResult result = ReadNode(r, t);
        if (result.result != mani_success)
        {
            return result;
        }
    }
}

// The version: the first comment's, else FBXHeaderExtension's
// FBXVersion.
static maniImportResult ReadVersion(maniFbx* fbx, uint32_t fromComment)
{
    uint32_t version = fromComment;
    if (version == 0)
    {
        uint32_t header = maniFbxFindChild(fbx, 0, "FBXHeaderExtension", 18);
        uint32_t node = header != MANI_FBX_NONE ? maniFbxFindChild(fbx, header, "FBXVersion", 10)
                                                : MANI_FBX_NONE;
        int64_t v = 0;
        bool given = node != MANI_FBX_NONE && fbx->nodes[node].propertyCount > 0 &&
                     fbx->properties[fbx->nodes[node].firstProperty].type == 'N' &&
                     maniFbxTextInteger(fbx, fbx->properties[fbx->nodes[node].firstProperty].offset,
                                        fbx->properties[fbx->nodes[node].firstProperty].encodedSize,
                                        0, UINT32_MAX, &v);
        version = given ? (uint32_t)v : 0;
    }
    if (version < 3000 || version > 7700)
    {
        return maniFbxFail(maniImportReasonVersion, 0, 0);
    }
    fbx->version = version;
    return maniFbxSuccess();
}

bool maniFbxAsciiNextNumber(const maniFbx* fbx, uint64_t* at, uint64_t end, uint64_t* startOut,
                            uint64_t* lengthOut)
{
    // The text was read whole at open: only numbers, commas, spaces and
    // comments lie between its numbers.
    Reader r = {.fbx = (maniFbx*)fbx, .at = *at, .firstComment = true};
    for (;;)
    {
        SkipSpace(&r);
        if (r.at >= end)
        {
            return false;
        }
        if (fbx->bytes[r.at] != ',')
        {
            break;
        }
        ++r.at;
    }
    Token t = NumberToken(&r);
    *startOut = t.start;
    *lengthOut = t.length;
    *at = r.at;
    return true;
}

maniImportResult maniFbxReadAscii(maniFbx* fbx)
{
    fbx->ascii = true;
    Reader r = {.fbx = fbx};
    maniImportResult result = ReadAll(&r);
    if (result.result != mani_success)
    {
        return result;
    }
    fbx->nodeCount = r.nodes;
    fbx->propertyCount = r.properties;
    fbx->nodes = maniAllocate(&fbx->allocator, r.nodes, sizeof(maniFbxNode), 8);
    fbx->properties = maniAllocate(&fbx->allocator, r.properties + 1, sizeof(maniFbxProperty), 8);
    if (fbx->nodes == NULL || fbx->properties == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    r.fill = true;
    result = ReadAll(&r);
    return result.result == mani_success ? ReadVersion(fbx, r.version) : result;
}
