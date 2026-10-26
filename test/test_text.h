// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Text a test writes piece by piece into a fixed buffer: each piece is
// kept only when it fits whole, and the text records whether every piece
// did, so a test checks that before it uses the text.

#ifndef MAUL_ANIM_TEST_TEXT_H
#define MAUL_ANIM_TEST_TEXT_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

typedef struct TestText
{
    char* data;
    size_t size;
    size_t length;
    bool fits;
} TestText;

static inline TestText TestTextIn(char* data, size_t size)
{
    if (size > 0)
    {
        data[0] = '\0';
    }
    return (TestText){data, size, 0, size > 0};
}

// Formats the piece as printf does; the compilers that can check the
// arguments against the format do.
#if defined(__GNUC__) || defined(__clang__)
#define TEST_TEXT_FORMAT __attribute__((format(printf, 2, 3)))
#else
#define TEST_TEXT_FORMAT
#endif
static inline void TestTextAppend(TestText* text, const char* format, ...) TEST_TEXT_FORMAT;

static inline void TestTextAppend(TestText* text, const char* format, ...)
{
    if (!text->fits)
    {
        return;
    }
    size_t room = text->size - text->length;
    va_list args;
    va_start(args, format);
    int n = vsnprintf(text->data + text->length, room, format, args);
    va_end(args);
    if (n < 0 || (size_t)n >= room)
    {
        // The piece did not fit: the text stays as it was.
        text->data[text->length] = '\0';
        text->fits = false;
        return;
    }
    text->length += (size_t)n;
}

#endif // MAUL_ANIM_TEST_TEXT_H
