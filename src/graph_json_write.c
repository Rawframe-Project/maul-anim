// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_json_write.h"

#include "decimal_format.h"

#include "maul-anim/base.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

void maniGraphJsonPut(maniGraphWriter* w, const char* bytes, size_t length)
{
    if (w->out != NULL)
    {
        memcpy(w->out + w->at, bytes, length);
    }
    w->at += length;
}

void maniGraphJsonPutText(maniGraphWriter* w, const char* text)
{
    maniGraphJsonPut(w, text, strlen(text));
}

void maniGraphJsonLine(maniGraphWriter* w, uint32_t depth)
{
    maniGraphJsonPutText(w, "\n");
    for (uint32_t k = 0; k < depth; ++k)
    {
        maniGraphJsonPutText(w, "  ");
    }
}

void maniGraphJsonPutString(maniGraphWriter* w, const char* bytes, size_t length)
{
    static const char kHex[] = "0123456789abcdef";
    maniGraphJsonPutText(w, "\"");
    for (size_t i = 0; i < length; ++i)
    {
        uint8_t c = (uint8_t)bytes[i];
        char escape[6] = {'\\', 'u', '0', '0', kHex[c >> 4], kHex[c & 15u]};
        const char* named = c == '"'    ? "\\\""
                            : c == '\\' ? "\\\\"
                            : c == '\n' ? "\\n"
                            : c == '\r' ? "\\r"
                            : c == '\t' ? "\\t"
                                        : NULL;
        if (named != NULL)
        {
            maniGraphJsonPutText(w, named);
        }
        else if (c < 0x20u)
        {
            maniGraphJsonPut(w, escape, sizeof escape);
        }
        else
        {
            maniGraphJsonPut(w, bytes + i, 1);
        }
    }
    maniGraphJsonPutText(w, "\"");
}

void maniGraphJsonPutKey(maniGraphWriter* w, const char* key)
{
    maniGraphJsonPutText(w, "\"");
    maniGraphJsonPutText(w, key);
    maniGraphJsonPutText(w, "\": ");
}

void maniGraphJsonPutFloat(maniGraphWriter* w, float value)
{
    char text[MANI_DECIMAL_MAX];
    maniGraphJsonPut(w, text, maniFormatBinary32(value, text));
}

void maniGraphJsonPutUnsigned(maniGraphWriter* w, uint64_t value)
{
    char reversed[20];
    uint32_t n = 0;
    do
    {
        reversed[n++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value != 0);
    while (n > 0)
    {
        maniGraphJsonPut(w, &reversed[--n], 1);
    }
}

void maniGraphJsonPutInt(maniGraphWriter* w, int32_t value)
{
    if (value < 0)
    {
        maniGraphJsonPutText(w, "-");
    }
    maniGraphJsonPutUnsigned(w, value < 0 ? (uint64_t)(-(int64_t)value) : (uint64_t)value);
}

void maniGraphJsonPutBool(maniGraphWriter* w, bool value)
{
    maniGraphJsonPutText(w, value ? "true" : "false");
}

void maniGraphJsonPutParameterName(maniGraphWriter* w, const maniGraphSource* source,
                                   uint32_t parameter)
{
    const maniGraphParameterSource* p = &source->parameters[parameter];
    maniGraphJsonPutString(w, p->name, p->nameLength);
}

void maniGraphJsonPutScalar(maniGraphWriter* w, const maniGraphSource* source, maniGraphScalar s)
{
    if (s.parameter == MANI_NO_PARAMETER)
    {
        maniGraphJsonPutFloat(w, s.value);
        return;
    }
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "parameter");
    maniGraphJsonPutParameterName(w, source, s.parameter);
    maniGraphJsonPutText(w, "}");
}

void maniGraphJsonPutScalars(maniGraphWriter* w, const maniGraphSource* source,
                             const maniGraphScalar* scalars, uint32_t count)
{
    maniGraphJsonPutText(w, "[");
    for (uint32_t k = 0; k < count; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutScalar(w, source, scalars[k]);
    }
    maniGraphJsonPutText(w, "]");
}
