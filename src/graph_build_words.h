// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Writing a graph file's data words: a binary32's bits, a
// scalar (its parameter, then a literal's bits or 0), words and floats,
// each returning the word after it.

#ifndef MAUL_ANIM_SRC_GRAPH_BUILD_WORDS_H
#define MAUL_ANIM_SRC_GRAPH_BUILD_WORDS_H

#include "bytes.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline void maniGraphPutFloat(uint8_t* p, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(p, bits);
}

static inline uint32_t maniGraphPutScalar(uint8_t* data, uint32_t at, maniGraphScalar s)
{
    maniStoreU32(data + 4 * (size_t)at, s.parameter);
    maniGraphPutFloat(data + 4 * (size_t)at + 4, s.parameter == MANI_NO_PARAMETER ? s.value : 0.0f);
    return at + 2;
}

static inline uint32_t maniGraphPutWords(uint8_t* data, uint32_t at, const uint32_t* words,
                                         uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        maniStoreU32(data + 4 * (size_t)(at + k), words[k]);
    }
    return at + count;
}

static inline uint32_t maniGraphPutFloats(uint8_t* data, uint32_t at, const float* values,
                                          uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        maniGraphPutFloat(data + 4 * (size_t)(at + k), values[k]);
    }
    return at + count;
}

#endif // MAUL_ANIM_SRC_GRAPH_BUILD_WORDS_H
