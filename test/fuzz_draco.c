// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco meshes (mani-0021): the input decoded as a stream of at most
// 2^16 points. Run as fuzz_draco <corpus dir>; the seeds are the
// streams of test/data/draco.

#include "arena.h"
#include "gltf_draco.h"

#include <stddef.h>
#include <stdint.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    maniAllocator allocator = {0};
    maniArena arena = maniArenaMake(&allocator);
    maniDracoMesh m;
    (void)maniDracoDecode(&arena, data, size, 1u << 16, &m);
    maniArenaRelease(&arena);
    return 0;
}
