// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF's meshopt streams: a 5-byte prefix chooses the mode,
// stride, count, whether version 1 is allowed and a filter, then the
// rest is decoded as a stream and, when valid attributes, filtered (a
// filter the stride allows). Run as fuzz_meshopt <corpus dir>; the seeds
// come from test_gltf_meshopt --seed <file>...

#include "gltf_meshopt.h"

#include <stddef.h>
#include <stdint.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static uint8_t s_out[1u << 16];

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size < 5)
    {
        return 0;
    }
    maniMeshoptMode mode = (maniMeshoptMode)(data[0] % 3u);
    uint32_t stride =
        mode == maniMeshoptAttributes ? 4u * (1u + data[1] % 64u) : ((data[1] & 1u) != 0 ? 4u : 2u);
    uint32_t count = (uint32_t)data[2] | (uint32_t)data[3] << 8;
    count -= mode == maniMeshoptTriangles ? count % 3u : 0u;
    if ((size_t)count * stride > sizeof s_out)
    {
        return 0;
    }
    bool v1 = (data[4] & 1u) != 0;
    if (!maniMeshoptDecode(s_out, count, stride, mode, v1, data + 5, size - 5) ||
        mode != maniMeshoptAttributes)
    {
        return 0;
    }
    maniMeshoptFilter filter = (maniMeshoptFilter)((data[4] >> 1) % 5u);
    bool allowed = filter == maniMeshoptNone || filter == maniMeshoptExponential ||
                   (filter == maniMeshoptQuaternion && stride == 8u) ||
                   ((filter == maniMeshoptOctahedral || filter == maniMeshoptColor) &&
                    (stride == 4u || stride == 8u));
    if (allowed)
    {
        maniMeshoptApplyFilter(s_out, count, stride, filter);
    }
    return 0;
}
