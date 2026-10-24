// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The FBX build options. Output axis `to` is file axis
// `from`, negated when exactly one of the two is.

#include "fbx_options.h"

#include "basis.h"
#include "builder.h"
#include "fbx_document.h"

#include <math.h>

bool maniFbxOptionsValid(const maniFbxOptions* options)
{
    const maniFbxOptions* o = options;
    return maniBasisAxesValid(o->axes) && o->convertAxes <= 1 && o->foldPlacement <= 1 &&
           o->inheritHelpers <= 1 && o->unitMeters >= 0.0 && isfinite(o->unitMeters);
}

bool maniFbxUsable(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options,
                   bool inRange)
{
    if (builder == NULL)
    {
        return false;
    }
    if (fbx == NULL || options == NULL || !maniFbxOptionsValid(options) || !inRange)
    {
        maniBuilderMisuse(builder);
        return false;
    }
    return true;
}

maniBasis maniFbxBasis(const maniFbx* fbx, const maniFbxOptions* options)
{
    const maniFbxGlobalSettings* s = &fbx->settings;
    const maniFbxOptions* o = options;
    maniAxis axes[3] = {maniAxisX, maniAxisY, maniAxisZ};
    for (uint32_t k = 0; o->convertAxes != 0 && k < 3; ++k)
    {
        bool negative = (s->signs[k] < 0) != (o->axes[k] >= maniAxisNegativeX);
        axes[o->axes[k] % 3] = s->axes[k] + (negative ? maniAxisNegativeX : maniAxisX);
    }
    double unit = o->unitMeters > 0.0 ? s->unitScale * 0.01 / o->unitMeters : 1.0;
    return maniBasisMake(axes, (float)unit);
}
