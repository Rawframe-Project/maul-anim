# FBX fixtures

Scenes written for Maul Anim (MIT, as the repository), the inputs of
`test/test_fbx_fixtures.c`. The ASCII files are scenes of
the unit tests, kept whole:

| File | Scene | Covers |
| --- | --- | --- |
| `curves.fbx` | `test_fbx_curve.c`'s curves, each driving a user property of one model over four seconds | interpolations, tangents, extrapolations |
| `inherit.fbx` | `test_fbx_world.c` | inherit types under a non-uniform scale |
| `layers.fbx` | `test_fbx_anim.c` | layers, blend modes, weights, stack times |
| `legacy.fbx` | `test_fbx_legacy.c` | FBX 5.0 |
| `shapes.fbx` | `test_fbx_blend.c` | blend shapes, in-betweens |
| `skins.fbx` | `test_fbx_skin.c` | skins, binds and their fallbacks |
| `take.fbx` | `test_fbx_take.c` | FBX 6.1 takes |
| `units.fbx` | `test_fbx_units.c` | skeletons from skins and limbs |

`tools/fbx_fixtures.py` writes the binary forms from them with
`tools/fbx_binary.py`: `layers_7400.fbx` and `take_6100.fbx` (13-byte
records), `shapes_7500.fbx` and `curves_7500.fbx` (25-byte records,
deflated arrays), `skins_7400_be.fbx` (big-endian).

The values the test checks are ufbx's, written to
`test/fbx_fixture_expect.h` by the differential:

    fbx_differential --deviations tools/fbx-deviations.txt \
        --expect test/fbx_fixture_expect.h test/data/fbx/*.fbx

The scheduled differential job writes them again and fails when the
header differs.
