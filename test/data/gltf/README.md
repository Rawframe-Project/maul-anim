# glTF sample models

Models from the Khronos glTF Sample Assets repository,
`https://github.com/KhronosGroup/glTF-Sample-Assets`, commit
`edc7c9e67c639d230715049ee31f9a96a6babbbe`, kept exactly as published.
Each is dedicated to the public domain under CC0 1.0,
`https://creativecommons.org/publicdomain/zero/1.0/`, as its
`metadata.json` in that repository states:

| File | Model and variant | Author |
| --- | --- | --- |
| `InterpolationTest.glb` | InterpolationTest, glTF-Binary | Khronos |
| `AnimatedMorphCube.glb` | AnimatedMorphCube, glTF-Binary | Microsoft |
| `SimpleMorph.gltf` | SimpleMorph, glTF-Embedded | Marco Hutter |
| `SimpleSkinEmbedded.gltf` | SimpleSkin, glTF-Embedded (`SimpleSkin.gltf` there) | Marco Hutter |
| `SimpleSkin/` | SimpleSkin, glTF | Marco Hutter |

`tools/gen_gltf_expect.py` reads them to write the values
`test/test_gltf_fixtures.c` checks.

SHA-256 of each file:

```text
214ee56160a50dbf22543a1d66dbf860986e87f0efac3d89feac1359d0e6aeab  AnimatedMorphCube.glb
a86eb331b4a083715e75fe19a1f747eac5692d5b9ff120f1eaa457c23ba72bca  InterpolationTest.glb
0ff029b843d95d3f162a67c84e2d04c7fc76218e53a0cc8c85ca2323d5f82758  SimpleMorph.gltf
7d0c3f48d0510d101269cb4fdd3ee035eaada0e04809a0899e0b4cfe8c38e68f  SimpleSkinEmbedded.gltf
ff21b1bc3d0abbe53f7665060ec6c22d873b6e85a7669171a2b6b8ae01c39347  SimpleSkin/SimpleSkin_animation.bin
35f0ca6bc07976c0aadc163d4737758989cfcc9870a99800004c4478fe698b63  SimpleSkin/SimpleSkin_geometry.bin
8d92e9888340eb98e82a65a1f8b037d9a1d9f09e5fd0b6ba6d40fd93700b3239  SimpleSkin/SimpleSkin.gltf
5844ab221cd2ad367420a248f6387b46e5f8498604427730a37ee96b4d4ec599  SimpleSkin/SimpleSkin_inverseBindMatrices.bin
413bc8a8c0da673df767874e6f05f0b00c9261756a08586b9d555bd5535414b5  SimpleSkin/SimpleSkin_skinningData.bin
```
