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

Three are those models compressed by gltfpack (meshoptimizer, MIT,
commit `f0a684f1`), with `EXT_meshopt_compression` or
`KHR_meshopt_compression`; they stay under CC0:

| File | Made by |
| --- | --- |
| `InterpolationTest-meshopt.glb` and `InterpolationTest-meshopt.fallback.bin` | `gltfpack -i InterpolationTest.glb -o InterpolationTest-meshopt.glb -noq -cf -kn` |
| `InterpolationTest-khr.glb` | `gltfpack -i InterpolationTest.glb -o InterpolationTest-khr.glb -cc -ce khr -kn` |
| `SimpleSkin-khr.glb` | `gltfpack -i SimpleSkinEmbedded.gltf -o SimpleSkin-khr.glb -cc -ce khr -kn` |

`tools/gen_gltf_expect.py` reads them (a compressed file from its
uncompressed fallback) to write the values `test/test_gltf_fixtures.c`
checks; `test/test_gltf_compression.c` reads the compressed ones.

Three are SimpleSkin compressed by glTF Transform (MIT; the CLI
`@gltf-transform/cli` 4.4.2 with its core at 4.5.1, Google's Draco
encoder as `draco3dgltf`) with `KHR_draco_mesh_compression`; they stay
under CC0, and `test/test_gltf_draco_skin.c` reads them:

| File | Made by |
| --- | --- |
| `SimpleSkin-draco.glb` | `gltf-transform draco SimpleSkinEmbedded.gltf SimpleSkin-draco.glb` |
| `SimpleSkin-draco-q.glb` | `gltf-transform quantize SimpleSkinEmbedded.gltf q.glb --quantize-weight 8`, then `gltf-transform draco q.glb SimpleSkin-draco-q.glb` |
| `SimpleSkinSeam-draco.glb` | `tools/draco_skin_seam.py SimpleSkinEmbedded.gltf SimpleSkinSeam.gltf` (a texture coordinate added, vertex 4 split in two), then `gltf-transform draco SimpleSkinSeam.gltf SimpleSkinSeam-draco.glb` |

SHA-256 of each file:

```text
214ee56160a50dbf22543a1d66dbf860986e87f0efac3d89feac1359d0e6aeab  AnimatedMorphCube.glb
ed5a9b5a3864b04ddf67f1277527b13d31023369be537983239606bed101f25b  InterpolationTest-khr.glb
180d834e1c4bc2763d069bb7e1b974b7918140ba99984840676cf95aeaf9688a  InterpolationTest-meshopt.fallback.bin
24d0ee909172ef3bbed568a63f5d6f8980636b2d530bc3c9ef2bc56077dbb86c  InterpolationTest-meshopt.glb
a86eb331b4a083715e75fe19a1f747eac5692d5b9ff120f1eaa457c23ba72bca  InterpolationTest.glb
0ff029b843d95d3f162a67c84e2d04c7fc76218e53a0cc8c85ca2323d5f82758  SimpleMorph.gltf
8ed78771c1a01910b61f38aa492bd5daa05343f9b73b57f53237ed06ed2c3379  SimpleSkin-draco-q.glb
15f137c9682867e2c361461a723cd1807bd2ac636743eb35242e7f6d8e3ccfb9  SimpleSkin-draco.glb
082d3e17aee50dd8a42b7e97bc372b2d722a7d50237c7124d611a2f467ee6672  SimpleSkin-khr.glb
8d92e9888340eb98e82a65a1f8b037d9a1d9f09e5fd0b6ba6d40fd93700b3239  SimpleSkin/SimpleSkin.gltf
ff21b1bc3d0abbe53f7665060ec6c22d873b6e85a7669171a2b6b8ae01c39347  SimpleSkin/SimpleSkin_animation.bin
35f0ca6bc07976c0aadc163d4737758989cfcc9870a99800004c4478fe698b63  SimpleSkin/SimpleSkin_geometry.bin
5844ab221cd2ad367420a248f6387b46e5f8498604427730a37ee96b4d4ec599  SimpleSkin/SimpleSkin_inverseBindMatrices.bin
413bc8a8c0da673df767874e6f05f0b00c9261756a08586b9d555bd5535414b5  SimpleSkin/SimpleSkin_skinningData.bin
7d0c3f48d0510d101269cb4fdd3ee035eaada0e04809a0899e0b4cfe8c38e68f  SimpleSkinEmbedded.gltf
f4d6949e316229860dc278cc98a012c4675ad23d725b415eb2b2fb341a674a4e  SimpleSkinSeam-draco.glb
```
