# Draco streams

Draco meshes (bitstream 2.2) that Google's Draco encoder (commit
`d429de8`, Apache-2.0) wrote from synthetic meshes, with the tool
`draco_fixtures.cc` of the library's research (record mani-0021): a
tube of quads wrapped around a cylinder with a texture seam, the same
with holes, a second component and a non-manifold edge (`holes`, and
larger, `largeholes`; and with positions alone, `bare` and
`largebare`), a closed sphere of 24 rings and 32 segments (`sphere`),
thirty discs of 36 triangles around a centre (`fans`), a small one and a
large one: positions, and but for the bare ones normals,
texture coordinates (not the small one), joints as four unsigned shorts
and weights as four floats.
The meshes are the project's own (MIT); the encoder's output carries no
licence of its own.

A name is the shape, the method (`seq` sequential, `eb` Edgebreaker),
the encoder's speed (`s0` to `s10`), and whether the attributes were
quantized (`q`: positions 14 bits, normals 10, texture coordinates 12,
generic 12) or stored raw. `test/test_gltf_draco.c` checks each one's
decoding against the FNV-1a 64 of Google's decoding.

`small-eb-s5-q-multi.drc` is `small-eb-s5-q.drc` with byte 85, its
positions' prediction scheme, changed from 1 (parallelogram) to 2
(multi-parallelogram): Google's decoder still reads that scheme, its
encoder no longer writes it. Its expected decoding is Google's decoder's
of the changed file.

SHA-256 of each file:

```text
a058a14529cf9c33be4c8c9b91dcf1d237eae8c322936f5879c4a4fda49bd419  bare-eb-s5-q.drc
c17a11166b1e9ec3d4c2497530e36cc7dbe21e4b12fbcafc68d84a5b82f6065d  fans-eb-s0-q.drc
6552666f5d3c89d353d24ff039524db4ad053fbcd3563880ed5ccd53a90fb00c  holes-eb-s2-q.drc
66ad7588f7ef427df5b57710600ff54c7a361871fa68ee4ce9c485e489ab4254  holes-seq-s5-q.drc
258774bbfca6f97285404b382f90d5e97fe2bb5b98433f730fe7f5793ace9632  large-eb-s0-q.drc
73e41bfbd4e75294baa90969d4190af0c5fd1173e52e9051b6b3b4bb30a32197  large-seq-s2-q.drc
091f489cb243267d6aeac8b95a75a1237d57290b55bf39191fa494d2bbd0bb35  largebare-eb-s0-q.drc
6edd469fab99e96663637871211c082d2dac065e19920cbb0f8e35b67b1b30f1  largeholes-eb-s2-q.drc
0fda0fb6f3af61d64234167dc8ee0c5993c05894aa956ce9b74eb93850ec526d  small-eb-s10-q.drc
048bbbb386c1908afdd4c732bc249c89093598ddc088f9f4083b9847f83ce3f9  small-eb-s5-q-multi.drc
8dee036a3e65f754b7dd403908f5a84c9c3ed1901683e7817fc294c0f3134923  small-eb-s5-q.drc
c52f005ddbf5c388725636bbd95c73b598bce23e9296dbc82a670a25b0ccad5c  small-seq-s0-q.drc
0f1f88cf98f5f76d80df17ee5ba8d068b95aa03890d3081bab56de86e687a461  small-seq-s8-raw.drc
cd2f2846f37b8741328adfc0212a1897cc25640d36a3572f842c8f8fddd85f01  sphere-eb-s0-q.drc
c1d2f28033e99c03d594bcd05e437c9ff7c4cd6feb2815a645aed01c9b32f599  sphere-eb-s5-q.drc
974ff139c8c88b40385e6bb89974ce6627fd47e9569f12c66467bf8a7d2aaedc  tube-eb-s0-q.drc
4f024a261d32951ed6758c6c0863b20b9a3c510846363a02a3777253e75fb26f  tube-seq-s0-q.drc
5e7f0b092bbe3d2770ef19ea61ddda023fed1e8a3a58a135529585564382f197  tube-seq-s10-raw.drc
```
