# Draco streams

Draco meshes (bitstream 2.2) that Google's Draco encoder (commit
`d429de8`, Apache-2.0) wrote from synthetic meshes, with the tool
`draco_fixtures.cc` of the library's research (record mani-0021): a
tube of quads wrapped around a cylinder with a texture seam, the same
with holes, a second component and a non-manifold edge, a small one and
a large one, each with positions, normals, texture coordinates (not the
small one), joints as four unsigned shorts and weights as four floats.
The meshes are the project's own (MIT); the encoder's output carries no
licence of its own.

A name is the shape, the method (`seq` sequential, `eb` Edgebreaker),
the encoder's speed (`s0` to `s10`), and whether the attributes were
quantized (`q`: positions 14 bits, normals 10, texture coordinates 12,
generic 12) or stored raw. `test/test_gltf_draco.c` checks each one's
decoding against the FNV-1a 64 of Google's decoding.

SHA-256 of each file:

```text
66ad7588f7ef427df5b57710600ff54c7a361871fa68ee4ce9c485e489ab4254  holes-seq-s5-q.drc
73e41bfbd4e75294baa90969d4190af0c5fd1173e52e9051b6b3b4bb30a32197  large-seq-s2-q.drc
c52f005ddbf5c388725636bbd95c73b598bce23e9296dbc82a670a25b0ccad5c  small-seq-s0-q.drc
0f1f88cf98f5f76d80df17ee5ba8d068b95aa03890d3081bab56de86e687a461  small-seq-s8-raw.drc
4f024a261d32951ed6758c6c0863b20b9a3c510846363a02a3777253e75fb26f  tube-seq-s0-q.drc
5e7f0b092bbe3d2770ef19ea61ddda023fed1e8a3a58a135529585564382f197  tube-seq-s10-raw.drc
```
