# Documentation

## Using the library

- [Getting started](user/getting-started.md): requirements, building, installing, build options, the CPU check, the model the API follows.
- [Skeletons, clips and poses](user/skeletons-and-clips.md): building data, sampling, cursors, blending, the model pass, root motion.
- [Importing](user/importing.md): glTF and FBX, a memory budget for untrusted files, compression.
- [Animation graphs](user/graphs.md): building, binding, instances, the frame, node kinds, debugging.
- [Inverse kinematics and physics](user/ik-and-physics.md): the solvers, spring bones, ragdolls.
- [Skinning and morph targets](user/skinning.md): skins, influences, palettes, the skinning jobs.
- [Retargeting and mirroring](user/retargeting-and-mirroring.md): maps, baked clips, the JSON form.
- [Motion matching](user/motion-matching.md): databases, the search, learned models and training.
- [API reference](https://rawframe-project.github.io/maul-anim/api.html), generated from the headers.

## How it works

- [Overview](internals/overview.md): the library profile and how the parts fit.
- [Glossary](internals/glossary.md).
- [Determinism](internals/determinism.md), [Memory and threads](internals/memory-and-threads.md), [File formats](internals/file-formats.md).
- [Skeletons, poses, clips and compression](internals/clips-and-compression.md), [Graphs](internals/graphs.md), [Motion matching](internals/motion-matching.md), [Inverse kinematics](internals/ik.md), [Spring bones and the ragdoll seam](internals/physics-seam.md), [Retargeting and mirroring](internals/retargeting-and-mirroring.md), [Skinning and morph targets](internals/skinning.md).
- [glTF import](internals/gltf-import.md), [FBX import](internals/fbx-import.md).
- [Size and performance](internals/size-and-performance.md): the code size budget and the targets against other libraries.
- [Maul conventions](internals/maul-conventions.md): the rules every Maul library follows.

## Maintaining it

- [Development](operations/development.md): toolchain, running tests, generators, differentials, code size, fuzz findings.
- [Release](operations/release.md).
