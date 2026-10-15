# mani-0017. FBX blend shapes and 6.x takes

Status: Accepted

## Context

FBX weighs blend shapes per channel: a channel's DeformPercent drives
one or more shapes, in-between shapes taking over at their target
weights (FullWeights), negative targets below zero. Files up to 7.1
may instead hold shapes as children of a mesh, weighted by the mesh's
property of the shape's name. Files before 7.0 have no stacks or curve
nodes at all: their animation is a list of takes, each naming models
and nesting channels whose keys are one flat list of times, values and
mode letters, and their models split properties between themselves and
an attribute (mesh, bone, camera) by name. The clip format's morph
weights are one float per target (mani-0012's glTF morph tracks).

## Decision

- **Blend shapes** become one float track per shape, named
  `mesh/shape`, holding the shape's effective weight at every sample as
  ufbx computes it: targets from FullWeights (all 1 in files of
  Blender's binary exporter, which writes per-vertex weights there),
  stably sorted, the two shapes around the channel weight sharing it
  linearly. A channel's tracks are kept when its mesh is a bone of the
  skeleton or skinned within it, and the stack drives its weight.
- **Before 7.2**, a mesh's Shape children (a 6.x Mesh model, a 7.0 or
  7.1 Geometry) are channels of one shape each, weighted by the mesh's
  number property of the shape's name.
- **6.x takes** are stacks with one base layer. Their channels bind to
  the models their keys name; `Transform`'s T, R and S are the Lcl
  properties; a component's Default is its static value; key lists are
  read as ufbx reads them, every mode, slope mode, weight mode and key
  version included. A channel on a property the model does not keep
  (neither a node property nor user-defined) drives its attribute and
  makes no float track.
- What ufbx fails on is refused; times that are not whole ticks or go
  back, and values that are not finite, are refused as for 7.x curves.
- Files before 6.0 (versions 3000 to 5999) are read, as ufbx reads
  them, into the same objects and connections: each top-level Model is
  an object keyed by its name, its `Type` (LimbNode, Light, Camera) its
  subclass, its `Children` list its children; a mesh's `Link` children
  are the clusters of one skin per mesh. Such a model has no property
  entries: the last channel driving one of its Lcl properties in file
  order, a take's or its own, gives that property's rest value. A
  `Settings` FrameRate, and a 6.x file's `Version5` one, is the frame
  rate where GlobalSettings gives none.
- In those files the takes are followed by one more stack,
  `(internal)`, of the top-level models' own channels, when there are
  any; its range is its keys' span. A channel name ending in
  ` (Shape)` names the shape without that suffix.

## Consequences

FBX morph weights reach the runtime as glTF's do, one weight per
target, so morph targets need no second weighting model; in-betweens
resolve at import. Old files import with the same clips ufbx evaluates,
which keeps the differential meaningful across versions.
