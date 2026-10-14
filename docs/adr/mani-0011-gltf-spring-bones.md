# mani-0011. Spring bone sources from `VRMC_springBone`

Status: Accepted

## Context

Spring bones (hair, cloth, tails) are secondary motion the library
simulates itself (mani-0001's scope). VRM's `VRMC_springBone` 1.0
extension is the open format that carries them with glTF assets, and
fixes their parameters and rules. The runtime and its file come later;
the importer can be finished now against the format.

## Decision

- **Sources** (`maniSpringSetSource`, offline): colliders (a bone, a
  sphere at an offset or a capsule from an offset to a tail, a radius,
  in the bone's local space), named collider groups, and springs: a
  chain of joints (a bone, hit radius, stiffness, gravity power and
  direction, drag), the collider groups it collides with, and a center
  bone for its simulation space. VRM's parameters are kept as they are.
- **`maniGltfReadSprings`** reads the root `extensions.VRMC_springBone`
  (an empty set when absent) onto an opened skeleton: nodes found by
  bone name (mani-0009), schema defaults applied, offsets, tails and
  gravity directions moved into the output basis. The arrays belong to
  the document until the next read or close.
- **Refused:** a `specVersion` other than "1.0"; values outside the
  schema's ranges (drag in [0, 1], radii, stiffness, gravity power and
  hit radius not below 0); a chain whose joints are not each an
  ancestor of the next; a joint in two chains; a center that is not the
  chain's first joint or its ancestor, or that is another chain's joint
  or below one; a shape that is not exactly one of sphere and capsule;
  indices out of range; a node that is not a bone of the skeleton.
- VRM 0.x's `secondaryAnimation` is another format and is not read.

## Consequences

The spring bone runtime starts from sources whose every rule the
importer has already checked. Hosts reading VRM 0.x files convert them
with VRM's tools first.
