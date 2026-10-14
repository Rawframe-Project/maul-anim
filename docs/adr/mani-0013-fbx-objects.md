# mani-0013. FBX objects, properties, connections and settings

Status: Accepted

## Context

An FBX scene is a flat list of objects (models, node attributes,
deformers, animation stacks, layers, curve nodes, curves) wired by a
list of connections, each object's properties in a `Properties70`
block whose missing entries default to per-kind templates in
`Definitions`. Version 6 files name objects instead of numbering them
and keep properties in `Properties60`. Every later part of the FBX
import walks this graph.

## Decision

- **Objects** are `Objects`' children. In 7.x an object's first value
  is its 64-bit id, its second its name and class (binary
  `Name\x00\x01Class`, ASCII `Class::Name`), its third its subclass; in
  6.x the first is `Class::Name`, which keys the object, and the second
  its subclass. An object without an id or name is skipped and counted,
  as ufbx skips them; an id given twice is refused (Property). Objects
  are found by id through a sorted index.
- **Connections** (`C` in 7.x, `Connect` in 6.x): `OO`, `OP`, `PO` and
  `PP`, child first, with the property names they give. Id 0 (6.x:
  `Model::Scene`) is the root. An entry of another type or naming no
  object is skipped and counted. Both ends are indexed, each object's
  connections in file order.
- **Properties:** an entry named twice is its later one; a name an
  object lacks comes from a `PropertyTemplate` of its kind whose class,
  without `Fbx`, is its subclass, or from any template of its kind for
  models, materials, animation stacks and layers, as ufbx chooses.
- **Settings:** `GlobalSettings`' right, up and front axes with signs
  (three different axes), centimetres a unit and the original, the
  time mode's frame rate as an exact rational (NTSC rates over 1001),
  or the custom rate, and the time span. Absent settings take FBX's
  defaults: X right, Y up, Z front, one centimetre, 30 a second.
  Settings out of range are refused (Property).

## Consequences

Skeletons, skins, curves and blend shapes are found by walking
connections from objects with properties that already include their
defaults, the same for 6.x and 7.x files. Resampled NTSC animation
lands on whole frames.
