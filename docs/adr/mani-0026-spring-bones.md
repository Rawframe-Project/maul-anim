# mani-0026. Spring bones

Status: Accepted

## Context

Hair, tails and cloth strips sway behind a character's motion without a
physics engine. VRM 1.0 carries them with avatars as
`VRMC_springBone-1.0` (chains of joints, colliders, centers) and two
complete extensions, `VRMC_springBone_extended_collider-1.0` (inside
spheres and capsules, planes) and `VRMC_springBone_limit-1.0` (cone,
hinge and spherical limits); mani-0011 reads the base extension into
sources. VRM's reference integrates once a rendered frame: the inertia
carries the last frame's displacement whole while stiffness and gravity
scale with the frame's time, so the motion changes with the frame rate,
and content is tuned at the rate its author previewed it (60 Hz on a
common display; 50 Hz for UniVRM's fixed update). The library's results
are the same bits everywhere (mani-0001) and must not depend on how a
host splits time.

## Decision

- **Sources:** colliders gain the plane shape and inside spheres and
  capsules; joints gain a limit (none, cone, hinge, spherical: angle,
  or pitch and yaw, and a rotation); a set gains its reference rate (60
  Hz by default) and step rate (the reference rate by default), both up
  to `MANI_SPRING_MAX_RATE`.
- **The file** (container kind 11): the skeleton's identity, the rates,
  colliders, springs (their joints, the colliders of their groups in
  order without repeats, a center bone, a name) and joints, each a
  segment from its bone to the next joint's. `maniBuildSpringSet`
  refuses what VRM forbids (a chain whose joints are not each below the
  last, a joint in two chains, a center neither the first joint nor
  above it, or on or below another chain's joint) and every value out
  of its range, each refusal with its reason and items.
- **Fixed steps:** each joint's drag d, stiffness and gravity, given per
  frame of the reference rate f0, are converted at build to a step of h
  seconds: the drag 1 - (1 - d)^(h f0), stiffness and gravity times
  h (dh / d) (h² f0 for d = 0). This makes the free motion (inertia,
  drag, stiffness, gravity) the same at any step, the solution of the
  equation the reference integrates exactly over each of its frames; at
  the default step, the reference's own.
- **Limits** keep their angles clamped as the extension says and a
  frame per joint built from the head's rest direction to its tail (the
  turn from +y, 180° about x for exactly -y) and the limit's rotation.
- `maniOpenSpringSet` opens a file in place, every record checked;
  the skeleton the set was built for is checked where a state is made.

## Consequences

A set's motion is decided once at build from its source and rates, and
plays the same on hosts of any frame rate; content tuned at another
rate than 60 Hz says so in its reference rate. The runtime (the state,
the steps with colliders, limits and centers, the output between
steps) and a graph node follow in this record as they are built.

Opening is fuzzed; every check is tested against a file changed at its
byte; the conversion is tested against its binary64 formula.
