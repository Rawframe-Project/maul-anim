# Glossary

Terms the code, the tests and the documentation use with one meaning.

- **Local / model:** a bone's transform relative to its parent (local) or to the skeleton's root frame (model). The model pass composes locals into model matrices.
- **Rest:** a bone's local transform in the skeleton file; what a channel without keys plays.
- **Identity (of a file):** a 64-bit hash that every file built against another records, so mismatched assets are refused. A skeleton's covers its bones, parents, rests and names.
- **View:** the small struct `maniOpen...` fills, pointing into the caller's file bytes.
- **Builder:** the offline owner object that turns sources into files (`maniBuilder`).
- **Source:** the struct or JSON text a builder reads.
- **Group:** eight consecutive bones of a pose, one eight-lane vector per component.
- **Difference:** a pose that holds changes from a basis rather than transforms; added onto poses by additive layers.
- **Basis:** what an additive clip's differences are taken from: the rest or the clip's first frame.
- **Sample:** one point of a clip's uniform grid; **sample rate** the grid's spacing.
- **Sub-track:** one channel (translation, rotation or scale) of one bone, or one float track, as the compressor classifies it: default, constant or animated.
- **Segment:** a run of about 16 samples that a compressed clip stores with its own ranges and bit rates.
- **Shell distance:** the radius of the points around each bone whose movement measures compression error.
- **Cursor:** caller memory that keeps decoded samples for forward play; **compact** when it keeps them without the group layout.
- **Root motion:** the move a clip's declared bone makes, taken out at build; a **move** is a translation and rotation in the character's frame.
- **Slot (graph):** a pose of the evaluation scratch that a node writes; also a named place for an asset in a graph (a clip slot, a database slot).
- **Binding:** a graph's slots matched to the host's assets, shared by every instance.
- **Instance:** one character's graph state in caller memory.
- **Update / evaluation:** advancing time (weights, playheads, events, root motion, machines) and writing the pose.
- **Weight / root weight:** a node's share of the output pose, and its share of the root motion, which differ below masks and additive layers.
- **Sync group / leader:** clips kept in phase; the leader moves by its own speed and the others follow its phase.
- **Marker:** a named time in a clip that sync groups align by.
- **Leading clip:** the most weighted clip beneath a machine state, whose position and events transitions may listen to.
- **Inertialization:** easing a jump by decaying the offset between the old output and the new input.
- **Feature / channel (matching):** a normalized number describing a frame; a channel is a kind of feature at a set of times.
- **Entry:** a span of a clip in a motion database, with its flags and tags.
- **Continuing frame:** where the current entry would play next; scored first in a search.
- **State (learned):** a learned model's features and latent values for one frame.
- **Effector:** a bone with a goal in full-body IK; the **part** is the top bone and every bone on a path to an effector.
- **Reference pose:** the pose two rigs are compared through when retargeting or mirroring; the rest unless one is given.
- **Partner:** the bone a mirror map pairs with another; itself when unpaired.
- **Placement:** the transform above a skeleton's root in an imported file, reported or folded into the root.
- **Helper bone:** a bone `<parent>#unscaled` the FBX importer adds where FBX's scale inheritance cannot be one local TRS.
- **Reference rate / step rate (springs):** the rate spring content was tuned at, and the rate the simulation steps at.
- **Backend:** the SIMD code a build uses: AVX2, NEON or scalar.
- **Misuse:** a call back in from the host's code, refused and counted on the object.
