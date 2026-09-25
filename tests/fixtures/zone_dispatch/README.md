# Zone benchmark TOP frames

Rendered from `zones/zone_open` (map version 2: 4 × 3 pickup grid at 0.6 m), seed 11, default goal (`sim.zone_arena.DEFAULT_GOAL`),
contact profile `local_contact_fine`, after the standard 1.3 s scene setup (no robot
commands). `start-top-west.jpg` is `cctv_top` (the approved TOP), `start-top-east.jpg`
is `cctv_top_east`. Used to check the RGB colour-blob box detector offline.

`wide-<camera>.jpg` are the four TOPs of `zones/zone_wide` (map version 1), seed 12,
goal `{"A":{"red":2},"B":{"cyan":2},"C":{"green":1,"yellow":1}}` with spares red 1 and cyan 1,
contact profile `local_contact_fine`, rendered after the standard setup plus 1.3 s of physics
with no robot commands.
