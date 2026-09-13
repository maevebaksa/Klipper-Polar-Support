# Validation record — 0.5.0

Tested against official Klipper commit
`f0892d82b0f1c1228454f09eb508eddde2250f4b` using real Klippy, compiled C helpers,
step generation/compression and hostsimulator MCU file-output mode. No printer
is contacted. Software tests do not establish physical print quality.

| Check | Result |
|---|---:|
| Unit, geometry, installer and update tests | 46 tests |
| Default feature-off endpoint assertions | 232 |
| Extended logical and motor endpoint assertions | 291 |
| Native curve spans checked against motor limits | 300 |
| Native junctions carrying nonzero speed | 247 |
| Automatically fitted G1 arcs, including active mesh | 120 |
| Explicit mesh arc spans | 121 |
| Center retractions with equal restoration | 135 |
| Center rotations with stationary radial/Z/E motors | 152 |
| Physical in-curve motor checks | 51 batches |
| Pressure advance in motion regressions | 0.04 s |
| Repeat installation and final full Klipper Git status | Clean |

These are synthetic regression counts, not a speed or surface-quality claim.
The machine-readable extended result is `tests/results/v05-extended.json`.

## What is checked

The regression retains the existing homing, near-origin lines, exact center
crossings, extruder validation, pressure advance and relative/absolute extrusion
checks. New cases cover:

- Full/partial CW and CCW arcs in all quadrants, changing endpoints and G92 XY/E
  offsets, M220/M221 overrides, and continuous queued arc sequences.
- Arc-to-line tangent junctions and preservation of extruder junction limits.
- Helical XYZ/E motion with 3D path length and Z velocity/acceleration limits.
- Native arcs through the origin, stopped reorientation, and center retraction.
- Bounded repair of rounded slicer endpoints, keeping both endpoints exact.
- Automatic G1 fitting with and without an active mesh, bounded chord deviation,
  unchanged endpoints and net E, and rejection of excessive geometric changes.
- Active mesh correction, interpolation cells, fade transitions, fade target and
  tool offset. Unit tests compare interpolated span heights against the mesh.
- Invalid complete-arc radial bounds and unsafe extrusion rejected atomically.
- Recorded MCU step counts sampled *inside* curves against expected radial,
  angular and Z positions, as well as endpoint and final extrusion checks.

Submitted trapezoids are sampled with independent analytic derivative formulas.
Junction tests compare actual incoming/outgoing tangents and motor velocities.
The persistent C timeline preserves full-circle winding and is pruned only after
step generation while retaining live-position history. The live position query
uses actual curve geometry; raw trapq dumps remain linear skeleton diagnostics.

## Mathematical bounds

Straight-segment bounds retain the v0.4 exact derivative extrema. For native
curves, acceleration includes tangential and centripetal terms. Centered and
origin-tangent circles have specialized bounds; other offset circles use
conservative bounds based on their minimum radius. Native junctions use curve
endpoint tangents with Klipper look-ahead and motor velocity-change limits.
This is not a finite-jerk planner at sharp corners.

For mesh correction with Lipschitz slope bound L over XY arc-length interval h,
linear interpolation error is at most L*h/2. The bound is computed from the
actual dense mesh table, clamping, fade target and fade slope, so it remains
valid across cell and fade boundaries. XY remains circular in every span.

G1 fitting preserves vertices and bounds the sagitta between each source chord
and its fitted arc. Active-mesh fits must also pass the mesh-height error bound.
Nominally constant Z and matching extrusion distribution are required.

## Reproduce

Install the plugin in the supported Klipper checkout and build its hostsimulator
dictionary, then run:

```bash
KLIPPER_PATH=/path/to/klipper PYTHONPATH=/path/to/klipper/klippy \
  python3 -m unittest discover -s tests -v
python3 tests/run_integration.py --features --extended --klipper /path/to/klipper \
  --dictionary /path/to/klipper/out/klipper.dict --output /tmp/polar-extended
```

GitHub Actions additionally runs the default regression, feature regression and
existing corner benchmark. Historical v0.3 JSON files in `tests/results/` remain
historical; they are not v0.5 timing claims. User printer logs and G-code are not
bundled.

Non-XY planes and unknown custom transforms retain upstream segmentation.
Mechanical center stops remain necessary. Physical torque, backlash, nozzle
calibration, adhesion and extrusion tuning require printer testing.
