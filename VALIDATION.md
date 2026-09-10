# Validation record — 0.3.0

Validated September 10, 2026 against official Klipper commit
`f0892d82b0f1c1228454f09eb508eddde2250f4b`.

| Check | Result |
|---|---:|
| Solver, corner, installer, and update safeguard tests | 29 passed |
| Real Klippy Cartesian/physical-step endpoint assertions | 232 passed |
| Center rotations preserving radial, Z, and extruder positions | 146 passed |
| Sampled submitted trajectories within configured limits | 170,598 passed |
| Rejected requests with no partial motion | 4 passed |
| Pressure advance | Enabled, 0.04 s |
| Full Klipper Git status after repeat installation | Clean |

The integration used Klipper's real host motion planner, compiled C helpers,
step generation, step compression, and host-simulator MCU file-output mode. It
covered six center-crossing directions, 120 reversals with 14:3 gearing,
center holds, Z/E-only center moves, helical crossings, near-center offsets,
arcs, relative motion/extrusion, pressure advance, homing after rotation, and
atomic rejection of invalid moves.

## Corner handling

The new limits use the signed instantaneous motor velocities on both sides of
each junction. For a Cartesian path unit vector `(ux, uy, uz)` and junction
position `(x, y)` at radius `r`, the motor velocities per unit path speed are:

- Arm: `(x*ux + y*uy) / r`.
- Bed: `(x*uy - y*ux) / r**2`.

The junction path speed is capped by each configured velocity-change limit
divided by the absolute difference in those coefficients. Native Cartesian,
extruder, speed, acceleration, and minimum-cruise-ratio constraints remain in
force. Junctions at the numerical center always stop. These bounds permit
instantaneous velocity changes; they do not prove finite acceleration at sharp
corners. No geometric blending or path rounding is performed.

4,000 seeded random 3D junctions verify the motor velocity-change bounds.
The real Klippy benchmark additionally samples submitted trapezoids and checks
velocity jumps across contiguous junctions, not just candidate Move limits.

| Replay | Exact-stop time | New corner time | Reduction | Extrusion |
|---|---:|---:|---:|---|
| Two circular walls, reversal and center crossings | 40.81 s | 22.34 s | 45.3% | Identical generated E steps |

The synthetic benchmark uses 50 mm/s Cartesian/radial velocity, 1 rad/s bed
velocity, 1500 mm/s² radial acceleration, and 100 rad/s² bed acceleration.
The comparison changes square corner velocity from 0 to 5 mm/s, with the new
1 mm/s radial and 0.02 rad/s angular velocity-change caps. Thermal waits and
probing are excluded. Benchmark pressure advance is zero; the separate motion
regression exercises 0.04 s pressure advance. Machine-readable synthetic results
are in `tests/results/`.

Reproduce after installing the plugin into a supported Klipper checkout and
building Klipper's hostsimulator dictionary:

```bash
KLIPPER_PATH=/path/to/klipper PYTHONPATH=/path/to/klipper/klippy \
  python3 -m unittest discover -s tests -v
python3 tests/run_corner_benchmark.py --klipper /path/to/klipper \
  --dictionary /path/to/klipper/out/klipper.dict --output /tmp/polar-corners
```

The optional `--gcode /path/to/benchy.gcode` replays layer 0 from an Orca export
with `M117 Printing Layer 0/...` markers. The user-supplied G-code is not bundled.

These are software results. They do not establish physical motor torque,
endstop or nozzle-center calibration, backlash, adhesion, or center seam quality.
