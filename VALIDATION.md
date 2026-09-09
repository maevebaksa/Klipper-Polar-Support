# Validation record — 0.2.0

Validated September 9, 2026 against official Klipper commit
`f0892d82b0f1c1228454f09eb508eddde2250f4b`.

| Check | Result |
|---|---:|
| Solver, installer, and update safeguard tests | 19 passed |
| Real Klippy Cartesian/physical-step endpoint assertions | 232 passed |
| Center rotations preserving radial, Z, and extruder positions | 146 passed |
| Sampled submitted trajectories within configured limits | 171,103 passed |
| Rejected requests with no partial motion | 4 passed |
| Pressure advance | Enabled, 0.04 s |
| Tracked Klipper source after repeat installation | Clean |

The integration used Klipper's real host motion planner, compiled C helpers,
step generation, step compression, and host-simulator MCU file-output mode. It
covered six center-crossing directions, 120 reversals with 14:3 gearing,
center holds, Z/E-only center moves, helical crossings, near-center offsets,
arcs, relative motion/extrusion, pressure advance, homing after rotation, and
atomic rejection of invalid moves.

These are software results. They do not establish physical motor torque,
endstop or nozzle-center calibration, backlash, adhesion, or center seam quality.
