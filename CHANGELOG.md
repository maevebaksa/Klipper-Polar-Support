# Changelog

## 0.5.0

- Queue native XY arcs through Klipper look-ahead with persistent radial, rotary
  and Z solvers. Compatible arc/arc and arc/line junctions can carry speed.
- Support helical Z motion and native origin passages with the required center
  stop, bed turn and optional retract/unretract.
- Support the built-in bed mesh, offsets and fade with bounded height error;
  XY remains circular.
- Repair small rounded-coordinate inconsistencies within `polar_arc_tolerance`
  while preserving requested endpoints. Reject larger inconsistencies.
- Add optional internal G1 extrusion fitting with `polar_auto_arcs`, including
  certified active-mesh fits. Bound geometric change with `polar_fit_tolerance`.
- Report fitted-arc and mesh-span counts, and correct live position for curves.
- Extend real Klippy and physical step-count regression coverage. Installation
  continues to preserve a clean Klipper checkout and unrelated user changes.

Non-XY arcs and unknown custom transforms retain segmentation. Exact center
passages still stop; this release does not provide splines or finite-jerk motion.

## 0.4.0

- Add optional center retraction with equal filament restoration.
- Use exact straight-segment motor derivative extrema.
- Add opt-in native XY circular interpolation, initially stopping at each arc.
- Fix full-circle winding and verify extrusion/coordinate overrides.

## 0.3.0

- Carry speed through ordinary corners using radial and angular velocity-change
  limits while preserving native Cartesian/extruder look-ahead constraints.
