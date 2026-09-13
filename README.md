# Klipper Polar Support

Experimental `polar_center` kinematics for Polar3D-style printers. Ordinary
Cartesian G-code can cross XY `(0,0)` without a slicer postprocessor. At an
exact center crossing, Klipper stops at the center, turns the rotary bed through
the shortest angle, and continues the radial move. Stock `kinematics: polar`
remains available.

The plugin keeps the main Klipper Git checkout clean. Its two Python kinematics files
are symlinked into Klipper and excluded through `.git/info/exclude`; its C solver
is compiled as a separate shared library under this repository. It does not
replace the tracked `kin_polar.c` or `chelper/__init__.py` files. The installer
also restores the recognized previous Polar patch while preserving backups.

Version **0.5.0** queues native curves through Klipper look-ahead, including
helical Z and active bed mesh. It adds bounded repair of rounded arc coordinates
and optional fitting of ordinary G1 extrusion segments. Exact center passages
use the same stopped reorientation and optional retraction as straight lines.

## Install

Run as the normal printer user while the printer is idle:

```bash
cd ~ && \
git clone https://github.com/maevebaksa/Klipper-Polar-Support.git && \
cd ~/Klipper-Polar-Support && \
chmod +x install.sh && \
./install.sh
```

The installer builds the small host-side C solver, installs the symlinks, restarts
Klipper and Moonraker, and adds **polar-support** to Moonraker's Update Manager.
An idle-only timer checks every six hours. When an update is available,
Moonraker pulls this repository, rebuilds/relinks the plugin, and restarts
Klipper. It does not update Klipper itself or flash the MCU.

## Update

With the printer idle and heater targets off, run:

```bash
cd ~/Klipper-Polar-Support && \
python3 auto_update.py --check-idle && \
git pull --ff-only && \
bash install.sh
```

Or use **polar-support** in Moonraker's Update Manager if it is already installed.
The installer restores only recognized legacy Polar changes, backs them up
outside the Klipper checkout, and excludes only the two managed Python symlinks.
It does not reset the repository, hide tracked edits, or delete other plugins.
Unrecognized edits to the old solver files stop installation before Klipper is
stopped; other unrelated changes are retained and listed in the installer output.

Then edit the existing `[printer]` section:

```ini
[printer]
kinematics: polar_center
max_velocity: 40
max_accel: 300
max_z_velocity: 3
max_z_accel: 30
square_corner_velocity: 1
max_angular_velocity: 1.0
max_angular_accel: 2.0
max_radial_velocity: 40
max_radial_accel: 300
polar_slow_radius: 5.0
max_radial_velocity_change: 1.0
max_angular_velocity_change: 0.02
```

Retain your verified `[stepper_arm]`, `[stepper_bed]`, `[stepper_z]`, pins,
rotation distances, calibrated home positions, and calibrated bed `gear_ratio`.
The arm must use `position_min: 0`. A reference file is provided at
`config/polar3d-printrboard-center.cfg`; do not overwrite a working printer
configuration with it.

## What it does

- Splits exact center crossings while interpolating Z and E at the same point.
- Stops extrusion during center reorientation, with an optional retract and equal unretract.
- Preserves the incoming bed angle for center dwells and Z/E-only center moves.
- Subdivides near-center lines without geometrically detouring them.
- Bounds angular and radial velocity and acceleration along each straight move.
- Limits arm and bed velocity changes at corners while retaining native Klipper
  Cartesian look-ahead and extruder junction limits.
- Preserves the unwrapped angular step-grid phase for noninteger gearing.
- Preflights every generated line leg, center retract pair, and complete native arc
  before queueing that request. Segmented fallback arcs retain upstream behavior.

The numerical center snap tolerance is `0.0000001 mm`; it is not an excluded
print region. With the supplied 1 rad/s and 2 rad/s² commissioning limits, a
180-degree center turn takes about 3.64 seconds.

## Corner tuning

Existing installations automatically get the new limits: **1 mm/s** maximum
instantaneous arm velocity change and **0.02 rad/s** maximum instantaneous bed
angular velocity change. These defaults are capped at the corresponding configured
motor velocity limits if those are lower. An existing `square_corner_velocity: 0`
still forces stops; change it to `1` to start testing. If the setting was omitted,
Klipper's existing default of `5` is retained. The installer does not edit printer.cfg.

The two new `max_*_velocity_change` settings bound the difference between motor
velocities before and after each corner. They are not acceleration or jerk limits.
Higher values allow faster corners but can increase vibration. The angular cap
naturally tightens Cartesian corner speeds near the center. The existing speed
and acceleration limits continue to apply along each move. This planner permits
bounded instantaneous velocity changes; it does not round the path or provide
mathematically continuous acceleration through a sharp corner.

To compare the old stop-at-corners behavior during an idle test, use
`SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=0`; restore the configured corner speed
afterward. Center reorientation stops remain enabled at all settings. Segmented G2/G3 arcs
benefit from the same corner limits; native arcs are an optional separate path.

## Native curves and center retraction

These options belong in your existing `[printer]` section:

```ini
polar_native_arcs: True
center_retract_length: 0.3
center_retract_speed: 20
center_unretract_speed: 10

# Optional fitting of ordinary G1 extrusion paths:
polar_auto_arcs: True
polar_fit_tolerance: 0.01

# Maximum nominal-circle repair and mesh-height approximation, in mm:
polar_arc_tolerance: 0.02
polar_mesh_tolerance: 0.005
```

Retraction and native arcs remain disabled unless enabled explicitly. G1 fitting
also defaults to disabled. Curve tolerance settings are read when native arcs
are enabled. The 0.3 mm retract is a starting value to tune.

### Continuous arc motion

XY G2/G3 commands with I/J now enter the normal Klipper look-ahead queue. They
can carry speed through tangent arc-to-arc and arc-to-line junctions. Junction
limits use the actual curve tangents, preserving Cartesian and extruder limits
and the existing radial/angular velocity-change caps. Real corners and reversals
can still require slowing or stopping. M400, dwells and other flushes still stop.

Persistent C solvers evaluate the curve directly without swapping steppers or
forcing a stop for every command. Winding remains continuous through full
circles. Helical arcs are supported: their linear Z component contributes to
path length, extrusion ratio, Z speed and Z acceleration limits. The live motion
position is corrected to the curve. Raw trapq diagnostic streams still contain
linear skeleton entries; use stepper data or the live-position query to inspect
actual curve motion.

### Rounded arc coordinates

Small endpoint inconsistencies can be repaired by moving the circle center to
the perpendicular chord bisector. Both requested endpoints remain exact. The
center displacement plus radius change is bounded by `polar_arc_tolerance`
(default 0.02 mm). This bounds the change of corresponding points on the nominal
circle. Larger or ill-conditioned inconsistencies are rejected before motion;
they are not silently accepted as a badly distorted curve.

### Bed mesh

The built-in bed mesh is supported with its offsets, fade and fade target.
XY stays circular. Mesh height is approximated by linear-Z arc spans, which
remain connected through look-ahead. A global slope bound on the actual dense
mesh table and fade function determines the span size so height approximation
error stays within `polar_mesh_tolerance` (default 0.005 mm). A request requiring
more than 4096 spans is rejected; inspect the mesh or adjust the tolerance.

A curved mesh path can still be slower because its Z speed and acceleration
limits apply. This approximates the interpolated mesh, not the physical bed.

### Ordinary G1 files

With `polar_auto_arcs: True`, compatible pairs of queued G1 extrusion moves can
be fitted to circular arcs inside Klipper. A postprocessor is no longer required
for those paths. The fitted arc passes through the original vertices; deviation
from the original chords is bounded by `polar_fit_tolerance` (default 0.01 mm).
This is an explicit geometric tolerance: details smaller than it can be rounded.

Fitting is deliberately selective: constant nominal Z, matching requested
speed, positive extrusion and matching extrusion distribution. Built-in bed
mesh is supported when a height-error bound can certify the fit; custom
transforms are preserved. Moves near the center, travel moves, extra-axis moves
and moves with callbacks are preserved. Existing G2/G3 commands are never
refitted. Slicer arc fitting or ArcWelder remains useful for producing longer
arc commands.

### Center passages

An XY arc through the origin is split exactly at the origin. It stops, optionally
retracts, turns the bed, restores the same filament length, then continues as a
curve. That stop is a mechanical requirement of a nonnegative-radius shuttle;
removing it would require a different mechanism or a changed toolpath. No
printable center exclusion is added.

Automatic retraction applies when extrusion arrives at the center and continues
after the turn. Net E is unchanged. Length and speeds are physical filament mm
and mm/s, independent of M221. Travel-only turns and an active G10 firmware
retract do not add a second retract. A separate E move at the center clears the
continuing-extrusion state. This is center-pause control, not general travel
retraction for an entire print.

### Remaining scope

Native interpolation supports XY-plane arcs, including helical Z. Non-XY planes,
tiny arcs and unknown third-party move transforms retain Klipper segmentation.
Absolute XYZ/IJ arc syntax follows Klipper. Splines, general sharp-corner rounding and finite-jerk trajectories are not implemented.
Stock cornering still permits bounded instantaneous motor velocity changes.

Kinematics status exposes `polar_center_retractions`, `polar_native_arc_count`,
`polar_native_arc_fallbacks`, `polar_fitted_arcs`, and `polar_mesh_arc_spans`.
Counts reflect queued operations. Enabling native curves automatically loads
the G2/G3 handlers; an existing `[gcode_arcs]` section is accepted.

Nozzle offsets, homing, steps/mm and extrusion calibration remain your settings.

## Acceleration calculations

Straight moves use exact extrema over each segment for angular velocity,
angular curvature, radial curvature and radial slope. Motor acceleration
combines the tangential term with the curvature term proportional to speed
squared. Budgets remain conservative. Native circles centered on or tangent to
the polar origin have specialized analytic bounds. Other offset circles use
conservative bounds based on swept minimum radius. These are not time-optimal
or jerk-limited trajectories.

## Initial dry run

Clear the bed and test without heat or extrusion:

```gcode
G28
G90
G1 Z10 F120
G1 X30 Y0 F300
G1 X-30 Y0 F300
M400
```

The nozzle should stop at the center while the bed turns 180 degrees, then
continue. Next test a center hold/departure and an offset pass:

```gcode
G1 X0 Y0 F300
G1 Z12 F120
G1 X0 Y30 F300
G1 X30 Y0.5 F300
G1 X-30 Y0.5 F300
M400
```

## Safety and compatibility

This has passed software motion regression tests but has not been physically
validated on a Polar3D printer. The supplied speeds are conservative starting
values, not tuned motor limits. Verify the true rotation center and homing
calibration before printing.

- `[input_shaper]` is rejected because its nonlinear solver wrapping is not
  supported. Pressure advance is supported by the test suite.
- Exact crossings physically require the nonnegative-radius shuttle to stop and
  reverse; center oozing or a seam remains possible.
- Bed mesh has software regression coverage; probes and third-party transforms
  are not physically validated.
- Signed negative radial travel and angular-index homing are not added.
- A compiler is required on the host, as with Klipper's normal C helper build.
- Compatibility checks reject untested changes to Klipper's motion interfaces.
  Other Klipper updates are permitted when those interfaces remain unchanged.
  The solver is built during installation or plugin updates, never every reboot.

Check status with:

```bash
git -C ~/klipper status --short
systemctl status polar-support-update.timer --no-pager
journalctl -u polar-support.service -n 50 --no-pager
```

The first command should be empty unless another local Klipper modification
exists. Automatic installation only occurs when Klipper is ready, the print
state is idle/complete/cancelled, `idle_timeout` is Idle, and every heater target
is zero.

## Validation and license

The prior v0.3.0 release passed 232 real Klippy endpoint assertions, 146 center-rotation checks,
170,598 trajectory samples, and the corner-speed and extrusion benchmark.
A synthetic curved-wall test took 45.3% less planned motion time with
the same generated extrusion.
These are software results, not promises of print time or surface quality.
CI also tests geometry, clean-checkout installation, and update safeguards. See
[`VALIDATION.md`](VALIDATION.md) for the tested Klipper version and results.

Distributed under GPL-3.0. Klipper's original Polar solver and kinematics are
copyright Kevin O'Connor and Klipper contributors.
