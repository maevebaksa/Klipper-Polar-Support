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

Version **0.4.0** adds optional center retraction, exact derivative bounds for
straight moves, and native XY circular arcs. The existing arm and bed corner
velocity-change limits remain available. Exact center crossings stop and reorient.

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

## Center retraction and native curves

Add these options to your existing `[printer]` section after installing v0.4.0:

```ini
center_retract_length: 0.3
center_retract_speed: 20
center_unretract_speed: 10
polar_native_arcs: True
```

The 0.3 mm retract is a starting value to tune, not a calibration result.
Omitting these options leaves retraction disabled (`0`) and native arcs disabled
(`False`). The acceleration calculation improvement needs no configuration.

Retraction occurs when extrusion arrives at the exact center and continues
after a bed turn. The sequence is stop, retract, rotate, unretract the same
length, resume. Net commanded E is unchanged. Length and speeds are physical
filament mm and mm/s, independent of M221 flow scaling. Normal extruder
temperature, distance, velocity, and acceleration checks remain enforced.
Travel-only turns and an already active G10 firmware retract do not add a
second automatic retract. A separate E move at the center clears the continuing
extrusion state. This is center-pause control, not automatic travel retraction
for the whole print.

Native arcs require **G2/G3 with I/J in the XY plane and constant Z**, using
Klipper's absolute-XYZ arc syntax. Slicer arc fitting or ArcWelder can supply
these commands. G1-only files remain polylines. G2/G3 handlers are loaded
automatically when enabled; an existing `[gcode_arcs]` section is accepted.

The C solver evaluates the circle directly from actual traveled arc length.
It maintains continuous winding across full turns and synchronizes extrusion
using that same arc length. M82/M83, G92, M220/M221, pressure advance, and normal
extruder limits are retained. Complete swept-radius bounds are checked before
motion. A standalone arc respects the single-move minimum-cruise cap.

**Each native arc stops at both ends.** This release does not blend lines into
arcs, blend adjacent arcs at speed, fit curves internally, or provide spline/
jerk-limited motion. Many short arcs can therefore be slower than G1 cornering.
Large arcs remove chord junctions within the arc but do not guarantee smoother
physical prints.

These cases retain Klipper's segmented arc path:

- An active mesh or other nonidentity move transform.
- Helical arcs, non-XY planes, or additional toolhead axes.
- Arcs passing through/within 0.000001 mm of the polar origin.
- Endpoints whose circle radii differ by more than 0.000001 mm, or tiny arcs.

An inactive built-in bed mesh with zero fade offset is allowed. The strict
endpoint tolerance can cause rounded slicer arcs to fall back; fallback counts
make that visible. `[gcode_arcs] resolution` affects fallback segmentation only.
Nozzle-center offsets, steps/mm, flow calibration, and physical tuning remain
your existing settings.

Kinematics status exposes `polar_center_retractions`,
`polar_native_arc_count`, and `polar_native_arc_fallbacks` (the arc counters
appear when native arcs are enabled).

## Acceleration calculations

Straight moves now use exact extrema over each segment for angular velocity,
angular curvature, radial curvature, and radial slope, including interior
angular-acceleration peaks. This removes some unnecessary slowing from the
previous conservative bounds. Motor acceleration combines the tangential
term (`position_derivative * path_acceleration`) with the curvature term
(`position_second_derivative * path_speed_squared`).

The planner still conservatively shares acceleration budgets between those
terms. Native offset circles use conservative analytic bounds, with a special
case for circles centered on the bed axis. This is not time-optimal trajectory
planning or a finite-jerk guarantee at corners.

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
- Bed mesh, probes, and third-party move transforms are not physically validated.
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
