# Klipper Polar Support

Experimental `polar_center` kinematics for Polar3D-style printers. Ordinary
Cartesian G-code can cross XY `(0,0)` without a slicer postprocessor. At an
exact center crossing, Klipper stops at the center, turns the rotary bed through
the shortest angle, and continues the radial move. Stock `kinematics: polar`
remains available.

The plugin keeps the main Klipper Git checkout clean. Its Python kinematics file
is symlinked into Klipper and excluded through `.git/info/exclude`; its C solver
is compiled as a separate shared library under this repository. It does not
replace the tracked `kin_polar.c` or `chelper/__init__.py` files. The installer
also restores the recognized previous Polar patch while preserving backups.

Version **0.3.0** carries speed through ordinary corners using separate arm and
bed velocity-change limits. Exact center crossings still stop and reorient.

## Install

Run as the normal printer user while the printer is idle:

```bash
cd ~ && \
git clone https://github.com/maevebaksa/Klipper-Polar-Support.git && \
cd ~/Klipper-Polar-Support && \
chmod +x install.sh && \
./install.sh
```

The installer builds the small host-side C solver, installs the symlink, restarts
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
outside the Klipper checkout, and excludes only the managed Python symlink.
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
- Stops extrusion at the center while the bed performs its bounded reorientation.
- Preserves the incoming bed angle for center dwells and Z/E-only center moves.
- Subdivides near-center lines without geometrically detouring them.
- Bounds angular and radial velocity and acceleration along each straight move.
- Limits arm and bed velocity changes at corners while retaining native Klipper
  Cartesian look-ahead and extruder junction limits.
- Preserves the unwrapped angular step-grid phase for noninteger gearing.
- Validates the complete request before queueing the first generated movement.

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
afterward. Center reorientation stops remain enabled at all settings. G2/G3 arcs
still use Klipper's line segmentation and benefit from the same corner limits.

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

Version 0.3.0 passed 232 real Klippy endpoint assertions, 146 center-rotation checks,
170,598 trajectory samples, and the corner-speed and extrusion benchmark.
A synthetic curved-wall test took 45.3% less planned motion time with
the same generated extrusion.
These are software results, not promises of print time or surface quality.
CI also tests geometry, clean-checkout installation, and update safeguards. See
[`VALIDATION.md`](VALIDATION.md) for the tested Klipper version and results.

Distributed under GPL-3.0. Klipper's original Polar solver and kinematics are
copyright Kevin O'Connor and Klipper contributors.
