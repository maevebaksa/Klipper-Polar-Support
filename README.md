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

Then edit the existing `[printer]` section:

```ini
[printer]
kinematics: polar_center
max_velocity: 40
max_accel: 300
max_z_velocity: 3
max_z_accel: 30
square_corner_velocity: 0
max_angular_velocity: 1.0
max_angular_accel: 2.0
max_radial_velocity: 40
max_radial_accel: 300
polar_slow_radius: 5.0
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
- Bounds angular and radial velocity and acceleration.
- Preserves the unwrapped angular step-grid phase for noninteger gearing.
- Validates the complete request before queueing the first generated movement.

The numerical center snap tolerance is `0.0000001 mm`; it is not an excluded
print region. With the supplied 1 rad/s and 2 rad/s² commissioning limits, a
180-degree center turn takes about 3.64 seconds.

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

The standalone implementation passed 232 real Klippy endpoint assertions,
146 center-rotation checks, and 171,103 trajectory limit checks. CI also tests
geometry, clean-checkout installation, and update safeguards. See
[`VALIDATION.md`](VALIDATION.md) for the tested Klipper version and results.

Distributed under GPL-3.0. Klipper's original Polar solver and kinematics are
copyright Kevin O'Connor and Klipper contributors.
