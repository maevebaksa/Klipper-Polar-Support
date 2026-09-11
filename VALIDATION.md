# Validation record — 0.4.0

Tested against official Klipper commit
`f0892d82b0f1c1228454f09eb508eddde2250f4b`. GitHub Actions runs the unit,
installer, default motion, corner benchmark, and optional feature tests.

The suite uses real Klippy, compiled C helpers, step generation/compression,
and a hostsimulator MCU dictionary in file-output mode. It contacts no printer.
See the checks on [PR #2](https://github.com/maevebaksa/Klipper-Polar-Support/pull/2)
for the result tied to each commit.

## New regression coverage

- 38 unit/installer/update tests, including sampled exact line derivatives,
  swept-arc radius extrema, continuous C solver winding, and transform gates.
- Default feature-off motion regression with 232 logical and motor endpoint
  assertions, 146 stationary center turns, and four atomic invalid-move checks.
- Optional feature regression: repeated center retraction with equal restoration,
  full and partial CW/CCW arcs, offset circles in all quadrants, and line/arc
  transitions. It verifies physical E steps with 0.04 s pressure advance,
  M82/M83, M220/M221, and G92 E/XY offsets.
- Samples submitted arc trapezoids against independent analytic radial/angular
  velocity and acceleration formulas, plus Cartesian centripetal acceleration.
- Rejects complete native arcs whose interior exceeds the radial limit and
  unsafe extrusion, without partial motor or commanded-position changes.
- Origin-touching and helical arcs use the fallback path. Unit tests reject
  active mesh/fade/custom transforms from native execution.
- Repeated installation leaves the full Klipper Git status clean. Both managed
  Python links are excluded; tracked and unrelated user changes remain protected.

## Acceleration calculation

For a straight segment, let `h` be perpendicular distance to the XY line,
`a` signed distance along that line from its closest point, and `q` the
ratio of XY to XYZ path length. The exact derivatives are:

- `|theta'| = q*h/(h*h+a*a)`
- `|theta''| = 2*q*q*h*abs(a)/(h*h+a*a)^2`
- `r'' = q*q*h*h/(h*h+a*a)^(3/2)`
- `|r'| = q*abs(a)/sqrt(h*h+a*a)`

Angular curvature peaks at `a = +/-h/sqrt(3)` when those points lie inside
the segment; all extrema are bounded over the actual finite segment.
Acceleration remains conservatively budgeted between tangential and curvature
terms. Corners retain bounded instantaneous motor velocity changes from v0.3,
not a claim of finite acceleration at a mathematically sharp corner.

Native circles centered on the polar origin use `r'=r''=theta''=0` and
`|theta'|=1/R`. Other circles use conservative derivative bounds based on the
swept minimum radius. Native arcs stop at their boundaries; no junction blending,
spline interpolation, or automatic G1 curve fitting is claimed.

The C arc solver computes continuous winding analytically. This matters because
Klipper updates a solver's commanded position at the end of a trapezoid phase;
nearest-angle unwrapping against that value would jump during long full-circle
cruises. The C regression explicitly holds that value fixed while sampling turns.

## Reproduce

Install into the supported Klipper checkout and build its hostsimulator dictionary:

```bash
KLIPPER_PATH=/path/to/klipper PYTHONPATH=/path/to/klipper/klippy \
  python3 -m unittest discover -s tests -v
python3 tests/run_integration.py --features --klipper /path/to/klipper \
  --dictionary /path/to/klipper/out/klipper.dict --output /tmp/polar-features
python3 tests/run_corner_benchmark.py --klipper /path/to/klipper \
  --dictionary /path/to/klipper/out/klipper.dict --output /tmp/polar-corners
```

Historical v0.3 benchmark JSON files remain in `tests/results/`; they are not
v0.4 timing claims. Current synthetic timings are printed by CI. User G-code and
printer logs are not bundled. Software results do not establish physical motor
torque, calibration, backlash, adhesion, extrusion tuning, or surface quality.
