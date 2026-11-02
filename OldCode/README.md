# Running and interpreting the legacy simulations

Reviewed against the Python source on 2026-11-01. These are standalone desktop
experiments, separate from the Teensy firmware and `platformio.ini`.

For antenna tracking demand, start with `pointer.py`. For a repeatable single
flight with rocket statistics, tracking statistics, and CSV output, use the new
`flight_stats.py`. Read the model issues below before using either for sizing.

## Setup on Windows

From the repository root in PowerShell, with a working Python 3 interpreter:

```powershell
cd OldCode
python -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r requirements.txt
& .\.venv\Scripts\python.exe -c "import numpy, matplotlib, ambiance; print('Imports OK')"
```

Use the explicit virtual-environment interpreter below; activation and execution
policy changes are unnecessary. NumPy supplies array math, Matplotlib supplies
plots, and Ambiance supplies atmospheric properties (and installs its own
dependencies). Requirements reflect imports, not a historically pinned environment.
For a reproducible environment after a successful run:

```powershell
& .\.venv\Scripts\python.exe -m pip freeze > requirements-resolved.txt
```

On the reviewed machine, `python` resolves to an inaccessible WindowsApps alias.
A working Python 3.11.7 exists at the following path, but has none of the three
simulation packages. It can be used to create the separate environment:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m venv .venv
```

Run that command from `OldCode` instead of `python -m venv .venv`, then install
the requirements as above. Do not install simulation packages into PlatformIO's
own environment. Select `OldCode/.venv/Scripts/python.exe` in the IDE as well.

## Programs and how they connect

| Program/module | What it does | How to use it |
| --- | --- | --- |
| `gen_main.py` | Standard-drag flight, Gaussian position/acceleration noise, altitude/XYZ/vertical-speed/Mach plots | `& .\.venv\Scripts\python.exe gen_main.py` |
| `pointer.py` | Ten randomized low-drag flights; ideal antenna yaw/pitch speed and acceleration peaks | `& .\.venv\Scripts\python.exe pointer.py` |
| `filter_main.py` | Standard-drag flight, synthetic GPS/barometer/IMU/quaternion measurements, six-state EKF, altitude plot and rocket acceleration printout | Experimental; known errors below. `& .\.venv\Scripts\python.exe filter_main.py` |
| `flight_stats.py` | One fixed flight, selectable legacy drag model, rocket/tracking peaks and motor-ratio conversion; exports JSON, CSV and generator diagnostics | Commands below |
| `gear_calculations/gear_analysis.py` | Prints dimensions/tooth counts for one planetary configuration | `& .\.venv\Scripts\python.exe gear_calculations/gear_analysis.py` |
| `gear_calculations/gear_analysis_old.py` | Torque/speed ratio estimate and discrete two-stage planetary search | `& .\.venv\Scripts\python.exe gear_calculations/gear_analysis_old.py` |
| `models/rocket.py` | Dimensions, mass depletion, thrust acceleration parameter, CoG/CoP, projected areas | Shared model; edit parameters in the entry point |
| `atmospheric_model/atmosphere.py` | Wraps Ambiance; returns density, pressure, temperature, sound speed, gravity and Reynolds number | Shared by generators, EKF and plots |
| `data_generators/base_generator.py` | Abstract generator interface | No standalone simulation |
| `data_generators/supersonic_generator.py` | Flight integrator with Mach/Reynolds/roughness drag, gravity, mass loss and quaternion orientation | Used by generation/filter programs |
| `data_generators/worst_case_pointer_supersonic_generator.py` | Near-duplicate integrator with drag coefficient fixed at 0.1 | Used by pointer; name does not establish a worst-case bound |
| `sensors/sensor.py` | Adds selected noise to a supplied truth vector; returns a column vector | Used for synthetic measurements |
| `noise_generators/*.py` | Abstract noise interface, Gaussian noise, drift, placeholder pink noise | Gaussian is active; pink is white Gaussian noise scaled by 0.5, not actual 1/f noise |
| `kalman_filters/*.py` | Abstract KF/EKF interfaces, covariance update and concrete position/velocity rocket EKF | Used only by `filter_main.py` |
| `plot_managers/plot_manager.py` | Plot registration and functions for altitude, XYZ, vertical speed and Mach | Plots generally block until their windows close |

Generator output is an in-memory dictionary, not a saved file: `time`, `r_x/y/z`,
`v_x/y/z`, `a_x/y/z`, `quaternion`, `angular_vx/y/z`, and
`angular_accel_x/y/z`. Position is metres, velocity m/s, translational acceleration
m/s^2, quaternion order `(w,x,y,z)`, rocket angular motion rad/s and rad/s^2.
Rocket angular motion describes the rocket's orientation, not antenna motor demand.
The legacy entry points do not export this dictionary.

`Sensor.measure()` combines the selected additive noise models. Drift increments
per component per call, not per second, so its behavior depends on sampling and
vector size. `progressive_measure()` passes an extra time-step argument unsupported
by the supplied noise implementations. Neither drift nor progressive measurement
is used by the active entry points.

## Repeatable rocket and motor-demand statistics

Run from `OldCode` after installing requirements:

```powershell
& .\.venv\Scripts\python.exe flight_stats.py --output results/standard
& .\.venv\Scripts\python.exe flight_stats.py --model pointer --output results/low-drag
```

Both use a vertical, still-air flight with rocket thrust-acceleration parameter
201.5 m/s^2, 4.83 s burn, 20 Hz sampling, 10 s prelaunch delay, and a level antenna
804.672 m (half a mile) from the launchpad. The seed defaults to 42, although these
fixed cases have no randomized inputs. This is not the ten-flight random sweep.

Example with a tilted flight and **example** 10:1 gear reductions:

```powershell
& .\.venv\Scripts\python.exe flight_stats.py --model pointer --launch-angle 5 --heading 0 --hz 100 --yaw-ratio 10 --pitch-ratio 10 --output results/tilted-100hz
```

Use `--help` to see all options. `--motor-accel` controls rocket thrust acceleration,
not the antenna motor. Heading is radians; launch tilt is degrees from vertical.
Dimensions, masses, roughness, CoG/CoP, zero wind, and launch delay are fixed in
this runner and recorded in the summary. Edit them there if your rocket differs.

Each output directory contains:

- `summary.json`: settings, sampled apogee, maximum speed, Mach, net acceleration
  in m/s^2 and g, last above-ground sample time, both axes' absolute speed and
  acceleration peaks, and motor-shaft demands after reduction. Each peak records
  its time relative to launch. Peak speed and peak acceleration may occur at
  different times.
- `history.csv`: all samples, including prelaunch and the final below-ground
  sample; trajectory, net acceleration, still-air Mach and pointing derivatives.
  Statistics exclude prelaunch and below-ground samples. Derivatives are calculated
  over the complete series, so boundary samples can still influence peaks.
- `generator.log`: the legacy generator's drag diagnostics and angle-of-attack
  warnings. Review it before accepting the numbers.

Reuse of an output directory overwrites those three files. Choose distinct paths
for different cases. This runner preserves the original physics and adds no
filter, motor capability model, control loop, recovery system or design margin.

## The original pointing sweep

`pointer.py` samples these settings independently on each of ten runs:

| Setting | Current value/range |
| --- | --- |
| Rocket thrust acceleration parameter | Uniform 180-220 m/s^2 |
| Burn duration | Uniform 4.5-5 s |
| Length / diameter | 3.835 m / 0.1524 m |
| Empty / full mass | 40.8 kg / 60.8 kg |
| Surface roughness | Uniform 1e-6 to 10e-6 m; has no effect on the fixed-Cd drag path |
| Launch tilt | Uniform -5 to +5 degrees from vertical |
| Heading | Uniform 0 to 2*pi radians |
| Wind velocity callback | `[U(0,0.2)*sin(t), U(0,0.2)*cos(t), 0]` m/s; new random draws every evaluation |
| Sampling / launch delay | 20 Hz / 10 s |
| Antenna position / orientation | `[-804.672,0,0]` m relative to launchpad, level, world-aligned |

For reproducibility without editing the original script:

```powershell
& .\.venv\Scripts\python.exe -c "import numpy as np; np.random.seed(42); import pointer; pointer.main()"
```

To change the number of flights, edit `num_simulations` inside `main()`. Change
`distance`, displacement, rotation, launch bounds, wind and rocket parameters
there as needed. `gen_main.py` instead uses 201.5 m/s^2 and 20 Hz;
`filter_main.py` uses 200 m/s^2 and 10 Hz. Both use zero tilt, zero wind,
4.83 s burn and the same dimensions/masses, with random heading.

Pointing subtracts antenna displacement from rocket position, rotates into the
antenna frame, computes yaw `atan2(y,x)` and elevation `asin(z/r)`, unwraps angles,
then applies `np.gradient` twice. Peaks are absolute magnitudes over the whole
legacy series. Median acceleration over a flight is not a peak sizing criterion.
Yaw is zero along +X; firmware azimuth is zero North. Align these frames when
transferring commands. A rocket directly above the antenna makes azimuth singular;
large apparent yaw requirements near that geometry need separate mechanical review.

These are required ideal output-axis motions, not tested motor maximum ratings.
For reduction `G = motor revolutions / antenna revolutions`:

```text
motor speed (deg/s)       = G * antenna speed (deg/s)
motor speed (RPM)         = G * antenna speed (deg/s) / 6
motor acceleration       = G * antenna acceleration (same angular units)
rad/s or rad/s^2          = (deg/s or deg/s^2) * pi / 180
```

Actual achievable acceleration requires inertia, torque available at operating
speed, gearing efficiency, gravity imbalance, wind/friction loads, driver/current
limits and control behavior. None is modeled by `pointer.py`. The repository's
old torque/speed assumptions do not establish which motors you will use.

## Model issues that affect interpretation

1. **Mass timing is inconsistent in both generators.** Force calculation uses
   `get_current_mass(time - pre_launch_delay)`, but acceleration and inertia use
   `get_current_mass(time)`. With a 10 s delay exceeding burn time, the launch
   force uses full mass while division uses empty mass. For a vertical still-air
   launch, the initial legacy acceleration is approximately
   `(motorAccel - 9.80665) * 60.8 / 40.8`, rather than `motorAccel - 9.80665`.
   Do not interpret `motorAccel` itself as the resulting peak net acceleration.
2. **Position integration adds too much acceleration displacement.** Velocity is
   updated first, then position uses the updated velocity plus `0.5*a*dt^2`.
   Relative to the old velocity this adds `1.5*a*dt^2`. Flight samples also store
   the updated state under the start-of-step timestamp. These affect geometry,
   timing, and numerical angular acceleration.
3. **The pointer generator uses fixed Cd=0.1.** Standard generation uses a variable
   coefficient. The low-drag case is an exploratory assumption, not a validated
   bound. No parachute/recovery is simulated; flight continues until ground
   crossing and records the first below-ground state.
4. **Flight acceleration is net inertial acceleration including gravity.** It is
   not IMU specific force. Prelaunch acceleration is initialized to -9.81 m/s^2
   despite stationary position/velocity. The original filter acceleration summary
   includes these samples; its near-max duration is total qualifying sample count
   times dt, not necessarily a contiguous interval.
5. **Rigid-body dynamics are approximate.** The code solves inertia against world
   torque and uses the result to advance quaternion angular velocity without a
   complete consistent body-frame Euler dynamics model. Stability, tumbling and
   resulting pointing demands need validation against flight data or a trusted
   trajectory before motor selection.
6. **Differentiation is sensitive to sampling and discontinuities.** Launch,
   burnout, ground crossing and randomized wind can affect acceleration spikes.
   Repeat fixed-input cases at 20, 50 and 100 Hz and inspect CSV histories for
   convergence. Changing Hz in the random sweep also changes the wind draws;
   a shared seed alone does not preserve the same physical wind history.
7. **Wind API implementation differs from its annotation.** Generators call
   `wind_affector(t)` unconditionally; supply a callable even for zero or constant
   wind. `None` and a bare vector fail despite being advertised as supported.

Fixing these changes historical results. The new runner deliberately leaves
them intact so reproduction can be distinguished from future model corrections.
Ten random flights do not establish a global maximum. Sweep realistic launch
tilt/heading, minimum antenna distance, trajectory and wind assumptions, inspect
peak cases, then apply an explicit engineering margin.

## EKF experiment limitations

You do not need `filter_main.py` to obtain rocket truth or pointing requirements.
It currently has source-level problems that can prevent a successful run:

- `max(measured_gps_positions[i][2], [0.0])` and the barometer equivalent mix
  array/list values when building the measurement array; this can create a ragged
  array. Measurements need explicit scalar extraction or consistent column shapes.
- `calculate_acceleration_inertial()` indexes `conditions['gravity'][0]` although
  the atmosphere wrapper normally returns scalar gravity for scalar altitude.
- `compute_jacobian_H()` divides by velocity magnitude without a zero-speed guard.
- Synthetic acceleration is rotated with `R @ a` into the claimed body frame,
  while the EKF measurement function uses `R.T @ a`. They disagree for tilted
  orientations. Quaternion noise is also a simple additive placeholder.
- The `use_simulated=False` comment promises real CSV loading, but no loader
  exists; noise variables/measurements used later would be missing.
- Filter time starts at a hard-coded 10 s, and estimated samples are plotted
  against `time[:-1]` despite being generated from indices starting at 1.

Treat the filter as a separate repair/validation task before using its estimates.

## Gear calculators

`gear_analysis.py` is a geometry calculator for a fixed-ring, sun-input,
carrier-output planetary. Its current example is a 12-tooth sun, 108-tooth ring,
48-tooth planets, three planets, module 2 mm and 10:1 reduction. It does not
calculate motor performance or tooth strength.

`gear_analysis_old.py` assumes 0.4237 Nm motor torque at an intended 250 RPM,
8.75 Nm required output torque and `40/6` RPM output speed (40 deg/s, despite a
comment mentioning 20). The example uses a 0.90 torque allowance, 0.90 efficiency
per stage, two stages, 20-tooth suns and four planets. Update its constants from
your mechanism and motor torque-speed data; torque times ratio alone does not
check acceleration or motor torque loss with speed. Its import executes the
example search immediately.

## What was verified in this review

All 19 pre-existing Python source files parsed successfully with Python 3.11.7.
Both gear scripts ran: the old search chose rings `[88, 88]`, output 8.57 RPM and
9.01 Nm under its own assumptions. The flight, plotting, filter and new statistics
runner were not executed: NumPy, Matplotlib and Ambiance are absent from the
available interpreter. No numerical flight or motor-demand result is claimed
as verified by this review.
