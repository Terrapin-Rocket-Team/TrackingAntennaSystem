# Running and interpreting the legacy simulations

Updated 2026-11-17. These are standalone desktop
experiments, separate from the Teensy firmware and `platformio.ini`.

## Start here: run the current simulation yourself

These commands are for **PowerShell** in VS Code. Open **Terminal > New
Terminal** and choose PowerShell from the terminal dropdown. Paste each command
without the surrounding Markdown backticks. The `&` runs the Python executable
at the quoted path; it is PowerShell syntax. `-B` prevents Python bytecode files.
No PlatformIO build, Teensy connection, or environment activation is needed.

### 1. Go to the simulation folder (each new terminal)

```powershell
cd "C:\Users\mattm\Documents\GitHub\TrackingAntennaSystem\OldCode"
```

All following paths are relative to this folder. If the checkout moves, change
this path to the actual `OldCode` directory.

### 2. Check the existing Python environment

```powershell
& ".\.venv\Scripts\python.exe" -B -c "import numpy, matplotlib, ambiance; print('Simulation environment ready')"
```

If it prints `Simulation environment ready`, proceed to step 3. If packages are
missing, install them once (and again when requirements change):

```powershell
& ".\.venv\Scripts\python.exe" -m pip install -r requirements.txt
```

If `.venv` does not exist, create it using the working PlatformIO Python, then
install the requirements. This creates a separate environment for simulations:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m venv .venv
& ".\.venv\Scripts\python.exe" -m pip install -r requirements.txt
```

If the existing environment reports that it cannot create a process, its base
Python may have moved or become unavailable. Create an additional environment
with `-m venv .venv-sim`, install requirements there, and substitute `.venv-sim`
for `.venv` in the commands below. Preserve the existing environment until the
replacement works. If the PlatformIO interpreter is also unavailable, use a
working installed Python 3 interpreter to create the environment.

### 3. Run both geometry optimization studies

```powershell
& ".\.venv\Scripts\python.exe" -B mount_study.py
```

This is the **current complete workflow**: create the 30,000-foot test flight,
search gamma1/d for minimum ideal torque and minimum maximum antenna tracking
error, simulate both with coupled torque dynamics, and save plots, motor torque,
angular velocity/acceleration peaks, and reusable configurations. Wait until
PowerShell returns its `PS ...>` prompt. Progress messages show generation,
candidate evaluation and refinements; plots are saved rather than opened.

### 4. Open the results

Use the VS Code Explorer under `OldCode/results/mount-study`:

| File | What to inspect |
| --- | --- |
| `study_summary.json` | Both selected geometries, maximum errors/torques, and angular-motion peaks |
| `comparison.png` | Tracking error and motor torque versus time for both designs |
| `optimization.png` | How gamma1 and d affect each objective |
| `minimum_torque/summary.json` | Detailed minimum-torque motion peaks, timestamps and assumptions |
| `minimum_tracking_error/summary.json` | Detailed minimum-error motion peaks, timestamps and assumptions |
| Either design's `prediction_error.png` | True rocket direction minus predicted direction, separately for azimuth/elevation |
| Either design's `tracking.png` | Required and actual antenna motion |
| Either design's `history.csv` | Full time history, including shaft/output torque |

`max_output_*` means **gearbox output / antenna axis**. `max_motor_*` means
**motor shaft**, using 50:1 azimuth and 10:1 elevation. These are sampled
requirements/achieved motions, not measured physical motor maximum ratings.

### 5. Rerun one selected design without searching again

After step 3 has produced the configurations, run either command:

```powershell
# Minimum-torque design
& ".\.venv\Scripts\python.exe" -B tracking_sim.py --mount-config results/mount-study/minimum_torque/mount_config.json --output results/chosen-mount
# Minimum-error design
& ".\.venv\Scripts\python.exe" -B tracking_sim.py --mount-config results/mount-study/minimum_tracking_error/mount_config.json --output results/chosen-error-mount
```

These reuse the chosen geometry/controller but regenerate the calibrated flight.
**Keep `--mount-config` in the command to use the coupled mechanical dynamics.**
Running `tracking_sim.py` alone uses the earlier kinematic comparison.

To test a radio outage with the chosen geometry:

```powershell
& ".\.venv\Scripts\python.exe" -B tracking_sim.py --mount-config results/mount-study/minimum_torque/mount_config.json --blackout 2 5 --comm-timeout 1 --output results/chosen-mount-outage
```

### 6. Check the code after simulation updates

```powershell
& ".\.venv\Scripts\python.exe" -B -m unittest test_tracking_sim test_mount_dynamics -v
```

Successful checks end in `OK`. Read failures before trusting changed results.
Rerun step 3 after changes to geometry, objectives, PID, dynamics or flight inputs
so saved selected configurations match the new model. Use a separate output
directory to preserve a previous run:

```powershell
& ".\.venv\Scripts\python.exe" -B mount_study.py --output results/mount-study-new
```

When using that directory, also change the `--mount-config` paths in step 5 to
`results/mount-study-new/...`. Reusing any output directory overwrites that run's
generated files. See the sections below for assumptions and detailed settings.

**Maintenance:** every simulation update must update this run process when
commands, dependencies, inputs, outputs or interpretation change, and record the
verification actually performed. Keep the repository README's simulation link current.

For the supplied coupled mechanical equations and the two geometry studies, use
`mount_study.py` and the saved configurations described below.

## Coupled dynamics and two geometry studies

From `OldCode`:

```powershell
& .\.venv\Scripts\python.exe -B mount_study.py
```

The script builds one calibrated 30,000-foot flight and one seeded telemetry
history, then uses both for every geometry. It searches **gamma1=0–90 degrees**
and **d=0.025–0.1 m**, with user-confirmed **a=s=0**, L=0.143 m and dish mass
3 kg. The default coarse grid is 19 by 7 with two local refinements around both
objectives. This is a bounded sampled search, not a global optimality proof.
Mechanical travel and collision limits are intentionally unspecified for this
provisional study. The two objectives optimize gamma1 and d only; PID gains,
trajectory, telemetry and other mechanical assumptions remain fixed:

- **Minimum torque:** minimize the largest absolute ideal motor-shaft torque
  across both axes and the flight, computed by inverse dynamics. Then run the
  torque-limited controller for that geometry and measure its tracking error.
  Using ideal trajectory torque avoids the trivial alternative of reducing
  torque by simply failing to follow the rocket.
- **Minimum tracking error:** minimize the maximum actual antenna/true-rocket
  boresight separation using the same torque PID controller; report its ideal,
  requested and actual peak motor torques.

The pure **rocket prediction error** is true rocket direction minus predicted
rocket direction; it is identical for both geometries and is plotted separately.
Geometry cannot change telemetry prediction error. Actual antenna tracking error
includes the mechanism/controller response and therefore can change with geometry.

Each study saves a `mount_config.json` and the usual trajectory, tracking,
prediction-error, summary and history files. `study_summary.json` compares both
designs, including peak angular velocity/acceleration for the ideal trajectory and
actual predicted PID motion. `candidates.csv` records all evaluated geometries;
`optimization.png` shows the objective surfaces; `comparison.png` compares
tracking error and actual motor-shaft torque versus time. Reported torques are
**motor-shaft N*m**, except CSV fields explicitly labeled `output_torque`.

To use a selected geometry in a new simulation:

```powershell
& .\.venv\Scripts\python.exe -B tracking_sim.py --mount-config results/mount-study/minimum_torque/mount_config.json --output results/chosen-mount
& .\.venv\Scripts\python.exe -B tracking_sim.py --mount-config results/mount-study/minimum_tracking_error/mount_config.json --output results/chosen-error-mount
```

Supplying `--mount-config` enables the coupled torque plant. Without that option,
`tracking_sim.py` retains its original kinematic acceleration-limited comparison.
The torque plant uses output-coordinate radians [phi,psi], converts sky elevation
with `psi = pi/2 - elevation - gamma1`, and integrates both coupled equations with
RK4 and zero-order-held torque. It has **no artificial acceleration or speed
clipping**: the speed-dependent motor torque envelope limits the applied torque.
The old `--max-speed` and `--max-accel` arguments apply only to the kinematic plant.
Torque PID gains, separately for azimuth/elevation, default to Kp=[12,8] N*m/rad,
Ki=[0.5,0.5] N*m/(rad*s), Kd=[4,2] N*m*s/rad. The controller adds gravity-holding
torque, uses derivative on measured angle and conditional integral antiwindup.
It assumes ideal angle/rate feedback and ideal torque actuation; this is not a
simulation of stepper commutation or an assertion that the DM556T exposes torque
commands or angle feedback.

Provisional inputs are explicit in each saved configuration: a uniform thin dish
of radius 0.25 m, base azimuth inertia 0.0904 kg*m² **excluding the added dish and
azimuth rotor**, zero extra elevation support inertia, zero damping, 50:1/10:1
reductions, and FY56ED420A rotor inertia 0.000068 kg*m². Approximate 24 V torque
curve points are transcribed from the supplied chart, then derated to **50%**
available torque; gearbox efficiency is assumed **80%**. Both can be varied with
`--usable-torque-fraction` and `--efficiency`. No measured torque-speed curve,
support COM/gravity, gearbox inertia, backlash, wind load or missed-step model is
available. Thus numerical results are conditional on these assumptions.

Output-equivalent input torque is eta*N*tau_motor. To account for motor rotor
acceleration consistently when eliminating the shaft coordinate, effective
inertia includes eta*N²*J_rotor (N²*J_rotor for unit efficiency). This is a
constant-efficiency drive approximation, not a quadrant-specific gearbox-loss
model. Required output motion maps to motor motion by N; a fixed gamma1 changes
gravity/inertia and achievable tracking, not ideal tracking velocity/acceleration.

The default vertical flight has zero ideal azimuth velocity/acceleration;
actual azimuth PID motion comes from noisy telemetry. These runs therefore cannot
establish azimuth worst-case requirements for an off-axis flight. Ideal elevation
requirements are common to both geometries; controller motion can exceed those
requirements while correcting errors. Inspect peak times and raw command jumps.

Verified study results on 2026-11-17 (329 evaluated candidates, seed 42):

| Geometry objective | gamma1 | d | Ideal peak shaft torque | Actual peak shaft torque | Maximum antenna tracking error |
| --- | --- | --- | --- | --- | --- |
| Minimum ideal torque | 55.56 deg | 25.69 mm | 0.3977 N*m | 0.4177 N*m | 7.6229 deg |
| Minimum maximum tracking error | 90 deg | 25 mm | 0.5158 N*m | 0.5175 N*m | 7.5916 deg |

The torque maxima above occur on elevation. Predicted-PID motion peaks:

| Design / axis | Output deg/s | Output deg/s² | Motor RPM | Motor deg/s² |
| --- | --- | --- | --- | --- |
| Minimum torque / azimuth | 1.665 | 32.573 | 13.874 | 1628.656 |
| Minimum torque / elevation | 28.760 | 116.444 | 47.934 | 1164.440 |
| Minimum tracking error / azimuth | 1.642 | 30.252 | 13.681 | 1512.616 |
| Minimum tracking error / elevation | 28.733 | 127.461 | 47.888 | 1274.610 |

Ideal elevation motion for both designs is 26.740 deg/s and 21.549 deg/s²;
the additional controller acceleration corrects lag/noise. Pure rocket-direction
prediction error is the same for both designs (2.650 deg maximum boresight error).
At 200 Hz with each geometry held fixed, peak tracking errors were 7.6336 and
7.6037 deg, and peak elevation motor torques were 0.4181 and 0.5175 N*m.
This timestep check preserves the tradeoff but does not establish robustness
across telemetry seeds, aerodynamic assumptions or physical hardware.
All 21 tracking/dynamics regression tests passed, including energy consistency,
batch/scalar dynamics agreement, holding under gravity compensation and motor
torque envelope checks. Selected configurations were also run through the main
tracking simulator at 200 Hz.

For a faster rerun of the already calibrated test flight (same 20 Hz integrator):

```powershell
& .\.venv\Scripts\python.exe -B mount_study.py --target-apogee-ft 0 --motor-accel 312.5837707519531
```

For PID tracking and pointing-error diagrams, start with `tracking_sim.py` below.
For ideal antenna tracking demand, use `pointer.py`. For a repeatable single
flight with rocket statistics, tracking statistics, and CSV output, use the new
`flight_stats.py`. Read the model issues below before using either for sizing.

## PID controller and 30,000-foot tracking scenario

From `OldCode`, after installing requirements:

```powershell
& .\.venv\Scripts\python.exe -B tracking_sim.py
```

### Maximum angular velocity and acceleration

Every PID run prints a **MAXIMUM ANGULAR MOTION** table and saves the same
measurements under `angular_motion` in `summary.json`. Each azimuth/elevation
entry reports maximum absolute output velocity (deg/s), output acceleration
(deg/s²), motor velocity (RPM), and motor acceleration (deg/s²), with the time
of each peak relative to launch. Only flight samples contribute to these peaks.

The report separates five sources:

- `ideal_tracking`: the motion required to point exactly at the simulated rocket.
- `hold_command` / `predicted_command`: motion implied by the unfiltered telemetry
  commands, without and with velocity prediction.
- `hold_pid` / `predicted_pid`: actual simulated PID motion after speed and
  acceleration limits.

Default motor reductions are **50:1 azimuth and 10:1 elevation**. Override with
`--azimuth-ratio` and `--elevation-ratio`; these convert reported shaft demands,
but do not change the assumed output-axis speed/acceleration limits. Likewise,
`flight_stats.py` now defaults to `--yaw-ratio 50 --pitch-ratio 10`.

Ideal angular derivatives are calculated on the flight generator's native
sample grid before interpolation for plotting. Command derivatives are sampled
on the control grid: noisy packet updates can create large jumps and apparent
acceleration spikes. Treat these as raw command discontinuities to be handled by
the controller, not motor sizing limits or physical rocket angular acceleration.
Repeat cases at multiple sampling rates before accepting peak requirements.
PID peaks reflect the configured plant limits, not measured motor capability;
none of these sampled peaks proves a global worst-case bound.

`history.csv` includes `required_*_rate_deg_s`, `required_*_accel_deg_s2`, and
`hold_*` / `predict_*` command derivatives alongside the existing actual PID
motion. `pointer.py` continues to report ideal output peaks across its randomized
flights; `flight_stats.py` reports ideal output/shaft peaks for one flight.

The script generates the corrected standard-drag trajectory, then adjusts the
rocket's **thrust acceleration parameter** until sampled apogee is within 1 metre
of **30,000 ft = 9,144 m above the launchpad**. It does not stretch altitude values.
This is a calibrated tracking test case, not validation that the physical rocket
will reach that height. An expected 30,000 ft MSL would require subtracting launch
site elevation first. Calibration retains 60.8/40.8 kg full/empty mass, 4.83 s burn,
3.835 m length, 0.1524 m diameter, legacy drag and still air. These assumptions
need thrust-curve, aerodynamic or flight-data validation for real predictions.

Your saved `results/flight/summary.json` reported 7,569.62 m using the old
integration. After correcting mass timing and integration, the same inputs
produce about 6,707 m at 20 Hz. The original `motorAccel=201.5` input therefore
does not represent a 30,000-foot trajectory in this model. Existing saved results
are not changed. `flight_stats.py`, `gen_main.py` and `pointer.py` use the corrected
integrators but **do not calibrate automatically**.

Files saved under `results/pid` (overwritten when that directory is reused):

- **`trajectory.png`**: altitude in feet and the target apogee.
- **`tracking.png`**: true direction, predicted command and actual PID pointing;
  required/actual angular speed and acceleration with assumed limits; signed
  azimuth/elevation pointing errors.
- **`pointing_error.png`**: total boresight angular error for hold-last-position
  versus velocity prediction, a beam threshold, telemetry age, and a link-status
  panel. Red shading marks telemetry timeouts.
- **`prediction_error.png`**: one signed error per axis, defined as true rocket
  angle minus predicted rocket angle relative to the ground system at the same
  timestamp. Azimuth is wrapped into [-180,180) degrees. Telemetry status and red
  timeout shading remain. `projected_actual_error.png` is a compatibility copy
  of this simplified graph. Antenna-following error stays in the other graphs.
- **`history.csv`**: rocket position, telemetry age, commands, actual motion,
  acceleration and errors for both strategies. Acceleration at row t acts over
  the next control interval. The final acceleration is zero (no next interval).
- **`summary.json`**: all settings, baseline and calibrated apogee, fitted rocket
  acceleration parameter, peak/RMS/95th-percentile errors and fraction inside
  the assumed half-power beam radius. Statistics cover flight only.
- **`generator.log`**: original drag and angle-of-attack diagnostics from baseline
  and calibration trials.

The PID plant is angle/velocity feedback with acceleration output:

```text
acceleration_command = Kp * angle_error + Ki * integral(error) - Kd * actual_rate
```

Derivative on measured angle avoids setpoint derivative kick. Conditional
integration limits windup at speed/acceleration saturation. Defaults are Kp=25
s^-2, Ki=0.5 s^-3, Kd=10 s^-1, output speed limits 40/40 deg/s, output acceleration
limits 100/100 deg/s² (azimuth/elevation), and a 100 Hz control loop. These gains
and motor limits are experiment settings, not measured hardware capabilities.
Motor shaft motion is output motion times 50 for azimuth and 10 for elevation.
Rotor/gearbox dynamics, gravity imbalance, backlash, missed steps and electrical
driver behavior are not simulated. Ideal angle/rate feedback is assumed; the
real open-loop steppers do not provide that feedback automatically.

Telemetry defaults: 5 Hz, 0.3 s latency, 0.05 s Gaussian latency jitter clipped at
zero, 3 m position noise per axis, 1 m/s velocity noise per axis, no packet drops.
Velocity is a synthetic measured input, not differentiated noisy position.
Only arrived packets can influence commands. Late packets older than the latest
measurement are ignored. Prediction uses measurement age, not fixed latency:
`predicted_position = measured_position + measured_velocity * age`.
Before any arrival, the antenna points to the known pad. Both strategies receive
identical packets and use identical PID gains/limits. Packet loss, telemetry
timing and noise are assumptions; measure them from radio logs when possible.

The default flight is vertical: true azimuth demand is zero, so azimuth mainly
responds to telemetry noise. `--launch-angle` (degrees) and `--heading` (radians)
allow nonvertical cases, but the legacy approximate angular dynamics still need
validation. A true overhead crossing makes azimuth ill-conditioned. No hard
elevation stops or azimuth cable-wrap limits are modeled.

Useful experiments (choose a separate output directory for each):

```powershell
# Change PID gains; compare against the default results.
& .\.venv\Scripts\python.exe -B tracking_sim.py --kp 49 --ki 0.5 --kd 14 --output results/pid-faster
# More telemetry lag, outages, and lower assumed acceleration.
& .\.venv\Scripts\python.exe -B tracking_sim.py --latency 1 --drop-probability 0.1 --max-accel 50 50 --output results/pid-lag
# Inspect the corrected uncalibrated rocket using the original thrust parameter.
& .\.venv\Scripts\python.exe -B tracking_sim.py --target-apogee-ft 0 --output results/pid-original-inputs
# Check sensitivity to trajectory sampling (calibration is repeated at this rate).
& .\.venv\Scripts\python.exe -B tracking_sim.py --flight-hz 50 --output results/pid-50hz
# Behavioral regression tests, without generating bytecode.
& .\.venv\Scripts\python.exe -B -m unittest test_tracking_sim -v
```

The beamwidth default is **an assumed 20-degree full half-power beamwidth**;
`--beamwidth` changes it. Error below 10 degrees therefore means inside this
assumed half-power contour, not guaranteed radio reception. No covariance or
statistical confidence band is claimed: the 95th percentile in the summary is
over time in one flight. Repeated seeded runs would be needed for Monte Carlo
bands; a validated estimator would be needed for covariance-based bands.

### Communication indicator and forced outages

The indicator reports **Receiving**, **Waiting** (initially), or **LOST**.
`--comm-timeout 1` declares loss once one second passes without an arriving
packet containing a newer measurement. After the first fresh packet arrives,
status returns to Receiving. An out-of-order old packet does not renew freshness.
Before any packet arrives, Waiting changes to LOST after the same timeout.
The timeout uses **time since arrival**, while telemetry age uses **time since
measurement**; a large radio latency is not itself a reception outage.

This indicates the simulator's missing telemetry, not a measured RF link budget.
The beam contour stays a separate alignment criterion: pointing outside it does
not automatically discard packets. During an outage the PID continues following
the available projection; the graph shows whether that projection becomes wrong.

To see a clear loss and recovery on the graph:

```powershell
& .\.venv\Scripts\python.exe -B tracking_sim.py --blackout 2 5 --comm-timeout 1 --output results/pid-outage
```

`--blackout START END` suppresses packet **arrivals** from START (inclusive) to END
(exclusive), in seconds since launch. With the one-second timeout, the LOST
indicator begins after the last arrival has aged a second, rather than exactly
at blackout start. It clears on the next fresh arrival. The graph can show no
loss under normal defaults; use this option or packet drops to test outages.

`history.csv` now also records signed projected-minus-actual and projected-minus-
true angles, their boresight errors, reception age and `communication_status`
(`1` receiving, `0` waiting, `-1` lost). `summary.json` records flight-time outage
intervals, total timed-out duration and projected/actual error peaks. Radio
status is shared by both controllers because they receive the same packets.

## Setup on Windows

### Coupled mount dynamics equations (2026-11-17)

`models/mount_dynamics.py` transcribes the supplied geometry and equations in SI
units. It includes the dish inertias, parallel-axis contributions, gravity torque,
both coupling terms, and constant reflected motor-rotor inertias. It provides
inverse dynamics (motion to output torque) and forward dynamics (torque to
acceleration). It is connected to `mount_study.py` and to `tracking_sim.py` when
`--mount-config` is supplied. Horizontal offsets `a=s=0` were confirmed by the user.

The supplied drawing defines `theta = psi + gamma1` from vertical, whereas sky
elevation is above the horizon. The conversion is therefore
`elevation = pi/2 - (psi + gamma1)`. Geometry is
`x0 = a + d*sin(gamma1)`, `z0 = L + d*cos(gamma1)`,
`x(psi) = L*sin(psi) + d*sin(psi+gamma1) + a*cos(psi)`,
`l2 = s + x`, and `G = mass*g*x`. The forward solver follows the supplied
elevation equation's sign: gravity drives positive psi, so holding torque is -G.

Known/proposed model inputs: dish mass 3 kg, L=0.143 m, d in [0.025,0.1] m,
thin-disk radius approximately 0.25 m from the sketch, rotor inertia
0.000068 kg*m² and reductions 50/10. The 0.0904 kg*m² azimuth assembly inertia
is treated as excluding the additional dish and azimuth rotor. Elevation support
inertia and damping default to zero (incomplete until supplied). These defaults
are explicit model assumptions, not CAD measurements. Constructor inputs `a`,
`s`, `gamma1` and `d` are required by the model; study-specific values are saved
in each configuration. Bounded geometry optima are conditional on the study assumptions.

Check the equations and existing tracker with:

```powershell
& .\.venv\Scripts\python.exe -B -m unittest test_tracking_sim test_mount_dynamics -v
```

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
| `tracking_sim.py` | Calibrated 30,000-foot scenario, delayed/noisy telemetry, PID motion, error metrics and saved plots | PID section above |
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
- `history.csv`: all above-ground samples, including prelaunch; trajectory,
  net acceleration, still-air Mach and pointing derivatives.
  Statistics exclude prelaunch. Derivatives are calculated
  over the complete series, so boundary samples can still influence peaks.
- `generator.log`: the legacy generator's drag diagnostics and angle-of-attack
  warnings. Review it before accepting the numbers.

Reuse of an output directory overwrites those three files. Choose distinct paths
for different cases. This runner uses the corrected integrators and adds no
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

1. **Fixed 2026-11-03: mass timing was inconsistent in both generators.** Force calculation used
   `get_current_mass(time - pre_launch_delay)`, but acceleration and inertia used
   `get_current_mass(time)`. With a 10 s delay exceeding burn time, the launch
   force used full mass while division used empty mass. For a vertical still-air
   launch, the initial legacy acceleration is approximately
   `(motorAccel - 9.80665) * 60.8 / 40.8`, rather than `motorAccel - 9.80665`.
   Both now use time since launch. Prelaunch mass is clamped to full mass.
   `motorAccel` remains thrust acceleration, not net acceleration including drag/gravity.
2. **Fixed 2026-11-03: position integration added too much displacement.** Velocity was
   updated first, then position used the updated velocity plus `0.5*a*dt^2`.
   Relative to the old velocity this added `1.5*a*dt^2`. Flight samples also stored
   the updated state under the start-of-step timestamp. These affect geometry,
   timing, and numerical angular acceleration. Position now advances from the old
   velocity, and state/acceleration samples match their timestamps. The first
   underground state is no longer stored. There is a 600-second flight guard.
3. **The pointer generator uses fixed Cd=0.1.** Standard generation uses a variable
   coefficient. The low-drag case is an exploratory assumption, not a validated
   bound. No parachute/recovery is simulated; flight continues until ground
   crossing and retains only above-ground samples.
4. **Flight acceleration is net inertial acceleration including gravity.** It is
   not IMU specific force. Prelaunch net acceleration is now zero, matching
   stationary position/velocity. The original filter acceleration summary
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
7. **Fixed 2026-11-03: wind API implementation differed from its annotation.** Generators called
   `wind_affector(t)` unconditionally and required a callable even for zero or
   constant wind. Both now accept `None`, a constant vector, or a callable.

These corrections change historical results. Old saved summaries are historical;
new runs use corrected integration. Remaining aerodynamic/rotation limitations
are retained and apply to the PID trajectory, including calibrated scenarios.
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

## Original review verification (2026-11-01)

All 19 pre-existing Python source files parsed successfully with Python 3.11.7.
Both gear scripts ran: the old search chose rings `[88, 88]`, output 8.57 RPM and
9.01 Nm under its own assumptions. The flight, plotting, filter and new statistics
runner were not executed: NumPy, Matplotlib and Ambiance are absent from the
available interpreter. No numerical flight or motor-demand result is claimed
as verified by this review.

## PID verification (2026-11-03)

The existing Python 3.13 environment now has the required packages. All eight
behavioral regression tests passed, and the full PID run generated its CSV,
summary and three PNG plots. With default settings, fitted `motorAccel` was
312.5837707519531 m/s² and sampled apogee was 9,144.623 m (30,002.04 ft).
Hold-last-position PID had 20.155-degree peak / 3.546-degree RMS error;
velocity-predicted PID had 11.419-degree peak / 1.938-degree RMS error. These
results are for this specific assumed telemetry/controller scenario, not hardware
measurements. Plot inspection confirmed the largest error occurs during early
ascent, with additional lag during the unmodeled-recovery ballistic descent.

Keeping that fitted thrust parameter fixed and increasing trajectory sampling
from 20 to 100 Hz gave 9,130.691 m apogee, a 13.93 m (0.15%) difference. The
one-metre calibration tolerance describes fitting at a chosen timestep; it does
not represent physical or numerical accuracy. Use separate output directories
for sampling studies. Existing `results/flight` remains the historical run.
