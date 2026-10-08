# Antenna Tracking System: guide for AI agents

Read this file first when working in this repository. It gives a new agent or
contributor the project boundaries, architecture, runtime flow, and checks needed
to navigate the code. Its scope is the Antenna Tracking System repository,
specifically BackendSoftware and OldCode. Read the relevant
implementation and tests before changing behavior; this guide is a map, not a
replacement for the source.

## Document ownership and required change history

- Created: **2026-10-06** (America/New_York).
- Created by: Divyansh Srivastava, Matt Urban
- Last updated: **2026-10-06**.
- Last updated by: OpenAI Codex (AI agent), at Matt Urban's request.
- Initial review: library version **0.1**, repository commit **11b4d7a** (metadata preserved from the supplied guide; not an ATS package version).
- ATS adaptation reviewed against: checkout at **d1bf4af**, including local edits to `src/atsmain.cpp`.

**Every change to this file must update `Last updated` and `Last updated by`, and
append a row to the change history below with the actual date, editor, and a
specific description.** Use `YYYY-MM-DD` dates and identify the human or AI agent
that made the edit. Preserve earlier entries and the creation metadata. Record
separate edits even if they happen on the same date. When changes to architecture,
public APIs, integration, or tooling make this guide inaccurate, update the
affected sections and record that update in the same change.

| Date | Changed by | Change |
| --- | --- | --- |
| 2026-10-06 | Matt Urban | Created the repository guide. |
| 2026-10-06 | OpenAI Codex (AI agent) | Adapted Astra-specific architecture, APIs, units, simulation, build commands, and maintenance references to ATS BackendSoftware and OldCode; documented current integration gaps. |

## What the Antenna Tracking System is and how it reaches the controller

**The Antenna Tracking System points a ground antenna at a rocket in flight.**
BackendSoftware is a C++ Arduino application built with PlatformIO for Teensy 4.1.
It combines ground-station GPS, received APRS telemetry, target prediction,
coordinate conversion, stepper control, and logging. It is not packaged as an
Astra library and has no `AstraConfig` or `astra::Astra` lifecycle.

**The intended deployment model is firmware running on the antenna's Teensy.**
The application supplies GPS, radio UART, motor pins, gearing, and logging sinks.
PlatformIO compiles and links the application and libraries into firmware, and
that firmware is uploaded/flashed to the controller. The integration in
`atsmain.cpp` is incomplete; consult the known mismatches below before treating
it as a working end-to-end deployment.

The repository's purpose is documented in [README.md](../README.md):

| Project area | Responsibility |
| --- | --- |
| **BackendSoftware** | Teensy tracking firmware, prediction, coordinate conversion, GPS, motor control, and logging. |
| **OldCode** | Python trajectory generation, noisy sensor experiments, extended Kalman filtering, antenna pointing analysis, and gear calculations. |
| **Teensy_Test** | Separate PlatformIO project for hardware experimentation; its configuration is not BackendSoftware's build configuration. |
| **Electronics** and other repository directories | Supporting system resources; outside this guide's detailed software scope. |

For the application entry point, actual pin assignments, and upload configuration,
consult [atsmain.cpp](src/atsmain.cpp), [motormain.cpp](src/motormain.cpp), and
[platformio.ini](platformio.ini). The two applications currently use different
motor pins. Keep firmware changes in BackendSoftware and Python analysis changes
in OldCode; an experiment is not automatically part of the deployed firmware.

## Repository map

Paths in this document are relative to BackendSoftware, where this guide lives.

| Location | Purpose and useful entry points |
| --- | --- |
| [src/atsmain.cpp](src/atsmain.cpp) | Intended tracking `setup()`/`loop()`: GPS origin, radio receive buffer, prediction, pointing, and motors. |
| [src/motormain.cpp](src/motormain.cpp) | Active motor test `setup()`/`loop()`, plus commented earlier tracking code. |
| [src/TargetPrediction](src/TargetPrediction) | `Extract` decodes APRS into local ENU; `State` stores position, velocity, acceleration; `Propagate` applies a kinematic transition. |
| [src/CoordinateConversion](src/CoordinateConversion) | `Conversion.h/.cpp` define `CoordConvert`, which computes azimuth and elevation. |
| [src/MotorControl](src/MotorControl) | `MotorPins.h/.cpp`: direction, pulse timing, gearing, and blocking moves to commanded angles. |
| [src/Sensors](src/Sensors) | `GPS.h/.cpp` wrap the SparkFun u-blox GNSS driver and report the ground-station position. |
| [src/FakeSensors](src/FakeSensors) | Native-guarded `FakeSAM_M10Q` generates changing GPS-like reporter values. |
| [src/Math](src/Math) | `Vector<N>`, `Quaternion`, and `Matrix` utilities. Preserve embedded third-party license notices. |
| [src/Data/DataReporter](src/Data/DataReporter) | `DataReporter`, `SimpleDataReporter`, and linked `DataPoint` columns. |
| [src/Data/DataLogging](src/Data/DataLogging) | Global `DataLogger` and `EventLogger`. |
| [src/Data/Backend](src/Data/Backend) | `ILogSink`, Print/UART/USB/file/circular-buffer adapters and serial compatibility support. |
| [src/Data/Storage](src/Data/Storage) | `IStorage`, `IFile`, and Teensy SD/file implementations. |
| [src/Data/Retrieval](src/Data/Retrieval) | `FileReader` storage retrieval utility. |
| [src/AstraUtilities](src/AstraUtilities) | Local circular-buffer utility; this directory does not supply an Astra system orchestrator. |
| [src/TerminalReporter.h](src/TerminalReporter.h), [src/TerminalPrint.h](src/TerminalPrint.h), [src/SDLog.cpp](src/SDLog.cpp) | Terminal/reporting helpers and a commented SD logging example. |
| [platformio.ini](platformio.ini) | Active Teensy build environment and declared external libraries; native configuration is commented out. |
| [OldCode entry points](../OldCode) | `gen_main.py` generates/plots noisy trajectories; `filter_main.py` runs an EKF experiment; `pointer.py` estimates pointing rates and accelerations. |
| [OldCode/data_generators](../OldCode/data_generators) | Base generator and two `RocketDataGenerator` implementations for flight and pointing experiments. |
| [OldCode/models](../OldCode/models), [OldCode/atmospheric_model](../OldCode/atmospheric_model) | `Rocket` parameters and `AtmosphereModel` used by simulations. |
| [OldCode/kalman_filters](../OldCode/kalman_filters) | `BaseKalmanFilter`, `BaseExtendedKalmanFilter`, and `RocketEKF`. |
| [OldCode/sensors](../OldCode/sensors), [OldCode/noise_generators](../OldCode/noise_generators) | `Sensor.measure()` and Gaussian, drift, and pink noise models. |
| [OldCode/plot_managers](../OldCode/plot_managers), [OldCode/gear_calculations](../OldCode/gear_calculations) | Plot management and gear analysis scripts. |

`.pio/` is PlatformIO output/dependency/cache state. `lib/` is the conventional
local-library slot; declared dependencies live in `platformio.ini` rather than a
vendored library tree. Treat build caches, coverage output, simulation CSVs,
Python bytecode, and generated compile commands as artifacts.

## Construction, lifetime, and startup

The tracking classes live in the global namespace. The core prediction pipeline
uses the following interfaces (a usage sketch, not a complete hardware application):

```cpp
#include <APRSTelem.h>
#include "TargetPrediction/Extract.h"
#include "TargetPrediction/Propagate.h"
#include "CoordinateConversion/Conversion.h"

APRSTelem telemetry;
Extract extractor(&telemetry);
Propagate predictor;
CoordConvert pointing;

// Set extractor.originLatDeg, originLngDeg, and originAltFt from the
// antenna location before processing a complete, validated APRSTelem packet.
void processTelemetry(const uint8_t* bytes, size_t length,
                      double packetIntervalSec, double timeSinceLaunchSec) {
    State measured = extractor.ExtractTelemetry(
        bytes, length, packetIntervalSec, timeSinceLaunchSec);
    predictor.update(measured);
    predictor.propagate(1.0);
    pointing.convert(predictor.state);
}
```

Configure all objects before initialization. Caller-supplied telemetry objects,
reporters, streams, log sinks, and sink arrays must remain alive while the system
uses them. `Extract` borrows its `APRSData*` and assumes it actually points to an
`APRSTelem`. `Propagate` owns a value `State`. `DataLogger` borrows reporters and
the sink array. `FileLogSink` borrows a supplied storage backend and owns the
opened `IFile`. Avoid copying reporters with owning column/name pointers.

`atsmain.cpp::setup()` starts Serial1 at 9600 baud and Wire, begins/registers the
GPS, attempts SD/logging setup, waits up to ten seconds for a GPS fix, sets the
extractor origin if a fix arrives, and attempts motor initialization.
`GPS::begin()` returns **0 on success** and **-1 on failure**. Logging/storage
`begin()` methods use booleans instead. There is no aggregate readiness result.
The current setup ignores several return values and can leave the origin at
zero after a GPS timeout; completion does not prove readiness to track.

## Runtime data flow and timing

```text
ground GPS ---------------------------> Extract origin
radio UART -> APRSTelem -> Extract ----> State (local ENU)
State -> Propagate -> CoordConvert ----> MotorPins -> antenna axes
GPS / other registered DataReporters --> DataLogger CSV -> ILogSink
event messages -----------------------> EventLogger -> ILogSink

OldCode: Rocket + AtmosphereModel -> generated trajectory -> noise / Sensor
         -> RocketEKF / plots / pointing-rate analysis
```

One intended tracking loop updates GPS and appends a log line, drains Serial1
into a 256-byte buffer, and processes buffered data when Serial1 is empty.
Packet spacing is `(now - lastPacketMs)/1000.0` seconds. A height above the
antenna origin exceeding two feet (0.6096 m) latches `launched`. After launch,
each measurement seeds the propagator, advances it by the fixed one-second
latency, converts to pointing angles, and commands azimuth then elevation.

UART emptiness is not a reliable complete-packet boundary; the current code has
no robust packet framing/overflow recovery or decode-success gate. Preserve the
distinction between packet interval and prediction horizon. `timeSinceLaunch`
is currently reset to zero for every packet before extraction; its later
back-calculation is not retained or used by that extraction.

Time passed to GPS/reporters and `Propagate::propagate(dt)` is in **seconds**.
`millis()` is converted explicitly. GPS configures a device measurement rate;
the loop has no fixed telemetry logging cadence. Motor commands execute blocking
pulse loops, so movement delays subsequent radio reads and logging. Use positive
rates and increasing simulation timestamps so prediction receives positive `dt`.

## Sensors and estimation contracts

`GPS` derives from `DataReporter`. `begin()` configures the u-blox device over
I2C; `update()` reads PVT and returns **0 for success**, **-1** when uninitialized,
no PVT is available, or LLH is invalid. `getHasFix()` currently means at least
four satellites, and `getFixQual()` returns satellite count. Health and
initialization are distinct: a later read can fail after successful initialization.
There is no generic sensor manager or attitude filter in the tracking pipeline.

`Extract::ExtractTelemetry(bytes, length, dt, timeSinceLaunch)` calls `decode()`,
casts the borrowed message to `APRSTelem`, converts position to local ENU, and
decomposes speed/heading into horizontal velocity. First-packet vertical velocity
uses `98.1 * timeSinceLaunch`; later packets use altitude difference divided by
positive `dt`. Acceleration remains zero. Despite old comments, this method
does not run a Kalman correction and does not support arbitrary APRS subclasses.

Preserve the established units and frames:

| Value | Convention |
| --- | --- |
| Ground GPS position | `(latitude degrees, longitude degrees, altitude metres)`; velocity getters use NED in m/s. |
| APRS position / extractor origin | Latitude/longitude in degrees, altitude in feet as consumed by `Extract`. |
| APRS speed / heading | Knots / degrees clockwise from North; converted with 0.514444 m/s per knot. |
| Target `State` | Local ENU: X East, Y North, Z Up; position in metres, velocity in m/s, acceleration in m/s^2. |
| `CoordConvert` output | Azimuth clockwise from North in [0, 360); elevation above the horizon in degrees, including negative values. |
| Motor command | `theta` is gearbox/output angle in degrees; `rpm` is output-axis RPM in the current step-rate calculation. |
| OldCode pointing | Antenna-frame yaw `atan2(y, x)` and pitch in radians; reported rates/accelerations convert to degrees/s and degrees/s^2. |
| OldCode quaternion | `(w, x, y, z)`; inspect each transform's direction before reusing it. |

`Extract::llaToENU()` uses WGS84 local metres-per-degree scalers at the origin
latitude, not a full global ECEF transformation. `Propagate::update()` copies
position/velocity and resets acceleration to zero. `propagate(dt)` multiplies
a nine-value `[px, py, pz, vx, vy, vz, ax, ay, az]` state by a kinematic matrix;
after `update()`, this is constant-velocity prediction, with no covariance fusion.

`CoordConvert::convert()` uses `atan2(East, North)` for azimuth and
`atan2(Up, horizontalDistance)` for elevation. OldCode's `compute_yaw_pitch()`
uses a different zero direction. Trace both transforms before changing orientation
to avoid applying the same rotation twice. Change prediction in
`src/TargetPrediction`, pointing geometry in `src/CoordinateConversion`, and
the experimental Python EKF in `OldCode/kalman_filters`.

## Telemetry, event logs, storage, and commands

`DataReporter` columns hold pointers to live values plus printf formats/labels;
keep those values valid for the reporter's lifetime. Constructors do not globally
register reporters. Call `DataLogger::registerReporter()` explicitly. The logger
has a **32-reporter total capacity**. Begin standalone custom reporters yourself;
`appendLine()` emits their current values without updating them.

`DataLogger` emits CSV header/data, optionally prefixed `TELEM/`. `EventLogger`
uses separate sink configuration and `LOGI`, `LOGW`, `LOGE`, `LOGD`.
`DataLogger::configure()` immediately initializes the sinks and prints headers;
all sink entries must already be valid. No successful telemetry sink means
no telemetry output. `ILogSink` extends Arduino `Print`: UART, USB, generic Print,
circular buffer, and file adapters are present.

`FileLogSink` uses `IStorage`/`IFile` with a caller-provided backend. File naming
avoids overwriting existing logs. The concrete SD backend is in
`Data/Storage/Teensy`; `SDCardStorage` is storage, not an `ILogSink`.
`Data/Retrieval/FileReader` provides retrieval functionality; inspect its blocking
interaction before inserting it into the regular tracking loop.

The radio receive path is direct Serial1 byte handling. There is no
`SerialMessageRouter`, `CMD/PING`, `CMD/PONG`, or `HITL/` command dispatcher in
the current application. Do not infer a command protocol from logging prefixes.

`DataLogger` and `EventLogger` are shared process state. Multiple tracking objects
do not have isolated logging state. Reset shared state in tests where supported
(`DataLogger::reset()`). Motor debug counters `steps` and `stepsOut` are also
file globals shared by both motor instances.

## HITL and SITL

This checkout has no integrated HITL/SITL transport or simulator-to-firmware
protocol. `FakeSAM_M10Q` is a native-guarded synthetic reporter, and `SDLog.cpp`
contains a commented example. The native PlatformIO environment is commented
out and does not establish a working host test application.

OldCode runs independent Python experiments:

- `gen_main.py`: creates a `Rocket`, generates a trajectory, adds sensor noise,
  and uses `PlotManager` to display position, velocity, and Mach plots.
- `filter_main.py`: generates noisy GPS/barometer/acceleration/quaternion
  measurements and iterates a six-state `RocketEKF` for position and velocity.
- `pointer.py`: runs ten randomized trajectories, transforms positions and
  velocities into an antenna frame, unwraps pointing angles, and differentiates
  them to estimate required output-axis rates and accelerations.
- `gear_calculations/`: separate gear analysis scripts.

These experiments use NumPy and Matplotlib. Inspect imports and script parameters
before running them. Randomized results are samples, not guaranteed worst-case
limits. `pointer.py` does not model backlash, torque limits, or motor dynamics;
motor RPM additionally depends on the reduction ratio. Match units, frames,
sampling, and noise assumptions before transferring results into firmware.

## Building and checking changes

The build configuration is [platformio.ini](platformio.ini). From BackendSoftware:

```sh
pio run -e teensy41
```

The first run downloads platforms/libraries and may take several minutes.
Hardware upload and device checks are separate checks requiring the corresponding
controller and wiring. Once a buildable application is selected and verified,
the upload command is `pio run -e teensy41 -t upload`.

| PlatformIO environment | Target / status | Application sources |
| --- | --- | --- |
| `teensy41` | Teensy 4.1, Arduino; active | `build_src_filter = +<*>` currently includes both application entry points. |
| `native` | Commented-out draft | No supported native build/test target is configured. |

Select exactly one application entry point when changing `build_src_filter`.
The current configuration includes duplicate `setup()`/`loop()` definitions and
the integration mismatches below, so this guide does not assert that the current
firmware builds. There is no BackendSoftware `test/` suite, `run_tests.py`,
or Astra-Support test workflow in this checkout.

For Python experiment checks, run the relevant script from OldCode:

```sh
python gen_main.py
python filter_main.py
python pointer.py
```

Plotting scripts may open windows and block. These are experiment entry points,
not automated tests. Add behavioral coverage to the matching component when a
change warrants it. Host checks cannot validate real GPS, SD storage, radio
framing, pulse timing, mechanical travel, or wiring.

Hardware dependencies are declared in `platformio.ini`: the RadioMessage Git
repository and SparkFun u-blox GNSS v3 `^3.1.13`. RadioMessage is not pinned to a
commit. Avoid platform-only includes leaking into native paths; preserve the
platform guards around storage and hardware code. No library manifest or
automatic `ASTRA_VERSION` injection is configured here.

## Documentation and maintenance workflow

Read the [repository README](../README.md), the relevant headers and
implementations, and [platformio.ini](platformio.ini) for configuration.
Use [Extract](src/TargetPrediction/Extract.h),
[State](src/TargetPrediction/State.h),
[Propagate](src/TargetPrediction/Propagate.h), and
[Conversion](src/CoordinateConversion/Conversion.h) for prediction contracts;
[GPS](src/Sensors/GPS.h) for sensor inputs;
[MotorPins](src/MotorControl/MotorPins.h) for outputs; and the
[OldCode scripts](../OldCode) for simulation assumptions.

There is no MkDocs tree or documented documentation-deployment/release workflow
in these software directories. Update this guide and relevant source documentation
when the architecture changes; do not substitute Astra's documentation commands.

When adding a sensor, implement the driver, register its telemetry, wire it
through application setup, and document its units and role. Keep
sensor reads non-blocking where possible and avoid unnecessary new allocations
in repeated update paths. Preserve the established math types and public include
paths. For an API change, check consumers in applications/experiments/tests and
update the relevant documentation and this file's dated history.

Resolve disagreements between older prose and current behavior by reading
headers, implementations, and tests. Known mismatches at ATS adaptation review:

- `atsmain.cpp` includes nonexistent `Data/Storage/SDCardStorage.h` and
  `CoordinateConversion/CoordConvert.h`; the files are under `Storage/Teensy`
  and named `Conversion.h`, respectively.
- Both `atsmain.cpp` and `motormain.cpp` define active Arduino entry points,
  and the source filter includes both.
- `atsmain.cpp` calls `motor_init()` with two arguments; the interface requires
  four. Motor constructor/init parameter names in the header put pulse before
  direction, while the implementation interprets direction before pulse.
- The static logging arrays in `atsmain.cpp` contain null entries and are never
  populated before `configure()` dereferences them. Calling `init()` again also
  repeats initialization already performed by `configure()`.
- Launch time is reset for each packet; a GPS timeout leaves a zero origin;
  UART silence is used as packet completion without validation.
- Old `Extract` comments promise arbitrary APRS subclasses, Kalman filtering,
  or zero vertical velocity. The implementation assumes `APRSTelem`, estimates
  vertical velocity, and leaves filtering to neither `Extract` nor `Propagate`.
- `State` member comments label X/Y as latitude/longitude, but the active
  extraction/conversion contract is X East, Y North. Elevation is not clamped
  to 0-90 degrees despite a source comment.
- `GPS::wrapLongitude()` tests `< 180` where the negative boundary would be
  expected, and `getDisplacement()` uses inconsistent latitude/longitude scalers;
  the tracking path uses `Extract::llaToENU()` instead.
- Motor direction settling uses millisecond `delay()` with a constant named
  in microseconds. The implementation repeats the default `stepsPerRev` argument
  already specified in the header. Movement is open-loop and blocking;
  `get_motor_angle()` tracks motor-shaft angle, not measured output-axis angle.

Before handing off a change, review the diff, run checks appropriate to what
changed, and report which checks actually ran and any remaining limitations.
