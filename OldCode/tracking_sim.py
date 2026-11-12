"""PID antenna tracking experiment using corrected legacy rocket trajectories.

Default: calibrate the rocket thrust-acceleration parameter to 30,000 ft AGL.
Calibration constructs a tracking test case, not a validated rocket prediction.
"""
import argparse
import contextlib
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from data_generators.supersonic_generator import RocketDataGenerator
from models.rocket import Rocket


def wrap(degrees):
    return (np.asarray(degrees) + 180) % 360 - 180


def pointing(relative):
    relative = np.asarray(relative)
    return np.degrees(np.stack((np.arctan2(relative[..., 1], relative[..., 0]),
                               np.arctan2(relative[..., 2], np.hypot(relative[..., 0], relative[..., 1]))), axis=-1))


def boresight(angles):
    yaw, pitch = np.moveaxis(np.radians(angles), -1, 0)
    return np.stack((np.cos(pitch)*np.cos(yaw), np.cos(pitch)*np.sin(yaw), np.sin(pitch)), axis=-1)


def separation(a, b):
    # atan2 is well conditioned for very small pointing errors.
    u, v = boresight(a), boresight(b)
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(u, v), axis=-1), np.sum(u*v, axis=-1)))


def angular_derivatives(angles, times):
    """Unwrap azimuth before differentiating sampled angles in degrees."""
    angles = np.array(angles, dtype=float, copy=True)
    angles[:, 0] = np.degrees(np.unwrap(np.radians(angles[:, 0])))
    rate = np.gradient(angles, times, axis=0)
    return rate, np.gradient(rate, times, axis=0)


def motion_peaks(times, rate, acceleration, mask, ratios):
    """Absolute per-axis peaks, their timestamps, and geared motor equivalents."""
    indices = np.flatnonzero(mask)
    if not len(indices):
        raise ValueError("No flight samples available for angular-motion statistics")
    axes = {}
    for i, (axis, ratio) in enumerate(zip(("azimuth", "elevation"), ratios)):
        def peak(values):
            index = indices[np.argmax(np.abs(values[indices]))]
            return {"value": float(abs(values[index])),
                    "time_since_launch_s": float(times[index]-10)}
        axes[axis] = {
            "max_output_velocity_deg_s": peak(rate[:, i]),
            "max_output_acceleration_deg_s2": peak(acceleration[:, i]),
            "max_motor_velocity_rpm": peak(rate[:, i]*ratio/6),
            "max_motor_acceleration_deg_s2": peak(acceleration[:, i]*ratio),
        }
    return axes


def communication_status(now, last_arrival, timeout):
    """1: fresh reception, 0: waiting initially, -1: reception timed out.

    Receipt time measures link health; measurement age measures prediction age.
    """
    if last_arrival is None:
        return -1 if now >= timeout else 0
    return -1 if now-last_arrival >= timeout else 1


def intervals(times, mask):
    """Half-open masked spans in the supplied time frame, capped at final time."""
    edges = np.diff(np.r_[False, mask, False].astype(int))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    return [(float(times[start]), float(times[min(end, len(times)-1)]))
            for start, end in zip(starts, ends)]


def show_communication(panel, times, status):
    panel.step(times, status, where="post", color="black", linewidth=1)
    panel.fill_between(times, -1.4, 1.4, where=status == 1, color="green",
                       alpha=.12, step="post")
    panel.set(yticks=[-1, 0, 1], yticklabels=["LOST", "Waiting", "Receiving"],
              ylim=(-1.4, 1.4), ylabel="Telemetry link", xlabel="Time since launch (s)")


def shade_outages(panel, spans):
    for i, (start, end) in enumerate(spans):
        panel.axvspan(start, end, color="red", alpha=.15,
                     label="Telemetry timed out" if i == 0 else None)


@dataclass
class AxisPID:
    """Ideal encoder-feedback PID commanding acceleration in output deg/s^2.

    Derivative on measured angle avoids setpoint derivative kick. Conditional
    integration prevents winding up against acceleration and speed saturation.
    This is a kinematic plant, not an electrical/torque or missed-step model.
    """
    angle: float = 0.0
    rate: float = 0.0
    integral: float = 0.0
    kp: float = 25.0
    ki: float = 0.5
    kd: float = 10.0
    max_speed: float = 40.0
    max_accel: float = 100.0
    circular: bool = False

    def step(self, target, dt):
        error = float(wrap(target-self.angle)) if self.circular else target-self.angle
        trial = self.integral + error*dt

        def action(integral):
            raw = self.kp*error + self.ki*integral - self.kd*self.rate
            accel = np.clip(raw, -self.max_accel, self.max_accel)
            rate = np.clip(self.rate + accel*dt, -self.max_speed, self.max_speed)
            return raw, float(rate)

        raw, rate = action(trial)
        effective = (rate-self.rate)/dt
        if error*(raw-effective) <= 1e-10:
            self.integral = trial
        else:
            _, rate = action(self.integral)
        accel = (rate-self.rate)/dt
        self.angle += 0.5*(self.rate+rate)*dt
        self.rate = rate
        return accel


def flight(args, motor_accel, log):
    rocket = Rocket(motor_accel, 4.83, 3.835, .1524, 40.8, 60.8, 5e-6)
    gen = RocketDataGenerator(rocket, loop_frequency=args.flight_hz, pre_launch_delay=10,
                              launch_angle=args.launch_angle, heading_angle=args.heading,
                              wind_affector=lambda t: np.zeros(3))
    with contextlib.redirect_stdout(log):
        return gen.generate()


def build_flight(args, log):
    original = flight(args, args.motor_accel, log)
    baseline = float(np.max(original["r_z"]))
    if args.target_apogee_ft == 0:
        return original, args.motor_accel, baseline
    target = args.target_apogee_ft*0.3048
    lo, hi = 15.0, max(args.motor_accel, 250.0)
    low_data, high_data = flight(args, lo, log), flight(args, hi, log)
    while np.max(high_data["r_z"]) < target and hi < 1000:
        hi *= 1.25
        high_data = flight(args, hi, log)
    if not np.max(low_data["r_z"]) <= target <= np.max(high_data["r_z"]):
        raise ValueError("Cannot bracket target apogee; use a realistic target or disable calibration")
    for _ in range(24):
        mid = (lo+hi)/2
        data = flight(args, mid, log)
        apogee = float(np.max(data["r_z"]))
        if abs(apogee-target) <= 1.0:
            return data, mid, baseline
        if apogee < target:
            lo = mid
        else:
            hi = mid
    raise RuntimeError("Apogee calibration did not converge to 1 metre")


def simulate(args, data):
    dt = 1/args.control_hz
    times = np.arange(0, data["time"][-1] + 1e-9, dt)
    position = np.column_stack([np.interp(times, data["time"], data[f"r_{a}"]) for a in "xyz"])
    velocity = np.column_stack([np.interp(times, data["time"], data[f"v_{a}"]) for a in "xyz"])
    antenna = np.array([-args.distance, 0.0, 0.0])
    truth = pointing(position-antenna)
    rng = np.random.default_rng(args.seed)
    packets = []
    for stamp in np.arange(0, times[-1]+1e-9, 1/args.telemetry_hz):
        if rng.random() < args.drop_probability:
            continue
        p = np.array([np.interp(stamp, times, position[:, i]) for i in range(3)])
        v = np.array([np.interp(stamp, times, velocity[:, i]) for i in range(3)])
        p += rng.normal(0, args.position_sigma, 3)
        v += rng.normal(0, args.velocity_sigma, 3)
        delay = max(0.0, args.latency + rng.normal(0, args.jitter))
        if args.blackout is not None and 10+args.blackout[0] <= stamp+delay < 10+args.blackout[1]:
            continue  # A forced receive blackout, in time since launch.
        packets.append((stamp+delay, stamp, p, v))
    packets.sort(key=lambda packet: packet[0])
    axes = [[AxisPID(angle=float(truth[0, i]), kp=args.kp, ki=args.ki, kd=args.kd,
                     max_speed=args.max_speed[i], max_accel=args.max_accel[i], circular=i == 0)
             for i in range(2)] for _ in range(2)]
    actual = np.zeros((len(times), 2, 2))  # time, strategy (hold/predict), axis
    rates, accelerations, commands = [np.zeros_like(actual) for _ in range(3)]
    ages = np.full(len(times), np.nan)
    valid_packet = np.zeros(len(times), dtype=bool)
    status = np.zeros(len(times), dtype=int)
    reception_age = np.full(len(times), np.nan)
    # Before the first arrival, point to the known pad without flight knowledge.
    latest = None
    last_arrival = None
    next_packet = 0
    for k, now in enumerate(times):
        while next_packet < len(packets) and packets[next_packet][0] <= now+1e-10:
            packet = packets[next_packet]
            # A late old packet must not replace a newer measurement.
            if latest is None or packet[1] > latest[1]:
                latest = packet
                last_arrival = packet[0]
            next_packet += 1
        if latest is None:
            target = np.tile(truth[0], (2, 1))
        else:
            _, stamp, p, v = latest
            age = max(0.0, now-stamp)
            ages[k], valid_packet[k] = age, True
            target = pointing(np.stack((p, p+v*age))-antenna)
        status[k] = communication_status(now, last_arrival, args.comm_timeout)
        if last_arrival is not None:
            reception_age[k] = max(0.0, now-last_arrival)
        commands[k] = target
        for strategy in range(2):
            for i, axis in enumerate(axes[strategy]):
                # State at now; target from now acts over the following interval.
                actual[k, strategy, i] = axis.angle
                rates[k, strategy, i] = axis.rate
                if k < len(times)-1:
                    accelerations[k, strategy, i] = axis.step(target[strategy, i], dt)
    errors = np.column_stack([separation(truth, actual[:, s]) for s in range(2)])
    projected_errors = np.column_stack([separation(commands[:, s], actual[:, s]) for s in range(2)])
    prediction_errors = np.column_stack([separation(commands[:, s], truth) for s in range(2)])
    return dict(time=times, position=position, velocity=velocity, truth=truth,
                actual=actual, rate=rates, accel=accelerations, command=commands,
                error=errors, projected_error=projected_errors, prediction_error=prediction_errors, age=ages,
                valid_packet=valid_packet, communication_status=status, reception_age=reception_age)


def export(args, data, result, fitted, baseline):
    t = result["time"]-10
    airborne = t >= 0
    # Differentiate truth on the flight integrator's native grid. Differentiating
    # linearly upsampled positions twice would add interpolation artifacts.
    native_angles = pointing(np.column_stack([data[f"r_{a}"] for a in "xyz"])
                             - np.array([-args.distance, 0, 0]))
    native_rate, native_accel = angular_derivatives(native_angles, data["time"])
    ratios = (args.azimuth_ratio, args.elevation_ratio)
    angular_motion = {"ideal_tracking": motion_peaks(data["time"], native_rate, native_accel,
                                                    data["time"] >= 10, ratios)}
    command_motion = []
    for s, name in enumerate(("hold", "predicted")):
        rate, acceleration = angular_derivatives(result["command"][:, s], result["time"])
        command_motion.append((rate, acceleration))
        angular_motion[f"{name}_command"] = motion_peaks(result["time"], rate, acceleration, airborne, ratios)
        angular_motion[f"{name}_pid"] = motion_peaks(result["time"], result["rate"][:, s],
                                                    result["accel"][:, s], airborne, ratios)
    outage_spans = intervals(t, result["communication_status"] == -1)
    flight_spans = [(max(0, start), end) for start, end in outage_spans if end > 0]
    metrics = {}
    for s, name in enumerate(("hold_last_position", "constant_velocity_prediction")):
        error = result["error"][airborne, s]
        metrics[name] = dict(peak_error_deg=float(np.max(error)),
                             rms_error_deg=float(np.sqrt(np.mean(error**2))),
                             p95_error_deg=float(np.percentile(error, 95)),
                             fraction_inside_beam=float(np.mean(error <= args.beamwidth/2)),
                             peak_projected_actual_error_deg=float(np.max(result["projected_error"][airborne, s])),
                             peak_prediction_error_deg=float(np.max(result["prediction_error"][airborne, s])))
    summary = dict(settings={**vars(args), "output": str(args.output)},
                   baseline_corrected_apogee_m=baseline,
                   calibrated_motor_accel_m_s2=fitted,
                   apogee_m=float(np.max(data["r_z"])),
                   apogee_ft=float(np.max(data["r_z"])/0.3048),
                   peak_rocket_speed_m_s=float(np.max(np.linalg.norm(result["velocity"], axis=1))),
                   metrics=metrics,
                   angular_motion=angular_motion,
                   angular_motion_notes=["Absolute peaks during flight, with times relative to launch.",
                                         "Ideal tracking derivatives use the native flight sampling grid.",
                                         "Command derivatives include packet/noise jumps and depend on control timestep; not physical motor capability.",
                                         "Actual PID peaks obey assumed kinematic limits, not a torque-speed motor model.",
                                         "Motor RPM = output deg/s * reduction / 6; motor acceleration = output acceleration * reduction."],
                   communication=dict(timeout_s=args.comm_timeout,
                                      loss_detected_during_flight=bool(flight_spans),
                                      lost_duration_s=sum(end-start for start, end in flight_spans),
                                      lost_intervals_since_launch_s=flight_spans,
                                      definition="No newer measurement received within timeout; not an RF link budget or beam-loss model."),
                   assumptions=["Target is altitude above launchpad (AGL), not sea level.",
                                "Apogee calibration changes thrust acceleration, not altitude samples.",
                                "Legacy atmosphere/drag and approximate rocket rotation retained; no recovery.",
                                "Ideal angle feedback; PID commands kinematic acceleration, not electrical torque.",
                                "Motor limits and beamwidth are scenario assumptions, not measured ratings.",
                                "No backlash, missed steps, mount bias, or covariance model.",
                                "Fixed masses 60.8/40.8 kg, burn 4.83 s; still air."])
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    columns = {"time_since_launch_s": t}
    for i, a in enumerate("xyz"):
        columns[f"rocket_{a}_m"] = result["position"][:, i]
    columns["telemetry_age_s"] = result["age"]
    columns["has_received_packet"] = result["valid_packet"].astype(int)
    columns["seconds_since_fresh_packet_arrival"] = result["reception_age"]
    columns["communication_status"] = result["communication_status"]
    for s, strategy in enumerate(("hold", "predict")):
        columns[f"{strategy}_pointing_error_deg"] = result["error"][:, s]
        columns[f"{strategy}_projected_actual_error_deg"] = result["projected_error"][:, s]
        columns[f"{strategy}_prediction_error_deg"] = result["prediction_error"][:, s]
        for i, axis in enumerate(("azimuth", "elevation")):
            columns[f"true_{axis}_deg"] = result["truth"][:, i]
            columns[f"required_{axis}_rate_deg_s"] = np.interp(result["time"], data["time"], native_rate[:, i])
            columns[f"required_{axis}_accel_deg_s2"] = np.interp(result["time"], data["time"], native_accel[:, i])
            columns[f"{strategy}_{axis}_command_rate_deg_s"] = command_motion[s][0][:, i]
            columns[f"{strategy}_{axis}_command_accel_deg_s2"] = command_motion[s][1][:, i]
            signed = result["command"][:, s, i]-result["actual"][:, s, i]
            columns[f"{strategy}_{axis}_projected_minus_actual_deg"] = wrap(signed) if i == 0 else signed
            prediction_error = result["command"][:, s, i]-result["truth"][:, i]
            columns[f"{strategy}_{axis}_projected_minus_true_deg"] = wrap(prediction_error) if i == 0 else prediction_error
            for key, unit in (("command", "deg"), ("actual", "deg"), ("rate", "deg_s"), ("accel", "deg_s2")):
                columns[f"{strategy}_{axis}_{key}_{unit}"] = result[key][:, s, i]
    np.savetxt(args.output/"history.csv", np.column_stack(list(columns.values())),
               delimiter=",", header=",".join(columns), comments="")
    # All plot comparisons use the same truth, packet draws and motor limits.
    fig, panels = plt.subplots(4, 2, figsize=(14, 13), sharex=True)
    unwrapped = np.degrees(np.unwrap(np.radians(result["truth"]), axis=0))
    required_rate = np.column_stack([np.interp(result["time"], data["time"], native_rate[:, i]) for i in range(2)])
    required_accel = np.column_stack([np.interp(result["time"], data["time"], native_accel[:, i]) for i in range(2)])
    for i, axis in enumerate(("Azimuth", "Elevation")):
        panels[0, i].plot(t, unwrapped[:, i], label="True direction")
        panels[0, i].plot(t, np.degrees(np.unwrap(np.radians(result["command"][:, 1, i]))), label="Predicted command", alpha=.7)
        panels[0, i].plot(t, result["actual"][:, 1, i], label="Actual PID angle")
        panels[0, i].set(title=axis, ylabel="Angle (deg)")
        panels[1, i].plot(t, required_rate[:, i], label="Required")
        panels[1, i].plot(t, result["rate"][:, 1, i], label="Actual")
        panels[2, i].plot(t, required_accel[:, i], label="Required")
        panels[2, i].plot(t, result["accel"][:, 1, i], label="Actual PID acceleration")
        for row, limit, label in ((1, args.max_speed[i], "Rate (deg/s)"),
                                  (2, args.max_accel[i], "Acceleration (deg/s²)")):
            panels[row, i].axhline(limit, color="red", linestyle="--", label="Assumed limit")
            panels[row, i].axhline(-limit, color="red", linestyle="--")
            panels[row, i].set_ylabel(label)
        signed = result["actual"][:, 1, i]-result["truth"][:, i]
        if i == 0:
            signed = wrap(signed)
        panels[3, i].plot(t, signed, label="Actual - true")
        panels[3, i].set(ylabel="Signed axis error (deg)", xlabel="Time since launch (s)")
    for panel in panels.flat:
        panel.grid(alpha=.3)
        panel.legend(fontsize=8)
        panel.set_xlim(0, t[-1])
    fig.suptitle("PID tracking with delayed, noisy telemetry — assumed motor limits")
    fig.tight_layout()
    fig.savefig(args.output/"tracking.png", dpi=160)
    plt.close(fig)
    fig, panels = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                               gridspec_kw={"height_ratios": [3, 2, 1]})
    for s, label in enumerate(("PID: hold last position", "PID: predict to current time")):
        panels[0].plot(t, result["error"][:, s], label=label)
    panels[0].axhline(args.beamwidth/2, color="red", linestyle="--", label="Assumed half-power beam radius")
    shade_outages(panels[0], outage_spans)
    panels[0].set(ylabel="Boresight separation (deg)", title="Total pointing error: prediction comparison")
    panels[0].legend()
    panels[1].plot(t, result["age"])
    panels[1].set(ylabel="Telemetry age (s)", xlabel="Time since launch (s)")
    show_communication(panels[2], t, result["communication_status"])
    shade_outages(panels[2], outage_spans)
    for panel in panels:
        panel.grid(alpha=.3)
        panel.set_xlim(0, t[-1])
    fig.tight_layout()
    fig.savefig(args.output/"pointing_error.png", dpi=160)
    plt.close(fig)
    fig, panels = plt.subplots(4, 1, figsize=(12, 11), sharex=True,
                               gridspec_kw={"height_ratios": [2, 2, 2, 1]})
    for i, axis in enumerate(("Azimuth", "Elevation")):
        signed = result["command"][:, 1, i]-result["actual"][:, 1, i]
        if i == 0:
            signed = wrap(signed)
        panels[i].plot(t, signed, label=f"{axis}: projected command - actual antenna")
        prediction_error = result["command"][:, 1, i]-result["truth"][:, i]
        panels[i].plot(t, wrap(prediction_error) if i == 0 else prediction_error,
                       linestyle="--", label=f"{axis}: projected command - true rocket")
        panels[i].axhline(0, color="gray", linewidth=.7)
        panels[i].set_ylabel(f"{axis} error (deg)")
    panels[0].set_title("Projected vs actual antenna angle — velocity-predicted PID")
    panels[2].plot(t, result["projected_error"][:, 1], label="Projected direction vs actual antenna")
    panels[2].plot(t, result["error"][:, 1], linestyle="--", label="True rocket direction vs actual antenna")
    panels[2].plot(t, result["prediction_error"][:, 1], linestyle=":", label="Projected direction vs true rocket")
    panels[2].set_ylabel("Boresight separation (deg)")
    show_communication(panels[3], t, result["communication_status"])
    for panel in panels:
        shade_outages(panel, outage_spans)
        panel.grid(alpha=.3)
        panel.set_xlim(0, t[-1])
    for panel in panels[:3]:
        panel.legend(fontsize=9)
    panels[3].set_title(f"LOST = no fresh packet for {args.comm_timeout:g} s; beam alignment is separate", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.output/"projected_actual_error.png", dpi=160)
    plt.close(fig)
    fig, panel = plt.subplots(figsize=(10, 5))
    panel.plot(data["time"]-10, data["r_z"]/0.3048, label="Simulated altitude")
    if args.target_apogee_ft:
        panel.axhline(args.target_apogee_ft, linestyle="--", color="red", label="Calibration target")
    panel.set(xlabel="Time since launch (s)", ylabel="Altitude above launchpad (ft)",
              title="Rocket trajectory — calibrated test case, not validated flight prediction", xlim=(0, t[-1]))
    panel.grid(alpha=.3)
    panel.legend()
    fig.tight_layout()
    fig.savefig(args.output/"trajectory.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print("\nMAXIMUM ANGULAR MOTION (absolute peaks during flight)")
    print(f"{'Source / axis':<30} {'Output deg/s':>13} {'Output deg/s^2':>15} {'Motor RPM':>12} {'Motor deg/s^2':>15}")
    for source, axes in angular_motion.items():
        for axis, peaks in axes.items():
            values = [peaks[key]["value"] for key in ("max_output_velocity_deg_s", "max_output_acceleration_deg_s2",
                                                     "max_motor_velocity_rpm", "max_motor_acceleration_deg_s2")]
            print(f"{source+' / '+axis:<30} {values[0]:13.3f} {values[1]:15.3f} {values[2]:12.3f} {values[3]:15.3f}")
    print("Command peaks include telemetry jumps. PID peaks use assumed limits, not measured motor capability.")


def positive(text):
    value = float(text)
    if not np.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def nonnegative(text):
    value = float(text)
    if not np.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("results/pid"))
    parser.add_argument("--target-apogee-ft", type=nonnegative, default=30000, help="AGL; 0 disables calibration")
    parser.add_argument("--motor-accel", type=positive, default=201.5, help="rocket thrust acceleration parameter")
    parser.add_argument("--flight-hz", type=positive, default=20)
    parser.add_argument("--control-hz", type=positive, default=100)
    parser.add_argument("--launch-angle", type=float, default=0, help="degrees from vertical")
    parser.add_argument("--heading", type=float, default=0, help="radians")
    parser.add_argument("--distance", type=positive, default=804.672)
    parser.add_argument("--telemetry-hz", type=positive, default=5)
    parser.add_argument("--latency", type=nonnegative, default=0.3)
    parser.add_argument("--jitter", type=nonnegative, default=0.05)
    parser.add_argument("--position-sigma", type=nonnegative, default=3, help="metres per axis")
    parser.add_argument("--velocity-sigma", type=nonnegative, default=1, help="m/s per axis")
    parser.add_argument("--drop-probability", type=nonnegative, default=0)
    parser.add_argument("--comm-timeout", type=positive, default=1, help="seconds since last fresh packet arrival before declaring telemetry lost")
    parser.add_argument("--blackout", nargs=2, type=nonnegative, default=None, metavar=("START", "END"),
                        help="forced reception blackout, seconds since launch")
    parser.add_argument("--max-speed", nargs=2, type=positive, default=[40, 40], metavar=("AZ", "EL"), help="output deg/s")
    parser.add_argument("--max-accel", nargs=2, type=positive, default=[100, 100], metavar=("AZ", "EL"), help="output deg/s^2")
    parser.add_argument("--azimuth-ratio", type=positive, default=50, help="motor revolutions per azimuth output revolution")
    parser.add_argument("--elevation-ratio", type=positive, default=10, help="motor revolutions per elevation output revolution")
    parser.add_argument("--kp", type=nonnegative, default=25)
    parser.add_argument("--ki", type=nonnegative, default=0.5)
    parser.add_argument("--kd", type=nonnegative, default=10)
    parser.add_argument("--beamwidth", type=positive, default=20, help="assumed full half-power beamwidth, deg")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.drop_probability > 1:
        parser.error("drop probability must be between 0 and 1")
    if args.blackout is not None and args.blackout[1] <= args.blackout[0]:
        parser.error("blackout END must be greater than START")
    if not np.isfinite(args.launch_angle) or abs(args.launch_angle) > 10 or not np.isfinite(args.heading):
        parser.error("heading must be finite and launch angle within +/-10 degrees for this experiment")
    args.output.mkdir(parents=True, exist_ok=True)
    print("Generating corrected flight and calibrating apogee (if enabled)...", flush=True)
    with (args.output/"generator.log").open("w", encoding="utf-8") as log:
        data, fitted, baseline = build_flight(args, log)
    print("Simulating PID and exporting plots...", flush=True)
    export(args, data, simulate(args, data), fitted, baseline)


if __name__ == "__main__":
    main()
