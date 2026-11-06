"""Export one reproducible flight and its ideal tracking demands.

This uses the corrected legacy integrators; see README.md before
using the results to size motors. No motor torque or capability is simulated.
"""
import argparse
import contextlib
import json
from pathlib import Path

import numpy as np

from atmospheric_model.atmosphere import AtmosphereModel
from models.rocket import Rocket


def positive(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=["standard", "pointer"], default="standard")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--hz", type=positive, default=20)
    parser.add_argument("--motor-accel", type=positive, default=201.5,
                        help="rocket thrust acceleration parameter, m/s^2")
    parser.add_argument("--burn-time", type=positive, default=4.83)
    parser.add_argument("--launch-angle", type=float, default=0, help="degrees from vertical")
    parser.add_argument("--heading", type=float, default=0, help="radians about world Z")
    parser.add_argument("--distance", type=positive, default=804.672, help="antenna distance, metres")
    parser.add_argument("--yaw-ratio", type=positive, default=1, help="motor revs per output rev")
    parser.add_argument("--pitch-ratio", type=positive, default=1)
    parser.add_argument("--output", type=Path, default=Path("results/flight"))
    args = parser.parse_args()
    if not np.isfinite(args.launch_angle) or abs(args.launch_angle) >= 90:
        parser.error("launch angle must be finite and between -90 and 90 degrees")
    if not np.isfinite(args.heading):
        parser.error("heading must be finite")

    if args.model == "standard":
        from data_generators.supersonic_generator import RocketDataGenerator
    else:
        from data_generators.worst_case_pointer_supersonic_generator import RocketDataGenerator

    np.random.seed(args.seed)
    rocket = Rocket(args.motor_accel, args.burn_time, 3.835, .1524, 40.8, 60.8, 5e-6)
    delay = 10.0
    generator = RocketDataGenerator(
        rocket=rocket, loop_frequency=args.hz, pre_launch_delay=delay,
        launch_angle=args.launch_angle, heading_angle=args.heading,
        wind_affector=lambda t: np.zeros(3),
    )
    args.output.mkdir(parents=True, exist_ok=True)
    # Keep the original drag diagnostics and AOA warnings available for review.
    with (args.output / "generator.log").open("w", encoding="utf-8") as log:
        with contextlib.redirect_stdout(log):
            data = generator.generate()

    time = data["time"]
    position = np.column_stack([data[f"r_{axis}"] for axis in "xyz"])
    velocity = np.column_stack([data[f"v_{axis}"] for axis in "xyz"])
    acceleration = np.column_stack([data[f"a_{axis}"] for axis in "xyz"])
    # Same level frame and antenna location as pointer.py.
    relative = position - np.array([-args.distance, 0, 0])
    horizontal = np.hypot(relative[:, 0], relative[:, 1])
    if np.any(horizontal < 1e-6):
        raise ValueError("Trajectory crosses the azimuth singularity above the antenna")
    angles = np.degrees(np.column_stack([
        np.unwrap(np.arctan2(relative[:, 1], relative[:, 0])),
        np.arctan2(relative[:, 2], horizontal),
    ]))
    rates = np.gradient(angles, time, axis=0)
    angular_accel = np.gradient(rates, time, axis=0)
    speed = np.linalg.norm(velocity, axis=1)
    accel = np.linalg.norm(acceleration, axis=1)
    atmosphere = AtmosphereModel()
    # Restrict statistics to above-ground flight rather than the prelaunch pad.
    airborne = (time >= delay) & (position[:, 2] >= 0)
    if not np.any(airborne):
        raise ValueError("No above-ground flight samples")
    mach = np.array([speed[i] / atmosphere.get_conditions(max(z, 0))["speed_of_sound"]
                     for i, z in enumerate(position[:, 2])])

    def peak(values):
        indices = np.flatnonzero(airborne)
        index = indices[np.argmax(values[airborne])]
        return {"value": float(values[index]), "time_since_launch_s": float(time[index] - delay)}

    summary = {
        "settings": {**vars(args), "output": str(args.output)},
        "fixed_settings": {"pre_launch_delay_s": delay, "wind_m_s": [0, 0, 0],
                           "length_m": 3.835, "diameter_m": .1524,
                           "mass_empty_kg": 40.8, "mass_full_kg": 60.8,
                           "roughness_m": 5e-6, "CoG_m": 2.7686, "CoP_m": 3.180},
        "warning": "Corrected legacy integration; approximate physics. Sampled tracking demands, not motor capability or guaranteed maxima.",
        "apogee_m": peak(position[:, 2]),
        "max_speed_m_s": peak(speed),
        "max_mach_still_air": peak(mach),
        "max_net_acceleration_m_s2": peak(accel),
        "max_net_acceleration_g": peak(accel / 9.80665),
        "last_above_ground_time_since_launch_s": float(time[airborne][-1] - delay),
    }
    for i, (axis, ratio) in enumerate((("yaw", args.yaw_ratio), ("pitch", args.pitch_ratio))):
        summary[f"{axis}_max_output_speed_deg_s"] = peak(np.abs(rates[:, i]))
        summary[f"{axis}_max_output_acceleration_deg_s2"] = peak(np.abs(angular_accel[:, i]))
        summary[f"{axis}_max_motor_speed_rpm"] = peak(np.abs(rates[:, i]) * ratio / 6)
        summary[f"{axis}_max_motor_acceleration_deg_s2"] = peak(np.abs(angular_accel[:, i]) * ratio)

    columns = {"time_s": time, "time_since_launch_s": time - delay}
    columns.update({key: data[key] for key in
                    [f"{prefix}_{axis}" for prefix in ("r", "v", "a") for axis in "xyz"]})
    columns.update(speed_m_s=speed, net_acceleration_m_s2=accel, mach_still_air=mach)
    for i, axis in enumerate(("yaw", "pitch")):
        columns[f"{axis}_deg"] = angles[:, i]
        columns[f"{axis}_deg_s"] = rates[:, i]
        columns[f"{axis}_deg_s2"] = angular_accel[:, i]
    np.savetxt(args.output / "history.csv", np.column_stack(list(columns.values())),
               delimiter=",", header=",".join(columns), comments="")
    report = json.dumps(summary, indent=2, allow_nan=False)
    (args.output / "summary.json").write_text(report + "\n", encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
