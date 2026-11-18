"""Two bounded geometry studies using the supplied coupled mount dynamics.

Optimize gamma1 and d only. Fixed torque-PID gains and identical telemetry are
used for all candidates. Results are scenario optima, not certified hardware limits.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from models.mount_dynamics import MountDynamics
from tracking_sim import (angular_derivatives, build_flight, export,
                          positive, nonnegative, separation, simulate)


def model_for(gamma, d, config):
    return MountDynamics(gamma1=np.radians(gamma), d=np.asarray(d), a=config["a_m"], s=config["s_m"],
                         mass=config["mass_kg"], radius=config["radius_m"], length=config["L_m"],
                         base_azimuth_inertia=config["base_azimuth_inertia"],
                         support_elevation_inertia=config["support_elevation_inertia"],
                         damping_phi=config["damping"][0], damping_psi=config["damping"][1],
                         gear_efficiency=config["efficiency"])


def torque_capacity(rate, config):
    """Motor torque cap, N*m; rough FY56ED420A 24 V curve from supplied image.

    Linear interpolation is an explicitly approximate actuator envelope, derated
    by usable_torque_fraction. It is not a measured pull-in or acceleration limit.
    """
    rpm = np.abs(rate)*np.array([50, 10])*60/(2*np.pi)
    return config["usable_torque_fraction"]*np.interp(
        rpm, config["curve_rpm"], config["curve_nm"], right=0)


def coupled_simulation(source, model, config, targets=None, history=False):
    """Vectorized candidate evaluation; torque is constant across each RK4 step.

    PID torque = Kp*angle_error + Ki*integral - Kd*rate + gravity compensation.
    Saturation is motor torque versus speed. No artificial acceleration clipping.
    """
    times = source["time"]
    count = np.size(model.gamma1)
    q = model.from_sky(np.tile(np.radians(source["truth"][0]), (count, 1)))
    rate, integral = np.zeros_like(q), np.zeros_like(q)
    ratios = np.array([50, 10])
    scale = config["efficiency"]*ratios
    kp, ki, kd = (np.array(config[key]) for key in ("kp", "ki", "kd"))
    peak_error, error_squared, samples = np.zeros(count), np.zeros(count), 0
    peak_motor = np.zeros((count, 2))
    peak_requested_motor = np.zeros_like(peak_motor)
    saved = {key: np.zeros((len(times), count, 2)) for key in
             ("actual", "rate", "accel", "motor_torque", "requested_motor_torque", "output_torque")} if history else None

    def rhs(state, applied):
        return np.concatenate((state[:, 2:], model.forward(state[:, :2], state[:, 2:], applied)), axis=1)

    for k, now in enumerate(times):
        sky = np.degrees(model.to_sky(q))
        if now >= 10:
            error = separation(source["truth"][k], sky)
            peak_error = np.maximum(peak_error, error)
            error_squared += error**2
            samples += 1
        if saved is not None:
            saved["actual"][k] = sky
            saved["rate"][k] = np.degrees(rate)*[1, -1]
        if k == len(times)-1:
            break
        dt = times[k+1]-now
        command = source["command"][k, 1] if targets is None else targets[k]
        target_q = model.from_sky(np.radians(command))
        error = target_q-q
        error[:, 0] = (error[:, 0]+np.pi) % (2*np.pi)-np.pi
        trial = integral+error*dt
        _, _, _, gravity = model.coefficients(q[:, 1])
        compensation = np.column_stack((np.zeros(count), -gravity))
        raw = kp*error+ki*trial-kd*rate+compensation
        cap = torque_capacity(rate, config)*scale
        applied = np.clip(raw, -cap, cap)
        # Prevent integral growth into saturation; allow it to unwind.
        accept = error*(raw-applied) <= 1e-12
        integral = np.where(accept, trial, integral)
        raw = kp*error+ki*integral-kd*rate+compensation
        applied = np.clip(raw, -cap, cap)
        acceleration = model.forward(q, rate, applied)
        if now >= 10:
            peak_motor = np.maximum(peak_motor, np.abs(applied/scale))
            peak_requested_motor = np.maximum(peak_requested_motor, np.abs(raw/scale))
        if saved is not None:
            saved["accel"][k] = np.degrees(acceleration)*[1, -1]
            saved["motor_torque"][k] = applied/scale
            saved["requested_motor_torque"][k] = raw/scale
            saved["output_torque"][k] = applied
        state = np.concatenate((q, rate), axis=1)
        a = rhs(state, applied)
        b = rhs(state+.5*dt*a, applied)
        c = rhs(state+.5*dt*b, applied)
        d = rhs(state+dt*c, applied)
        state += dt*(a+2*b+2*c+d)/6
        q, rate = state[:, :2], state[:, 2:]
        if not np.all(np.isfinite(state)) or np.max(np.abs(state)) > 1e6:
            raise RuntimeError("Unstable integration; reduce control timestep or check PID gains")
    return dict(peak_error_deg=peak_error, rms_error_deg=np.sqrt(error_squared/max(samples, 1)),
                peak_motor_torque_nm=peak_motor, peak_requested_motor_torque_nm=peak_requested_motor,
                history=saved)


def ideal_torques(data, model, efficiency):
    angles = np.degrees(np.stack((np.arctan2(data["r_y"], data["r_x"]+804.672),
                                  np.arctan2(data["r_z"], np.hypot(data["r_x"]+804.672, data["r_y"]))), axis=-1))
    rates, accel = angular_derivatives(angles, data["time"])
    q = model.from_sky(np.radians(angles[:, None, :]))
    rate = np.radians(rates[:, None, :])*[1, -1]
    acceleration = np.radians(accel[:, None, :])*[1, -1]
    motor = model.inverse(q, rate, acceleration)/(efficiency*np.array([50, 10]))
    return np.max(np.abs(motor[data["time"] >= 10]), axis=0)


def evaluate(gamma, d, source, data, config):
    model = model_for(gamma, d, config)
    performance = coupled_simulation(source, model, config)
    required = ideal_torques(data, model, config["efficiency"])
    return np.column_stack((gamma, d, np.max(required, axis=1), performance["peak_error_deg"],
                            performance["rms_error_deg"], required,
                            performance["peak_motor_torque_nm"], performance["peak_requested_motor_torque_nm"]))


def run_configured_mount(source, config):
    """Use a saved study geometry/controller in tracking_sim, for both strategies."""
    model = model_for(np.repeat(config["gamma1_deg"], 2), np.repeat(config["d_m"], 2), config)
    result = coupled_simulation(source, model, config, targets=source["command"], history=True)
    output = dict(source)
    output.update(result["history"])
    output["error"] = np.column_stack([separation(source["truth"], output["actual"][:, i]) for i in range(2)])
    output["projected_error"] = np.column_stack([separation(source["command"][:, i], output["actual"][:, i]) for i in range(2)])
    output["mount_config"] = config
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("results/mount-study"))
    parser.add_argument("--gamma-points", type=int, default=19)
    parser.add_argument("--d-points", type=int, default=7)
    parser.add_argument("--refinements", type=int, default=2)
    parser.add_argument("--control-hz", type=positive, default=100)
    parser.add_argument("--target-apogee-ft", type=nonnegative, default=30000)
    parser.add_argument("--motor-accel", type=positive, default=201.5)
    parser.add_argument("--efficiency", type=positive, default=.8)
    parser.add_argument("--usable-torque-fraction", type=positive, default=.5)
    parser.add_argument("--support-elevation-inertia", type=nonnegative, default=0)
    parser.add_argument("--base-azimuth-inertia", type=nonnegative, default=.0904)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if min(args.gamma_points, args.d_points) < 2 or not 0 <= args.refinements <= 4:
        parser.error("At least two grid points per dimension and 0-4 refinements are required")
    if max(args.efficiency, args.usable_torque_fraction) > 1:
        parser.error("Efficiency and usable torque fraction must not exceed 1")
    args.output.mkdir(parents=True, exist_ok=True)
    config = dict(mass_kg=3, radius_m=.25, L_m=.143, a_m=0, s_m=0,
                  base_azimuth_inertia=args.base_azimuth_inertia,
                  support_elevation_inertia=args.support_elevation_inertia,
                  efficiency=args.efficiency, usable_torque_fraction=args.usable_torque_fraction,
                  damping=[0, 0], kp=[12, 8], ki=[.5, .5], kd=[4, 2],
                  curve_rpm=[0, 30, 60, 150, 240, 300, 600, 900, 1500, 2100],
                  curve_nm=[3, 3, 3.05, 3.05, 3, 2.9, 2.3, 1.65, .6, 0])
    sim_args = SimpleNamespace(output=args.output, target_apogee_ft=args.target_apogee_ft,
                              motor_accel=args.motor_accel, flight_hz=20, control_hz=args.control_hz,
                              launch_angle=0, heading=0, distance=804.672, telemetry_hz=5,
                              latency=.3, jitter=.05, position_sigma=3, velocity_sigma=1,
                              drop_probability=0, comm_timeout=1, blackout=None, max_speed=[40, 40],
                              max_accel=[100, 100], azimuth_ratio=50, elevation_ratio=10,
                              kp=25, ki=.5, kd=10, beamwidth=20, seed=args.seed)
    print("Generating shared flight and telemetry...", flush=True)
    with (args.output/"generator.log").open("w", encoding="utf-8") as log:
        data, fitted, baseline = build_flight(sim_args, log)
    source = simulate(sim_args, data)
    gamma, distance = np.meshgrid(np.linspace(0, 90, args.gamma_points), np.linspace(.025, .1, args.d_points))
    print(f"Evaluating {gamma.size} geometries using coupled torque dynamics...", flush=True)
    table = evaluate(gamma.ravel(), distance.ravel(), source, data, config)
    spacing = np.array([90/(args.gamma_points-1), .075/(args.d_points-1)])
    for level in range(args.refinements):
        candidates = []
        for objective in (2, 3):
            best = table[np.argmin(table[:, objective]), :2]
            g, d = np.meshgrid(np.linspace(max(0, best[0]-spacing[0]), min(90, best[0]+spacing[0]), 7),
                               np.linspace(max(.025, best[1]-spacing[1]), min(.1, best[1]+spacing[1]), 7))
            candidates.extend(zip(g.ravel(), d.ravel()))
        candidates = np.unique(candidates, axis=0)
        print(f"Refinement {level+1}: {len(candidates)} candidates...", flush=True)
        table = np.vstack((table, evaluate(candidates[:, 0], candidates[:, 1], source, data, config)))
        spacing /= 3
    header = "gamma1_deg,d_m,ideal_peak_motor_torque_nm,peak_tracking_error_deg,rms_tracking_error_deg,ideal_azimuth_motor_nm,ideal_elevation_motor_nm,actual_azimuth_motor_nm,actual_elevation_motor_nm,requested_azimuth_motor_nm,requested_elevation_motor_nm"
    np.savetxt(args.output/"candidates.csv", table, delimiter=",", header=header, comments="")
    studies, chosen_history = {}, {}
    for name, objective in (("minimum_torque", 2), ("minimum_tracking_error", 3)):
        best = table[np.argmin(table[:, objective])]
        chosen = {**config, "gamma1_deg": float(best[0]), "d_m": float(best[1])}
        directory = args.output/name
        directory.mkdir(exist_ok=True)
        (directory/"mount_config.json").write_text(json.dumps(chosen, indent=2)+"\n", encoding="utf-8")
        sim_args.output = directory
        result = run_configured_mount(source, chosen)
        export(sim_args, data, result, fitted, baseline)
        chosen_history[name] = result
        run_summary = json.loads((directory/"summary.json").read_text(encoding="utf-8"))
        studies[name] = dict(gamma1_deg=float(best[0]), d_m=float(best[1]),
                             ideal_peak_motor_torque_nm=float(best[2]), max_tracking_error_deg=float(best[3]),
                             rms_tracking_error_deg=float(best[4]),
                             actual_peak_motor_torque_nm={"azimuth": float(best[7]), "elevation": float(best[8])},
                             requested_peak_motor_torque_nm={"azimuth": float(best[9]), "elevation": float(best[10])},
                             ideal_peak_motor_torque_by_axis_nm={"azimuth": float(best[5]), "elevation": float(best[6])},
                             angular_motion={key: run_summary["angular_motion"][key] for key in ("ideal_tracking", "predicted_pid")})
    report = dict(studies=studies, apogee_ft=float(np.max(data["r_z"])/.3048),
                  evaluated_candidates=len(table), settings={**vars(args), "output": str(args.output)},
                  scope=["Bounded sampled search, not proof of a global optimum.",
                         "Gamma1 0-90 degrees; d .025-.1 m; no mechanical travel stops.",
                         "Torque objective: highest ideal inverse-dynamics motor torque across axes and flight.",
                         "Error objective: maximum actual antenna-vs-true-rocket boresight error with fixed PID gains.",
                         "Prediction error is independent of mount geometry.",
                         "Vertical flight: ideal azimuth demand is zero; azimuth PID responds to noisy telemetry.",
                         "Uniform thin disk radius .25 m; elevation support inertia and damping assumed zero.",
                         "Motor envelope approximately digitized from supplied 24V chart, derated by usable fraction.",
                         "Ideal torque control and angle feedback; stepper resonances/missed steps/backlash not modeled."])
    (args.output/"study_summary.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    fig, panels = plt.subplots(1, 2, figsize=(12, 5))
    for panel, column, label in ((panels[0], 2, "Ideal peak motor torque (N m)"), (panels[1], 3, "Maximum tracking error (deg)")):
        plot = panel.scatter(table[:, 0], table[:, 1]*1000, c=table[:, column], s=25)
        fig.colorbar(plot, ax=panel, label=label)
        for name, study in studies.items():
            panel.scatter(study["gamma1_deg"], study["d_m"]*1000, marker="x", s=100, label=name)
        panel.set(xlabel="Fixed mount angle gamma1 (deg)", ylabel="COM offset d (mm)", title=label)
        panel.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output/"optimization.png", dpi=160)
    plt.close(fig)
    fig, panels = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    t = source["time"]-10
    for name, result in chosen_history.items():
        label = f"{name}: gamma={studies[name]['gamma1_deg']:.2f} deg, d={studies[name]['d_m']*1000:.1f} mm"
        panels[0].plot(t, result["error"][:, 1], label=label)
        for i, axis in enumerate(("Azimuth", "Elevation")):
            panels[i+1].plot(t, result["motor_torque"][:, 1, i], label=name)
            panels[i+1].set_ylabel(f"{axis} motor torque (N m)")
    panels[0].set(ylabel="Actual antenna tracking error (deg)", title="Two geometry objectives, identical flight/telemetry and fixed torque PID")
    panels[-1].set_xlabel("Time since launch (s)")
    for panel in panels:
        panel.set_xlim(0, t[-1])
        panel.grid(alpha=.3)
        panel.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output/"comparison.png", dpi=160)
    plt.close(fig)
    print("\nTWO-STUDY COMPARISON\n"+json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
