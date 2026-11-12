"""Behavior checks: python -B -m unittest test_tracking_sim -v (from OldCode)."""
import unittest
from types import SimpleNamespace

import numpy as np

from data_generators.supersonic_generator import RocketDataGenerator
from data_generators.worst_case_pointer_supersonic_generator import RocketDataGenerator as LowDragGenerator
from models.rocket import Rocket
from tracking_sim import AxisPID, angular_derivatives, motion_peaks, communication_status, intervals, pointing, separation, simulate, wrap


class FlightTests(unittest.TestCase):
    def generate(self, generator_class, delay):
        rocket = Rocket(20, .5, 1, .1, 1, 2, 0)
        generator = generator_class(rocket, loop_frequency=20, pre_launch_delay=delay,
                                    wind_affector=np.zeros(3))
        generator.atmosphere = SimpleNamespace(get_conditions=lambda **kw: {"gravity": 9.81})
        generator.drag_force_inertial = lambda *args: np.zeros(3)
        return generator.generate()

    def test_launch_delay_does_not_change_flight_mass_or_dynamics(self):
        for cls in (RocketDataGenerator, LowDragGenerator):
            with self.subTest(generator=cls.__module__):
                immediate, delayed = self.generate(cls, 0), self.generate(cls, 1)
                for key in ("r_z", "v_z", "a_z"):
                    np.testing.assert_allclose(immediate[key], delayed[key][20:], atol=1e-10)
                np.testing.assert_allclose(delayed["a_z"][:20], 0)

    def test_position_velocity_and_timestamp_agree(self):
        for cls in (RocketDataGenerator, LowDragGenerator):
            data = self.generate(cls, 0)
            self.assertAlmostEqual(data["r_z"][0], 0)
            self.assertAlmostEqual(data["v_z"][0], 0)
            self.assertAlmostEqual(data["a_z"][0], 20-9.81)
            self.assertAlmostEqual(data["r_z"][1], .5*(20-9.81)*.05**2)
            self.assertAlmostEqual(data["v_z"][1], (20-9.81)*.05)
            self.assertTrue(np.all(data["r_z"] >= 0))

    def test_prelaunch_mass_does_not_exceed_full_mass(self):
        rocket = Rocket(20, .5, 1, .1, 1, 2, 0)
        self.assertEqual(rocket.get_current_mass(-10), 2)


class PIDTests(unittest.TestCase):
    def test_motion_peaks_include_negative_extrema_gearing_and_flight_times(self):
        times = np.array([9., 10., 11.])
        rates = np.array([[999, 999], [2, -5], [-4, 3]])
        accel = np.array([[999, 999], [-8, 2], [3, -6]])
        peaks = motion_peaks(times, rates, accel, times >= 10, (50, 10))
        self.assertEqual(peaks["azimuth"]["max_output_velocity_deg_s"], {"value": 4, "time_since_launch_s": 1})
        self.assertAlmostEqual(peaks["azimuth"]["max_motor_velocity_rpm"]["value"], 4*50/6)
        self.assertEqual(peaks["elevation"]["max_motor_acceleration_deg_s2"]["value"], 60)

    def test_angular_derivatives_unwrap_azimuth_without_velocity_spike(self):
        rates, accel = angular_derivatives([[178, 0], [179, 2], [-180, 4], [-179, 6]], [0, 1, 2, 3])
        np.testing.assert_allclose(rates, [[1, 2]]*4, atol=1e-10)
        np.testing.assert_allclose(accel, 0, atol=1e-10)

    def test_limits_and_integral_recovery(self):
        axis = AxisPID(kp=25, ki=1, kd=10, max_speed=5, max_accel=10)
        for _ in range(200):
            acceleration = axis.step(100, .01)
            self.assertLessEqual(abs(acceleration), 10+1e-9)
            self.assertLessEqual(abs(axis.rate), 5+1e-9)
        self.assertAlmostEqual(axis.integral, 0)
        target = axis.angle
        for _ in range(3000):
            axis.step(target, .01)
        self.assertLess(abs(axis.angle-target), .1)

    def test_zero_gains_leave_antenna_stationary(self):
        axis = AxisPID(kp=0, ki=0, kd=0)
        for _ in range(100):
            axis.step(40, .01)
        self.assertEqual(axis.angle, 0)

    def test_shortest_azimuth_path(self):
        axis = AxisPID(angle=179, circular=True)
        self.assertGreater(axis.step(-179, .01), 0)
        self.assertAlmostEqual(float(wrap(-358)), 2)

    def test_spherical_error_handles_wrap_and_zenith(self):
        self.assertAlmostEqual(float(separation([179, 0], [-179, 0])), 2)
        self.assertAlmostEqual(float(separation([0, 90], [180, 90])), 0, places=10)

    def test_packets_are_unavailable_before_arrival(self):
        times = np.arange(0, 3.01, .1)
        data = {"time": times}
        for axis in "xyz":
            data[f"r_{axis}"] = 10*times if axis == "z" else np.zeros_like(times)
            data[f"v_{axis}"] = np.full_like(times, 10) if axis == "z" else np.zeros_like(times)
        args = SimpleNamespace(control_hz=10, distance=100, seed=42, telemetry_hz=2,
                               drop_probability=0, position_sigma=0, velocity_sigma=0,
                               latency=1, jitter=0, kp=25, ki=.5, kd=10,
                               comm_timeout=1, blackout=None,
                               max_speed=[40, 40], max_accel=[100, 100])
        result = simulate(args, data)
        self.assertFalse(result["valid_packet"][9])
        self.assertTrue(result["valid_packet"][10])
        np.testing.assert_allclose(result["command"][:10], 0)
        self.assertAlmostEqual(result["command"][10, 0, 1], 0)
        self.assertAlmostEqual(result["command"][10, 1, 1], pointing([100, 0, 10])[1])

    def test_communication_wait_timeout_and_recovery(self):
        self.assertEqual(communication_status(.5, None, 1), 0)
        self.assertEqual(communication_status(1, None, 1), -1)
        self.assertEqual(communication_status(2, .5, 1), -1)
        self.assertEqual(communication_status(2, 1.9, 1), 1)

    def test_blackout_loses_and_recovers_without_discarding_prediction(self):
        times = np.arange(0, 13.01, .1)
        data = {"time": times}
        for axis in "xyz":
            data[f"r_{axis}"] = 10*times if axis == "z" else np.zeros_like(times)
            data[f"v_{axis}"] = np.full_like(times, 10) if axis == "z" else np.zeros_like(times)
        args = SimpleNamespace(control_hz=10, distance=100, seed=42, telemetry_hz=2,
                               drop_probability=0, position_sigma=0, velocity_sigma=0,
                               latency=0, jitter=0, kp=25, ki=.5, kd=10,
                               max_speed=[40, 40], max_accel=[100, 100],
                               comm_timeout=.5, blackout=[0, 1.5])
        result = simulate(args, data)
        self.assertEqual(result["communication_status"][99], 1)
        self.assertEqual(result["communication_status"][104], -1)
        self.assertEqual(result["communication_status"][115], 1)
        self.assertTrue(result["valid_packet"][104])
        self.assertGreater(result["command"][104, 1, 1], result["command"][104, 0, 1])
        np.testing.assert_allclose(result["projected_error"][:, 1],
                                   separation(result["command"][:, 1], result["actual"][:, 1]))
        np.testing.assert_allclose(result["prediction_error"][:, 1],
                                   separation(result["command"][:, 1], result["truth"]))
        self.assertEqual(intervals(times-10, result["communication_status"] == -1), [(0, 1.5)])


if __name__ == "__main__":
    unittest.main()
