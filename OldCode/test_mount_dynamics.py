"""Checks for supplied geometry, coupling, gravity signs and angular-error convention."""
import unittest

import numpy as np

from models.mount_dynamics import MountDynamics
from tracking_sim import prediction_axis_error
from mount_study import coupled_simulation, model_for, torque_capacity


class MountTests(unittest.TestCase):
    def setUp(self):
        # Test fixtures only; these offsets are not estimates of the real mount.
        self.mount = MountDynamics(gamma1=.4, d=.025, a=.02, s=.01)

    def test_com_matches_rotated_zero_geometry(self):
        x0, z0 = self.mount.com_at_zero
        for psi in (-.5, 0, .8):
            x, z, lever = self.mount.geometry(psi)
            self.assertAlmostEqual(x, x0*np.cos(psi)+z0*np.sin(psi))
            self.assertAlmostEqual(z, z0*np.cos(psi)-x0*np.sin(psi))
            self.assertAlmostEqual(lever, self.mount.s+x)

    def test_analytic_inertia_derivative(self):
        for psi in (-1, 0, .7):
            h = 1e-6
            numerical = (self.mount.coefficients(psi+h)[0]-self.mount.coefficients(psi-h)[0])/(2*h)
            self.assertAlmostEqual(self.mount.coefficients(psi)[1], numerical, places=8)

    def test_inverse_forward_are_consistent_with_both_couplings(self):
        q, velocity, acceleration = [.6, .3], [.8, -.5], [-.2, .7]
        torque = self.mount.inverse(q, velocity, acceleration)
        np.testing.assert_allclose(self.mount.forward(q, velocity, torque), acceleration, atol=1e-12)

    def test_gravity_matches_potential_gradient(self):
        psi, h = .4, 1e-6
        derivative = (self.mount.potential_energy([0, psi+h])-self.mount.potential_energy([0, psi-h]))/(2*h)
        gravity = self.mount.coefficients(psi)[3]
        self.assertAlmostEqual(derivative, -gravity, places=8)
        np.testing.assert_allclose(self.mount.forward([0, psi], [0, 0], [0, -gravity]), 0, atol=1e-12)

    def test_sky_angle_conversion(self):
        sky = np.radians([[10, 0], [40, 30], [20, 90]])
        np.testing.assert_allclose(self.mount.to_sky(self.mount.from_sky(sky)), sky, atol=1e-12)

    def test_prediction_error_is_true_minus_predicted(self):
        error = prediction_axis_error([[179, 30], [-179, 50]], [[-179, 25], [179, 60]])
        np.testing.assert_allclose(error, [[-2, 5], [2, -10]])

    def test_batched_mounts_match_scalar_equations(self):
        batch = MountDynamics(gamma1=np.array([.2, .6]), d=np.array([.025, .1]), a=0, s=0)
        q = np.array([[.1, .4], [.2, -.2]])
        rate, accel = np.array([[.2, .3], [-.6, .1]]), np.array([[.3, -.1], [.1, -.7]])
        expected = [MountDynamics(gamma1=g, d=d, a=0, s=0).inverse(q[i], rate[i], accel[i])
                    for i, (g, d) in enumerate(zip(batch.gamma1, batch.d))]
        np.testing.assert_allclose(batch.inverse(q, rate, accel), expected, atol=1e-12)

    def test_coupling_preserves_mechanical_energy_without_torque_or_damping(self):
        q, rate = np.array([.1, .4]), np.array([.5, -.3])
        acceleration = self.mount.forward(q, rate, [0, 0])
        h = 1e-6
        def energy(q, v):
            return self.mount.kinetic_energy(q, v)+self.mount.potential_energy(q)
        derivative = (energy(q+h*rate, rate+h*acceleration)-energy(q-h*rate, rate-h*acceleration))/(2*h)
        self.assertAlmostEqual(derivative, 0, places=8)

    def test_gravity_compensation_holds_multiple_geometries(self):
        config = dict(a_m=0, s_m=0, mass_kg=3, radius_m=.25, L_m=.143,
                      base_azimuth_inertia=.0904, support_elevation_inertia=0,
                      efficiency=.8, usable_torque_fraction=.5, damping=[0, 0],
                      kp=[12, 8], ki=[.5, .5], kd=[4, 2], curve_rpm=[0, 1000, 2000], curve_nm=[3, 1, 0])
        times = np.arange(0, 1.01, .01)
        source = dict(time=times, truth=np.zeros((len(times), 2)), command=np.zeros((len(times), 2, 2)))
        model = model_for(np.array([0, 90]), np.array([.025, .1]), config)
        result = coupled_simulation(source, model, config, history=True)
        np.testing.assert_allclose(result["history"]["actual"], 0, atol=1e-10)
        capacity = torque_capacity(np.zeros((2, 2)), config)
        self.assertTrue(np.all(np.abs(result["history"]["motor_torque"]) <= capacity+1e-12))
        np.testing.assert_allclose(torque_capacity([[10, 100]], config), 0)


if __name__ == "__main__":
    unittest.main()
