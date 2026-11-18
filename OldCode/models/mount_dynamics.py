"""SI-unit coupled mount model transcribed from the supplied mechanism drawing.

q = [phi, psi] are gearbox-output radians. theta = psi + gamma1 is
boresight tilt FROM VERTICAL; sky elevation is pi/2 - theta.
The dish is approximated as a uniform thin circular plate.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MountDynamics:
    gamma1: float
    d: float
    a: float
    s: float
    mass: float = 3.0
    radius: float = .25
    length: float = .143
    base_azimuth_inertia: float = .0904
    support_elevation_inertia: float = 0.0
    rotor_inertia: float = .000068
    azimuth_ratio: float = 50.0
    elevation_ratio: float = 10.0
    damping_phi: float = 0.0
    damping_psi: float = 0.0
    gravity: float = 9.80665
    gear_efficiency: float = 1.0

    def __post_init__(self):
        if any(not np.all(np.isfinite(value)) for value in vars(self).values()):
            raise ValueError("Mount parameters must be finite")
        for name in ("mass", "radius", "length", "azimuth_ratio", "elevation_ratio"):
            if np.any(np.asarray(getattr(self, name)) <= 0):
                raise ValueError(f"{name} must be positive")
        for name in ("d", "base_azimuth_inertia", "support_elevation_inertia", "rotor_inertia", "damping_phi", "damping_psi", "gravity"):
            if np.any(np.asarray(getattr(self, name)) < 0):
                raise ValueError(f"{name} must be nonnegative")
        if not 0 < self.gear_efficiency <= 1:
            raise ValueError("gear_efficiency must lie in (0,1]")

    @property
    def com_at_zero(self):
        return np.stack([self.a+self.d*np.sin(self.gamma1),
                         self.length+self.d*np.cos(self.gamma1)], axis=-1)

    def geometry(self, psi):
        theta = np.asarray(psi) + self.gamma1
        x = self.length*np.sin(psi) + self.d*np.sin(theta) + self.a*np.cos(psi)
        z = self.length*np.cos(psi) + self.d*np.cos(theta) - self.a*np.sin(psi)
        return x, z, self.s+x

    def coefficients(self, psi):
        """Return J_phi, dJ_phi/dpsi, J_psi and G (gravity driving torque).

        base_azimuth_inertia excludes the dish and azimuth motor rotor.
        support_elevation_inertia excludes the dish and elevation motor rotor.
        Constant reflected rotor inertias do not contribute to dJ_phi/dpsi.
        With tau_output = eta*N*tau_motor, eliminating rotor acceleration
        contributes eta*N^2*J_rotor to effective inertia (N^2*J at eta=1).
        """
        theta = np.asarray(psi) + self.gamma1
        x, z, lever = self.geometry(psi)
        diameter_inertia = .25*self.mass*self.radius**2
        normal_inertia = .5*self.mass*self.radius**2
        j_phi = (self.base_azimuth_inertia + self.gear_efficiency*self.azimuth_ratio**2*self.rotor_inertia
                 + diameter_inertia*np.sin(theta)**2 + normal_inertia*np.cos(theta)**2
                 + self.mass*lever**2)
        j_prime = (2*(diameter_inertia-normal_inertia)*np.sin(theta)*np.cos(theta)
                   + 2*self.mass*lever*z)
        j_psi = (self.support_elevation_inertia + self.gear_efficiency*self.elevation_ratio**2*self.rotor_inertia
                 + diameter_inertia + self.mass*np.sum(self.com_at_zero**2, axis=-1))
        return j_phi, j_prime, j_psi, self.mass*self.gravity*x

    def inverse(self, q, rate, acceleration):
        """Required gearbox-output torque for requested motion; last axis phi,psi."""
        q, rate, acceleration = map(np.asarray, (q, rate, acceleration))
        j_phi, j_prime, j_psi, gravity = self.coefficients(q[..., 1])
        phi_rate, psi_rate = rate[..., 0], rate[..., 1]
        tau_phi = j_phi*acceleration[..., 0] + j_prime*psi_rate*phi_rate + self.damping_phi*phi_rate
        tau_psi = j_psi*acceleration[..., 1] - .5*j_prime*phi_rate**2 - gravity + self.damping_psi*psi_rate
        return np.stack((tau_phi, tau_psi), axis=-1)

    def forward(self, q, rate, torque):
        """Solve the supplied coupled equations for output acceleration, rad/s^2."""
        q, rate, torque = map(np.asarray, (q, rate, torque))
        j_phi, j_prime, j_psi, gravity = self.coefficients(q[..., 1])
        phi_rate, psi_rate = rate[..., 0], rate[..., 1]
        phi_accel = (torque[..., 0] - j_prime*psi_rate*phi_rate - self.damping_phi*phi_rate)/j_phi
        psi_accel = (torque[..., 1] + .5*j_prime*phi_rate**2 + gravity - self.damping_psi*psi_rate)/j_psi
        return np.stack((phi_accel, psi_accel), axis=-1)

    def from_sky(self, angles):
        """Convert [azimuth, elevation] radians to [phi, psi] radians."""
        angles = np.asarray(angles)
        return np.stack(np.broadcast_arrays(angles[..., 0], np.pi/2-angles[..., 1]-self.gamma1), axis=-1)

    def to_sky(self, q):
        q = np.asarray(q)
        return np.stack((q[..., 0], np.pi/2-q[..., 1]-self.gamma1), axis=-1)

    def kinetic_energy(self, q, rate):
        j_phi, _, j_psi, _ = self.coefficients(np.asarray(q)[..., 1])
        return .5*(j_phi*np.asarray(rate)[..., 0]**2+j_psi*np.asarray(rate)[..., 1]**2)

    def potential_energy(self, q):
        _, z, _ = self.geometry(np.asarray(q)[..., 1])
        return self.mass*self.gravity*z
