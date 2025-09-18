from typing import Optional
from .base_extended_kalman_filter import BaseExtendedKalmanFilter
from atmospheric_model.atmosphere import AtmosphereModel
import numpy as np
import math

class RocketEKF(BaseExtendedKalmanFilter):
    """
    A concrete implementation of an extended Kalman Filter 
    that sets up the F, H, Q, R matrices for a 6x6 state, etc.
    """

    def __init__(self, 
                 rocket,
                 initial_state: np.ndarray,
                 initial_covariance: np.ndarray,
                 measurement_noise: np.ndarray,
                 launch_angle: float,
                 heading_angle: float,
                 process_noise: np.ndarray,
                pre_launch_delay: float = 0.0 # realistically can detect when launch occurs to run algorithm
                ):
        
        super().__init__(initial_state, initial_covariance)
        self.Q = process_noise 
        self.R = measurement_noise
        self.rocket = rocket
        self.atmosphere = AtmosphereModel()
        roll, pitch, yaw = 0, launch_angle, heading_angle
        # Convert initial roll, pitch, yaw to quaternion
        self.curr_quaternion = self.euler_to_quaternion(roll, pitch, yaw)
        self.time = 10.0 
        # Useful Constants from Atmospheric Model
        self.R_earth = 6371000  # Mean radius of the Earth (m)
        self.g0 = 9.80665       # Standard gravity at sea level (m/s²)
        self.pre_launch_delay = pre_launch_delay

    def iterate(self, dt, measurement, quaternion, true_state: Optional[np.ndarray] = None):
        """
        One cycle of predict -> update.
        
        :param dt: Time step
        :param measurement: Measurement vector (7x1)
        :param quaternion: Quaternion vector (4x1) from complementary filter
        :param true_state: True state vector (6x1) for NEES calculation (only for generated data)
        """
        print("\n--- EKF Iteration ---")
        self.time += dt
        print(f"Time: {self.time}")

        self.curr_quaternion = quaternion

        self.predict_state(dt)
        
        # print("\nAfter Predict State:")
        # print("Predicted State:", self.x)
        # print("Predicted Covariance:", self.P)
        
        # print("\nMeasurement Input:")
        # print(measurement)
        
        self.update_with_measurement(measurement, true_state)
        
        # print("\nAfter Update:")
        # print("Updated State:", self.x)
        # print("Updated Covariance:", self.P)

    def state_transition_function(self, state, dt):
        """Defines the nonlinear state transition model f(x, u, dt)."""
        new_state = np.copy(state)
        current_mass = self.rocket.get_current_mass(self.time - self.pre_launch_delay)
        a = self.calculate_acceleration_inertial(self.curr_quaternion, current_mass, state[2], state[3:6], self.time).reshape(3, 1)
        # print(a)
        new_state[0:3] += state[3:6] * dt + 1/2 * a[0:3]*dt**2
        new_state[2] = max(new_state[2], 0) # logic is I know it won't be below 0, so 0 has to be more correct
        new_state[3:6] += a[0:3]*dt
        return new_state

    def compute_jacobian_F(self, state, dt):
        """Computes the Jacobian of f(x, u, dt)."""
        # Get atmospheric conditions
        current_mass = self.rocket.get_current_mass(self.time - self.pre_launch_delay)
        velocity_mag = np.linalg.norm(state[3:6])
        conditions = self.atmosphere.get_conditions(
                altitude=state[2],
                velocity=velocity_mag,  # Using velocity magnitude
                characteristic_length=self.rocket.length
        )
        mach_number = velocity_mag / conditions["speed_of_sound"]
        reynolds = conditions["reynolds_number"]
        C_d = self.total_drag_coefficient(reynolds, mach_number)
        density = conditions["density"]
        F = np.zeros((6, 6))
        F[0, 0] = 1
        F[0, 3] = dt - .5*C_d*density*state[3]*self.rocket.topCrossSectionalArea/current_mass*dt**2
        F[1, 1] = 1
        F[1, 4] = dt - .5*C_d*density*state[4]*self.rocket.topCrossSectionalArea/current_mass*dt**2
        F[2, 2] = 1 - self.g0 * (self.R_earth**2) / ((self.R_earth + state[2]) ** 3)*dt**2
        F[2, 5] = dt - .5*C_d*density*state[5]*self.rocket.topCrossSectionalArea/current_mass*dt**2
        F[3, 3] = 1 - .5*C_d*density*state[3]*self.rocket.topCrossSectionalArea/current_mass*dt
        F[4, 4] = 1 - .5*C_d*density*state[4]*self.rocket.topCrossSectionalArea/current_mass*dt
        F[5, 2] = -self.g0 * (self.R_earth**2) / ((self.R_earth + state[2]) ** 3)*dt
        F[5, 5] = 1 - .5*C_d*density*state[5]*self.rocket.topCrossSectionalArea/current_mass*dt
        return F

    def measurement_function(self, state):
        """Defines the nonlinear measurement model h(x)."""
        current_mass = self.rocket.get_current_mass(self.time - self.pre_launch_delay)
        rotation_matrix = self.quaternion_to_rotation_matrix(self.curr_quaternion)
        a = self.calculate_acceleration_inertial(self.curr_quaternion, current_mass, state[2], state[3:6], self.time)
        rotation_matrix_T = rotation_matrix.T
        a_body = rotation_matrix_T @ a
        # print(a_body)
        h =  np.zeros((7, 1))
        h[0, 0] = state[0]
        h[1, 0] = state[1]
        h[2, 0] = state[2]
        h[3, 0] = state[2]
        h[4, 0] = a_body[0]
        h[5, 0] = a_body[1]
        h[6, 0] = a_body[2]
        return h

    def compute_jacobian_H(self, state):
        """Computes the Jacobian of h(x)."""
        current_mass = self.rocket.get_current_mass(self.time - self.pre_launch_delay)
        rotation_matrix = self.quaternion_to_rotation_matrix(self.curr_quaternion)
        velocity_mag = np.linalg.norm(state[3:6])
        conditions = self.atmosphere.get_conditions(
                altitude=state[2],
                velocity=velocity_mag,  # Using velocity magnitude
                characteristic_length=self.rocket.length
        )
        mach_number = velocity_mag / conditions["speed_of_sound"]
        reynolds = conditions["reynolds_number"]
        C_d = self.total_drag_coefficient(reynolds, mach_number)
        density = conditions["density"]
        rotation_matrix_T = rotation_matrix.T
        r_body = rotation_matrix_T @ state[0:3]
        v_body = rotation_matrix_T @ state[3:6]
        v_mag = np.linalg.norm(v_body)
    
        H = np.zeros((7, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        H[3, 2] = 1
        H[6, 2] = -2 * self.g0 * (self.R_earth**2) / ((self.R_earth + state[2]) ** 3)
        H[4, 3] = -2 * self.g0 * (self.R_earth**2) / ((self.R_earth + r_body[2]) ** 3)
        temp = -C_d * density * self.rocket.topCrossSectionalArea / current_mass * (np.eye(3) * v_mag + (v_body @ v_body.T) / v_mag)        
        H[4:7, 3:6] = temp @ rotation_matrix_T

        return H

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        
        :param roll: Roll angle in radians
        :param pitch: Pitch angle in radians
        :param yaw: Yaw angle in radians
        :return: Quaternion as [w, x, y, z]
        """
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])

    def quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to rotation matrix.
        
        :param q: Quaternion as [w, x, y, z]
        :return: 3x3 rotation matrix
        """
        w, x, y, z = q
        
        # Normalize quaternion
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # Quaternion to rotation matrix
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        R = np.squeeze(np.array([
            [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
            [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
        ], dtype=float))
        return R
    
    def calculate_friction_coefficient(self, reynolds: float, mach: float) -> float:
        """Calculate skin friction coefficient"""
        if not self.rocket.surface_roughness:
            # Perfect finish (partially laminar)
            if reynolds < 1.0e4:
                cf = 1.33e-2
            elif reynolds < 5.39e5:
                cf = 1.328 / math.sqrt(reynolds)
            else:
                cf = 1.0 / pow(1.50 * math.log(reynolds) - 5.6, 2) - 1700/reynolds
        else:
            # Turbulent
            if reynolds < 1.0e4:
                cf = 1.48e-2
            else:
                cf = 1.0 / pow(1.50 * math.log(reynolds) - 5.6, 2)

        # Compressibility corrections
        if mach < 0.9:
            cf *= 1 - 0.1 * mach**2
        elif mach > 1.1:
            if not self.rocket.surface_roughness:
                cf *= 1.0 / pow(1 + 0.045 * mach**2, 0.25)
            else:
                cf *= 1.0 / pow(1 + 0.15 * mach**2, 0.58)
        else:
            # Linear interpolation between M=0.9 and M=1.1
            if not self.rocket.surface_roughness:
                c1 = 1 - 0.1 * 0.9**2
                c2 = 1.0 / pow(1 + 0.045 * 1.1**2, 0.25)
            else:
                c1 = 1 - 0.1 * 0.9**2
                c2 = 1.0 / pow(1 + 0.15 * 1.1**2, 0.58)
            cf *= c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
            
        return cf

    def calculate_stagnation_cd(self, mach: float) -> float:
        """Calculate stagnation pressure coefficient"""
        if mach <= 1:
            pressure = 1 + mach**2/4 + mach**4/40
        else:
            pressure = 1.84 - 0.76/mach**2 + 0.166/mach**4 + 0.035/mach**6
        return 0.85 * pressure

    def calculate_base_cd(self, mach: float) -> float:
        """Calculate base drag coefficient"""
        if mach <= 1:
            return 0.12 + 0.13 * mach**2
        else:
            return 0.25 / mach

    def calculate_roughness_correction(self, mach: float) -> float:
        """Calculate roughness effects"""
        if not self.rocket.surface_roughness:
            return 0
        
        if mach < 0.9:
            correction = 1 - 0.1 * mach**2
        elif mach > 1.1:
            correction = 1 / (1 + 0.18 * mach**2)
        else:
            c1 = 1 - 0.1 * 0.9**2
            c2 = 1 / (1 + 0.18 * 1.1**2)
            correction = c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
            
        return 0.032 * pow(self.rocket.surface_roughness/self.rocket.length, 0.2) * correction

    def total_drag_coefficient(self, reynolds, mach_number: float) -> float:
        """
        Calculate the total drag coefficient by summing wave drag, base drag, and friction drag.
        :param reynolds: Reynolds Number
        :param mach_number: Mach number (ratio of rocket velocity to speed of sound)
        """
        # Calculate friction coefficient
        cf = self.calculate_friction_coefficient(reynolds, mach_number)
        
        # Calculate roughness effects
        roughness_cd = self.calculate_roughness_correction(mach_number)
        
        # Use maximum of Cf and roughness-limited value
        friction_cd = max(cf, roughness_cd)
        
        # Calculate pressure drag
        stagnation_cd = self.calculate_stagnation_cd(mach_number)
        
        # Calculate base drag
        base_cd = self.calculate_base_cd(mach_number)
        
        total_cd = friction_cd + stagnation_cd + base_cd
        return total_cd
    
    def cd_aoa_multiplier(self, aoa) -> float:
        """
        Calculate angle of attack correction.
        Handles limited range of AOA
        
        :param aoa: angle of attack (radians)
        :return: AOA-multiplier
        """
        
        aoa_degrees = math.degrees(abs(aoa))
        
        # Handle symmetry: if AOA > 90°
        if abs(aoa_degrees) > 90:
            aoa_degrees = 180 - abs(aoa_degrees)
        else:
            aoa_degrees = abs(aoa_degrees)
        
        # Only apply correction for reasonable AOA range
        if aoa_degrees >= 17:
            print(f"Warning: AOA ({aoa_degrees:.1f}°), correction may be inaccurate")
        
        aoa_clamped = min(aoa_degrees, 17)
        
        # Apply correction
        sin_aoa = math.sin(math.radians(aoa_clamped))
        multiplier = 1.0 + min(sin_aoa, 0.30)
        return multiplier
    
    def drag_force_inertial(self, conditions, velocity, velocity_mag, aoa = 0):
        """
        Calculate the drag force in the body frame
        """

         # Calculate the relative velocity vector (rocket relative to air)
        
        if velocity_mag == 0:
            return np.zeros(3)
    
        mach_number = velocity_mag / conditions["speed_of_sound"]
        reynolds = conditions["reynolds_number"]
        C_d = self.total_drag_coefficient(reynolds, mach_number)
        density = conditions["density"]
    
        A_eff = self.rocket.topCrossSectionalArea * abs(np.cos(aoa)) + self.rocket.sideCrossSectionalArea * abs(np.sin(aoa))
        # Calculate pure aerodynamic drag (opposite to velocity direction) 
        drag_force = -0.5 * C_d * density * A_eff * velocity_mag**2 * (velocity / velocity_mag) # top dominates
        multiplier = self.cd_aoa_multiplier(aoa)
        return drag_force * multiplier

    def calculate_acceleration_inertial(self, quaternion, mass, altitude, velocity, time):
        """
        Calculate acceleration in inertial frame from quaternion, motor acceleration, and atmospheric conditions.
        
        :param quaternion: Current orientation as quaternion [w, x, y, z]
        :param mass: Current mass of the rocket
        :param altitude: Current altitude for atmospheric conditions
        :param velocity: Current velocity vector [vx, vy, vz] in inertial frame
        :param time: Current simulation time (used to check if engine is burning)
        :return: Acceleration vector [ax, ay, az] in inertial frame
        """
        adjusted_time = time - self.pre_launch_delay
        # Calculate velocity magnitude
        velocity_mag = np.linalg.norm(velocity)

        # Get atmospheric conditions
        conditions = self.atmosphere.get_conditions(
                altitude=altitude,
                velocity=velocity_mag,  # Using velocity magnitude
                characteristic_length=self.rocket.length
        )
        
        # Convert quaternion to rotation matrix (body frame to inertial frame)
        rotation_matrix = self.quaternion_to_rotation_matrix(quaternion)
        
        # Calculate thrust force in body frame (along z-axis in body frame)
        if adjusted_time >= 0 and adjusted_time <= self.rocket.burnTime:
            thrust_force_body = np.array([0.0, 0.0, self.rocket.motorAccel * mass])
        else:
            thrust_force_body = np.zeros(3)
        
        # Convert thrust force to inertial frame
        
        thrust_force_inertial = rotation_matrix @ thrust_force_body
        
        # Calculate gravity force in inertial frame (always points down)
        grav = conditions["gravity"][0]
        gravity_force_inertial = np.array([0, 0, (-grav * mass)])
        
        drag_force_inertial = self.drag_force_inertial(conditions, velocity, velocity_mag).flatten()
        
        # Calculate total force
        total_force_inertial = thrust_force_inertial + gravity_force_inertial + drag_force_inertial
        
        # Calculate acceleration from force
        acceleration_inertial = total_force_inertial / mass
        return acceleration_inertial