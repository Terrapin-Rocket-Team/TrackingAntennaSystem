import numpy as np
import matplotlib.pyplot as plt

# Import your classes
from models.rocket import Rocket
from data_generators.worst_case_pointer_supersonic_generator import RocketDataGenerator

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.
    Angles must be in radians.
    Returns [w, x, y, z]
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

def random_angle(min, max, step_size):
    range = np.linspace(min, max, int((max - min)/step_size) + 1)
    return np.random.choice(range)

def quaternion_to_rotation_matrix(q):
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
        
        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
            [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
        ])
        
        return R

def convert_rocket_to_antenna(x, y, z, vx, vy, vz, displacement, rotation_matrix):
    """
    Convert position and velocity vectors from rocket stand frame to antenna frame.
    
    :param x, y, z: Position components in rocket stand frame
    :param vx, vy, vz: Velocity components in rocket stand frame
    :param rotation_matrix: 3x3 rotation matrix from rocket stand frame to antenna frame
    :return: (pos_antenna, vel_antenna) as np.arrays in antenna frame
    """
    pos_rocket = np.array([x, y, z])
    vel_rocket = np.array([vx, vy, vz])
    
    # Rotate vectors into antenna frame
    pos_antenna = rotation_matrix @ (pos_rocket - displacement)
    vel_antenna = rotation_matrix @ vel_rocket
    
    return pos_antenna, vel_antenna

def compute_yaw_pitch(target_vec):
    """
    Given a direction vector in the antenna frame, return yaw and pitch angles
    to rotate the antenna to point in that direction.
    
    target_vec: np.array([x, y, z]) in antenna frame
    Returns: yaw (rad), pitch (rad)
    """
    # Normalize vector
    vec = target_vec / np.linalg.norm(target_vec)
    
    # yaw: rotation about z-axis (azimuth)
    yaw = np.arctan2(vec[1], vec[0])
    
    # pitch: rotation about y-axis (elevation)
    pitch = np.arcsin(vec[2])  # assuming z-up, x-forward

    return yaw, pitch


def main():
    num_simulations = 10
    
    all_max_yaw_accel = []
    all_max_pitch_accel = []
    all_median_yaw_accel = []
    all_median_pitch_accel = []
    
    for sim_num in range(num_simulations):
        print(f"Running simulation {sim_num + 1}/{num_simulations}...")
        
        # 1. Initializes rocket model and local variables
        rocket = Rocket(
            motorAccel=np.random.uniform(180, 220), 
            burnTime=np.random.uniform(4.5, 5), 
            length=3.835,
            diameter=.1524,
            mass_empty=40.8,
            mass_full=60.8,
            surface_roughness=np.random.uniform(1e-6, 10e-6),
        )     

        # 2. Generate simulated data
        generator = RocketDataGenerator(
            rocket=rocket, 
            loop_frequency=20, 
            pre_launch_delay=10,
            launch_angle=np.random.uniform(-5, 5),
            heading_angle=np.random.uniform(0, 2 * np.pi),
            wind_affector=(lambda t: np.array([np.random.uniform(0, .2) * np.sin(t), 
                                               np.random.uniform(0, .2)*np.cos(t), 0]))
        )
        sim_data = generator.generate()

        # Extract simulated ground truth
        sim_time_data = sim_data["time"]
        r_x, r_y, r_z = sim_data["r_x"], sim_data["r_y"], sim_data["r_z"]
        v_x, v_y, v_z = sim_data["v_x"], sim_data["v_y"], sim_data["v_z"]

        # 3. Convert to antenna frame
        theta = random_angle(-60, 60, 1)
        phi = random_angle(0, 20, 1)
        distance = np.random.uniform(0.5, 1.5) * 1609.34
        x = distance * np.cos(theta) * np.cos(phi)
        y = distance * np.sin(theta) * np.cos(phi)
        z = distance * np.sin(phi)

        displacement = np.array([x, y, z])

        # Convert degrees to radians
        yaw = np.radians(theta)
        pitch = np.radians(phi)
        roll = 0.0

        q = euler_to_quaternion(roll, pitch, yaw)
        rotation_matrix = quaternion_to_rotation_matrix(q)
        
        antenna_positions = []
        antenna_velocities = []
        antenna_yaw = []
        antenna_pitch = []
        
        for i in range(len(sim_time_data)):
            pos_antenna, vel_antenna = convert_rocket_to_antenna(
                r_x[i], r_y[i], r_z[i], v_x[i], v_y[i], v_z[i], 
                displacement, rotation_matrix
            )
            antenna_positions.append(pos_antenna)
            antenna_velocities.append(vel_antenna)
            yaw_angle, pitch_angle = compute_yaw_pitch(pos_antenna)
            antenna_yaw.append(yaw_angle)
            antenna_pitch.append(pitch_angle)

        antenna_yaw = np.unwrap(antenna_yaw)
        antenna_pitch = np.unwrap(antenna_pitch)

        # Compute angular rates (rad/s)
        dt_uniform = sim_time_data[1] - sim_time_data[0]
        yaw_rate = np.gradient(antenna_yaw, dt_uniform)
        pitch_rate = np.gradient(antenna_pitch, dt_uniform)

        # Compute angular accelerations (rad/s²)
        yaw_accel = np.gradient(yaw_rate, dt_uniform)
        pitch_accel = np.gradient(pitch_rate, dt_uniform)

        # Convert to degrees/s² and store statistics
        yaw_accel_deg = np.degrees(yaw_accel)
        pitch_accel_deg = np.degrees(pitch_accel)
        
        all_max_yaw_accel.append(np.max(np.abs(yaw_accel_deg)))
        all_max_pitch_accel.append(np.max(np.abs(pitch_accel_deg)))
        all_median_yaw_accel.append(np.median(np.abs(yaw_accel_deg)))
        all_median_pitch_accel.append(np.median(np.abs(pitch_accel_deg)))
    
    # Print results
    print("\n" + "="*60)
    print("ANGULAR ACCELERATION ANALYSIS RESULTS")
    print("="*60)
    print(f"\nYaw Angular Acceleration (deg/s²):")
    print(f"  Max across all simulations:    {np.max(all_max_yaw_accel):.2f}")
    print(f"  Median of max values:          {np.median(all_max_yaw_accel):.2f}")
    print(f"  Median of median values:       {np.median(all_median_yaw_accel):.2f}")
    
    print(f"\nPitch Angular Acceleration (deg/s²):")
    print(f"  Max across all simulations:    {np.max(all_max_pitch_accel):.2f}")
    print(f"  Median of max values:          {np.median(all_max_pitch_accel):.2f}")
    print(f"  Median of median values:       {np.median(all_median_pitch_accel):.2f}")
    print("="*60)

if __name__ == "__main__":
    main()

         


