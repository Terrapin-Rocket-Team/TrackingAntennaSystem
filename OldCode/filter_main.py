
import numpy as np 
import matplotlib.pyplot as plt
import plot_managers.plot_manager as pm

# Import your classes
from models.rocket import Rocket
from data_generators.supersonic_generator import RocketDataGenerator

from kalman_filters.extended_kalman_filter import RocketEKF

from noise_generators.gaussian_noise_generator import GaussianNoiseGenerator
# from noise_generators.pink_noise_generator import PinkNoiseGenerator
# from noise_generators.drift_noise_generator import DriftNoiseGenerator

from sensors.sensor import Sensor
# from sensors.acceleration_sensor import AccelerationSensor

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
    
    R = np.squeeze(np.array([
        [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
        [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
    ], dtype=float))
    return R

def main():
    # 1. Choose data source: simulated
    use_simulated = True  # set to False to load real CSV

    actual_state = []

    measured_gps_positions = []
    measured_barometer_positions = []
    measured_accelerations = []
    measured_accelerations_body = []
    measured_quaternions = []

    rocket = Rocket(
        motorAccel=200, 
        burnTime=4.83, 
        length=3.835,
        diameter=.1524,
        mass_empty=40.8,
        mass_full=60.8,
        surface_roughness=5e-6,
    )    
  
    launch_angle = 0
    heading_angle=np.random.uniform(0, 2 * np.pi)

    generator = RocketDataGenerator(
        rocket=rocket, 
        loop_frequency=10, 
        pre_launch_delay=10,
        launch_angle=launch_angle,
        heading_angle=heading_angle,
        wind_affector=(lambda t: np.array([0*np.sin(t), 0*np.cos(t), 0]))
    )
    data_dict = generator.generate()

    # save everything as ground truth
    time_data = data_dict["time"]
    r_x = data_dict["r_x"]
    r_y = data_dict["r_y"]
    r_z = data_dict["r_z"]
    v_x = data_dict["v_x"]
    v_y = data_dict["v_y"]
    v_z = data_dict["v_z"]
    a_x = data_dict["a_x"]
    a_y = data_dict["a_y"]
    a_z = data_dict["a_z"] # In the future add gravity (9.81) since IMU doesn't account for it (simulation does)
    quat = data_dict["quaternion"]

    # 2. if simulated, add noise to the data to get "measured" data
    if use_simulated:
        # in the future need to get actual sensor noise specifications from datasheet
        gps_noise = .1
        position_gps_sensor = Sensor(noise_generators=[GaussianNoiseGenerator(sigma=gps_noise)])

        barometer_noise = .1
        position_barometer_sensor = Sensor(noise_generators=[GaussianNoiseGenerator(sigma=barometer_noise)])

        imu_noise = .1
        acceleration_sensor = Sensor(noise_generators=[GaussianNoiseGenerator(sigma=imu_noise)])
        # Realistically, quaternions are taken from the complementary filter
        quaternion_noise = .2
        quaternion_sensor = Sensor(noise_generators=[GaussianNoiseGenerator(sigma=quaternion_noise)]) 

        for i in range(len(time_data)):
            actual_state = np.array([r_x[i], r_y[i], r_z[i], v_x[i], v_y[i], v_z[i]])
            pos_gps_measured = position_gps_sensor.measure(np.array([r_x[i], r_y[i], r_z[i]]))
            measured_gps_positions.append(pos_gps_measured)

            pos_barometer_measured = position_barometer_sensor.measure(np.array([r_x[i], r_y[i], r_z[i]]))
            measured_barometer_positions.append(pos_barometer_measured)

            a_measured = acceleration_sensor.measure(np.array([a_x[i], a_y[i], a_z[i]]))
            measured_accelerations.append(a_measured)

            q_measured = quaternion_sensor.measure(np.array([quat[i][0], quat[i][1], quat[i][2], quat[i][3]]))
            measured_quaternions.append(q_measured)

            rotation_matrix = quaternion_to_rotation_matrix(q_measured)
            measured_accelerations_body.append(rotation_matrix @ a_measured)


    # 3. Set up the Kalman filter
    #   A typical initial covariance
    initial_covariance = 500 * np.eye(6)
    #   An initial state (6x1)
    initial_state = np.zeros((6,1))

    # Construct R matrix
    R = np.diag([gps_noise**2, gps_noise**2, gps_noise**2, barometer_noise**2, imu_noise**2, imu_noise**2, imu_noise**2]) # Needs to be tuned

    # Construct Q Matrix
    Q = np.diag([0.5e-1, 0.5e-1, 0.5e-1, 0.5e-1, 0.5e-1, 0.5e-1])  # Needs to be tuned

    kf = RocketEKF(
        rocket=rocket,
        initial_state=initial_state,
        initial_covariance=initial_covariance,
        measurement_noise=R,
        launch_angle=launch_angle,
        heading_angle=heading_angle,
        process_noise=Q,
        pre_launch_delay=10      
    )

    # 4. Run the filter in a loop
    estimated_positions = []
    estimated_velocities = []
    estimated_times = []

    for i in range(1, len(time_data)):
        if time_data[i] < 10:
            estimated_positions.append(np.array([[0], [0], [0]]))  # x, y, z from the filter
            estimated_velocities.append(np.array([[0], [0], [0]]))  # vx, vy, vz from the filter
        else:
            dt = time_data[i] - time_data[i-1]
            # measurement is 7x1:
            measurement = np.array([measured_gps_positions[i][0], measured_gps_positions[i][1], max(measured_gps_positions[i][2], [0.0]),
                                    max(measured_barometer_positions[i][2], [0.0]), measured_accelerations_body[i][0], measured_accelerations_body[i][1], 
                                    measured_accelerations_body[i][2]])
            # quaternion from complementary filter
            quaternion = measured_quaternions[i]

            # One iteration
            # print(f"Real State:\n {actual_state}")
            kf.iterate(dt, measurement, quaternion)

            estimated_positions.append(kf.get_state()[:3])  # x, y, z from the filter
            estimated_velocities.append(kf.get_state()[3:])  # vx, vy, vz from the filter
            estimated_times.append(time_data[i])

    # 5. Plot results
    data = {
        "time": time_data,
        "r_x": r_x,
        "r_y": r_y,
        "r_z": r_z,
        "estimated_r_x": [x[0] for x in estimated_positions],
        "estimated_r_y": [y[1] for y in estimated_positions],
        "estimated_r_z": [z[2] for z in estimated_positions],
        "v_x": v_x,
        "v_y": v_y,
        "v_z": v_z,
        "estimated_v_x": [vx[0] for vx in estimated_velocities],
        "estimated_v_y": [vy[1] for vy in estimated_velocities],
        "estimated_v_z": [vz[2] for vz in estimated_velocities],
        "a_x": a_x,
        "a_y": a_y,
        "a_z": a_z,
        "measured_a_x": [x[0] for x in measured_accelerations],
        "measured_a_y": [y[1] for y in measured_accelerations],
        "measured_a_z": [z[2] for z in measured_accelerations]
    }

    manager = pm.PlotManager(data)

    manager.add_plot("z_position", lambda: pm.plot_z_position(manager))
    # manager.add_plot("xyz_position", lambda: pm.plot_xyz_position(manager))
    # manager.add_plot("z_velocity", lambda: pm.plot_z_velocity(manager))
    # manager.add_plot("mach_number", lambda: pm.plot_mach_number(manager))

    # Show all plots
    manager.show_all()

if __name__ == "__main__":
    main()