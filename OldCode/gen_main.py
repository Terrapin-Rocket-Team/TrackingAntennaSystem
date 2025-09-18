import numpy as np
import matplotlib.pyplot as plt
import plot_managers.plot_manager as pm

# Import your classes
from models.rocket import Rocket
from data_generators.supersonic_generator import RocketDataGenerator

from noise_generators.gaussian_noise_generator import GaussianNoiseGenerator
# from noise_generators.pink_noise_generator import PinkNoiseGenerator
# from noise_generators.drift_noise_generator import DriftNoiseGenerator

from sensors.sensor import Sensor
# from sensors.acceleration_sensor import AccelerationSensor

def main():
    # 1. Initializes rocket model and local variables

    # Separate arrays for each data type
    sim_positions = []
    sim_accelerations = []
    real_positions = []
    real_accelerations = []

    rocket = Rocket(
        motorAccel=201.5, 
        burnTime=4.83, 
        length=3.835,
        diameter=.1524,
        mass_empty=40.8,
        mass_full=60.8,
        surface_roughness=5e-6,
    )     

    # 2. Generate simulated data
    generator = RocketDataGenerator(
        rocket=rocket, 
        loop_frequency=20, 
        pre_launch_delay=10,
        launch_angle=0,
        heading_angle=np.random.uniform(0, 2 * np.pi),
        wind_affector=(lambda t: np.array([0 * np.sin(t), 0*np.cos(t), 0]))
    )
    sim_data = generator.generate()

    # Extract simulated ground truth
    sim_time_data = sim_data["time"]
    r_x, r_y, r_z = sim_data["r_x"], sim_data["r_y"], sim_data["r_z"]
    v_x, v_y, v_z = sim_data["v_x"], sim_data["v_y"], sim_data["v_z"]
    a_x, a_y, a_z = sim_data["a_x"], sim_data["a_y"], sim_data["a_z"]

    # For simulated data create "measured" values by adding noise
    position_sensor = Sensor(noise_generators=[GaussianNoiseGenerator(sigma=.1)])
    acceleration_sensor = Sensor(noise_generators=[GaussianNoiseGenerator(sigma=0.1)])

    for i in range(len(sim_time_data)):
        # Simulated noisy data
        sim_positions.append(position_sensor.measure(np.array([r_x[i], r_y[i], r_z[i]])))
        sim_accelerations.append(acceleration_sensor.measure(np.array([a_x[i], a_y[i], a_z[i]])))

    print("Data processing complete.")

    # 4. Store information in dictionaries for plotting
    sim = {
    "time": sim_time_data,
    "r_x": r_x,
    "r_y": r_y,
    "r_z": r_z,
    "measured_r_x": [x[0] for x in sim_positions],
    "measured_r_y": [y[1] for y in sim_positions],
    "measured_r_z": [z[2] for z in sim_positions],
    
    # Check if estimated_positions are available
    "estimated_r_x": None,
    "estimated_r_y": None,
    "estimated_r_z": None,
    
    "v_x": v_x,
    "v_y": v_y,
    "v_z": v_z,
    
    # Check if estimated_velocities are available
    "estimated_v_x": None,
    "estimated_v_y": None,
    "estimated_v_z": None,
    
    "a_x": a_x,
    "a_y": a_y,
    "a_z": a_z,
    
    # Check if measured_accelerations are available
    "measured_a_x": [x[0] for x in sim_accelerations],
    "measured_a_y": [y[1] for y in sim_accelerations],
    "measured_a_z": [z[2] for z in sim_accelerations]
    }

    # 5. Plots to make sure that simulated data has a good shape
    
    sim_manager = pm.PlotManager(sim, run_kf=False)  # run_kf=False since estimated values are None
    sim_manager.add_plot("z_position", lambda: pm.plot_z_position(sim_manager))
    sim_manager.add_plot("xyz_position", lambda: pm.plot_xyz_position(sim_manager))
    sim_manager.add_plot("z_velocity", lambda: pm.plot_z_velocity(sim_manager))
    sim_manager.add_plot("mach_number", lambda: pm.plot_mach_number(sim_manager))
    # To show all plots
    sim_manager.show_all()

if __name__ == "__main__":
    main()
