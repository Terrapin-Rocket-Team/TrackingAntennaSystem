from itertools import product

def gear_analysis(
    motor_torque_nm,
    motor_speed_rpm,
    torque_required_nm,
    speed_required_rpm,
    safety_factor=0.9,
    efficiency=1.0
):
    """
    Calculate gear ratio, output speed, and check if requirements are met.

    Parameters:
    - motor_torque_nm: motor max torque (Nm)
    - motor_speed_rpm: motor speed (RPM)
    - torque_required_nm: required output torque (Nm)
    - speed_required_rpm: required minimum output speed (RPM)
    - safety_factor: fraction of motor torque to use (e.g., 0.8 for 80% safe torque)
    - efficiency: drivetrain efficiency (1.0 for direct, <1.0 for planetary gear losses)

    Returns:
    - gear_ratio: required gear ratio
    - output_speed_rpm: output speed after gearing
    - meets_speed_req: True if output_speed_rpm >= speed_required_rpm else False
    """

    safe_torque = motor_torque_nm * safety_factor * efficiency
    gear_ratio = torque_required_nm / safe_torque
    output_speed_rpm = motor_speed_rpm / gear_ratio
    meets_speed_req = output_speed_rpm >= speed_required_rpm

    return gear_ratio, output_speed_rpm, meets_speed_req

def find_all_valid_gear_ratios(
    motor_torque_nm,
    motor_speed_rpm,
    torque_needed_nm,
    speed_needed_rpm,
    sun_teeth=20,
    ring_teeth_range=range(10, 50),
    stages=1,
    safety_factor=0.9,
    efficiency=0.9,
    num_planets=3  # Typically 3 or 4
):

    def stage_speed_ratio(Ns, Nr):
        return Ns / (Ns + Nr)

    def is_valid_gear_set(Ns, Nr, num_planets):
        if (Nr - Ns) % 2 != 0:
            return False  # must allow integer planet gear
        if (Ns + Nr) % num_planets != 0:
            return False  # evenly spaced planets
        Np = (Nr - Ns) // 2
        if Ns < 12 or Nr < 24 or Np < 12:
            return False  # minimum practical tooth count
        return True

    effective_motor_torque = motor_torque_nm * safety_factor
    efficiency_total = efficiency ** stages
    valid_solutions = []

    for ring_combo in product(ring_teeth_range, repeat=stages):
        # Validate all stages
        if not all(is_valid_gear_set(sun_teeth, Nr, num_planets) for Nr in ring_combo):
            continue

        speed_ratio_total = 1.0
        for Nr in ring_combo:
            speed_ratio_total *= stage_speed_ratio(sun_teeth, Nr)


        torque_ratio_total = 1.0 / speed_ratio_total
        output_speed_rpm = motor_speed_rpm * speed_ratio_total
        output_torque_nm = effective_motor_torque * torque_ratio_total * efficiency_total

        # print(f"Trying ring combo {ring_combo}: speed={output_speed_rpm:.2f}, torque={output_torque_nm:.2f}")

        if output_speed_rpm >= speed_needed_rpm and output_torque_nm >= torque_needed_nm:
            valid_solutions.append({
                'ring_teeth_per_stage': list(ring_combo),
                'total_speed_rpm': output_speed_rpm,
                'total_torque_nm': output_torque_nm,
                'gear_ratio': torque_ratio_total,
                'total_teeth_count': sum(ring_combo),
            })

    return valid_solutions


motor_torque = .4237  # Nm, motor max torque --> 60 in-oz
motor_speed = 250  # rpm, intended operating motor speed

torque_needed = 8.75  # Nm, output torque you need --> 2ft long, 5 lb heavy antenna (conservative)
speed_needed = 40/6  # rpm, min output speed you want --> worst case from simulation yielded 20 deg/s (conservative)

# Scenario 1: Direct coupling (efficiency = 1)
gear_ratio1, output_speed1, ok1 = gear_analysis(
    motor_torque,
    motor_speed,
    torque_needed,
    speed_needed,
    safety_factor=0.90,
    efficiency=1.0
)

print(f"Direct coupling: Gear ratio = {gear_ratio1:.2f}, Output speed = {output_speed1:.1f} rpm, Meets speed? {ok1}")

results = find_all_valid_gear_ratios(
    motor_torque_nm=motor_torque,
    motor_speed_rpm=motor_speed,
    torque_needed_nm=torque_needed,
    speed_needed_rpm=speed_needed,
    sun_teeth=20,
    ring_teeth_range=range(20, 200),  # limit search range to reduce compute
    stages=2,
    safety_factor=0.90,
    efficiency=0.90,
    num_planets=4
)

if results:
    best = min(results, key=lambda x: (x['total_teeth_count'], -x['total_torque_nm']))
    print("Found gear ratios meeting requirements for Planetary Gearbox:")
    print(f"Ring teeth per stage: {best['ring_teeth_per_stage']}")
    print(f"Output speed (RPM): {best['total_speed_rpm']:.2f}")
    print(f"Output torque (Nm): {best['total_torque_nm']:.2f}")
else:
    print("No gear ratio combination found to meet requirements.")