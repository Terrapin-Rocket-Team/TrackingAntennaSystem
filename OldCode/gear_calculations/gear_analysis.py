def simple_planetary_gear_config(
    sun_teeth,
    gear_ratio,
    num_planets=3,
    module=2.0
):
    print(f"{'Sun Teeth':>12} {'Ring Teeth':>12} {'Planet Teeth':>12} {'Gear Ratio':>12} {'Sun Ø (mm)':>12} {'Ring Ø (mm)':>12} {'Planet Ø (mm)':>12}")
    print("-" * 100)

    ring_teeth = int(sun_teeth * (gear_ratio - 1))

    if ((ring_teeth + sun_teeth) % num_planets != 0 or ((ring_teeth - sun_teeth) % 2 != 0)):
        print('Invalid Planetary Gear Setup')
    else:
        planet_teeth = (ring_teeth - sun_teeth) // 2

        sun_diameter = module * sun_teeth
        ring_diameter = module * ring_teeth
        planet_diameter = module * planet_teeth

        print(f"{sun_teeth:12} {ring_teeth:12} {planet_teeth:12} {gear_ratio:12.2f} {sun_diameter:12.2f} {ring_diameter:12.2f} {planet_diameter:12.2f}")

# Example usage
simple_planetary_gear_config(
    sun_teeth=12,
    gear_ratio=10,
    num_planets=3,
    module=2
)
