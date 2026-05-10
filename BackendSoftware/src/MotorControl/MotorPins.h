//Created By Divyansh Srivastava on 5/10/2026, this file will defien the hardware
//level pin assignments and timing rules for the stepper motors, as well as helper functions to send pulses and set direction.
#ifndef MOTORPINS_H
#define MOTORPINS_H
#include <Arduino.h>

// =============================================================================
// MotorPins.h
// Motor Control — PUL/DIR pin definitions and signal helpers
// Teensy 4.1 → Logic Level Shifter → Stepper Driver
//
// ENA is NC (not connected) on both drivers.
// Timing rules from driver datasheet:
//   t1: ENA ahead of DIR by >5µs  — N/A (ENA not connected)
//   t2: DIR must settle >5µs before first PUL edge
//   t3: PUL HIGH width >2.5µs
//   t4: PUL LOW width >2.5µs
// =============================================================================

// -----------------------------------------------------------------------------
// Pin assignments
// -----------------------------------------------------------------------------
constexpr int MOTOR1_PUL = 8;
constexpr int MOTOR1_DIR = 9;

constexpr int MOTOR2_PUL = 24;
constexpr int MOTOR2_DIR = 25;

// -----------------------------------------------------------------------------
// Direction constants
// -----------------------------------------------------------------------------
constexpr bool MOTOR_DIR_CW  = HIGH;
constexpr bool MOTOR_DIR_CCW = LOW;

// -----------------------------------------------------------------------------
// Timing constants (microseconds)
// All values exceed datasheet minimums with small margin
// -----------------------------------------------------------------------------
constexpr uint32_t T2_DIR_SETUP_US = 6;   // DIR settle before PUL  (min 5µs)
constexpr uint32_t T3_PUL_HIGH_US  = 3;   // PUL HIGH width         (min 2.5µs)
constexpr uint32_t T4_PUL_LOW_US   = 3;   // PUL LOW width          (min 2.5µs)

// -----------------------------------------------------------------------------
// Initialisation — call once in setup()
// -----------------------------------------------------------------------------
inline void motorPins_init()
{
    pinMode(MOTOR1_PUL, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PUL, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);

    // Default both motors to CW, idle pulse line LOW
    digitalWriteFast(MOTOR1_DIR, MOTOR_DIR_CW);
    digitalWriteFast(MOTOR2_DIR, MOTOR_DIR_CW);
    digitalWriteFast(MOTOR1_PUL, LOW);
    digitalWriteFast(MOTOR2_PUL, LOW);

    // t2: let DIR settle after init before any pulse can arrive
    delayMicroseconds(T2_DIR_SETUP_US);
}

// -----------------------------------------------------------------------------
// Set direction for a motor
// Always call this before stepping if direction has changed.
// Includes t2 settling delay so caller does not need to.
// -----------------------------------------------------------------------------
inline void motor1_setDir(bool dir)
{
    digitalWriteFast(MOTOR1_DIR, dir);
    delayMicroseconds(T2_DIR_SETUP_US);   // t2: DIR must settle >5µs before PUL
}

inline void motor2_setDir(bool dir)
{
    digitalWriteFast(MOTOR2_DIR, dir);
    delayMicroseconds(T2_DIR_SETUP_US);   // t2
}

// -----------------------------------------------------------------------------
// Send a single step pulse on the given PUL pin
// Caller must have already set direction and waited t2.
// -----------------------------------------------------------------------------
inline void motor1_pulse()
{
    digitalWriteFast(MOTOR1_PUL, HIGH);
    delayMicroseconds(T3_PUL_HIGH_US);    // t3: HIGH >2.5µs
    digitalWriteFast(MOTOR1_PUL, LOW);
    delayMicroseconds(T4_PUL_LOW_US);     // t4: LOW >2.5µs
}

inline void motor2_pulse()
{
    digitalWriteFast(MOTOR2_PUL, HIGH);
    delayMicroseconds(T3_PUL_HIGH_US);
    digitalWriteFast(MOTOR2_PUL, LOW);
    delayMicroseconds(T4_PUL_LOW_US);
}

// -----------------------------------------------------------------------------
// Step N pulses at a given speed (RPM), with direction
// steps     : number of step pulses to send
// rpm       : motor speed in RPM
// stepsPerRev: full steps per revolution × microstep setting (e.g. 1600)
// -----------------------------------------------------------------------------
inline void motor1_step(bool dir, uint32_t steps, float rpm, uint32_t stepsPerRev = 1600)
{
    motor1_setDir(dir);                    // sets DIR + waits t2

    // Period between pulses in µs, minus the fixed HIGH+LOW time already spent
    uint32_t periodUs = (uint32_t)(60000000.0f / (rpm * stepsPerRev));
    uint32_t lowUs    = (periodUs > T3_PUL_HIGH_US) ? (periodUs - T3_PUL_HIGH_US) : T4_PUL_LOW_US;
    if (lowUs < T4_PUL_LOW_US) lowUs = T4_PUL_LOW_US;  // never violate t4

    for (uint32_t i = 0; i < steps; i++)
    {
        digitalWriteFast(MOTOR1_PUL, HIGH);
        delayMicroseconds(T3_PUL_HIGH_US);
        digitalWriteFast(MOTOR1_PUL, LOW);
        delayMicroseconds(lowUs);
    }
}

inline void motor2_step(bool dir, uint32_t steps, float rpm, uint32_t stepsPerRev = 1600)
{
    motor2_setDir(dir);

    uint32_t periodUs = (uint32_t)(60000000.0f / (rpm * stepsPerRev));
    uint32_t lowUs    = (periodUs > T3_PUL_HIGH_US) ? (periodUs - T3_PUL_HIGH_US) : T4_PUL_LOW_US;
    if (lowUs < T4_PUL_LOW_US) lowUs = T4_PUL_LOW_US;

    for (uint32_t i = 0; i < steps; i++)
    {
        digitalWriteFast(MOTOR2_PUL, HIGH);
        delayMicroseconds(T3_PUL_HIGH_US);
        digitalWriteFast(MOTOR2_PUL, LOW);
        delayMicroseconds(lowUs);
    }
}

#endif // MOTORPINS_H