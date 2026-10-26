// Created By Divyansh Srivastava on 5/10/2026, this file will defien the hardware
// level pin assignments and timing rules for the stepper motors, as well as helper functions to send pulses and set direction.
#ifndef MOTORPINS_H
#define MOTORPINS_H
#include <Arduino.h>

// first, don't write any code here. Create a cpp file called MotorPins.cpp and write all the code there.
// Then, include the function prototypes here in the header file.
// This way, we can keep the implementation separate from the interface, which is a good practice in C++ programming.

// =============================================================================
// MotorPins.h
// Motor Control — PUL/DIR pin definitions and signal helpers
// Teensy 4.1 → Logic Level Shifter → Stepper Driver
//
// ENA is NC (not connected) on both drivers.
// Timing rules from driver datasheet:
//   t1: ENA ahead of DIR by >5µs  — N/A (ENA not connected) we did not connect enable pin, so we can ignore t1
//   t2: DIR must settle >5µs before first PUL edge
//   t3: PUL HIGH width >2.5µs
//   t4: PUL LOW width >2.5µs
// =============================================================================
class MotorPins
{
public:
    MotorPins();    
    MotorPins(int motor_dir, int motor_pul, int gear); 

    // -----------------------------------------------------------------------------
    // Initialisation — call once in setup()
    // -----------------------------------------------------------------------------
    void motor_init(int dir_pin, int pulse_pin, double initialAntennaAngle, int gear);

    // -----------------------------------------------------------------------------
    // Set direction for a motor
    // Always call this before stepping if direction has changed.
    // Includes t2 settling delay so caller does not need to.
    // -----------------------------------------------------------------------------
    void motor_setDir(bool dir);

    // -----------------------------------------------------------------------------
    // Send a single step pulse on the given PUL pin
    // Caller must have already set direction and waited t2.
    // -----------------------------------------------------------------------------
    bool motor_pulse(bool dir);

    void update(uint32_t nowUs);

    void setTargetAngle(double degrees);

    void setMaxSpeedDegPerSec(double degreesPerSecond);

    int32_t getCurrentSteps() const { return currentSteps; }
    bool isMoving() const { return currentSteps != targetSteps || pulseHigh; }

    /// Function to drive motor to a certain angle by converting degrees to steps and revolutions
    void motor_set_angle(double theta, float rpm, long int stepsPerRev = 1600);

    inline float get_motor_angle() { return currentDeg; }

    // Testing Functions
    int returnSteps();
    int returnStepsout();


private:
    // -----------------------------------------------------------------------------
    // Pin assignments
    // MOTOR1_PUL -> LOW == CLOCKWISE, HIGH == CCW
    // MOTOR1_DIR -> HIGH == ROTATE, LOW == DO NOT ROTATE
    // -----------------------------------------------------------------------------
    int MOTOR_PUL; //each motor oject will have its own motor pins, only have one pul pin and one dir pin 
    int MOTOR_DIR;

    int32_t currentSteps = 0;
    int32_t targetSteps = 0;
    const int32_t stepsPerRev = 1600;

    uint32_t stepIntervalUs = 1000;
    uint32_t lastRiseUs = 0;
    uint32_t lastFallUs = 0;
    uint32_t directionChangedUs = 0;
    
    bool pulseHigh = false;
    bool directionPositive = true;
    bool initialized = false;

    // -----------------------------------------------------------------------------
    // Direction constants
    // -----------------------------------------------------------------------------
    const bool MOTOR_DIR_CCW = LOW; //these are constants we are physically going to have to check 
    const bool MOTOR_DIR_CW = HIGH;

    // -----------------------------------------------------------------------------
    // Angle constants
    // -----------------------------------------------------------------------------
    int32_t currentDeg = 0;

    // -----------------------------------------------------------------------------
    // Gear Ratio constants
    // -----------------------------------------------------------------------------
    int gearRatio; 
    const int microSteps = 8; //we cannot change microsteps, just have it as a constant no point in having a it as a field 

    // -----------------------------------------------------------------------------
    // Timing constants (microseconds)
    // All values exceed datasheet minimums with small margin
    // -----------------------------------------------------------------------------
    const uint32_t T2_DIR_SETUP_US = 6; // DIR settle before PUL  (min 5µs)
    const uint32_t T3_PUL_HIGH_US = 4;  // PUL HIGH width         (min 2.5µs)
    const uint32_t T4_PUL_LOW_US = 60;   // PUL LOW width          (min 2.5µs)
    unsigned long pulseTime = T3_PUL_HIGH_US + T4_PUL_LOW_US; // Pulse time for use with time intervals between pulses for RPM
};

#endif // MOTORPINS_H
