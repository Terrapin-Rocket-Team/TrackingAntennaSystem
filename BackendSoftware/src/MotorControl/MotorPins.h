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
    MotorPins(int motor_pul, int motor_dir, int gear);  //remove micro 
    // -----------------------------------------------------------------------------
    // Initialisation — call once in setup()
    // -----------------------------------------------------------------------------
    inline void motorPins_init();

    // -----------------------------------------------------------------------------
    // Set direction for a motor
    // Always call this before stepping if direction has changed.
    // Includes t2 settling delay so caller does not need to.
    // -----------------------------------------------------------------------------
    inline void motor_setDir(bool dir);

    // -----------------------------------------------------------------------------
    // Send a single step pulse on the given PUL pin
    // Caller must have already set direction and waited t2.
    // -----------------------------------------------------------------------------
    inline bool motor_pulse(bool dir);

    /// Function to drive motor to a certain angle by converting degrees to steps and revolutions
    inline void motor_set_angle(float theta, float rpm, uint32_t stepsPerRev = 1600);

    //getter functions
    float get_motor_angle();
private:
    // -----------------------------------------------------------------------------
    // Pin assignments
    // -----------------------------------------------------------------------------
    const int MOTOR1_PUL = 8; //each motor oject will have its own motor pins, only have one pul pin and one dir pin 
    const int MOTOR1_DIR = 9;

    const int MOTOR2_PUL = 24;
    const int MOTOR2_DIR = 25;

    // -----------------------------------------------------------------------------
    // Direction constants
    // -----------------------------------------------------------------------------
    const bool MOTOR_DIR_CW = HIGH; //these are constants we are physically going to have to check 
    const bool MOTOR_DIR_CCW = LOW;

    // -----------------------------------------------------------------------------
    // Angle constants
    // -----------------------------------------------------------------------------

    static float motor_angle;
    // -----------------------------------------------------------------------------
    // Gear Ratio constants
    // -----------------------------------------------------------------------------
    int gearRatio; 
    const int steps = 200;
    const int microSteps = 8; //we cannot change microsteps, just have it as a constant no point in having a it as a field 

    // -----------------------------------------------------------------------------
    // Timing constants (microseconds)
    // All values exceed datasheet minimums with small margin
    // -----------------------------------------------------------------------------
    const uint32_t T2_DIR_SETUP_US = 6; // DIR settle before PUL  (min 5µs)
    const uint32_t T3_PUL_HIGH_US = 3;  // PUL HIGH width         (min 2.5µs)
    const uint32_t T4_PUL_LOW_US = 3;   // PUL LOW width          (min 2.5µs)

};

#endif // MOTORPINS_H

/*

// -----------------------------------------------------------------------------
// Step N pulses at a given speed (RPM), with direction
// steps     : number of step pulses to send
// rpm       : motor speed in RPM
// stepsPerRev: full steps per revolution × microstep setting (e.g. 1600)
// -----------------------------------------------------------------------------

//this funciton is not helpful
//its hard to calculate the number of steps needed to move a certain angle, so instead we can just have a function that takes in the
//desired angle and calculates the steps needed to get there. We can also have a global variable
//that keeps track of the current angle of the motor, so we can calculate the delta angle and convert that to steps. This way, we can just call set_angle with the desired angle and it will take care of the rest.
inline void motor1_step(bool dir, uint32_t steps, float rpm, uint32_t stepsPerRev = 1600) //change the default steps per rev, our two diff motors have diff gear ratios
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


//same issue with this
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
*/
