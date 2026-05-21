//Created By Divyansh Srivastava on 5/10/2026, this file will defien the hardware
//level pin assignments and timing rules for the stepper motors, as well as helper functions to send pulses and set direction.
#ifndef MOTORPINS_H
#define MOTORPINS_H
#include <Arduino.h>

//first, don't write any code here. Create a cpp file called MotorPins.cpp and write all the code there. 
//Then, include the function prototypes here in the header file. 
//This way, we can keep the implementation separate from the interface, which is a good practice in C++ programming.



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
// Angle constants
// -----------------------------------------------------------------------------

static float motor1_angle = 0.0f;
// static float motor2_angle = 0.0f; why is this commented out? we need to keep track of the angle of both motors, so we should have a global variable for each motor's angle. we can initialize them to 0, and then update them whenever we call set_angle. this way, we can always calculate the delta angle correctly.

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
    digitalWriteFast(MOTOR1_PUL, LOW); //change this to high. REMEMBER, LOW = TURNS ON. HIGH = TURNS OFF. WE ARE USING LOW ACTIVE ENABLE
    digitalWriteFast(MOTOR2_PUL, LOW); //change this to high 

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
inline void motor1_pulse() //this funciton is wrong, need to change it because LOW = active 
{
    digitalWriteFast(MOTOR1_PUL, HIGH);
    delayMicroseconds(T3_PUL_HIGH_US);    // t3: HIGH >2.5µs
    digitalWriteFast(MOTOR1_PUL, LOW);
    delayMicroseconds(T4_PUL_LOW_US);     // t4: LOW >2.5µs
}

inline void motor2_pulse() //same issue with this 
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


/// Function to drive motor to a certain angle by converting degrees to steps and revolutions
inline void motor1_set_anngle(float theta, float rpm = 10.0f, uint32_t stepsPerRev = 1600) { //why is the rpm a default value?
    //need to also take into account the 10:1/50:1 gear ratio which means that the 
    //motor shaft needs to turn 10/50 times more than the output shaft, so we need to multiply 
    //the steps per revolution by the gear ratio.
    float delta = theta - motor1_angle;
    if (delta == 0) return;

    bool dir = (delta > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;

    uint32_t steps = (uint32_t)(fabsf(delta) / 360.0f * (float)stepsPerRev);

    if (steps == 0) return;

    motor1_step(dir, steps, rpm, stepsPerRev);
    motor1_angle = theta;
}

// can copy over final set_angle function for motor 2
// TODO - set angle of motor during initialization

#endif // MOTORPINS_H