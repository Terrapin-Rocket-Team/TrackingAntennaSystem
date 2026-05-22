/// Created by Matt Urban 5/21/26
/// Utilizes MotorPins.h and executes motor functions
#include <MotorPins.h>
#include <Arduino.h>

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
    digitalWriteFast(MOTOR1_PUL, LOW);
    delayMicroseconds(T3_PUL_HIGH_US);    // t3: HIGH >2.5µs
    digitalWriteFast(MOTOR1_PUL, HIGH);
    delayMicroseconds(T4_PUL_LOW_US);     // t4: LOW >2.5µs
}

inline void motor2_pulse() //same issue with this 
{
    digitalWriteFast(MOTOR2_PUL, LOW);
    delayMicroseconds(T3_PUL_HIGH_US);
    digitalWriteFast(MOTOR2_PUL, HIGH);
    delayMicroseconds(T4_PUL_LOW_US);
}


/// Function to drive motor to a certain angle by converting degrees to steps and revolutions
inline void motor50(float theta, float rpm, uint32_t stepsPerRev = 1600) { //why is the rpm a default value?
    //need to also take into account the 10:1/50:1 gear ratio which means that the 
    //motor shaft needs to turn 10/50 times more than the output shaft, so we need to multiply 
    //the steps per revolution by the gear ratio.
    float delta = theta - motor1_angle;
    if (delta == 0) return;

    bool dir = (delta > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;

    uint32_t steps = (uint32_t)(fabsf(delta) / 360.0f * (float)stepsPerRev * MOTOR1_RATIO);

    if (steps == 0) return;k 

    motor_setDir(dir);
    uint32_t periodUs = (uint32_t)(60000000.0f / (rpm * stepsPerRev)); 
    uint32_t lowUs    = (periodUs > T3_PUL_HIGH_US) ? (periodUs - T3_PUL_HIGH_US) : T4_PUL_LOW_US;
    if (lowUs < T4_PUL_LOW_US) lowUs = T4_PUL_LOW_US;  // never violate t4
    for (uint32_t i = 0; i < steps; i++)
    {
        motor1_pulse();
    }

    motor1_angle = theta;
}