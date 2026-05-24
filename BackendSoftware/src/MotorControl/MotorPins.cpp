/// Created by Matt Urban 5/21/26
/// Utilizes MotorPins.h and executes motor functions
#include "MotorPins.h"


// -----------------------------------------------------------------------------
// Initialisation — call once in setup()
// -----------------------------------------------------------------------------
MotorPins::MotorPins(){
    // default constructor
}

MotorPins::MotorPins(int motor_pul, int motor_dir, int gear, int micro){
    gearRatio = gear;
    microSteps = micro;
    pinMode(motor_pul, OUTPUT);
    pinMode(motor_dir, OUTPUT);

    // Default both motors to CW, idle pulse line LOW
    digitalWriteFast(motor_dir, MOTOR_DIR_CW);
    digitalWriteFast(motor_pul, HIGH); //change this to high. REMEMBER, LOW = TURNS ON. HIGH = TURNS OFF. WE ARE USING LOW ACTIVE ENABLE

    // t2: let DIR settle after init before any pulse can arrive
    delayMicroseconds(T2_DIR_SETUP_US);
}

// -----------------------------------------------------------------------------
// Set direction for a motor
// Always call this before stepping if direction has changed.
// Includes t2 settling delay so caller does not need to.
// -----------------------------------------------------------------------------
inline void MotorPins::motor_setDir(bool dir)
{
    digitalWriteFast(MOTOR1_DIR, dir);
    delayMicroseconds(T2_DIR_SETUP_US);   // t2: DIR must settle >5µs before PUL
}



// -----------------------------------------------------------------------------
// Send a single step pulse on the given PUL pin
// Caller must have already set direction and waited t2.
// -----------------------------------------------------------------------------
inline void MotorPins::motor_pulse() 
{   // at every pulse, we have to add our angle which we can calculate by taking the steps we just sent,
    // dividing by the steps per revolution, and multiplying by 360.
    // then we can add that to our global angle variable. 
    // this way, we can keep track of the current angle of the motor and use that to calculate the delta angle for the next set_angle call.
    digitalWriteFast(MOTOR1_PUL, LOW);
    delayMicroseconds(T3_PUL_HIGH_US);    // t3: HIGH >2.5µs
    digitalWriteFast(MOTOR1_PUL, HIGH);
    delayMicroseconds(T4_PUL_LOW_US);     // t4: LOW >2.5µs
}

/// Function to drive motor to a certain angle by converting degrees to steps and revolutions
inline void MotorPins::motor_set_angle(float theta, float rpm, uint32_t stepsPerRev = 1600) {
    //need to also take into account the 10:1/50:1 gear ratio which means that the 
    //motor shaft needs to turn 10/50 times more than the output shaft, so we need to multiply 
    //the steps per revolution by the gear ratio.
    float delta = theta - motor_angle;
    if (delta == 0) return;

    bool dir = (delta > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;

    uint32_t steps = (uint32_t)(fabsf(delta) / 360.0f * (float)stepsPerRev * (float)gearRatio);

    if (steps == 0) return;

    motor_setDir(dir);
    uint32_t periodUs = (uint32_t)(60000000.0f / (rpm * stepsPerRev)); 
    uint32_t lowUs    = (periodUs > T3_PUL_HIGH_US) ? (periodUs - T3_PUL_HIGH_US) : T4_PUL_LOW_US;
    if (lowUs < T4_PUL_LOW_US) lowUs = T4_PUL_LOW_US;  // never violate t4
    for (uint32_t i = 0; i < steps; i++)
    {
        motor_pulse();
        motor_angle += ((dir) ? 1.80f/((float)microSteps*(float)gearRatio) : -1.0f * 1.80f/((float)microSteps*(float)gearRatio)); // changed angle per microstep depends on the formula -> angle_per_step*step/microstep*1/Gear_ratio
    }

    //motor_angle = theta;
}

float MotorPins::get_motor_angle(){
    return motor_angle;
}

int MotorPins::get_gear_ratio(){
    return gearRatio;
}

int MotorPins::get_micro_steps(){
    return microSteps;
}
