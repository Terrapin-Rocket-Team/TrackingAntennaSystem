/// Created by Matt Urban 5/21/26
/// Utilizes MotorPins.h and executes motor functions
#include "MotorPins.h"

// Define static member declared in header
float MotorPins::motor_angle = 0.0f;


// -----------------------------------------------------------------------------
// Initialisation — call once in setup()
// -----------------------------------------------------------------------------
MotorPins::MotorPins(){
    // default constructor
}

MotorPins::MotorPins(int motor_pul, int motor_dir, int gear){
    gearRatio = gear;
    pinMode(motor_pul, OUTPUT);
    pinMode(motor_dir, OUTPUT);

    // Default both motors to CW, idle pulse line LOW
    digitalWriteFast(motor_dir, MOTOR_DIR_CW);
    digitalWriteFast(motor_pul, HIGH);  
    // t2: let DIR settle after init before any pulse can arrive
    delayMicroseconds(T2_DIR_SETUP_US);
}


void MotorPins::motor_init(int motor_pul, int motor_dir, int gear)
{
    gearRatio = gear;
    pinMode(motor_pul, OUTPUT);
    pinMode(motor_dir, OUTPUT);

    // Default both motors to CW, idle pulse line LOW
    digitalWriteFast(motor_dir, MOTOR_DIR_CW);
    digitalWriteFast(motor_pul, HIGH);  
    // t2: let DIR settle after init before any pulse can arrive
    delayMicroseconds(T2_DIR_SETUP_US);
}


// -----------------------------------------------------------------------------
// Set direction for a motor
// Always call this before stepping if direction has changed.
// Includes t2 settling delay so caller does not need to.
// -----------------------------------------------------------------------------
void MotorPins::motor_setDir(bool dir)
{
    digitalWriteFast(MOTOR1_DIR, dir);
    delayMicroseconds(T2_DIR_SETUP_US);   // t2: DIR must settle >5µs before PUL
}



// -----------------------------------------------------------------------------
// Send a single step pulse on the given PUL pin
// Caller must have already set direction and waited t2.
// -----------------------------------------------------------------------------
bool MotorPins::motor_pulse(bool dir) //add the angle stuff here, also i'd like you to return a bool for whether the pulse was successful or not, so we can use that in our 
//set angle function to make sure we only update the angle if the pulse was successful.
{   // at every pulse, we have to add our angle which we can calculate by taking the steps we just sent,
    // dividing by the steps per revolution, and multiplying by 360.
    // then we can add that to our global angle variable. 
    // this way, we can keep track of the current angle of the motor and use that to calculate the delta angle for the next set_angle call.
    digitalWriteFast(MOTOR1_PUL, LOW);
    delayMicroseconds(T3_PUL_HIGH_US);    // t3: HIGH >2.5µs
    digitalWriteFast(MOTOR1_PUL, HIGH);
    delayMicroseconds(T4_PUL_LOW_US);     // t4: LOW >2.5µs
    motor_angle += ((dir) ? 1.80f/((float)microSteps) : -1.0f * 1.80f/((float)microSteps));         // changed angle per microstep depends on the formula -> angle_per_step*step/microstep*1/Gear_ratio
    return true;
}

/// Function to drive motor to a certain angle by converting degrees to steps and revolutions
void MotorPins::motor_set_angle(float theta, float rpm, uint32_t stepsPerRev = 1600) {
    float delta = theta*gearRatio - motor_angle; //Theta refers to the gearbox angle. theta*gearRatio returns the desire angle for the motor to rotate to.
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
        motor_pulse(dir);
    }

    //motor_angle = theta;
}

