/// Created by Matt Urban 5/21/26
/// Utilizes MotorPins.h and executes motor functions
#include "MotorPins.h"
#include <math.h>

// Define static member declared in header
//float motor_angle = 0.0f;

// -----------------------------------------------------------------------------
// Testing Variables
// -----------------------------------------------------------------------------
long int stepsOut = 0;
long int steps;

// -----------------------------------------------------------------------------
// Initialisation — call once in setup()
// -----------------------------------------------------------------------------
MotorPins::MotorPins(){
    // default constructor
}

MotorPins::MotorPins(int motor_dir, int motor_pul, int gear){
    gearRatio = gear;
    MOTOR_DIR = motor_dir;
    MOTOR_PUL = motor_pul;
    pinMode(motor_pul, OUTPUT);
    pinMode(motor_dir, OUTPUT);

    // Default both motors to CW, idle pulse line LOW
    digitalWrite(motor_dir, HIGH);
    digitalWrite(motor_pul, HIGH);  
    // t2: let DIR settle after init before any pulse can arrive
    delay(T2_DIR_SETUP_US);
}


void MotorPins::motor_init(int dir_pin, int pulse_pin, double initialAntennaAngle, int gear)
{
    gearRatio = gear;
    MOTOR_DIR = dir_pin;
    MOTOR_PUL = pulse_pin;

    pinMode(pulse_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);

    currentSteps = static_cast<int32_t>(lround(initialAntennaAngle*gearRatio*stepsPerRev/360.0));

    targetSteps = currentSteps;

    pulseHigh = false;
    directionPositive = true;

    uint32_t nowUs = micros();

    lastRiseUs = nowUs;
    lastFallUs = nowUs;

    directionChangedUs = nowUs;

    initialized = true;

    // Default both motors to CW, idle pulse line LOW
}

void MotorPins::setTargetAngle(double degrees)
{
    if (!initialized || !isfinite(degrees))
    {
        return;
    }

    targetSteps = static_cast<int32_t>(lround(degrees*gearRatio*stepsPerRev/360.0));
}

void MotorPins::update(uint32_t nowUs)
{
    if (!initialized)
    {
        return;
    }

    //Finish an active pulse before pulsing another step
    if (pulseHigh)
    {
        if (uint32_t(nowUs-lastRiseUs) >= T3_PUL_HIGH_US)
        {
            digitalWrite(MOTOR_PUL, LOW);
            lastFallUs = micros();
            pulseHigh = false;
        }
        return;
    }

    if (currentSteps == targetSteps) {
        return;
    }

    bool nextDirectionPositive = targetSteps > currentSteps;

    // Change direction only while step is low.
    if (nextDirectionPositive != directionPositive)
    {
        directionPositive = nextDirectionPositive;

        digitalWrite(MOTOR_DIR, directionPositive ? MOTOR_DIR_CW : MOTOR_DIR_CCW);

        directionChangedUs = micros();
        return;
    }

    if (uint32_t(nowUs - directionChangedUs) < T2_DIR_SETUP_US || 
        uint32_t(nowUs - lastFallUs) < T4_PUL_LOW_US || 
        uint32_t(nowUs - lastRiseUs) < stepIntervalUs) {
            return;
        }
    digitalWrite(MOTOR_PUL, HIGH);
    lastRiseUs = micros();
    pulseHigh = true;

    //track steps on the rising edge
    currentSteps += directionPositive ? 1 : -1;
}

    

// -----------------------------------------------------------------------------
// Set direction for a motor 
// Always call this before stepping if direction has changed.
// Includes t2 settling delay so caller does not need to.
// -----------------------------------------------------------------------------
void MotorPins::motor_setDir(bool dir)
{
    digitalWrite(MOTOR_DIR, dir);
    delay(T2_DIR_SETUP_US);   // t2: DIR must settle >5µs before PUL
}



// -----------------------------------------------------------------------------
// Send a single step pulse on the given PUL pin
// Caller must have already set direction and waited t2.
// -----------------------------------------------------------------------------
bool MotorPins::motor_pulse(bool dir)
{   
    // Send one STEP pulse
    digitalWriteFast(MOTOR_PUL, HIGH);
    delayMicroseconds(T3_PUL_HIGH_US);   // must meet driver minimum pulse width

    digitalWriteFast(MOTOR_PUL, LOW);
    delayMicroseconds(T4_PUL_LOW_US);    // must meet driver minimum low time
    motor_angle += ((dir) ? (1.80/((double)microSteps)) : (-1.80/((double)microSteps)));         // changed angle per microstep depends on the formula -> angle_per_step*step/microstep*1/Gear_ratio
    //stepsOut++; used for Debugging
    return true;
}

/// Function to drive motor to a certain angle by converting degrees to steps and revolutions
void MotorPins::motor_set_angle(double theta, float rpm, long int stepsPerRev) {
    double delta = (theta*((double)gearRatio)) - motor_angle; //Theta refers to the gearbox angle. theta*gearRatio returns the desire angle for the motor to rotate to.
    if (delta == 0) return;

    bool dir = (delta > 0.0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;

    steps = (long int)(fabs(delta)/360.0*(float)stepsPerRev); //(long int)(fabsf(delta) /360.0f * (float)stepsPerRev * (float)gearRatio);

    if (steps == 0) return;

    // ---------------------------
    // IMPORTANT NOTE
    // with a gear ratio of 1:10, the maximum rpm is 57.7 rpm before the motor fails.
    // with a gear ratio of 1:50 the maximum rpm is 11.5 rpm before the motor fails.
    // ---------------------------
    double maxOutputRPM = 1000000/(float)pulseTime*60.0/(float)stepsPerRev/(float)gearRatio;
    if (rpm > maxOutputRPM) rpm = maxOutputRPM;


    double stepsPerSecond = (double)rpm*(double)stepsPerRev*(double)gearRatio/60.0;
    double stepIntervalMicros = 1000000.0/stepsPerSecond; /// intervals for RPM
    ///stepsOut = 0; Debugging use 
    motor_setDir(dir);
    for (long int i = 0; i < steps; i++)
    {
        motor_pulse(dir);
        (stepIntervalMicros < T4_PUL_LOW_US) ? delayMicroseconds(T4_PUL_LOW_US): delayMicroseconds(stepIntervalMicros);
    }

}


int MotorPins::returnSteps(){
    return steps;
}

int MotorPins::returnStepsout(){
    return stepsOut;
}