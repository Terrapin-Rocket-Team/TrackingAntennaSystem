/*
Created by Jack Yeulenski 3/24/26

PredictStep is responsible for:
1. Taking in radio telemetry bytes.
2. Decoding those bytes using the RadioMessage library.
3. Converting the decoded telemetry into a measurement.
4. Running a Kalman filter update step.
5. Predicting the target state forward by about 0.5 seconds.
*/

#ifndef PREDICTSTEP_H
#define PREDICTSTEP_H

#include <stdint.h>
#include <RadioMessage.h>
#include "State.h"
#include "RadioMeasurement.h"
#include <Math/Matrix.h>

//what is this class for? Is it not done?

class PredictStep {
    /*
        static method to take in telemetry and return a state with a postion prediction.
        Need 2 methods:
            - Method that takes in telemetry, decodes it, converts it into a 9x1 Matrix
                Need to talk to Joseph
            - Method to propagate dt and return a predicted state.
                DONE
    */
        
};

#endif