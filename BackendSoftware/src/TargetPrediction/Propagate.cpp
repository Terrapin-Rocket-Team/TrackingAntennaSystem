/*
    Created by Jack yeulenski 3/28/26
    
    implementation ofstatic method to propogate a state forward dt with a transition matrix.
*/

#include "Propagate.h"

using astra::Matrix;

State Propagate::propagate(const Matrix& curState, double dt) {
    //position input is in meters NEU from Tracking Antenna System
    State predicted;

    double Fdata[] = {
        1, dt, 0.5*dt*dt, 0, 0, 0, 0, 0, 0,
        0, 1, dt, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, dt, 0.5*dt*dt, 0, 0, 0,
        0, 0, 0, 0, 1, dt, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, dt, 0.5*dt*dt,
        0, 0, 0, 0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 0, 0, 0, 1
    };

    Matrix transitionMatrix(9, 9, Fdata);
    Matrix predictedState = transitionMatrix * curState;

    predicted.posXM = predictedState(0, 0);
    predicted.posYM = predictedState(3, 0);
    predicted.posZM = predictedState(6, 0);

    return predicted;
}