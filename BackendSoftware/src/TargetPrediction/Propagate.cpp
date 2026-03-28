/*
    Created by Jack yeulenski 3/28/26
    
    implementation ofstatic method to propogate a state forward dt with a transition matrix.
*/

#include "Propagate.h"

using astra::Matrix;

State Propagate::propagate(const Matrix& curState, double dt) {
    //position input is in meters NEU from Tracking Antenna System
    //note that we will not be using an NEU frame, we will have our own local frame
    //with the origin being our ATS and the x axis being latitude, and the y axis being longitude, and the z axis being altitude. 
    //This is because we will be using a Kalman filter that assumes a local frame, and it will be easier to work with a local frame 
    //than to convert back and forth between NEU and ECEF.
    State predicted;

    double Fdata[] = { //where did you get this from? I want to see some comments 
        //explaining how you derived this transition matrix, and what each element represents.
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

    return predicted; //i want this to return coordinates, not a state struct. 
    //I say this so Lokesh can implement coordinate conversion trig
}