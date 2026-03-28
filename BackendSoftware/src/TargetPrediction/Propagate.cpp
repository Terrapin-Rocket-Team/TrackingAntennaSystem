/*
    Created by Jack yeulenski 3/28/26
    
    implementation ofstatic method to propogate a state forward dt with a transition matrix.
*/

#include "Propagate.h"

using astra::Matrix;

State Propagate::propagate(const Matrix& curState, double dt) {
    //note that we will not be using an NEU frame, we will have our own local frame
    //with the origin being our ATS and the x axis being latitude, and the y axis being longitude, and the z axis being altitude. 
    //This is because we will be using a Kalman filter that assumes a local frame, and it will be easier to work with a local frame 
    //than to convert back and forth between NEU and ECEF.
    State predicted;

    /*
    curState is a 9x1 matrix with the following format:
        [Px]
        [Py]
        [Pz]
        [Vx]
        [Vy]
        [Vz]
        [Ax]
        [Ay]
        [Az]
    */

    double Fdata[] = {
        1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
        //Px = Px + Vx*dt + 0.5*Ax*dt^2
        0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
        //Py = Py + Vy*dt + 0.5*Ay*dt^2
        0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
        //Pz = Pz + Vz*dt + 0.5*Az*dt^2
        0, 0, 0, 1, 0, 0, dt, 0, 0,
        //Vx = Vx + Ax*dt
        0, 0, 0, 0, 1, 0, 0, dt, 0,
        //Vy = Vy + Ay*dt
        0, 0, 0, 0, 0, 1, 0, 0, dt,
        //Vz = Vz + Az*dt
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        //Ax = Ax
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        //Ay = Ay
        0, 0, 0, 0, 0, 0, 0, 0, 1
        //Az = Az
    };
        
    

    Matrix transitionMatrix(9, 9, Fdata);
    Matrix predictedState = transitionMatrix * curState;

    predicted.posXM = predictedState(0, 0);
    predicted.posYM = predictedState(3, 0);
    predicted.posZM = predictedState(6, 0);

    return predicted; //i want this to return coordinates, not a state struct. 
    //I say this so Lokesh can implement coordinate conversion trig
}