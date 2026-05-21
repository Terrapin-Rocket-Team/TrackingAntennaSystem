/*
    Created by Jack Yeulenski 3/28/26
*/

#include "Propagate.h"

void Propagate::update(double posX, double posY, double posZ,
                       double velX, double velY, double velZ)
{
    state.setPosition(posX, posY, posZ);
    state.setVelocity(velX, velY, velZ);
    state.setAcceleration(0.0, 0.0, 0.0);
}

void Propagate::propagate(double dt)
{
    /*
    Kinematic transition matrix F (9x9):

        Px_new = Px + Vx*dt + 0.5*Ax*dt^2     (Ax = 0, simplifies to Px + Vx*dt)
        Py_new = Py + Vy*dt + 0.5*Ay*dt^2     (Ay = 0, simplifies to Py + Vy*dt)
        Pz_new = Pz + Vz*dt + 0.5*Az*dt^2     (Az = 0, simplifies to Pz + Vz*dt)
        Vx_new = Vx + Ax*dt                    (Ax = 0, simplifies to Vx)
        Vy_new = Vy + Ay*dt                    (Ay = 0, simplifies to Vy)
        Vz_new = Vz + Az*dt                    (Az = 0, simplifies to Vz)
        Ax_new = Ax                             (= 0)
        Ay_new = Ay                             (= 0)
        Az_new = Az                             (= 0)
    */
    double fdata[] = {
        1, 0, 0, dt, 0,  0,  0.5*dt*dt, 0,        0,
        0, 1, 0, 0,  dt, 0,  0,         0.5*dt*dt, 0,
        0, 0, 1, 0,  0,  dt, 0,         0,         0.5*dt*dt,
        0, 0, 0, 1,  0,  0,  dt,        0,         0,
        0, 0, 0, 0,  1,  0,  0,         dt,        0,
        0, 0, 0, 0,  0,  1,  0,         0,         dt,
        0, 0, 0, 0,  0,  0,  1,         0,         0,
        0, 0, 0, 0,  0,  0,  0,         1,         0,
        0, 0, 0, 0,  0,  0,  0,         0,         1
    };

    Matrix f(9, 9, fdata);

    double xData[] = {
        state.getPosX(),
        state.getPosY(),
        state.getPosZ(),
        state.getVelX(),
        state.getVelY(),
        state.getVelZ(),
        state.getAccX(),  // 0
        state.getAccY(),  // 0
        state.getAccZ()   // 0
    };

    Matrix x(9, 1, xData);

    Matrix xPred = f * x;

    state.setPosition    (xPred(0, 0), xPred(1, 0), xPred(2, 0));
    state.setVelocity    (xPred(3, 0), xPred(4, 0), xPred(5, 0));
    state.setAcceleration(xPred(6, 0), xPred(7, 0), xPred(8, 0));
}