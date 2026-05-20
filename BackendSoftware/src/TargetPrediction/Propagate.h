/*
    Created by Jack Yeulenski 3/28/26

    Propagates the rocket's state forward by dt seconds to compensate for
    telemetry latency. Uses a 9x1 kinematic state vector (position + velocity + acceleration).
    Acceleration is always zero as it is not available in the telemetry.

    State vector format (9x1):
        [ Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az ]^T

    Propagation:
        state = F * state
*/

#ifndef PROPAGATE_H
#define PROPAGATE_H

#include "State.h"
#include "Math/Matrix.h"

class Propagate
{
public:

    // The single state estimate. Seeded by update() on each telemetry packet,
    // then propagated forward by propagate() to compensate for latency.
    // Acceleration rows are always zero.
    State state;

    /*
        update

        Called each time a new telemetry packet arrives.
        Seeds position and velocity from Extract. Acceleration stays zero.

        Parameters:
            posX, posY, posZ — ENU position (meters)
            velX, velY, velZ — ENU velocity (m/s)
    */
    void update(double posX, double posY, double posZ,
                double velX, double velY, double velZ);

    /*
        propagate

        Applies the 9x9 kinematic transition matrix F to the current state,
        updating it in place. Call after update() to advance the state
        forward by the latency dt.

        Parameters:
            dt — latency time step in seconds
    */
    void propagate(double dt);
};

#endif // PROPAGATE_H