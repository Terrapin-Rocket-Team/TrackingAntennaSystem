//Created by Jack Yeulenski 3/24/26
#ifndef PREDICTEDTARGETSTATE_H
#define PREDICTEDTARGETSTATE_H

/*
PredictedTargetState

This is the output of PredictStep.
It stores the current best estimate of where the target will be after the
prediction horizon has been applied.

These values are intentionally simple
*/
class State {

    public:
    // -------------------------
    // POSITION SETTERS / GETTERS
    // -------------------------
    void setPosition(double x, double y, double z)
    {
        posXM = x;
        posYM = y;
        posZM = z;
    }

    void getPosition(double &x, double &y, double &z) const
    {
        x = posXM;
        y = posYM;
        z = posZM;
    }

    double getPosX() const { return posXM; }
    double getPosY() const { return posYM; }
    double getPosZ() const { return posZM; }

    // -------------------------
    // VELOCITY SETTERS / GETTERS
    // -------------------------
    void setVelocity(double vx, double vy, double vz)
    {
        velXM = vx;
        velYM = vy;
        velZM = vz;
    }

    void getVelocity(double &vx, double &vy, double &vz) const
    {
        vx = velXM;
        vy = velYM;
        vz = velZM;
    }

    double getVelX() const { return velXM; }
    double getVelY() const { return velYM; }
    double getVelZ() const { return velZM; }

    // -------------------------
    // ACCELERATION SETTERS / GETTERS
    // -------------------------
    void setAcceleration(double ax, double ay, double az)
    {
        accXM = ax;
        accYM = ay;
        accZM = az;
    }

    void getAcceleration(double &ax, double &ay, double &az) const
    {
        ax = accXM;
        ay = accYM;
        az = accZM;
    }

    double getAccX() const { return accXM; }
    double getAccY() const { return accYM; }
    double getAccZ() const { return accZM; }





    private: 
    // Position in meters local frame with the origin at the Tracking Antenna System
    double posXM = 0.0; //latitude = x-axis
    double posYM = 0.0; //longitude = y-axis
    double posZM = 0.0; //altitude = z-axis
 

    //velocity and acceleration needed because of the propogate step 


    // velocity (meters/sec in local frame)
    double velXM = 0.0;
    double velYM = 0.0;
    double velZM = 0.0;

    // acceleration (meters/sec^2 in local frame)
    double accXM = 0.0;
    double accYM = 0.0;
    double accZM = 0.0;
};


#endif // STATE_H