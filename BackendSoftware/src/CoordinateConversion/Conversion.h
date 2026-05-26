//Created by Divyansh Srivastava 5/25/2026
//This file will be to convert the distances in the x,y, and z direction of the rocket from the ATS
//Using the ATS as the origin, into an angle the motor will have to move 
#ifndef COORDCONVERT_H
#define COORDCONVERT_H
 
#include "TargetPrediction/State.h"
 
class CoordConvert
{
public:
 
    // Output angles — set by convert(), read by motor control
    double azimuthDeg   = 0.0;  // horizontal angle CW from North (degrees)
    double elevationDeg = 0.0;  // vertical angle above horizon (degrees)
 
    /*
        convert
 
        Takes the predicted rocket state in ENU (meters) and computes
        azimuth and elevation angles for the antenna to point at.
 
        Parameters:
            state — predicted rocket state from Propagate (ENU, meters)
    */
    void convert(const State& state);
};
 
#endif // COORDCONVERT_H
 