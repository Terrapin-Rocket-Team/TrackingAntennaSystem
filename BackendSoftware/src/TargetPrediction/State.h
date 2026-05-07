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
struct State
{
    // Position in meters local frame with the origin at the Tracking Antenna System
    double posXM = 0.0; //latitude = x-axis
    double posYM = 0.0; //longitude = y-axis
    double posZM = 0.0; //altitude = z-axis
};

#endif // STATE_H