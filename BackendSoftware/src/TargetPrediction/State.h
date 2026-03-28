//Created by Jack Yeulenski 3/24/26

#ifndef PREDICTEDTARGETSTATE_H
#define PREDICTEDTARGETSTATE_H

/*
PredictedTargetState

This is the output of PredictStep.
It stores the current best estimate of where the target will be after the
prediction horizon has been applied.

These values are intentionally simple and filter-facing:
- latitude / longitude / altitude describe the predicted position
- valid indicates whether the estimate is safe to use
*/
struct State
{
    // Position in meters NEU from Tracking Antenna System
    double posXM = 0.0;
    double posYM = 0.0;
    double posZM = 0.0;
};

#endif // STATE_H