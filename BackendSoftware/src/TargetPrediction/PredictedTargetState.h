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
struct PredictedTargetState
{
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeFt = 0.0;
    bool valid = false;
};

#endif // PREDICTEDTARGETSTATE_H