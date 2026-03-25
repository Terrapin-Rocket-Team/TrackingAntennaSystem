//Created by Jack Yeulenski 3/24/26

#ifndef RADIOMEASUREMENT_H
#define RADIOMEASUREMENT_H


#include <stdint.h>

/*
RadioMeasurement

This is an internal "clean" measurement format for PredictStep.
The RadioMessage library gives us an APRSTelem object directly, but the filter
should not need to know about the external library's full class layout.

Instead, we convert the decoded APRSTelem packet into this smaller struct and
then update the filter with it.
*/
struct RadioMeasurement
{
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeFt = 0.0;
    double speedKnots = 0.0;
    double headingDeg = 0.0;
    double orientationDeg[3] = {0.0, 0.0, 0.0};
    uint32_t stateFlags = 0;
    uint32_t receiveTimeMs = 0;
    bool valid = false;
};

#endif // RADIOMEASUREMENT_H