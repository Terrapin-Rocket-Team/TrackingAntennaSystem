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
struct RadioMeasurement //I believe the radio message library already gives us
//a function to decode aprs telem messages, why are we making our own struct? 
//I want to see some comments explaining the motivation for this design choice.
{
    double latitudeDeg = 0.0; //is there a way to get a delta meters given we have latitude and 
    //longitude in degrees
    double longitudeDeg = 0.0;
    double altitudeFt = 0.0; //is altitude in feet? 
    double speedKnots = 0.0; //what is this speed for? Is this derived from the velocity vector of the rocket?
    //if so, we must convert from the rocket's frame to our local frame 
    double headingDeg = 0.0;
    double orientationDeg[3] = {0.0, 0.0, 0.0};
    uint32_t stateFlags = 0;
    uint32_t receiveTimeMs = 0; //what is this recieve time for?
    bool valid = false;
};

#endif // RADIOMEASUREMENT_H