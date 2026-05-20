/*
Created by Jack Yeulenski 3/24/26

PredictStep is responsible for:
1. Taking in radio telemetry bytes.
2. Decoding those bytes using the RadioMessage library.
3. Converting the decoded telemetry into a measurement.
4. Running a Kalman filter update step.
5. Predicting the target state forward by about 0.5 seconds.
*/

#ifndef PREDICTSTEP_H
#define PREDICTSTEP_H

#include <stdint.h>
#include <RadioMessage.h>
#include "State.h"
#include <./Math/Matrix.h>

/*
    Extract
 
    Decodes any APRS packet (APRSTelem, APRSTextMessage, etc.) and converts it
    into a State object expressed in a local ENU frame whose origin is the
    Antenna Tracking System (ATS). The ENU frame is defined as follows:
        - X axis points East (derived from longitude delta)
        - Y axis points North (derived from latitude delta)
        - Z axis points Up (derived from altitude delta)
 
    Takes a pointer to any APRSData subclass. Polymorphism handles calling the
    correct decode() and field access for each packet type.
 
    Coordinate convention (ENU):
        posXM — East  (meters), derived from longitude delta
        posYM — North (meters), derived from latitude delta
        posZM — Up    (meters), derived from altitude
 
    Velocity convention:
        Speed (knots) and heading (degrees CW from North) are decomposed into
        ENU components. Vertical velocity is zeroed — APRSTelem carries no climb-rate.
*/
class Extract {

   public:
 
    // Origin of the local ENU frame (Tracking Antenna System location).
    // Must be set before calling ExtractTelemetry.
    double originLatDeg = 0.0;  // TAS latitude  (decimal degrees)
    double originLngDeg = 0.0;  // TAS longitude (decimal degrees)
    double originAltFt  = 0.0;  // TAS altitude  (feet)
 
    /*
        Constructor
 
        Parameters:
            message — pointer to any APRSData subclass (APRSTelem, APRSTextMessage, etc.)
                      The correct decode() is resolved at runtime via polymorphism.
    */
    Extract(APRSData* message);
 
    /*
        ExtractTelemetry
 
        Calls decode() on the stored APRSData pointer, then casts to APRSTelem
        to read lat/lng/alt/spd/hdg and populate a State in the local ENU frame.
 
        Parameters:
            telemetryBytes — raw byte buffer received over radio
            length         — number of valid bytes in the buffer
            dt             — time delta (seconds) reserved for a propagation step
 
        Returns:
            State with position (m, ENU) and velocity (m/s, ENU) populated.
            Acceleration is left at zero — not available in APRSTelem.
    */
    State ExtractTelemetry(const uint8_t* telemetryBytes, size_t length, double dt);
 
private:
 
    // Pointer to any APRSData subclass — decode() resolved via polymorphism.
    APRSData* _message;
 
    /*
        llaToENU
 
        Converts a geodetic position (lat/lng in decimal degrees, alt in feet)
        to local East-North-Up coordinates (meters) relative to a given origin.
 
        Uses the flat-Earth approximation — valid for ranges up to tens of km.
 
        Outputs:
            east  — positive East  (meters)
            north — positive North (meters)
            up    — positive Up    (meters)
    */
    static void llaToENU(double lat,       double lng,       double altFt,
                         double originLat, double originLng, double originAltFt,
                         double &east,     double &north,    double &up);
};

#endif