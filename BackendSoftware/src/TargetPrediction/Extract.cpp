/* Created by Jack Yeulenski 3/24/26
CPP file for the PredictStep class, which is responsible for taking in telemetry data
and extrapolating the future position of the target based on that data. 


*/
#include "TargetPrediction/Extract.h"

#include <cmath>
 
// WGS84 constants
static constexpr double EARTH_RAD   = 6378.137e3;                      // meters
static constexpr double RAD         = 3.14159265358979323846 / 180.0; //from angles in degrees to radians
static constexpr double EARTH_FLAT  = 1.0 / 298.257223563;
static constexpr double ECC_SQRD    = EARTH_FLAT * (2.0 - EARTH_FLAT);
static constexpr double M           = RAD * EARTH_RAD;
 
// Conversion factors
static constexpr double FT_TO_M     = 0.3048; //feet to meters
static constexpr double KNOTS_TO_MS = 0.514444; //meters per second in one knot

 
// Burn phase constants, used to calculate altitude during the burn (first 6 seconds after launch)
static constexpr double BURN_ACCEL_MS2  = 98.1;  // 10g vertical acceleration during burn (m/s^2)
static constexpr double BURN_DURATION_S = 6.0;    // burn phase duration (seconds)
 
// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
Extract::Extract(APRSData* message)
    : _message(message)
{
}
 
// ----------------------------------------------------------------------------
// ExtractTelemetry
// ----------------------------------------------------------------------------
State Extract::ExtractTelemetry(const uint8_t* telemetryBytes, size_t length, double dt, double timeSinceLaunch)
{
    // 1. Decode bytes into the APRSData subclass via polymorphism
    _message->decode(const_cast<uint8_t*>(telemetryBytes), static_cast<uint16_t>(length));
 
    // 2. Cast to APRSTelem to access telemetry fields
    APRSTelem* telem = static_cast<APRSTelem*>(_message);
 
    // 3. Compute WGS84 meters-per-degree scalers at the origin latitude
    double east, north, up;
    llaToENU(telem->lat, telem->lng, telem->alt,
             originLatDeg, originLngDeg, originAltFt,
             east, north, up);
 
    // 4. Decompose speed + heading into ENU velocity (m/s)
    //    heading is degrees CW from North
    //    velX = East, velY = North, velZ = Up (zeroed, no climb rate)
    double speedMS = telem->spd * KNOTS_TO_MS; //meters per second 
    double hdgRad  = telem->hdg * RAD; //measured from north clockwise, converted to radians
    double velX    =  speedMS * std::sin(hdgRad);  // East
    double velY    =  speedMS * std::cos(hdgRad);  // North
    

   // 5. Estimate vertical velocity (velZ)
    //
    //    First packet after launch: no previous altitude reading yet, use dynamic
    //    model (constant 10g acceleration) to estimate velZ.
    //    All subsequent packets: estimate from consecutive altitude readings.
    //        velZ = (alt_now - alt_prev) / dtPacket
    //
    double velZ = 0.0;
    double altM = telem->alt * FT_TO_M;  // current altitude in meters

    if (!_hasPrevAlt) // First packet — use dynamic model until next packet arrives
    {

        velZ = BURN_ACCEL_MS2 * timeSinceLaunch;
    }
    else if (dt > 0.0 && _hasPrevAlt) // sanity check to avoid division by zero
    {
        // Subsequent packets — estimate from altitude delta
        velZ = (altM - _prevAltM) / dt;
    }

    // Store altitude for next packet
    _prevAltM   = altM;
    _hasPrevAlt = true;


    // 6. Populate and return State
    State state;
    state.setPosition(east, north, up);
    state.setVelocity(velX, velY, velZ);
    // Acceleration left at (0,0,0) — no source in APRSTelem
 
    return state;
}
 
// ----------------------------------------------------------------------------
// llaToENU
// ----------------------------------------------------------------------------
void Extract::llaToENU(double lat,       double lng,       double altFt,
                       double originLat, double originLng, double originAltFt,
                       double &east,     double &north,    double &up)
{
    // Compute WGS84 meters-per-degree scalers at the origin latitude
    const double coslat = std::cos(originLat * RAD);
    const double w2     = 1.0 / (1.0 - ECC_SQRD * (1.0 - coslat * coslat));
    const double w      = std::sqrt(w2);
 
    const double kx = M * w * w2 * (1.0 - ECC_SQRD); // meters per degree latitude
    const double ky = M * w * coslat;                 // meters per degree longitude
 
    north = (lat - originLat) * kx; //figuring out north component of rocket from ATS in meters
    east  = (lng - originLng) * ky;
    up    = (altFt - originAltFt) * FT_TO_M;
}