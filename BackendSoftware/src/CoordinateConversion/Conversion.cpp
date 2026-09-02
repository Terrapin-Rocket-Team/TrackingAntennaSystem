#include "Conversion.h"
#include <cmath>
 
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
 
void CoordConvert::convert(const State& state)
{
    double posX = state.getPosX();  // East  (meters)
    double posY = state.getPosY();  // North (meters)
    double posZ = state.getPosZ();  // Up    (meters)
 
    // Horizontal distance from ATS to rocket
    double horizontalDist = std::sqrt(posX * posX + posY * posY);
 
    // Azimuth — angle CW from North to rocket, 0-360 degrees
    // atan2(East, North) gives angle CCW from North, so we normalize
    double az = std::atan2(posX, posY) * RAD_TO_DEG;
    if (az < 0.0) az += 360.0;
    azimuthDeg = az;
 
    // Elevation — angle above horizon to rocket, 0-90 degrees
    elevationDeg = std::atan2(posZ, horizontalDist) * RAD_TO_DEG;
}
 