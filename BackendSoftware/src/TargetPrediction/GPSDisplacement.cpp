//ported from astra
//untested

#include "GpsDisplacement.h"

#include <cmath>

struct GpsCoord {
    double lat; // degrees
    double lon; // degrees
    double alt; // meters
};

struct Displacement {
    double x; // north/south meters
    double y; // east/west meters
    double z; // altitude difference meters
};

static double wrapLongitude(double lonDelta)
{
    while (lonDelta > 180.0)
        lonDelta -= 360.0;
    while (lonDelta < -180.0)
        lonDelta += 360.0;
    return lonDelta;
}

Displacement displacementBetweenGps(const GpsCoord& origin, const GpsCoord& position)
{
    constexpr double EARTH_RAD = 6378.137e3; // meters
    constexpr double RAD = 3.14159265358979323846 / 180.0;
    constexpr double EARTH_FLAT = 1.0 / 298.257223563;
    constexpr double ECC_SQRD = EARTH_FLAT * (2.0 - EARTH_FLAT);

    // behavior: compute scale factors at the current latitude.
    const double m = RAD * EARTH_RAD;
    const double coslat = std::cos(position.lat * RAD);
    const double w2 = 1.0 / (1.0 - ECC_SQRD * (1.0 - coslat * coslat));
    const double w = std::sqrt(w2);

    const double ky = m * w * coslat;                // meters per longitude degree
    const double kx = m * w * w2 * (1.0 - ECC_SQRD); // meters per latitude degree

    Displacement displacement;
    displacement.x = (position.lat - origin.lat) * kx;
    displacement.y = wrapLongitude(position.lon - origin.lon) * ky;
    displacement.z = position.alt - origin.alt;

    return displacement;
}
