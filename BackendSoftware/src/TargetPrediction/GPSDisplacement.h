//untested

#ifndef GPS_DISPLACEMENT_H
#define GPS_DISPLACEMENT_H

struct GpsCoord {
    double lat; // degrees
    double lon; // degrees
    double alt; // meters
};

struct GpsDisplacement {
    double x; // north/south meters
    double y; // east/west meters
    double z; // altitude difference meters
};

GpsDisplacement displacementBetweenGps(const GpsCoord& origin, const GpsCoord& position);

#endif // GPS_DISPLACEMENT_H
