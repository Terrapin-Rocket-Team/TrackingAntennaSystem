#if NATIVE
#include "Fake-SAM-M10Q.h"

FakeSAM_M10Q::FakeSAM_M10Q() : DataReporter("Fake SAM-M10Q")
{
    addColumn("%.3f", &timeSentSec, "Time Sent (s)");
    addColumn("%.6f", &latitude, "Latitude");
    addColumn("%.6f", &longitude, "Longitude");
    addColumn("%.2f", &altitudeMeters, "Altitude (m)");
    addColumn("%.2f", &headingDegrees, "Heading (deg)");
    addColumn("%d", &fixQual, "Fix Quality");
    addColumn("%d", &hasFix, "Has Fix");
}

int FakeSAM_M10Q::begin()
{
    fixQual = 1;
    hasFix = 1;
    initialized = true;
    return 0;
}

int FakeSAM_M10Q::update(double currentTime)
{
    timeSentSec = currentTime;

    latitude += 0.000010;
    longitude -= 0.000012;
    altitudeMeters += 0.03;
    headingDegrees += 1.5;
    if (headingDegrees >= 360.0)
    {
        headingDegrees -= 360.0;
    }

    return 0;
}

double FakeSAM_M10Q::getLatitude() const { return latitude; }
double FakeSAM_M10Q::getLongitude() const { return longitude; }
double FakeSAM_M10Q::getAltitudeMeters() const { return altitudeMeters; }
double FakeSAM_M10Q::getHeadingDegrees() const { return headingDegrees; }
int FakeSAM_M10Q::getFixQual() const { return fixQual; }
bool FakeSAM_M10Q::getHasFix() const { return hasFix != 0; }

#endif // FAKE_SAM_M10Q_CPP
