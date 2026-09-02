
#if NATIVE

 

#include "RecordData/DataReporter/DataReporter.h"

class FakeSAM_M10Q : public DataReporter
{
public:
    FakeSAM_M10Q();

    int begin() override;
    int update(double currentTime = -1) override;

    double getLatitude() const;
    double getLongitude() const;
    double getAltitudeMeters() const;
    double getHeadingDegrees() const;
    int getFixQual() const;
    bool getHasFix() const;

private:
    double timeSentSec = 0.0;
    double latitude = 38.989700;
    double longitude = -76.937800;
    double altitudeMeters = 12.0;
    double headingDegrees = 0.0;
    int fixQual = 0;
    int hasFix = 0;
};

#endif // FAKE_SAM_M10Q_H
