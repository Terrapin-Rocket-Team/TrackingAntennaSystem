//Created by Divyansh Srivastava on 4/5/2026
//Implementation file for the SAM-M10Q GPS module, which is used for tracking the position of the antenna system
#include "GPS.h"

GPS::GPS() {
    //default constructor  
}

GPS::GPS(const char *name, TwoWire &wirePort, uint8_t address) : DataReporter(name) {
    this->wire = &wirePort;
    this->address = address; // use the passed address, not hardcoded
    this->hz = 10.0;         // default frequency

    addColumn("%0.7f", &position.x(), "Lat (deg)");
    addColumn("%0.7f", &position.y(), "Lon (deg)");
    addColumn("%0.2f", &position.z(), "Alt (m)");
    addColumn("%0.2f", &velocity.x(), "VelN (m/s)");
    addColumn("%0.2f", &velocity.y(), "VelE (m/s)");
    addColumn("%0.2f", &velocity.z(), "VelD (m/s)");
    addColumn("%d", &fixQual, "SIV");
}

GPS::GPS(const char *name, TwoWire &wirePort, double hz) : DataReporter(name) {
    this->wire = &wirePort;
    this->address = 0x42; // default I2C address for SAM-M10Q

    // clamp to SAM-M10Q supported range (0.1 - 10 Hz)
    if (hz < 0.1) hz = 0.1;
    if (hz > 10.0) hz = 10.0;
    this->hz = hz;

    addColumn("%0.7f", &position.x(), "Lat (deg)");
    addColumn("%0.7f", &position.y(), "Lon (deg)");
    addColumn("%0.2f", &position.z(), "Alt (m)");
    addColumn("%0.2f", &velocity.x(), "VelN (m/s)");
    addColumn("%0.2f", &velocity.y(), "VelE (m/s)");
    addColumn("%0.2f", &velocity.z(), "VelD (m/s)");
    addColumn("%d", &fixQual, "SIV");
}


GPS::~GPS(){
    // no memory to clean
}




int GPS::begin(){
    if (!sam_m10q.begin(*wire, address)){
        initialized = false;
        isHealthy = false;
        return -1;
    }

    uint16_t measIntervalMs = (uint16_t)(1000.0 / this->hz); // convert Hz to ms interval

    sam_m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM);
    sam_m10q.setMeasurementRate(measIntervalMs, VAL_LAYER_RAM);
    sam_m10q.setNavigationRate(1, VAL_LAYER_RAM);        // solve every measurement
    sam_m10q.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    sam_m10q.setAutoPVTrate(1, VAL_LAYER_RAM);           // push PVT every cycle

    initialized = true;
    isHealthy = true;
    return 0;
}





int GPS::update(double currentTime){
    if (!initialized || !sam_m10q.getPVT() || sam_m10q.getInvalidLlh()){ // see if new data is available
        return -1;
    }


    //get all vals
    position.x() = sam_m10q.getLatitude() / 10000000.0;
    position.y() = sam_m10q.getLongitude() / 10000000.0;
    position.z() = sam_m10q.getAltitude() / 1000.0;

    velocity.x() = sam_m10q.getNedNorthVel() / 1000.0;
    velocity.y() = sam_m10q.getNedEastVel() / 1000.0;
    velocity.z() = sam_m10q.getNedDownVel() / 1000.0;

    heading = sam_m10q.getHeading() / 100000.0;
    fixQual = sam_m10q.getSIV();

    hr = sam_m10q.getHour();
    min = sam_m10q.getMinute();
    sec = sam_m10q.getSecond();
    day = sam_m10q.getDay();
    month = sam_m10q.getMonth();
    year = sam_m10q.getYear();



    hasFix = (fixQual >= 4); // update fix status
    isHealthy = hasFix;
    if (hasFix && !hasFirstFix){
        hasFirstFix = true;
        calcInitialValuesForDistance(); // for kx/ky
    }

    //format the time componenet nicely too using this: 
    if (hasFix) {
        hr += hrOffset;
        hr = (hr % 24 + 24) % 24; // handles negative wrap in cpp
        min = min % 60;
        sec = sec % 60;
        snprintf(tod, 12, "%02d:%02d:%02d", hr, min, sec);
    }

    return 0;
}



//meters per degree scalers at current altitude
void GPS::calcInitialValuesForDistance(){ 
    //this is the correct code:  
    constexpr auto EARTH_RAD = 6378.137e3;               // meters
    constexpr auto RAD = 3.14159265358979323846 / 180.0; // lol

    constexpr auto EARTH_FLAT = 1.0 / 298.257223563; // flattening of the earth. IDK what this means

    constexpr auto ECC_SQRD = EARTH_FLAT * (2.0 - EARTH_FLAT); // eccentricity squared. IDK what this means

    constexpr auto m = RAD * EARTH_RAD;
    const auto coslat = cos(position.x() * RAD);
    const auto w2 = 1.0 / (1.0 - ECC_SQRD * (1.0 - coslat * coslat)); // IDK what this means
    const auto w = sqrt(w2);                                          // IDK what this means

    ky = m * w * coslat;                // IDK what this means
    kx = m * w * w2 * (1.0 - ECC_SQRD); // IDK what this means

}

// wrap long val difference 
double GPS::wrapLongitude(double val) const {
    while (val > 180.0){
        val -= 360.0;
    }

    while (val < 180.0){
        val += 360.0;
    }

    return val;
}



// returns disp in meters from origin to current pos
Vector<3> GPS::getDisplacement(Vector<3> origin) const {
    Vector<3> disp;
    disp.x() = (position.x() - origin.x()) * ky;
    disp.y() = wrapLongitude(position.y() - origin.y()) * kx; 
    disp.z() = position.z() - origin.z();
    return disp;
}



// returns hh:mm:ss str adjusted by the offset 
const char *GPS::getTimeOfDay() const {
    static char buf[9];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hr, min, sec);
    return buf;
}


//getters

Vector<3> GPS::getPos() const {return position;}
Vector<3> GPS::getVel() const {return velocity;}
int GPS::getFixQual() const {return fixQual;}
double GPS::getHeading() const {return heading;}
bool GPS::getHasFix() const {return hasFix;}

int8_t GPS::getHour() const {return hr;}
int8_t GPS::getMinute() const {return min;}
int8_t GPS::getSecond() const {return sec;}
uint8_t GPS::getDay() const {return day;}
uint8_t GPS::getMonth()const {return month;}
uint16_t GPS::getYear() const {return year;}

