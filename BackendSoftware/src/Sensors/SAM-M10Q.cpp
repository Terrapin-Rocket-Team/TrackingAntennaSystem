//Created by Divyansh Srivastava on 4/5/2026
//Implementation file for the SAM-M10Q GPS module, which is used for tracking the position of the antenna system
#include "SAM-M10Q.h"

GPS::GPS() {
    //default constructor  
}

GPS::GPS(String name, TwoWire &wirePort, u_int8_t address) : DataReporter(name) {
    this->wire = &wirePort;
    this->address = address; //default i2c address for SAM-M10Q is 0x42
    //because SAM is a data reporter, we want to add columns for all the data it can report,
    //ex:  addColumn("%0.7f", &position.x(), "Lat (deg)");
    //check astra for examples of how to use addColumn
    //add time stuff (hr, min, sec, day, month, year) as well for easier debugging and health monitoring

}

GPS::~GPS(){
    // no memory to clean
}


int GPS::begin(){
    if (!sam_m10q.begin(*wire, address)){ // try to initialize SAM over i2c
        initialized = false;
        return -1;
    }

    sam_m10q.setI2COutput(COM_TYPE_UBX);
    sam_m10q.setNavigationFrequency(10);
    sam_m10q.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    sam_m10q.setAutoPVT(true);
    sam_m10q.saveConfiguration();
    initialized = true;
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
    heading = sam_m10q.getHeading();
    fixQual = sam_m10q.getSIV();
    hr = sam_m10q.getHour();
    min = sam_m10q.getMinute();
    sec = sam_m10q.getSecond();
    day = sam_m10q.getDay();
    month = sam_m10q.getMonth();
    year = sam_m10q.getYear();



    hasFix = (fixQual >= 4); // update fix status
    if (hasFix && !hasFirstFix){
        hasFirstFix = true;
        calcInitialValuesForDistance(); // for kx/ky
    }

    updateHealth(0, currentTime);
    return 0;
}


// distance helpers

//meters per degree scalers at current altitude
void GPS::calcInitialValuesForDistance(){ 
    double latRad = position.x() * M_PI / 180.0; //this calculation is too simplistic, too many assumptions 
    //the Earth's radius must be taken into accoutn because rememebr the gps is on the ground, not at the center of the Earth, 
    //so the curvature of the Earth matters for how much distance corresponds to a degree of lat/lon
    //check the article astra has on this to understand the math better, but this is a common approximation for small distances

    ky = 111320.0;
    kx = 111320.0 * cos(latRad);
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
    disp.x() = (position.x() - origin.x()) * ky; //multiple by ky or kx here
    disp.y() = wrapLongitude(position.y() - origin.y()) * kx; //same issue here 
    disp.z() = position.z() - origin.z();
    return disp;
}


//timezone helper

//utc hour offset from current longitude (approx)
void GPS::findTimeZone(){ //don't even really need this, might as well just delete this
    hrOffset = static_cast<int8_t>(round(position.y() / 15.0))
}


//health
void GPS::updateHealth(int readErr, double currentTime){ //delete this, use the boolean health member variable instead and just set it to false
    // if readErr != 0 or if the gps doesn't have a fix, and true otherwise. Then use this in the main loop to decide whether or not to update the gps data

    DataReporter::updateHealth(readErr, currentTime); //there is no update health in data reporter, i did that on purpose
}



// returns hh:mm:ss str adjusted by the offset 
const char *GPS::getTimeOfDay() const {
    static char buf[9];
    int8_t localHr = hr + hrOffset;
    localHr = static_cast<int8_t>(((localHr % 24) + 24) % 24);
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", localHr, min, sec);
    return buf;
}

//where is the update and begin function?? 


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

