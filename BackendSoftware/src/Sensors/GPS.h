//created by Divyansh Srivastava on 4/5/2026
//This file will be used for the SAM-M10Q GPS module, which is used for tracking the position of the antenna system
//Right now, I am keeping it to one layer of abstraction (unlike ASTRA)
#ifndef SAM_M10Q_H
#define SAM_M10Q_H
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include "Math/Vector.h"
#include "RecordData/DataReporter/DataReporter.h"

class GPS : public DataReporter { //a gps is an object that can report data, so it inherits from DataReporter
    public: 
        GPS(); //default constructor
        GPS(const char *name, TwoWire &wirePort, u_int8_t address); //constructor with parameters
        GPS(const char *name, TwoWire &wirePort, double hz); //constructor with parameters, default address, diff update address 
        //I WILL DO THIS ONE, do not implement this one yet, just add it to the header file. 
        virtual ~GPS() = default;



        int begin() override; //initializes the gps module, returns true if successful
        int update(double currentTime = -1) override; //updates the gps data, returns true if successful
        
        // Distance-related calculations
        void calcInitialValuesForDistance();
        double kx, ky;
        double wrapLongitude(double val) const;
        void findTimeZone();


        void updateHealth(int readErr, double currentTime);

        Vector<3> getPos() const;
        Vector<3> getVel() const; // NED frame
        int getFixQual() const;
        double getHeading() const;
        bool getHasFix() const;
        
        Vector<3> getDisplacement(Vector<3> origin) const;

        const char *getTimeOfDay() const;

        int8_t getHour() const;
        int8_t getMinute() const;
        int8_t getSecond() const;
        uint8_t getDay() const;
        uint8_t getMonth() const;
        uint16_t getYear() const;

        private:
        
        bool isHealthy = false; // whether or not the gps is healthy (if init and update/read functions work)

        Vector<3> position; // latitude, longitude, alt(m)
        Vector<3> velocity; // vN (m/s), vE (m/s), vD (m/s)

        int fixQual = 0;    // number of satellite connections
        bool hasFix = false;        // whether or not GPS is currently connected to >= 4 satellites
        bool hasFirstFix = false;   // the first time it gets a fix
        bool initialized = false;
        double heading = 0;
        double hz; //data rate frequency 

        int8_t hr = 0, min = 0, sec = 0;
        uint8_t day = 0, month = 0;
        uint16_t year = 0;
        int8_t hrOffset = 0;
        char tod[9];

        SFE_UBLOX_GNSS_SUPER sam_m10q; //gps object has-a relation w/ sam-m10q
        TwoWire *wire; //i2c bus 
        u_int8_t address; //i2c address of the gps module (0x42 for SAM-M10Q)




};


#endif //SAM_M10Q_H