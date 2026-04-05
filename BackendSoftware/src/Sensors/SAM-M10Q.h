//created by Divyansh Srivastava on 4/5/2026
//This file will be used for the SAM-M10Q GPS module, which is used for tracking the position of the antenna system
//Right now, I am keeping it to one layer of abstraction (unlike ASTRA)
#ifndef SAM_M10Q_H
#define SAM_M10Q_H
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include "Math/Vector.h"

class GPS {
    public: 
        SFE_UBLOX_GNSS_SUPER sam_m10q; //gps object has-a relation w/ sam-m10q
        TwoWire *wire; //i2c bus 
        u_int8_t address; //i2c address of the gps module (0x42 for SAM-M10Q)


        GPS(); //default constructor
        GPS(String name, TwoWire &wirePort, u_int8_t address); //constructor with parameters
        ~GPS(); //destructor

        Vector<3> position(); //holds the position of the gps module in 3D space (x, y, z)
        


        bool begin(); //initializes the gps module, returns true if successful
        bool update(); //updates the gps data, returns true if successful



}












#endif //SAM_M10Q_H