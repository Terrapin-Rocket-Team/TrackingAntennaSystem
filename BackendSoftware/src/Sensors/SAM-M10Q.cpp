//Created by Divyansh Srivastava on 4/5/2026
//Implementation file for the SAM-M10Q GPS module, which is used for tracking the position of the antenna system
#include "SAM-M10Q.h"

GPS::GPS() {
    //default constructor  
}

GPS::GPS(String name, TwoWire &wirePort, u_int8_t address) {
    DataReporter(name);
    this->wire = &wirePort;
    this->address = address = 0x42; //default i2c address for SAM-M10Q

}






