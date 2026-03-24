//Created by Divyansh Srivastava on 3/23/2026
//this file holds the lowest level of data; a singular data point
//each data point will have a timestamp and a value 

#ifndef DATAPOINT_H
#define DATAPOINT_H
#include <stdint.h>

struct DataPoint { //using a struct since members are public by default 

    //copied from ASTRA 
    const char *fmt = nullptr;                          // printf format for this value
    const char *label = nullptr;                        // column label
    DataPoint *next = nullptr;                          // next in list
    const void *data = nullptr;                         // pointer to the value
};




#endif

