#include "SimpleDataReporter.h"


// no static variables
int SimpleDataReporter::begin() override
{
   (if _beginFunc != nullptr)
   {
       return _beginFunc() ? 0 : 1; // error code on if auto call worked or not
   }
   else
   {
       // print out an error function
       log("Data Reporter %s was not given an init function", getName())
       return -1;
   }
}

// TODO: implement auto calls to begin & update in cpp file