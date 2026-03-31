#ifndef SIMPLE_DATA_REPORTER_H
#define SIMPLE_DATA_REPORTER_H
#include "DataReporter.h"

/**
* @brief A simplified DataReporter that uses function callbacks for initialization and updates.
*
* This class makes it easy to add simple single-datapoint logging without creating a full class.
* Just provide begin() and update() functions, and the reporter handles the rest.
*/

template <typename T>
class SimpleDataReporter : DataReporter
{
   // This class can work with Generic datatypes. It is a child of Data Reporter.
   public:
       // reference a pointer to a function that returns a boolean
       typedef bool (*BeginFuncCB)();
       // reference a pointer to a function that returns a generic type
       typedef T (*UpdateFuncCB)();


       // constructor & destructor


       // we define our constructor parameters, but also include an initialization list
       // these values must be initialized for our constructor call
       // initialization list requires an implementation in the header :(
       SimpleDataReporter(
           const char* name = "Simple Data Reporter",
           const char* fmt = "%f",
           const char* label = "Default Label",
           BeginFuncCB beginFunc = nullptr,
           UpdateFuncCB updateFunc = nullptr,
           T defaultValue = T{}
       ) : DataReporter(name), _beginFunc(beginFunc), _updateFunc(updateFunc), loggedVariable(defaultValue) {addColumn(fmt, &loggedVariable, label);}


       virtual ~SimpleDataReporter() = default; // use the compiler generated default


       // basic methods that get autocalled
       int begin() override;
       int update(double currentTime = -1) override;


   private:
       // our instance fields expect pointer references to method
       // these methods get passed in when the class is instantiated 
       BeginFuncCB _beginFunc;
       UpdateFuncCB _updateFunc;
       T loggedVariable; // except for this generic field
}
#endif
