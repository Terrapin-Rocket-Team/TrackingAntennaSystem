#include "Data/DataReporter/DataReporter.h"

// TerminalReporter class will be an implementation of DataReporter to be used in the terminal
class TerminalReporter : public DataReporter {
    public:
        TerminalReporter(); // initializes a datareporter w/ columns for each category

        // these methods are tied to the DataReporter class
        int begin() override;
        int update(double currentTime = -1) override;

        // setters - we should re-implement these to put in data from hardware
        void updateLatitude();
        void updateLongitude();
        void updateDegrees();

        // getters - we will use these in our logs. We can override to reformat how we want our logging to look
        double &getLatitude();
        double &getLongitude();
        double &getDegrees();
        
    private:
        // these variables are the actual incoming data 
        // when switching to embedded data, we just update these values
        // as a result, we make them private to override getters & setters

        // im initializing to prevent refercing issues, this will be initialized accordingly to hardware
        double latitude = 0.0;
        double longitude = 0.0;
        double degrees = 0.0;

        double timeSentSec = 0.0;

};