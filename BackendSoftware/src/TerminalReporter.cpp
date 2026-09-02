#include "./TerminalReporter.h"

TerminalReporter::TerminalReporter() : DataReporter("Terminal Reporting")
{
    // we define categories of what we log
    // we return a reference to the output of our method
    // the method will ideally be used to format variable outputs in the future, so we should reference to avoid expensive copies
    addColumn("%.3f", &timeSentSec, "Time Sent (s)");
    addColumn("%.2f", &latitude, "Latitude"); // we should just return a reference to the instance field here. if we chose to format here, use methods
    addColumn("%.2f", &longitude, "Longitude");
    addColumn("%.2f", &degrees, "Degrees");
}

int TerminalReporter::begin()
{
    initialized = true; // can be used to handle handshaking in the datareporter
    return 0;
}

int TerminalReporter::update(double currentTime)
{
    timeSentSec = currentTime; // so that it gets relogged

    // these methods will be implemented to handle real life data
    updateLatitude();
    updateLongitude();
    updateDegrees();

    return 0;
}

// handle updating these instance variables later w/ actual hardware
void TerminalReporter::updateLatitude()
{
    latitude += 0.001;
}

void TerminalReporter::updateLongitude()
{
    longitude += 0.001;
}

void TerminalReporter::updateDegrees()
{
    degrees += 0.001;
}




