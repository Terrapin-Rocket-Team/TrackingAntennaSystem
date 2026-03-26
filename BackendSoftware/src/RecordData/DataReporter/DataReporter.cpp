#include "DataReporter.h"

// Static variables
int DataReporter::numReporters = 0;

// constructor
DataReporter::DataReporter(const char *name)
{
    DataReporter::numReporters += 1;

    // initialize reporter name
    if (name == nullptr)
    {
        // base case - default name
        this->name = new char[13]; 
        snprintf(this->name, 13, "Reporter #%d", numReporters); // populate the array w/ a default name
    }
    else
    {
        // formatted to fit parameter size
        this->name = new char[strlen(name) + 1];
        snprintf(this->name, strlen(name)+1, "%s", name);
    }

    //TODO: DataLogging function
}

DataReporter::~DataReporter()
{
    // Utilizing a virtual method as a destructor
    // TODO: DataLogging function
    
    auto current = first;
    while (current != nullptr) {
        auto saveVal = current->next;
        delete current; // delete object, not the reference variable
        current = saveVal;
    }

    delete[] name; // clear the parameter
}

// getters and setters -- name
const char* DataReporter::getName() const { return name; }
void DataReporter::setName(const char *n)
{
    delete[] name; // clear the object, not the reference variable

    name = new char[strlen(n) + 1];
    snprintf(name, strlen(n)+1, "%s", n); // populate array w/ pointer
}

// getters and setters for relevant data entries
int DataReporter::getNumColumns() { return numColumns; }

DataPoint* DataReporter::getDataPoints() { return first; }

DataPoint* DataReporter::getLastPoint() { return last; }

// checker methods
bool DataReporter::isInitialized() const { return initialized; }

void DataReporter::setAutoUpdate(bool update) { autoUpdate = update; }

bool DataReporter::getAutoUpdate() const { return autoUpdate; }
