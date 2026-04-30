#include <chrono>
#include <thread>

#include "TerminalPrint.h"
#include "TerminalReporter.h"
#include "RecordData/DataLogging/DataLogger.h"
#include "RecordData/DataLoggingSupporterClasses/PrintLog.h"

TerminalPrint terminal;
PrintLog terminalLog(terminal, true);
ILogSink *sinks[] = { &terminalLog };

TerminalReporter terminalReporter; //sensor is datareporter, don't need 

static std::chrono::steady_clock::time_point startTime;

void setup()
{
    startTime = std::chrono::steady_clock::now(); // find our initial time to log

    terminalReporter.begin(); // sends handshake
    DataLogger::registerReporter(&terminalReporter); // setup our datalogger child appropriately
    DataLogger::configure(sinks, 1); 
}

void loop() 
{
    auto now = std::chrono::steady_clock::now(); // get a new time
    double elapsedSec = std::chrono::duration<double>(now - startTime).count();

    terminalReporter.update(elapsedSec); // update w/ new time
    DataLogger::instance().appendLine(); // "write" it
}

// in this environment, we must use it. in hardware implementations, this will probably get more complicated
#ifdef NATIVE
int main()
{
    setup();

    while (true)
    {
        loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
#endif