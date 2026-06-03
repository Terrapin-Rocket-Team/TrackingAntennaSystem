// #if NATIVE


// #include <Arduino.h>
// #include <SD.h>

// #include "FakeSensors/Fake-SAM-M10Q.h"
// #include "Data/DataLogging/DataLogger.h"
// #include "Data/Backend/PrintLog.h"

// File csvFile;

// // all of these are standard cpp classes. we will just format them according to our logging framework
// PrintLog serialLog(Serial, true);
// PrintLog sdLog(csvFile, false);

// ILogSink *serialOnly[] = { &serialLog };
// ILogSink *serialAndSd[] = { &serialLog, &sdLog };

// // convert to the appropriate logging framework, after testing w/ hardware
// FakeSAM_M10Q gpsReporter;

// static unsigned long startTimeMs = 0;

// void setup()
// {
//     Serial.begin(9600);
//     while (!Serial && millis() < 3000) {}

//     bool sdReady = false;

//     if (SD.begin(BUILTIN_SDCARD))
//     {
//         csvFile = SD.open("LOG.csv", FILE_WRITE);
//         sdReady = csvFile;
//     }

//     startTimeMs = millis();

//     gpsReporter.begin();
//     DataLogger::registerReporter(&gpsReporter);

//     if (sdReady)
//     {
//         DataLogger::configure(serialAndSd, 2);
//         Serial.println("Logging to Serial and LOG.csv");
//     }
//     else
//     {
//         DataLogger::configure(serialOnly, 1);
//         Serial.println("SD unavailable; logging to Serial only");
//     }
// }

// void loop()
// {
//     double elapsedSec = (millis() - startTimeMs) / 1000.0;

//     gpsReporter.update(elapsedSec);
//     DataLogger::instance().appendLine();

//     delay(100);
// }


// #endif // SDLOG_H