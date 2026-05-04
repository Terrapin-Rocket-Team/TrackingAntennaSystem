#include <Arduino.h>

#include "FakeSensors/Fake-SAM-M10Q.h"
#include "RecordData/DataLogging/DataLogger.h"
#include "RecordData/DataLoggingSupporterClasses/PrintLog.h"

PrintLog serialLog(Serial, true);
ILogSink *sinks[] = { &serialLog };

FakeSAM_M10Q gpsReporter;

static unsigned long startTimeMs = 0;

void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000) {}

    startTimeMs = millis();

    gpsReporter.begin();
    DataLogger::registerReporter(&gpsReporter);
    DataLogger::configure(sinks, 1);
}

void loop()
{
    double elapsedSec = (millis() - startTimeMs) / 1000.0;

    gpsReporter.update(elapsedSec);
    DataLogger::instance().appendLine();

    delay(100);
}
