// // #include <Arduino.h>
// // #include <SD.h>

// // #include "Sensors/GPS.h"
// // #include "Data/DataLogging/DataLogger.h"
// // #include "Data/Backend/PrintLog.h"

// // File csvFile;

// // // all of these are standard cpp classes. we will just format them according to our logging framework
// // PrintLog serialLog(Serial, true);
// // PrintLog sdLog(csvFile, false);

// // ILogSink *serialOnly[] = { &serialLog };
// // ILogSink *serialAndSd[] = { &serialLog, &sdLog };

// // // convert to the appropriate logging framework, after testing w/ hardware
// // GPS gpsReporter;

// // static unsigned long startTimeMs = 0;

// // void setup()
// // {
// //     Serial.begin(9600);
// //     while (!Serial && millis() < 3000) {}

// //     bool sdReady = false;

// //     if (SD.begin(BUILTIN_SDCARD))
// //     {
// //         csvFile = SD.open("LOG.csv", FILE_WRITE);
// //         sdReady = csvFile;
// //     }

// //     startTimeMs = millis();

// //     gpsReporter.begin();
// //     DataLogger::registerReporter(&gpsReporter);

// //     if (sdReady)
// //     {
// //         DataLogger::configure(serialAndSd, 2);
// //         Serial.println("Logging to Serial and LOG.csv");
// //     }
// //     else
// //     {
// //         DataLogger::configure(serialOnly, 1);
// //         Serial.println("SD unavailable; logging to Serial only");
// //     }
// // }

// // void loop()
// // {
// //     double elapsedSec = (millis() - startTimeMs) / 1000.0;

// //     gpsReporter.update(elapsedSec);
// //     DataLogger::instance().appendLine();

// //     delay(100);
// // }
// /*
//     main.cpp — Ground Station Teensy 4.1

//     Pipeline:
//     1. Receive APRS telemetry bytes over Serial1 (UART from radio)
//     2. Decode and convert to local ENU state (Extract)
//     3. Propagate state forward by latency (Propagate)
//     4. Convert ENU to azimuth/elevation (CoordConvert)
//     5. Command motors to point antenna at predicted rocket position
// */

// #include <Arduino.h>
// #include <Wire.h>

// #include "TargetPrediction/Extract.h"
// #include "TargetPrediction/Propagate.h"
// //#include "TargetPrediction/CoordConvert.h"
// #include "MotorControl/MotorPins.h"
// #include "APRSTelem.h"

// // ----------------------------------------------------------------------------
// // APRS / Radio
// // ----------------------------------------------------------------------------
// static APRSTelem telemMessage;
// static Extract   extract(&telemMessage);

// // ----------------------------------------------------------------------------
// // State / Propagation
// // ----------------------------------------------------------------------------
// static Propagate    propagator;
// static CoordConvert coordConvert;

// // ----------------------------------------------------------------------------
// // Motors
// //   Elevation — gearbox ratio 10:1
// //   Azimuth   — gearbox ratio 50:1
// // ----------------------------------------------------------------------------
// static MotorPins elevationMotor(/* pins */ ELEV_PUL_PIN, ELEV_DIR_PIN, ELEV_ENA_PIN, /* gearRatio */ 10.0f);
// static MotorPins azimuthMotor  (/* pins */ AZIM_PUL_PIN, AZIM_DIR_PIN, AZIM_ENA_PIN, /* gearRatio */ 50.0f);

// // ----------------------------------------------------------------------------
// // Telemetry packet buffer
// // ----------------------------------------------------------------------------
// static constexpr size_t   PACKET_BUF_SIZE = 256;
// static uint8_t            packetBuf[PACKET_BUF_SIZE];
// static size_t             packetLen = 0;

// // ----------------------------------------------------------------------------
// // Timing
// // ----------------------------------------------------------------------------
// static unsigned long lastPacketMs  = 0;
// static unsigned long launchTimeMs  = 0;
// static bool          launched      = false;

// // Launch detection threshold — 2 feet converted to meters
// static constexpr double LAUNCH_THRESHOLD_M = 2.0 * 0.3048;

// // Estimated latency to compensate for (seconds)
// static constexpr double LATENCY_S = 1.0;

// // Motor RPM for slewing
// static constexpr float MOTOR_RPM = 60.0f;

// // ----------------------------------------------------------------------------
// // Setup
// // ----------------------------------------------------------------------------
// void setup()
// {
//     // USB serial for debug (optional)
//     Serial.begin(115200);

//     // Serial1 — APRS radio UART
//     Serial1.begin(9600);

//     // I2C bus
//     Wire.begin();

//     // Set TAS origin (lat/lng/alt of the antenna — set before flight)
//     extract.originLatDeg = 0.0;  // TODO: set to actual TAS latitude
//     extract.originLngDeg = 0.0;  // TODO: set to actual TAS longitude
//     extract.originAltFt  = 0.0;  // TODO: set to actual TAS altitude (feet)

//     // Enable motors
//     elevationMotor.motor_enable();
//     azimuthMotor.motor_enable();

//     lastPacketMs = millis();

//     Serial.println("Ground station ready. Waiting for telemetry...");
// }

// // ----------------------------------------------------------------------------
// // Loop
// // ----------------------------------------------------------------------------
// void loop()
// {
//     // --- 1. Read incoming APRS bytes from radio over Serial1 ----------------
//     while (Serial1.available())
//     {
//         uint8_t b = Serial1.read();
//         if (packetLen < PACKET_BUF_SIZE)
//         {
//             packetBuf[packetLen++] = b;
//         }
//     }

//     // --- 2. Process packet when we have enough bytes ------------------------
//     //        APRSTelem packets are fixed length — adjust size as needed
//     if (packetLen >= telemMessage.minPacketSize())  // TODO: confirm packet size
//     {
//         unsigned long now = millis();
//         double dt = (now - lastPacketMs) / 1000.0;
//         lastPacketMs = now;

//         // Decode and convert to ENU state
//         State measured = extract.ExtractTelemetry(packetBuf, packetLen, dt);
//         packetLen = 0;  // clear buffer for next packet

//         // Launch detection — more than 2 feet above TAS
//         if (!launched && measured.getPosZ() > LAUNCH_THRESHOLD_M)
//         {
//             launched     = true;
//             launchTimeMs = millis();
//             Serial.println("Launch detected!");
//         }

//         if (launched)
//         {
//             // Seed propagator with fresh measured state
//             propagator.update(measured);

//             // Propagate forward by latency to predict current position
//             propagator.propagate(LATENCY_S);

//             // Convert predicted ENU state to azimuth/elevation
//             coordConvert.convert(propagator.state);

//             Serial.print("Az: ");  Serial.print(coordConvert.azimuthDeg);
//             Serial.print(" El: "); Serial.println(coordConvert.elevationDeg);

//             // Command motors to point at predicted position
//             azimuthMotor.motor_set_angle  (coordConvert.azimuthDeg,   MOTOR_RPM);
//             elevationMotor.motor_set_angle(coordConvert.elevationDeg, MOTOR_RPM);
//         }
//     }
// }

// ---------------- Motor Testing ---------------


#include <Arduino.h>
#include <Wire.h>

#include "TargetPrediction/Extract.h"
#include "TargetPrediction/Propagate.h"
//#include "TargetPrediction/CoordConvert.h"
#include "MotorControl/MotorPins.h"

#define ELEV_PUL_PIN    8
#define ELEV_DIR_PIN    9
#define AZIM_PUL_PIN    24
#define AZIM_DIR_PIN    25

static MotorPins elevationMotor;
static MotorPins azimuthMotor;


void setup(){
    Serial.begin(115200);
    //Serial1.begin(9600);

    Wire.begin();
    //elevationMotor.motor_init(ELEV_PUL_PIN, ELEV_DIR_PIN, 10);
    azimuthMotor.motor_init(AZIM_PUL_PIN, AZIM_DIR_PIN, 1);
    Serial.println("Motors ready to test.");
}

void loop() {
    //elevationMotor.motor_set_angle(45.0, 10.0, 1600);
    //delay(1000);
    //elevationMotor.motor_set_angle(-45.0, 10.0, 1600);
    Serial.printf("\nAzimuth at theta = %f\n", azimuthMotor.get_motor_angle());
    delay(1000);
    azimuthMotor.motor_set_angle(90.0, 20.0, 1600);
    Serial.printf("\nAzimuth at theta = %f\n", azimuthMotor.get_motor_angle());
    delay(1000);
    azimuthMotor.motor_set_angle(-90.0, 20.0, 1600);
    Serial.printf("\nAzimuth at theta = %f\n", azimuthMotor.get_motor_angle());
    delay(1000);
    //azimuthMotor.motor_pulse(LOW);
}