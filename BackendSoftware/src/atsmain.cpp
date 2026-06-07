//made by Divyansh Srivastava on 6/6/2026
//this will be the main file for the antenna tracking system
//responsible for: creatign the backend data base, initaitng the state,
//starting the motors, adn continuosly tracking the rocket and updating the motors to point at the rocket
#if NATIVE

/*
    main.cpp — Ground Station Teensy 4.1

    Pipeline:
    1. Initialize GPS, SD logging, motors
    2. Receive APRS telemetry bytes over Serial1 (UART from radio)
    3. Decode and convert to local ENU state (Extract)
    4. Propagate state forward by latency (Propagate)
    5. Convert ENU to azimuth/elevation (CoordConvert)
    6. Command motors to point antenna at predicted rocket position
*/

#include <Arduino.h>
#include <Wire.h>

// Data logging
#include "Data/DataLogging/DataLogger.h"
#include "Data/Backend/PrintLog.h"
#include "Data/Storage/SDCardStorage.h"

// Sensors
#include "Sensors/GPS.h"

// Target prediction
#include "TargetPrediction/Extract.h"
#include "TargetPrediction/Propagate.h"
#include "TargetPrediction/State.h"

// Coordinate conversion
#include "CoordinateConversion/CoordConvert.h"

// Motor control
#include "MotorControl/MotorPins.h"

// APRS
#include "APRSTelem.h"

// ----------------------------------------------------------------------------
// Motor pin definitions — set to your actual Teensy pin numbers
// ----------------------------------------------------------------------------
static constexpr int ELEV_PUL_PIN = 2;   // Elevation motor pulse pin
static constexpr int ELEV_DIR_PIN = 3;   // Elevation motor direction pin
static constexpr int AZIM_PUL_PIN  = 4;  // Azimuth motor pulse pin
static constexpr int AZIM_DIR_PIN  = 5;  // Azimuth motor direction pin

// Gear ratios
static constexpr int ELEV_GEAR_RATIO = 10;  // 10:1
static constexpr int AZIM_GEAR_RATIO = 50;  // 50:1

// Motor slew speed
static constexpr float MOTOR_RPM = 60.0f;

// ----------------------------------------------------------------------------
// Storage + Logging
// ----------------------------------------------------------------------------
static SDCardStorage sdStorage;

static ILogSink *serialOnly[1];
static ILogSink *serialAndSd[2];

// ----------------------------------------------------------------------------
// GPS (ground station antenna location)
// ----------------------------------------------------------------------------
static GPS groundGPS("GroundGPS", Wire, (uint8_t)0x42);

// ----------------------------------------------------------------------------
// APRS / Extract
// ----------------------------------------------------------------------------
static APRSTelem  telemMessage;
static Extract    extract(&telemMessage);

// ----------------------------------------------------------------------------
// Propagation + Coordinate Conversion
// ----------------------------------------------------------------------------
static Propagate    propagator;
static CoordConvert coordConvert;

// ----------------------------------------------------------------------------
// Motors
// ----------------------------------------------------------------------------
static MotorPins elevationMotor(ELEV_PUL_PIN, ELEV_DIR_PIN, ELEV_GEAR_RATIO);
static MotorPins azimuthMotor  (AZIM_PUL_PIN, AZIM_DIR_PIN, AZIM_GEAR_RATIO);

// ----------------------------------------------------------------------------
// Telemetry packet buffer
// ----------------------------------------------------------------------------
static constexpr size_t PACKET_BUF_SIZE = 256;
static uint8_t          packetBuf[PACKET_BUF_SIZE];
static size_t           packetLen = 0;

// ----------------------------------------------------------------------------
// Timing / launch state
// ----------------------------------------------------------------------------
static unsigned long lastPacketMs = 0;
static bool          launched     = false;

static constexpr double LAUNCH_THRESHOLD_M = 2.0 * 0.3048;  // 2 feet in meters
static constexpr double LATENCY_S          = 1.0;            // worst case latency (seconds)

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
    // Serial1 — APRS radio UART
    Serial1.begin(9600);



    // --- GPS init ---
    groundGPS.begin();
    DataLogger::registerReporter(&groundGPS);

    // --- SD storage init ---
    bool sdReady = sdStorage.begin();

    // --- Data logging ---
    // PrintLog needs an ILogSink — using SDCardStorage as the sink backend
    // Wire up serial-only or serial+SD depending on SD availability
    if (sdReady)
    {
        DataLogger::configure(serialAndSd, 2);
    }
    else
    {
        DataLogger::configure(serialOnly, 1);
    }

    DataLogger::instance().init();

    // --- Set ATS origin from GPS once it gets a fix ---
    // Block until GPS has a fix (timeout 10s)
    unsigned long gpsTimeout = millis();
    while (millis() - gpsTimeout < 10000)
    {
        groundGPS.update();
        if (groundGPS.getHasFix())
        {
            Vector<3> pos = groundGPS.getPos();
            extract.originLatDeg = pos(0);  // latitude
            extract.originLngDeg = pos(1);  // longitude
            extract.originAltFt  = pos(2) * 3.28084;  // meters to feet
            break;
        }
        delay(100);
    }

    // --- Motors init ---
    elevationMotor.motor_init(ELEV_PUL_PIN, ELEV_GEAR_RATIO);
    azimuthMotor.motor_init  (AZIM_PUL_PIN, AZIM_GEAR_RATIO);

    lastPacketMs = millis();
}

// ----------------------------------------------------------------------------
// Loop
// ----------------------------------------------------------------------------
void loop()
{
    double elapsedSec = millis() / 1000.0;

    // --- Update GPS and log ---
    groundGPS.update(elapsedSec);
    DataLogger::instance().appendLine();

    // --- Read incoming APRS bytes from radio over Serial1 ---
    while (Serial1.available())
    {
        uint8_t b = (uint8_t)Serial1.read();
        if (packetLen < PACKET_BUF_SIZE)
        {
            packetBuf[packetLen++] = b;
        }
    }

    // --- Process packet when buffer has data ---
    // APRSTelem::decode() will consume what it needs and return bytes read
    if (packetLen > 0)
    {
        unsigned long now = millis();
        double dt = (now - lastPacketMs) / 1000.0;
        lastPacketMs = now;

        // Back-calculate timeSinceLaunch from altitude on first packet
        double altM = 0.0;
        double timeSinceLaunch = 0.0;

        // Decode and convert to ENU state
        State measured = extract.ExtractTelemetry(packetBuf, packetLen, dt, timeSinceLaunch);
        packetLen = 0;

        // Launch detection — more than 2 feet above ATS
        if (!launched && measured.getPosZ() > LAUNCH_THRESHOLD_M)
        {
            launched = true;

            // Back-calculate timeSinceLaunch from altitude: h = 0.5*a*t^2 -> t = sqrt(2h/a)
            altM = measured.getPosZ();
            timeSinceLaunch = sqrt(2.0 * altM / 98.1);
        }

        if (launched)
        {
            // Seed propagator with fresh measured state
            propagator.update(measured);

            // Propagate forward by latency
            propagator.propagate(LATENCY_S);

            // Convert predicted ENU to azimuth/elevation
            coordConvert.convert(propagator.state);

            // Command motors
            azimuthMotor.motor_set_angle  ((double)coordConvert.azimuthDeg,   MOTOR_RPM);
            elevationMotor.motor_set_angle((double)coordConvert.elevationDeg, MOTOR_RPM);
        }
    }
}


#endif