#include <Arduino.h>
#include "MotorControl/MotorPins.h"

// Matches motormain.cpp wiring. Position both axes at physical zero before boot.
static MotorPins elevation;
static MotorPins azimuth;
enum class Phase { Idle, Outbound, Retargeted, Returning };
static Phase phase = Phase::Idle;
static uint32_t startedMs = 0;
static uint32_t lastStatusMs = 0;


#define ELEV_DIR_PIN    0
#define ELEV_PUL_PIN    1
#define AZIM_DIR_PIN    24
#define AZIM_PUL_PIN    25

void setup()
{
    Serial.begin(115200);
    elevation.motor_init(0, 1, 0.0, 10);  // DIR, STEP, output angle, gearbox
    azimuth.motor_init(24, 25, 0.0, 10);
}

void loop()
{
    elevation.update(micros());
    azimuth.update(micros());
    const uint32_t nowMs = millis();

    // No automatic motion on boot. Send 's' through the serial monitor to start.
    if (Serial.available()) {
        const char command = Serial.read();
        if (command == 's' && phase == Phase::Idle) {
            elevation.setTargetAngle(180.0);
            azimuth.setTargetAngle(180.0);
            startedMs = nowMs;
            phase = Phase::Outbound;
            Serial.println("START: both targets +10 deg");
        }
    }

    if (phase == Phase::Outbound && uint32_t(nowMs - startedMs) >= 200) {
        elevation.setTargetAngle(10.0);
        azimuth.setTargetAngle(25.0);
        phase = Phase::Retargeted;
        Serial.println("RETARGET: elevation +2 deg, azimuth +0.5 deg");
    }

    if (phase == Phase::Retargeted && !elevation.isMoving() && !azimuth.isMoving()) {
        const bool passed = elevation.getCurrentSteps() == 89 && azimuth.getCurrentSteps() == 111;
        Serial.println(passed ? "PASS: replacement targets reached" : "FAIL: unexpected step counts");
        elevation.setTargetAngle(0.0);
        azimuth.setTargetAngle(0.0);
        phase = Phase::Returning;
    }

    if (phase == Phase::Returning && !elevation.isMoving() && !azimuth.isMoving()) {
        const bool passed = elevation.getCurrentSteps() == 0 && azimuth.getCurrentSteps() == 0;
        Serial.println(passed ? "PASS: returned to zero; send s to repeat" : "FAIL: return position");
        phase = Phase::Idle;
    }

    // Small, infrequent messages; skip rather than wait for USB buffer space.
    if (uint32_t(nowMs - lastStatusMs) >= 1000 && Serial.availableForWrite() >= 80) {
        lastStatusMs = nowMs;
        Serial.printf("Send s to test | elev steps=%ld az steps=%ld active=%d\n",
                      static_cast<long>(elevation.getCurrentSteps()),
                      static_cast<long>(azimuth.getCurrentSteps()),
                      phase != Phase::Idle);
    }
}
