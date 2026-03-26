/*
Created by Jack Yeulenski 3/24/26

PredictStep is responsible for:
1. Taking in radio telemetry bytes.
2. Decoding those bytes using the RadioMessage library.
3. Converting the decoded telemetry into a measurement.
4. Running a Kalman filter update step.
5. Predicting the target state forward by about 0.5 seconds.
*/

#ifndef PREDICTSTEP_H
#define PREDICTSTEP_H

#include <stdint.h>
#include <RadioMessage.h>
#include "PredictedTargetState.h"
#include "RadioMeasurement.h"


class PredictStep
{
public:
    /*
    Construct a PredictStep object.

    predictionHorizonSeconds:
    How far into the future we want to predict the target after each update.
    The default is 0.5 seconds to match the design goal in your class notes.
    */
    explicit PredictStep(float predictionHorizonSeconds = 0.5f);

    /*
    ingestAPRSPacket

    RADIO MESSAGE REPO: DIRECT ENTRY POINT
    This function is meant for raw radio bytes that represent a plain APRS
    telemetry packet encoded with the RadioMessage library.

    Expected decode flow:
    1. Copy the raw bytes into a RadioMessage::Message object.
    2. Decode that Message into a RadioMessage::APRSTelem object.
    3. Convert the APRSTelem fields into a RadioMeasurement.
    4. Run the filter update.

    Returns true when decoding and filter update succeed.
    Returns false if the packet is malformed or unusable.
    */
    bool ingestAPRSPacket(const uint8_t *rawPacket,
                          uint16_t packetSize,
                          uint32_t receiveTimeMs);

    /*
    ingestGroundStationPacket

    RADIO MESSAGE REPO: WRAPPED ENTRY POINT
    Use this if the incoming radio bytes are wrapped in a RadioMessage::GSData
    container first.

    Expected decode flow:
    1. Decode raw bytes into Message.
    2. Decode Message into GSData.
    3. Inspect GSData::dataType to confirm the payload is APRSTelem.
    4. Decode the GSData payload buffer into APRSTelem.
    5. Convert APRSTelem into RadioMeasurement.
    6. Run the filter update.

    This is useful when the transport layer multiplexes multiple payload types.
    */
    bool ingestGroundStationPacket(const uint8_t *rawPacket,
                                   uint16_t packetSize,
                                   uint32_t receiveTimeMs);

    /*
    predict

    Returns the best estimated target state after predicting forward by the
    configured horizon, which defaults to about 0.5 seconds.
    */
    PredictedTargetState predict() const;

    /*
    reset

    Clears any saved state so the next packet starts a fresh estimate.
    */
    void reset();

private:
    /*
    decodeAPRSTelem

    RADIO MESSAGE REPO: APRS DECODER HELPER
    This helper handles the exact RadioMessage decode pattern for APRS:

        Message msg;
        msg.fill(rawPacket, packetSize);
        APRSTelem telem(config);
        msg.decode(&telem);

    outputTelem is populated only when decoding succeeds.
    */
    bool decodeAPRSTelem(const uint8_t *rawPacket,
                         uint16_t packetSize,
                         APRSTelem &outputTelem) const;

    /*
    decodeAPRSTelemFromGSData

    RADIO MESSAGE REPO: GSData + APRSTelem DECODER HELPER
    This helper handles the two-stage decode path:

        Message outer;
        outer.fill(rawPacket, packetSize);
        GSData gs;
        outer.decode(&gs);

        Message inner;
        inner.fill(gs.buf, gs.size);
        APRSTelem telem(config);
        inner.decode(&telem);

    This keeps transport unpacking separate from the filter logic.
    */
    bool decodeAPRSTelemFromGSData(const uint8_t *rawPacket,
                                   uint16_t packetSize,
                                   APRSTelem &outputTelem) const;

    /*
    convertAPRSTelemToMeasurement

    RADIO MESSAGE REPO: DATA MAPPING HELPER
    Converts a decoded RadioMessage::APRSTelem object into the simpler internal
    RadioMeasurement format used by PredictStep.

    This is the boundary between:
    - external library types (APRSTelem)
    - internal predictor/filter types (RadioMeasurement)
    */
    RadioMeasurement convertAPRSTelemToMeasurement(const APRSTelem &telem,
                                                   uint32_t receiveTimeMs) const;

    /*
    updateKalmanFilter

    Consumes one normalized measurement and updates the internal target state.
    The actual Kalman math will live in PredictStep.cpp.
    */
    void updateKalmanFilter(const RadioMeasurement &measurement);

    /*
    projectForward

    Takes the current internal state estimate and projects it forward by
    dtSeconds. This is the final prediction step used to answer "where will
    the target be in about 0.5 seconds?"
    */
    PredictedTargetState projectForward(float dtSeconds) const;

    /*
    RADIO MESSAGE REPO: APRS CONFIG
    APRSTelem stores an APRSConfig and expects one during construction.
    We keep a copy here so every decode helper can construct APRSTelem objects
    consistently.

    Note:
    When decoding, RadioMessage will overwrite these fields with whatever was
    actually found in the packet header.
    */
    APRSConfig aprsConfig_;

    /*
    Filter / predictor bookkeeping.

    These are intentionally simple placeholders for now. You can later replace
    them with a full Kalman state vector and covariance matrix.
    */
    PredictedTargetState currentState_;
    uint32_t lastUpdateTimeMs_ = 0;
    float predictionHorizonSeconds_ = 0.5f;
    bool hasState_ = false;
};

#endif