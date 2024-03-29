#include "FlightRadio.h"
#include "PID.h"
#include "PowerControl.h"

ListenTransmitterAction listenTransmitterAction = ListenTransmitterAction();
SustainConnectionAction sustainConnectionAction = SustainConnectionAction();

void ListenTransmitterAction::onStart() {
    if (cancel != NULL)
        cancel->cancel();
    state = LISTENING;
    FDOS_LOG.println("ListenTransmitterAction started");
}

void ListenTransmitterAction::onStop() {
    if (cancel != NULL)
        cancel->cancel();
    cancel = NULL;
}

void ListenTransmitterAction::onReceive(uint8_t length, uint8_t *data, bool responseExpected) {
    switch (state) {
    case LISTENING:
        if (data[0] == RADIO_MSG_ID::TRANSMITTER_BEACON) {
            transmitterId = data[1];
            state = RESPONDING;
            respondAttempt = 0;
            FDOS_LOG.println("Beacon heard!");
            digitalWrite(LED_PIN, true);
            cancel = EXECUTOR.schedule((RunnableTask *)this, 100000, 200000); // Send confirmations repeatedly for 1 second
        }
        break;
    case CONNECTING_CONFIRM_WAIT:
        if (data[0] == RADIO_MSG_ID::TRANSMITTER_RECOGNIZE_CONFIRM) {
            if (data[2] == RECEIVER_ID && data[1] == transmitterId) {
                state = CONNECTED;
                RADIOTASK.removeAction(this);
                digitalWrite(LED_PIN, true);
                FDOS_LOG.println("CONNECTED!");
                RADIOTASK.addAction((RadioAction *)&sustainConnectionAction);
                return;
                // START sustain connection radio task
            } else {
                state = OTHER_RECEIVER;
                RADIOTASK.removeAction(this);
                digitalWrite(LED_PIN, false);
                FDOS_LOG.println("Competing recevier detected, shutting down!");
            }
        }
        break;
    default:
        break;
    }
}

void ListenTransmitterAction::run(TIME_INT_t time) {
    FDOS_LOG.print("!");
    switch (state) {
    case RESPONDING:
        digitalWrite(LED_PIN, respondAttempt % 2);
        respondAttempt++;
        if (respondAttempt == 5) {
            state = CONNECTING_CONFIRM_WAIT;
        } else {
            requestSend();
        }
        FDOS_LOG.println("responding...");
        break;
    case CONNECTING_CONFIRM_WAIT:
        respondAttempt++;
        if (respondAttempt == 10) {
            state = LISTENING; // Timed out waiting for connection confirmation
            FDOS_LOG.println("Timeout waiting for response from transmitter");
            digitalWrite(LED_PIN, false);
            cancel->cancel();
            cancel = NULL;
        }
        break;

    default:
        break;
    }
}

uint8_t ListenTransmitterAction::onSendReady(uint8_t *data, bool &responseExpected) {
    switch (state) {
    case RESPONDING:
        data[0] = RADIO_MSG_ID::RECEIVER_RECOGNIZE;
        data[1] = RECEIVER_ID;
        data[2] = transmitterId;
        responseExpected = true;
        FDOS_LOG.println("Sending response");
        return 3;

    default:
        return 0;
        break;
    }
}

void SustainConnectionAction::onStart() {
    if (cancel != NULL)
        cancel->cancel();
    cancel = EXECUTOR.schedule((RunnableTask *)this, EXECUTOR.getTimingPair(1, FrequencyUnitEnum::second));
    lastReceivedTime = microsSinceEpoch();
    FDOS_LOG.println("Sustain connection starting...");
}

void SustainConnectionAction::onStop() {
    if (cancel != NULL)
        cancel->cancel();
    cancel = NULL;
    nav->connectESC(false);
    FDOS_LOG.println("Sustain Connection stopped.");
}

bool hbLedFlip = false;
void SustainConnectionAction::onReceive(uint8_t length, uint8_t *data, bool responseExpected) {
    if (data[0] == RADIO_MSG_ID::TRANSMITTER_DISCONNECT) {
        FDOS_LOG.println("Disconnecting due to disconnect message");
        disconnect();
        return;
    }

    if (responseExpected) {
        requestSend();
    }

    if (data[0] == RADIO_MSG_ID::TRANSMITTER_HEARTBEAT) {
        lastReceivedTime = microsSinceEpoch();
        msgFromBytes(&transmitterState, data, sizeof(transmitter_heartbeat_t));
        FDOS_LOG.println("HB heard!");
        transmitterState.print(&FDOS_LOG);
        nav->connectESC(transmitterState.flightModeEnabled);
        nav->setControlMode(transmitterState.isDirectYaw(), transmitterState.isDirectPitch(), transmitterState.isDirectRoll());
        nav->setTelemCapture(transmitterState.isTelemEnabled());
        return;
    }

    if (data[0] == RADIO_MSG_ID::FLIGHT_CONFIG) {

        lastReceivedTime = microsSinceEpoch();
        flight_config_t config;
        msgFromBytes(&config, data, sizeof(flight_config_t));

        FDOS_LOG.println("Config Received!");
        config.print(&FDOS_LOG);

        nav->updatePIDConfiguration(config.yaw_KP, config.yaw_KI, config.yaw_KD, config.yaw_MAX_I, config.roll_KP, config.roll_KI, config.roll_KD,
                                    config.roll_MAX_I, config.pitch_KP, config.pitch_KI, config.pitch_KD, config.pitch_MAX_I);
        return;
    }

    if (data[0] == RADIO_MSG_ID::PID_TELEMETRY_REQUEST) {

        lastReceivedTime = microsSinceEpoch();
        pid_request_telemetry_t config;
        msgFromBytes(&config, data, sizeof(pid_request_telemetry_t));

        FDOS_LOG.println("Telemetry request received!");

        telemRequestIndex = config.index;
        telemRequestYPR = config.telemDimensionYPR;

        return;
    }

    if (data[0] == RADIO_MSG_ID::DIRECT_ESC) {
        lastReceivedTime = microsSinceEpoch();
        direct_esc_t escCommand;

        msgFromBytes(&escCommand, data, sizeof(direct_esc_t));
        FDOS_LOG.println("Direct ESC Received!");
        escCommand.print(&FDOS_LOG);

        uint16_t speeds[4];
        for (uint8_t i = 0; i < 4; i++) {
            speeds[i] = escCommand.escVals[i] * (1000.0 / 255.0);
        }

        nav->getESC()->runESCDirectCommand(escCommand.commandDurationSeconds, speeds);

        return;
    }

    if (data[0] == RADIO_MSG_ID::FLIGHT_INPUT) {
        lastReceivedTime = microsSinceEpoch();
        msgFromBytes(&inputState, data, sizeof(flight_input_t));
        nav->recordInput(inputState.joyH, inputState.joyV, inputState.slideH, inputState.throttleInput);
#ifdef USB_SERIAL_HID
        inputState.print(&FDOS_LOG);
        Joystick.slider((inputState.throttleInput * 1023) / 254);
        Joystick.Zrotate(((inputState.slideH + 127) * 1023) / 254);
        Joystick.X(((inputState.joyH + 127) * 1023) / 254);
        Joystick.Y(((inputState.joyV + 127) * 1023) / 254);
        Joystick.send_now();
#endif
    }
}

uint8_t SustainConnectionAction::onSendReady(uint8_t *data, bool &responseExpected) {
    digitalWrite(LED_PIN, hbLedFlip = !hbLedFlip);

    if (inErrorState[0] != '\0') {
        data[0] = CRITICAL_MESSAGE;
        strcpy((char *)(data + 1), inErrorState);
        inErrorState[0] = '\0';
        disconnect();
        return strlen((char *)(data + 1)) + 1;
    }

    if (telemRequestIndex != -1) {

        pid_response_telemetry_t response;

        response.totalTelemCount = nav->getTelemCount();
        response.sampleDurationMicros = nav->getTelemSampleDurationMicros();

        if (telemRequestIndex >= response.totalTelemCount)
            response.sampleStartIndex = response.totalTelemCount;
        else
            response.sampleStartIndex = telemRequestIndex;

        uint8_t responseSize;

        if (response.totalTelemCount - response.sampleStartIndex < PID_SAMPLE_SIZE)
            responseSize = response.totalTelemCount - response.sampleStartIndex;
        else
            responseSize = PID_SAMPLE_SIZE;

        for (uint8_t i = 0; i < responseSize; i++) {
            nav->getTelemSample(telemRequestIndex + i, telemRequestYPR, response.samples[i]);
        }

        msgToBytes(&response, data, response.getSizeInBytes());

        responseExpected = true;
        telemRequestIndex = -1;

        return response.getSizeInBytes();
    }

    receiverState.batV = POWER.getBatteryVoltage();
    receiverState.cur = POWER.getBatteryCurrent();

    float snr = RADIO.getSNR();
    receiverState.snr = snr * 10;

    receiverState.headings[0] = nav->currentOrientation.yaw;
    receiverState.headings[1] = nav->currentOrientation.pitch;
    receiverState.headings[2] = nav->currentOrientation.roll;
    receiverState.pressure = nav->currentPressure;

    receiverState.targetHeadings[0] = nav->targetOrientation.yaw;
    receiverState.targetHeadings[1] = nav->targetOrientation.pitch;
    receiverState.targetHeadings[2] = nav->targetOrientation.roll;

    receiverState.speeds[0] = nav->getESC()->getSpeed(0) / 4;
    receiverState.speeds[1] = nav->getESC()->getSpeed(1) / 4;
    receiverState.speeds[2] = nav->getESC()->getSpeed(2) / 4;
    receiverState.speeds[3] = nav->getESC()->getSpeed(3) / 4;

    FDOS_LOG.println("HB response:");
    receiverState.print((Print *)&FDOS_LOG);

    msgToBytes(&receiverState, data, sizeof(receiver_heartbeat_t));

    responseExpected = true;

    return sizeof(receiver_heartbeat_t);
}

void SustainConnectionAction::run(TIME_INT_t time) {
    if (time - lastReceivedTime > RECEIVER_HB_ABANDON_MICROS) {
        FDOS_LOG.println("Disconnecting due to lack of HB");
        disconnect();
        return;
    }
}

void SustainConnectionAction::disconnect() {
    FDOS_LOG.println("SustainConnectionAction disconnect called.");
    RADIOTASK.removeAction(this);
    RADIOTASK.addAction((RadioAction *)&listenTransmitterAction);
    nav->connectESC(false);
}