#include "FlightRadio.h"

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
    cancel = EXECUTOR.schedule((RunnableTask *)this, EXECUTOR.getTimingPair(RECEIVER_HB_ABANDON_MICROS / 4, FrequencyUnitEnum::micro));
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
        msgFromBytes(&transmitterState, data, transmitter_heartbeat_t::size);
        FDOS_LOG.printf("HB heard! size = %i.  enabled = %i\n",length,transmitterState.flightModeEnabled);
        nav->connectESC(transmitterState.flightModeEnabled);
        nav->setControlMode(transmitterState.isDirectYaw(), transmitterState.isDirectPitch(), transmitterState.isDirectRoll());
  
        return;
    }

    if (data[0] == RADIO_MSG_ID::FLIGHT_INPUT) {
        lastReceivedTime = microsSinceEpoch();
        msgFromBytes(&inputState, data, flight_input_t::size);
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

    uint16_t raw = analogRead(BATTERY_SENSE_PIN);
    receiverState.batV = raw;
    raw = analogRead(CURRENT_SENSE_PIN);
    receiverState.cur = raw;

    FDOS_LOG.printf("Voltage:%i current:%i\n", receiverState.batV , receiverState.cur );

    float snr = RADIO.getSNR();
    receiverState.snr = snr * 10;

    receiverState.headings[0] = convertHeading(nav->currentOrientation.yaw);
    receiverState.headings[1] = convertHeading(nav->currentOrientation.pitch);
    receiverState.headings[2] = convertHeading(nav->currentOrientation.roll);
    receiverState.pressure = convertPressure(nav->currentPressure);

    receiverState.targetHeadings[0] = nav->targetOrientation.yaw;
    receiverState.targetHeadings[1] = nav->targetOrientation.pitch;
    receiverState.targetHeadings[2] = nav->targetOrientation.roll;

    receiverState.speeds[0] = nav->getESC()->getSpeed(0) / 4;
    receiverState.speeds[1] = nav->getESC()->getSpeed(1) / 4;
    receiverState.speeds[2] = nav->getESC()->getSpeed(2) / 4;
    receiverState.speeds[3] = nav->getESC()->getSpeed(3) / 4;

    FDOS_LOG.println("HB response:");
    receiverState.print((Print *)&FDOS_LOG);

    msgToBytes(&receiverState, data, receiver_heartbeat_t::size);

    responseExpected = true;

    return receiver_heartbeat_t::size;
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