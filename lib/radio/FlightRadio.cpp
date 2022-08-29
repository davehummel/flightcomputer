#include "FlightRadio.h"

ListenTransmitterAction listenTransmitterAction = ListenTransmitterAction();

void ListenTransmitterAction::onStart() {
    if (cancel != NULL)
        cancel->cancel();
    state = LISTENING;
}

void ListenTransmitterAction::onStop() {
    if (cancel != NULL)
        cancel->cancel();
    cancel = NULL;
}

void ListenTransmitterAction::onReceive(uint8_t length, uint8_t *data) {
    switch (state) {
    case LISTENING:
        if (data[0] == RADIO_MSG_ID::TRANSMITTER_BEACON) {
            transmitterId = data[1];
            state = RESPONDING;
            respondAttempt = 0;
            cancel = EXECUTOR.schedule((RunnableTask *)this, 100000, 200000); // Send confirmations repeatedly for 1 second
        }
        break;
    case CONNECTING_CONFIRM_WAIT:
        if (data[0] == RADIO_MSG_ID::TRANSMITTER_RECOGNIZE_CONFIRM) {
            if (data[2] == RECEIVER_ID && data[1] == transmitterId) {
                state = CONNECTED;
                RADIOTASK.removeAction(this);
                FDOS_LOG.println("CONNECTED!");
                return;
                // START sustain connection radio task
            } else {
                state = OTHER_RECEIVER;
                RADIOTASK.removeAction(this);
                // TODO this is an error state
            }
        }
    default:
        break;
    }
}

void ListenTransmitterAction::run(TIME_INT_t time) {
    switch (state) {
    case RESPONDING:
        respondAttempt++;
        requestSend();
        if (respondAttempt == 5) {
            state = CONNECTING_CONFIRM_WAIT;
        }
        break;
    case CONNECTING_CONFIRM_WAIT:
        respondAttempt++;
        if (respondAttempt == 10) {
            state = LISTENING; // Timed out waiting for connection confirmation
            cancel->cancel();
            cancel=NULL;
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
        return 3;

    default:
        return 0;
        break;
    }
}
