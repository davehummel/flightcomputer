#ifndef __flight_radio_H__
#define __flight_radio_H__

#include "RadioTask.h"
#include "VMExecutor.h"


class ListenTransmitterAction : RadioAction, RunnableTask {
  private:
    ScheduledLink *cancel = NULL;
    uint8_t transmitterId = 0;

    uint8_t respondAttempt = 0;

  public:

    enum {LISTENING,RESPONDING, CONNECTING_CONFIRM_WAIT,CONNECTED,OTHER_RECEIVER} state;

    void onStart();

    void onStop();

    void onReceive(uint8_t length, uint8_t *data,bool responseExpected);

    uint8_t onSendReady(uint8_t *data, bool &responseExpected);

    void run(TIME_INT_t time);

};

extern ListenTransmitterAction listenTransmitterAction;

#endif