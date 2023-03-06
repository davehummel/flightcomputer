#ifndef __flight_radio_H__
#define __flight_radio_H__

#include "RadioTask.h"
#include "VMExecutor.h"

#include "TargetOrientationNav.h"

class ListenTransmitterAction : RadioAction, RunnableTask {
  private:
    ScheduledLink *cancel = NULL;
    uint8_t transmitterId = 0;

    uint8_t respondAttempt = 0;

  public:
    enum { LISTENING, RESPONDING, CONNECTING_CONFIRM_WAIT, CONNECTED, OTHER_RECEIVER } state;

    void onStart();

    void onStop();

    void onReceive(uint8_t length, uint8_t *data, bool responseExpected);

    uint8_t onSendReady(uint8_t *data, bool &responseExpected);

    void run(TIME_INT_t time);
};

class SustainConnectionAction : RadioAction, RunnableTask {
  private:

    ScheduledLink *cancel = NULL;

    receiver_heartbeat_t receiverState;

    transmitter_heartbeat_t transmitterState;

    flight_input_t inputState;

    TIME_INT_t lastReceivedTime = 0;
 
    TargetOrientationNav *nav; 

    char inErrorState[254]  = "\0";


  public:

    void setNavSource(TargetOrientationNav *navSource){ nav = navSource; }

    void onStart();

    void onStop();

    void onReceive(uint8_t length, uint8_t *data, bool responseExpected);

    uint8_t onSendReady(uint8_t *data, bool &responseExpected);

    void run(TIME_INT_t time);

    bool navIsYawDirect() { return transmitterState.isDirectYaw(); }

    bool navIsRollDirect() { return transmitterState.isDirectRoll(); }

    bool navIsPitchDirect() { return transmitterState.isDirectPitch(); }

    void disconnect();

};

extern ListenTransmitterAction listenTransmitterAction;

extern SustainConnectionAction sustainConnectionAction;


#endif