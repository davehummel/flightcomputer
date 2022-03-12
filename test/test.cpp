#include <Arduino.h>
#include <SPI.h>

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib

#include "FDOS_LOG.h"
#include "VMExecutor.h"
#include "VMTime.h"

#include "RadioTask.h"

SX1276 radio(new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN));

Logger FDOS_LOG(&Serial);

VMExecutor executor;

RadioTask radioTask(&radio);

#define PING_PONG_INITIATOR true

class PinpPongAction : RadioAction, RunnableTask {

    ScheduledLink *cancel = NULL;

    uint8_t count[4] = {0};

    uint32_t actionSeen = 0;

    void onStart() {
        if (cancel != NULL)
            cancel->cancel();

        if (PING_PONG_INITIATOR)
            cancel = executor.schedule((RunnableTask *)this, executor.getTimingPair(1, FrequencyUnitEnum::per_second));
    }

    void onStop() {
        if (cancel != NULL)
            cancel->cancel();
        cancel = NULL;
    }

    void onReceive(uint8_t length, uint8_t *data) {
        if (length < 4) {
            FDOS_LOG.println("INCOMPLETE data");
            while (true) {
                delay(1000);
            }
        }
        if (data[0] != (uint8_t)(count[0] + 1)) {
            FDOS_LOG.println("BAD data");
            if (data[0] != count[0] && data[0] != (uint8_t)count[0] - 1) {
                FDOS_LOG.println("Not even late!");
                while (true) {
                    delay(1000);
                }
            }
        }
        count[0] = data[0];
        count[1] = data[1];
        count[2] = data[2];
        count[3] = data[3];
        count[0]++;
        if (count[0] == 0) {
            count[1]++;
            if (count[1] == 0) {
                count[2]++;
                if (count[2] == 0) {
                    count[3]++;
                }
            }
        }
        actionSeen++;
        requestSend();
    }

    // returns length of data to send
    uint8_t onSendReady(uint8_t *data) {
        data[0] = count[0];
        data[1] = count[1];
        data[2] = count[2];
        data[3] = count[3];
        return 4;
    }

    bool blink = false;
    void run(TIME_INT_t time) {
        if (actionSeen == 0) {
            FDOS_LOG.print("Forcing send due to no action.  transmitting=");
            FDOS_LOG.println(requestSend());
        } else {
            FDOS_LOG.print(actionSeen);
            FDOS_LOG.println(" actions per second");
        }
        actionSeen = false;
        digitalWrite(LED_PIN, blink = !blink);
    }
} findReceiver;

void radioInterrupt(void) { radioTask.interruptTriggered(); }

void startRadioActions() { radioTask.addAction((RadioAction *)&findReceiver); }

void setup(void) {
    // PowerEnable pin must be enabled ASAP to keep system on after power is
    // released
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);

    Serial.begin(921600);

    delay(5000);

    FDOS_LOG.println("Radio Starting ...");

    int state = radio.begin(RADIO_CARRIER_FREQ, RADIO_LINK_BANDWIDTH, RADIO_SPREADING_FACTOR); //-23dBm
    if (state == RADIOLIB_ERR_NONE) {
        FDOS_LOG.println("success!");
    } else {
        FDOS_LOG.print("SX1276 failed. Code:");
        FDOS_LOG.println(state);
    }
    // set output power to 10 dBm (accepted range is -3 - 17 dBm)
    // NOTE: 20 dBm value allows high power operation, but transmission
    //       duty cycle MUST NOT exceed 1%
    if (radio.setOutputPower(RADIO_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        FDOS_LOG.println("Invalid Power!");
    }

    radio.setDio0Action(radioInterrupt);

    executor.schedule((RunnableTask *)&radioTask, executor.getTimingPair(RADIO_INTERVAL_MILLIS, FrequencyUnitEnum::milli));

    startRadioActions();
}

void loop(void) {
    uint32_t delayTime = executor.runSchedule();
    // You may want to allow the loop to finish and avoid long delays
    //    if you are using background arduino features
    // if (delayTime > 10000)
    //     delayTime = 10000;

    if (delayTime > MIN_MICRO_REST)
        delayMicroseconds(delayTime - MIN_MICRO_REST);
}
