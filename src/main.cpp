#include "CubeCell_NeoPixel.h"
#include <RadioLib.h>

#include "FDOS_LOG.h"
#include "VMExecutor.h"
#include "VMTime.h"
#include <Arduino.h>

#include <RadioTask.h>

Logger FDOS_LOG(&Serial);
CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);

SX1262 radio = new Module(RADIOLIB_BUILTIN_MODULE);
void beginReceive(void) { radio.startReceive(); }

VMExecutor executor;

RadioTask radioTask((PhysicalLayer *)&radio, 50000ULL, beginReceive);

void radioInterrupt(void) { radioTask.interruptTriggered(); }

void startRadioActions();

class ControlInputAction : RadioAction, RunnableTask {

    public:
    int8_t joyH,joyV;
    uint8_t slideH,slideV;

   void onReceive(uint8_t length, uint8_t *data) {
       if (data[0] == RADIO_MSG_ID::TRANSMIT_CONTROLS){
           joyH = data[1];
           joyV = data[2];
           slideH = data[3];
           slideV = data[4];
           FDOS_LOG.printf("JH:%i JV:%i SH:%i SV:%i\n",joyH,joyV,slideH,slideV);
       }
   }


    void onStart() {
       joyH = joyV = 0;
       slideH = slideV = 0;
    }

    void onStop() {
    }
};

class SustainConnectionAction : RadioAction, RunnableTask {

    uint8_t missedHBCount = 0;
    ScheduledLink *cancel = NULL;

    void onReceive(uint8_t length, uint8_t *data) {
        if (data[0] == HEARTBEAT) {
            FDOS_LOG.println("Heartbeat received from receiver");
            missedHBCount = 0;
            requestSend();
        }
    }

    // returns length of data to send
    uint8_t onSendReady(uint8_t *data) {
        float snr = radio.getSNR();
        FDOS_LOG.print("SNR:");
        FDOS_LOG.println(snr);

        data[0] = HEARTBEAT;
        data[1] = (getBatteryVoltage()-3100)/11;
        data[2] = (uint8_t)snr;

        FDOS_LOG.print("Batt:");
        FDOS_LOG.println(getBatteryVoltage());

        return 3;
    }

    void run(TIME_INT_t time) {
        if (missedHBCount > 2) {
            FDOS_LOG.println("DISCONNECTING TRANSMITTER - No heartbeat in 3 cycles");
            radioTask->removeAllActions();
            startRadioActions();
            return;
        }
        missedHBCount++;
    }

  public:
    void onStart() {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();

        missedHBCount = 0;

        if (cancel != NULL)
            cancel->cancel();
        cancel = executor.schedule((RunnableTask *)this, executor.getTimingPair(TRANSMITTER_HB_SECONDS, FrequencyUnitEnum::second));
    }

    void onStop() {
        if (cancel != NULL)
            cancel->cancel();
        cancel = NULL;
    }

} sustainAction;

class BeaconAction : RadioAction, RunnableTask {

    ScheduledLink *cancel = NULL;
    bool blink = false;

    bool isConfirming = false;
    uint8_t transmitterId = 0;

    void onReceive(uint8_t length, uint8_t *data) {
        if (data[0] == TRANSMITTER_RECOGNIZE && data[2] == RECEIVER_ID) {
            if (!requestSend())
                return;
            transmitterId = data[1];
            pixels.setPixelColor(0, pixels.Color(100, 100, 100));
            pixels.show();
            if (cancel != NULL)
                cancel->cancel();
            cancel = NULL;
            radioTask->addAction((RadioAction *)&sustainAction);
        }
    }

    // returns length of data to send
    uint8_t onSendReady(uint8_t *data) {
        if (transmitterId == 0) {
            data[0] = RADIO_MSG_ID::RECEIVER_BEACON;
            data[1] = RECEIVER_ID;
            return 2;
        } else {
            data[0] = RADIO_MSG_ID::RECEIVER_RECOGNIZE_CONFIRM;
            data[1] = transmitterId;
            return 2;
        }
    }

    void run(TIME_INT_t time) {
        pixels.setPixelColor(0, pixels.Color(0, 0, (blink = !blink) * 100));
        pixels.show();
        requestSend();
    }

  public:
    void onStart() {
        if (cancel != NULL)
            cancel->cancel();
        transmitterId = 0;
        cancel = executor.schedule((RunnableTask *)this, executor.getTimingPair(2, FrequencyUnitEnum::second));
    }

} beaconAction;

void startRadioActions() { radioTask.addAction((RadioAction *)&beaconAction); }

void setup() {
    Serial.begin(921600);

    // put your setup code here, to run once:
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW); // SET POWER
    pixels.begin();          // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.clear();          // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(10, 10, 10));
    pixels.show();


   FDOS_LOG.println("[SX1262] Initializing ...");
    SPI.begin();
    delay(300);                          // Seems to need this delay to start reliably
    int state = radio.begin(910.0, 500); //-23dBm
    if (state == RADIOLIB_ERR_NONE) {
        FDOS_LOG.println("success!");
    } else {
        FDOS_LOG.print("SX1262 failed. Code:");
        FDOS_LOG.println(state);
        for(uint8_t i = 0 ; i < 10 ; i++)  {
            pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            pixels.show();
            delay(200);
            pixels.setPixelColor(0, pixels.Color(10, 0, 0));
            pixels.show();
            delay(100);
        }
        HW_Reset(0);
    }

    if (radio.setOutputPower(17) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        FDOS_LOG.println("Invalid Power!");
        while (true) {
            pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            pixels.show();
            delay(100);
            pixels.setPixelColor(0, pixels.Color(10, 0, 0));
            pixels.show();
            delay(200);
        }
    }

    radio.setDio1Action(radioInterrupt);

    pixels.setPixelColor(0, pixels.Color(0, 10, 0));
    pixels.show();

    executor.schedule((RunnableTask *)&radioTask, executor.getTimingPair(1, FrequencyUnitEnum::second));
    startRadioActions();

    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
}

void loop() {
    uint32_t delayTime = executor.runSchedule();
    if (delayTime > 10000)
        delayTime = 10000;
    if (delayTime > MIN_MICRO_REST)
        delayMicroseconds(delayTime - MIN_MICRO_REST);
}
