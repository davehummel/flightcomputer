#include <Arduino.h>
#include <RadioLib.h>

#include "FDOS_LOG.h"
#include "I2Cdev.h"
#include "IMU.h"
#include "Sensor_cal.h"
#include "USFSMAX.h"
#include "VMExecutor.h"
#include "VMTime.h"

#include "FlightMessages.h"
#include <MotionTask.h>
#include <RadioTask.h>

Logger FDOS_LOG(&Serial);

SX1276 radio(new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN));

void beginReceive(void) { radio.startReceive(); }

VMExecutor executor;

MotionTask motionSensor;

class ESC : RunnableTask {
  private:
    uint8_t initStep;

    uint8_t motorPins[MOTOR_COUNT] = MOTOR_PINS;

    float speeds[MOTOR_COUNT] = {0};

    void setAllSpeeds(float speed) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            setSpeed(i, speed);
        }
    }

    void run(TIME_INT_t time) {
        FDOS_LOG.print('.');
        initStep++;
        switch (initStep) {
        case 1:
            setAllSpeeds(0);
            return;
        case 2:
            setAllSpeeds(.5);
            return;
        case 3:
            setAllSpeeds(1);
            return;
        case 4:
            setAllSpeeds(.25);
            return;
        default:
            setAllSpeeds(0);
            FDOS_LOG.println(" Completed!");
            return;
        }
    }

    TIME_INT_t getNextInterval(TIME_INT_t lastInterval) {
        switch (initStep) {
        case 1:
            return 2 * MICROS_PER_SECOND;
        case 2:
            return .25 * MICROS_PER_SECOND;
        case 3:
            return .5 * MICROS_PER_SECOND;
        case 4:
            return .25 * MICROS_PER_SECOND;
        default:
            return -1;
        }
    }

  public:
    void initMotors() {
        FDOS_LOG.print("PWM ESC init Started.");
        initStep = 0;
        executor.schedule(this, VMExecutor::getTimingPair(0, FrequencyUnitEnum::second));
    }

    void setSpeed(uint8_t motorNum, float speed) {
        if (motorNum >= MOTOR_COUNT)
            return;
        if (speed < 0)
            speed = 0;
        else if (speed > 1)
            speed = 1;
        speeds[motorNum] = speed;
        analogWrite(motorPins[motorNum], 204 + 206 * speed);
    }

    ESC() {
        analogWriteResolution(12);
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            pinMode(motorPins[i], OUTPUT);
            analogWriteFrequency(motorPins[i], MOTOR_PWM_FREQ);
            setSpeed(i, 0);
        }
    }

    float getSpeed(uint8_t motorNum) {
        if (motorNum >= MOTOR_COUNT)
            return -1;
        return speeds[motorNum];
    }

} esc;

RadioTask radioTask(&radio);

void radioInterrupt(void) { radioTask.interruptTriggered(); }

void startRadioActions();

class ControlInputAction : RadioAction {

  public:
    int8_t joyH = 0;
    int8_t joyV = 0;
    uint8_t slideH = 0;
    uint8_t slideV = 0;

    void onReceive(uint8_t length, uint8_t *data) {
        if (data[0] == RADIO_MSG_ID::TRANSMIT_CONTROLS) {
            joyH = data[1];
            joyV = data[2];
            slideH = data[3];
            slideV = data[4];

            FDOS_LOG.printf("JH:%i JV:%i SH:%i SV:%i\n", joyH, joyV, slideH, slideV);
            // This is a test using radio source to change motor speeds
        }
    }

    void onStart() {
        joyH = joyV = 0;
        slideH = slideV = 0;
    }

    void onStop() {
        joyH = joyV = 0;
        slideH = slideV = 0;
    }
} controlInputAction;

class SustainConnectionAction : RadioAction, RunnableTask {

    uint8_t missedComCount = 0;
    ScheduledLink *cancel = NULL;

    fc_heartbeat_t lastFCHB;
    ct_heartbeat_t lastCTHB;

    uint16_t getBatteryVoltage() { return analogRead(BATTERY_SENS_PIN); }

    void onReceive(uint8_t length, uint8_t *data) {
        if (missedComCount == 1) { // If greater than one, already sent on hb
            requestSend();
        }
        missedComCount = 0;
        if (data[0] == CT_HEARTBEAT) {
            FDOS_LOG.println(F("Heartbeat received from controller"));
            msgFromBytes(lastCTHB, data + 1);
        }
    }

    // returns length of data to send
    uint8_t onSendReady(uint8_t *data) {
        float snr = radio.getSNR();
        FDOS_LOG.print(F("SNR:"));
        FDOS_LOG.println(snr);

        lastFCHB.batV = (getBatteryVoltage() - 3430) / 3.2;
        lastFCHB.snr = snr * 10;

        lastFCHB.headings[0] = convertHeading(motionSensor.yaw);
        lastFCHB.headings[1] = convertHeading(motionSensor.pitch);
        lastFCHB.headings[2] = convertHeading(motionSensor.roll);
        lastFCHB.pressure = convertPressure(motionSensor.pressureHPA);

        lastFCHB.speeds[0] = esc.getSpeed(0) * 255;
        lastFCHB.speeds[0] = esc.getSpeed(1) * 255;
        lastFCHB.speeds[0] = esc.getSpeed(2) * 255;
        lastFCHB.speeds[0] = esc.getSpeed(3) * 255;

        // lastFCHB.print((Print*)&FDOS_LOG);

        data[0] = FC_HEARTBEAT;
        msgToBytes(lastFCHB, data + 1);

        FDOS_LOG.print(F("Batt:"));
        FDOS_LOG.println(getBatteryVoltage());

        return lastFCHB.size + 1;
    }

    void run(TIME_INT_t time) {
        if (missedComCount > FC_DC_TIMEOUT_SECONDS) {
            FDOS_LOG.println(F("DISCONNECTING TRANSMITTER - Timeout"));
            radioTask->removeAllActions();
            startRadioActions();
            return;
        } else if (missedComCount %2 == 1) {
            requestSend();
        }
        missedComCount++;
    }

  public:
    void onStart() {

        missedComCount = 0;

        if (cancel != NULL)
            cancel->cancel();
        cancel = executor.schedule((RunnableTask *)this, executor.getTimingPair(1, FrequencyUnitEnum::second));
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
            if (cancel != NULL)
                cancel->cancel();
            cancel = NULL;
            radioTask->addAction((RadioAction *)&sustainAction);
            radioTask->addAction((RadioAction *)&controlInputAction);
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
        digitalWrite(LED_PIN, blink);
        blink = !blink;
        requestSend();
    }

  public:
    void onStart() {
        if (cancel != NULL)
            cancel->cancel();
        transmitterId = 0;
        cancel = executor.schedule((RunnableTask *)this, 5000000LL, 2000000LL);
    }

} beaconAction;

void startRadioActions() { radioTask.addAction((RadioAction *)&beaconAction); }

void setup() {
    analogReadResolution(12);
    Serial.begin(921600);
    pinMode(BATTERY_SENS_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, false);

    // put your setup code here, to run once:

    FDOS_LOG.println("[SX1276] Initializing ...");
    // Seems to need this delay to start reliably
    int state = radio.begin(RADIO_CARRIER_FREQ, RADIO_LINK_BANDWIDTH, RADIO_SPREADING_FACTOR); //-23dBm

    if (state == RADIOLIB_ERR_NONE) {
        FDOS_LOG.print(" started, configuring ...");
    } else {
        FDOS_LOG.print("SX1276 failed. Code:");
        FDOS_LOG.println(state);
        while (true) {
            digitalWrite(LED_PIN, true);
            delay(200);
            digitalWrite(LED_PIN, false);
            delay(100);
        }
    }

    if (radio.setOutputPower(RADIO_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        FDOS_LOG.println(F("Invalid Power!"));
        while (true) {
            digitalWrite(LED_PIN, true);
            delay(100);
            digitalWrite(LED_PIN, false);
            delay(200);
        }
    }
    FDOS_LOG.println(F("success!"));

    radio.setDio0Action(radioInterrupt);

    FDOS_LOG.println("Initializing esc signals");
    esc.initMotors();

    FDOS_LOG.println("Initializing Motion Sensor");
    motionSensor.initSensors();
    FDOS_LOG.println("Calibrating , hope you didnt move!");
    motionSensor.calibrateGyro();

    executor.schedule((RunnableTask *)&motionSensor, VMExecutor::getTimingPair(USFSMAX_IMU_RATE, FrequencyUnitEnum::per_second));

    executor.schedule((RunnableTask *)&radioTask, executor.getTimingPair(RADIO_INTERVAL_MILLIS, FrequencyUnitEnum::milli));

    startRadioActions();
}

void loop() {
    uint32_t delayTime = executor.runSchedule();
    // You may want to allow the loop to finish and avoid long delays
    //    if you are using background arduino features
    // if (delayTime > 100000)
    //     delayTime = 100000;
    if (delayTime > MIN_MICRO_REST)
        delayMicroseconds(delayTime - MIN_MICRO_REST);
}
