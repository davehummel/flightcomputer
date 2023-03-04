#include <Arduino.h>
#include <RadioLib.h>

#include "FDOS_LOG.h"
#include "I2Cdev.h"
#include "IMU.h"
#include "Sensor_cal.h"
#include "USFSMAX.h"
#include "VMExecutor.h"
#include "VMTime.h"
#include "PowerControl.h"

#include "FlightMessages.h"
#include <FlightRadio.h>
#include <MotionTask.h>
#include <Navigation.h>
#include <RadioTask.h>

#include "TargetOrientationNav.h"
// #include "TestNav.h"

Logger FDOS_LOG(&Serial);


SX1276 RADIO = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN, SPI, SPISettings(SPI_RADIO_FREQ, MSBFIRST, SPI_MODE0));

void beginReceive(void) { RADIO.startReceive(); }

MotionTask motionSensor;

ESC esc(&EXECUTOR);

RadioTask RADIOTASK(&RADIO);

TargetOrientationNav nav(&esc, &motionSensor);
// DirectInputNav nav(&esc, &motionSensor);  // Test Nav



void radioInterrupt(void) { RADIOTASK.interruptTriggered(); }

void startRadioActions();

void startRadioActions() {
    sustainConnectionAction.setNavSource(&nav);
    RADIOTASK.addAction((RadioAction *)&listenTransmitterAction);
}


void setup() {

    Serial.begin(921600);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(BUTTON_SENSE_PIN, INPUT_PULLDOWN);
#ifdef USB_SERIAL_HID
    Joystick.begin();
    Joystick.useManualSend(true);
#endif
#if defined(M0_FEATHER)
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
#endif

    delay(2000);

    POWER.start();

    Serial.println("[SX1276] Initializing ...");
    SPI.begin();
    int state = RADIO.begin(RADIO_CARRIER_FREQ, RADIO_LINK_BANDWIDTH, RADIO_SPREADING_FACTOR, RADIO_CODING_RATE);

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

    if (RADIO.setOutputPower(RADIO_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        FDOS_LOG.println(F("Invalid Power!"));
        while (true) {
            digitalWrite(LED_PIN, true);
            delay(100);
            digitalWrite(LED_PIN, false);
            delay(200);
        }
    }
    FDOS_LOG.println(F("success!"));

    RADIO.setDio0Action(radioInterrupt);

    FDOS_LOG.println("Initializing Motion Sensor");
    motionSensor.initSensors();
    FDOS_LOG.println("Calibrating , hope you didnt move!");
    motionSensor.calibrateGyro();
    EXECUTOR.schedule((RunnableTask *)&motionSensor, VMExecutor::getTimingPair(USFSMAX_IMU_RATE, FrequencyUnitEnum::per_second));
    EXECUTOR.schedule((RunnableTask *)&nav, VMExecutor::getTimingPair(NAV_RATE, FrequencyUnitEnum::per_second));
    EXECUTOR.schedule((RunnableTask *)&RADIOTASK, VMExecutor::getTimingPair(RADIO_INTERVAL_MILLIS, FrequencyUnitEnum::milli));
    startRadioActions();

    FDOS_LOG.println("Initializing esc signals");
    esc.initMotors();
    for (uint8_t i = 0; i < 10; i++) {
        digitalWrite(LED_PIN, true);
        delay(50);
        digitalWrite(LED_PIN, false);
        delay(20);
    }
}

void loop() {
    uint32_t delayTime = EXECUTOR.runSchedule();
    if (delayTime > 1000000)
        delayTime = 1000000;
    if (delayTime > MIN_MICRO_REST)
        delayMicroseconds(delayTime - MIN_MICRO_REST);
}
