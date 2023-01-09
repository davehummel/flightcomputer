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

// class ControlInputAction : RadioAction {

//   public:
//     int8_t joyH = 0;
//     int8_t joyV = 0;
//     uint8_t slideH = 0;
//     uint8_t slideV = 0;

//     ct_config_t controlConfig;

//     void onReceive(uint8_t length, uint8_t *data) {
//         switch (data[0]) {
//         case RADIO_MSG_ID::FLIGHT_MODE_UPDATE:
//             nav.connectESC(data[1]);
//             FDOS_LOG.print("Flight mode ");
//             FDOS_LOG.println(data[1] ? "On" : "Off");
//             return;

//         case RADIO_MSG_ID::TRANSMIT_CONTROLS:
//             joyH = data[1];
//             joyV = data[2];
//             slideH = data[3];
//             slideV = data[4];

//             FDOS_LOG.printf("JH:%i JV:%i SH:%i SV:%i\n", joyH, joyV, slideH, slideV);

//             nav.recordInput(joyH, joyV, slideH, slideV);
//             return;

//         case RADIO_MSG_ID::RESET_ORIENTATION:
//             FDOS_LOG.println("HOLDING new orientation");
//             joyH = 0;
//             joyV = 0;
//             slideH = 0;
//             nav.holdOrientation();
//             return;

//         case RADIO_MSG_ID::CHANGE_CONFIG:
//             msgFromBytes(&controlConfig, data + 1, controlConfig.size);
//             nav.setControlMode(ct_config_t::DIRECT == controlConfig.yawMode, ct_config_t::DIRECT == controlConfig.pitchMode,
//                                ct_config_t::DIRECT == controlConfig.rollMode);
//             FDOS_LOG.printf("Control changes y:%i p:%i r:%i\n", controlConfig.yawMode, controlConfig.pitchMode, controlConfig.rollMode);
//             return;
//         }
//     }

//     void onStart() {
//         joyH = joyV = 0;
//         slideH = slideV = 0;
//         nav.recordInput(joyH, joyV, slideH, slideV);
//     }

//     void onStop() {
//         joyH = joyV = 0;
//         slideH = slideV = 0;
//         nav.recordInput(joyH, joyV, slideH, slideV);
//     }
// } controlInputAction;

// class SustainConnectionAction : RadioAction, RunnableTask {

//     uint8_t missedComCount = 0;
//     ScheduledLink *cancel = NULL;

//     fc_heartbeat_t lastFCHB;
//     ct_heartbeat_t lastCTHB;

//     uint16_t getBatteryVoltage() { return analogRead(BATTERY_SENS_PIN); }

//     void onReceive(uint8_t length, uint8_t *data) {
//         if (data[0] == CT_HEARTBEAT) {
//             FDOS_LOG.print("Heartbeat received from controller : enabled = ");
//             msgFromBytes(&lastCTHB, data + 1, ct_heartbeat_t::size);
//             FDOS_LOG.println(lastCTHB.flightModeEnabled);
//             // if (lastCTHB.flightModeEnabled!=nav.isESCConnected())
//             //     nav.connectESC(lastCTHB.flightModeEnabled);
//             requestSend();
//             missedComCount = 0;
//         }
//     }

//     // returns length of data to send
//     uint8_t onSendReady(uint8_t *data) {
//         float snr = radio.getSNR();
//         FDOS_LOG.print(F("SNR:"));
//         FDOS_LOG.println(snr);
// #if defined(TEENSYDUINO)
//         lastFCHB.batV = (getBatteryVoltage() - 3430) / 3.2;
// #elif defined(M0_FEATHER)
//         lastFCHB.batV = ((getBatteryVoltage() * 2 * 3.3) / 4096.0 - 3.2) * 100;
// #endif
//         lastFCHB.snr = snr * 10;

//         lastFCHB.headings[0] = convertHeading(motionSensor.yaw);
//         lastFCHB.headings[1] = convertHeading(motionSensor.pitch);
//         lastFCHB.headings[2] = convertHeading(motionSensor.roll);
//         lastFCHB.pressure = convertPressure(motionSensor.pressureHPA);

//         lastFCHB.targetHeadings[0] = nav.targetOrientation.yaw;
//         lastFCHB.targetHeadings[1] = nav.targetOrientation.pitch;
//         lastFCHB.targetHeadings[2] = nav.targetOrientation.roll;

//         lastFCHB.speeds[0] = esc.getSpeed(0) / 4;
//         lastFCHB.speeds[1] = esc.getSpeed(1) / 4;
//         lastFCHB.speeds[2] = esc.getSpeed(2) / 4;
//         lastFCHB.speeds[3] = esc.getSpeed(3) / 4;

//         // lastFCHB.print((Print*)&FDOS_LOG);

//         data[0] = FC_HEARTBEAT;
//         msgToBytes(&lastFCHB, data + 1, fc_heartbeat_t::size);

//         FDOS_LOG.print(F("Batt:"));
//         FDOS_LOG.print(getBatteryVoltage());
//         FDOS_LOG.print(F(" -> "));
//         FDOS_LOG.println(lastFCHB.batV);

//         return lastFCHB.size + 1;
//     }

//     void run(TIME_INT_t time) {
//         if (missedComCount > FC_DC_TIMEOUT_SECONDS) {
//             FDOS_LOG.println(F("DISCONNECTING TRANSMITTER - Timeout"));
//             nav.connectESC(false);
//             radioTask->removeAllActions();
//             startRadioActions();
//             return;
//         }
//         missedComCount++;
//     }

//   public:
//     void onStart() {

//         missedComCount = 0;
//         nav.setControlMode(false, false, false);
//         if (cancel != NULL)
//             cancel->cancel();
//         cancel = executor.schedule((RunnableTask *)this, executor.getTimingPair(1, FrequencyUnitEnum::second));
//     }

//     void onStop() {
//         if (cancel != NULL)
//             cancel->cancel();
//         cancel = NULL;
//     }

// } sustainAction;

// class BeaconAction : RadioAction, RunnableTask {

//     ScheduledLink *cancel = NULL;
//     bool blink = false;

//     bool isConfirming = false;
//     uint8_t transmitterId = 0;

//     void onReceive(uint8_t length, uint8_t *data) {
//         if (data[0] == TRANSMITTER_RECOGNIZE && data[2] == RECEIVER_ID) {
//             if (!requestSend())
//                 return;
//             transmitterId = data[1];
//             if (cancel != NULL)
//                 cancel->cancel();
//             cancel = NULL;
//             radioTask->addAction((RadioAction *)&sustainAction);
//             radioTask->addAction((RadioAction *)&controlInputAction);
//         }
//     }

//     // returns length of data to send
//     uint8_t onSendReady(uint8_t *data) {
//         if (transmitterId == 0) {
//             data[0] = RADIO_MSG_ID::RECEIVER_BEACON;
//             data[1] = RECEIVER_ID;
//             return 2;
//         } else {
//             data[0] = RADIO_MSG_ID::RECEIVER_RECOGNIZE_CONFIRM;
//             data[1] = transmitterId;
//             return 2;
//         }
//     }

//     void run(TIME_INT_t time) {
//         digitalWrite(LED_PIN, blink);
//         blink = !blink;
//         requestSend();
//     }

//   public:
//     void onStart() {
//         if (cancel != NULL)
//             cancel->cancel();
//         transmitterId = 0;
//         cancel = executor.schedule((RunnableTask *)this, 5000000LL, 2000000LL);
//     }

// } beaconAction;

void startRadioActions() { RADIOTASK.addAction((RadioAction *)&listenTransmitterAction); }

void setup() {
    analogReadResolution(12);
    Serial.begin(921600);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BATTERY_SENS_PIN, INPUT_PULLUP);
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

    Serial.println("[SX1276] Initializing ...");
    SPI.begin();
    int state = RADIO.begin(RADIO_CARRIER_FREQ, RADIO_LINK_BANDWIDTH, RADIO_SPREADING_FACTOR,RADIO_CODING_RATE);

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

    // FDOS_LOG.println("Initializing Motion Sensor");
    // motionSensor.initSensors();
    // FDOS_LOG.println("Calibrating , hope you didnt move!");
    // motionSensor.calibrateGyro();
    // EXECUTOR.schedule((RunnableTask *)&motionSensor, VMExecutor::getTimingPair(USFSMAX_IMU_RATE, FrequencyUnitEnum::per_second));
    // EXECUTOR.schedule((RunnableTask *)&nav, VMExecutor::getTimingPair(NAV_RATE, FrequencyUnitEnum::per_second));


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
