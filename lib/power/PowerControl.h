#ifndef power__control_H__
#define power__control_H__
#include "FDOS_LOG.h"
#include "VMExecutor.h"
#include "VMTime.h"
#include <Arduino.h>


typedef void (*FunctionPointer)();

class Power : RunnableTask {
  public:
    Power(int hold_power_pin, int power_btn_sense_pin, int bat_sense_pin, int en_bat_sense_pin);
    void start();
    void run(TIME_INT_t time);
    bool isPowerBtnPressed();
    void powerDown();
    float getBatteryVoltage();
    uint8_t getBatteryPercent();
    void onShutdownSubscribe(FunctionPointer listener);
    void onShutdownUnsubscribe(FunctionPointer listener);

  private:
    const int holdPowerPin;
    const int powerBtnSensePin;
    const int batSensePin;
    const int enBatSensePin;

    bool initialBtnRelease = false;
    bool firstBtnOffDetected = false;

    bool isPoweredDown = false;

    const static uint8_t MAX_LISTENERS = 10;
    FunctionPointer listeners[MAX_LISTENERS] = {0};
};

extern Power POWER;

#endif