#ifndef power__control_H__
#define power__control_H__
#include "FDOS_LOG.h"
#include "VMExecutor.h"
#include "VMTime.h"
#include <Arduino.h>


typedef void (*FunctionPointer)();

class Power : RunnableTask {
  public:
    Power(uint8_t sliding_avg_samples, uint16_t interval_ms, int voltage_sense_pin, int current_sense_pin);
    void start();
    void run(TIME_INT_t time);


    uint16_t getBatteryVoltage();
    uint16_t getBatteryCurrent();



  private:

    const int voltageSensePin;
    const int currentSensePin;
    const uint32_t slidingAvgSamples;
    const uint16_t intervalMS;

    uint32_t voltage;
    uint32_t current;

};

extern Power POWER;

#endif