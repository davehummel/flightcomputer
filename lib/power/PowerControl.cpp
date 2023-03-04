#include "PowerControl.h"

Power POWER(POWER_CONTROL_AVG_SAMPLES, POWER_CONTROL_SAMPLE_RATE_MS, BATTERY_SENSE_PIN, CURRENT_SENSE_PIN);

Power::Power(uint8_t sliding_avg_samples, uint16_t interval_ms, int voltage_sense_pin, int current_sense_pin)
    : voltageSensePin(voltage_sense_pin), currentSensePin(current_sense_pin), slidingAvgSamples(sliding_avg_samples), intervalMS(interval_ms) {

    analogReadResolution(ANALOG_READ_RESOLUTION);
    analogReadAveraging(ANALOG_READ_AVERAGING);
    pinMode(voltageSensePin, INPUT_PULLUP);
    pinMode(currentSensePin, INPUT);
}

void Power::start() {
    voltage = analogRead(voltageSensePin);
    current = analogRead(currentSensePin);
    EXECUTOR.schedule(this, EXECUTOR.getTimingPair(intervalMS, FrequencyUnitEnum::milli));
}

uint16_t Power::getBatteryVoltage() { return voltage; }

uint16_t Power::getBatteryCurrent() { return current; }

void Power::run(TIME_INT_t time) {

    voltage = (voltage * (slidingAvgSamples - 1) + analogRead(voltageSensePin)) / slidingAvgSamples;
    current = (current * (slidingAvgSamples - 1) + analogRead(currentSensePin)) / slidingAvgSamples;

}