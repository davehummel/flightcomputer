#ifndef __MOTIONTASK_H__
#define __MOTIONTASK_H__

#include "Arduino.h"

#include "VMExecutor.h"

class MotionTask : public RunnableTask {
  private:
    // Calculate geomagnetic calibration parameters for your location (set in "config.h")
    bool barometerReady = false;

    void fetchOrientationData();
    void fetchBarometerData();

  public:
    float pressureHPA = 0;
    float yaw,pitch,roll = 0; 

    void initSensors();

    void calibrateGyro();

    void run(TIME_INT_t time);
};

#endif