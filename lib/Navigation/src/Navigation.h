
#ifndef NAVTASK_H__
#define NAVTASK_H__

#include "Arduino.h"
#include "FDOS_LOG.h"
#include "FlightMessages.h"
#include "MotionTask.h"
#include "VMExecutor.h"
#include "esc.h"

struct orient_attr {
    uint16_t pitch, yaw, roll;
};

class GenericNavTask : public RunnableTask {
  private:
    esc_objective_attr output;

    ESC *esc;
    MotionTask *motion;

    TIME_INT_t lastFrameTime;

    bool escConnected = false;

    void setCurrentOrientation(float yaw, float pitch, float roll, float pressureHPA) {
        currentOrientation.yaw = convertHeading(yaw);
        currentOrientation.pitch = convertHeading(pitch);
        currentOrientation.roll = convertHeading(roll);
        currentPressure = convertPressure(pressureHPA);
    }

  protected:
    virtual esc_objective_attr nextFrame(TIME_INT_t intervalMicros) = 0;
    virtual void activateESCEvent(bool activated) = 0;

    int8_t joy1H;
    int8_t joy1V;
    int8_t joy2H;
    uint8_t joy2V;

  public:
    orient_attr currentOrientation;
    orient_attr targetOrientation;

    uint16_t currentPressure;

    GenericNavTask(ESC *_esc, MotionTask *_motion) : esc(_esc), motion(_motion) {}

    void recordInput(int8_t joyH, int8_t joyV, uint8_t slideH, uint8_t slideV);

    void run(TIME_INT_t time);

    void connectESC(bool connected);

    bool isESCConnected() { return escConnected; }

    ESC *getESC() { return esc; }

    virtual void holdOrientation() {}
};

#endif