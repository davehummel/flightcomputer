
#ifndef TARGET_ORIENTATION_NAV_H__
#define TARGET_ORIENTATION_NAV_H__

#include "Navigation.h"
#include "PID.h"

class TargetOrientationNav : public GenericNavTask {
  public:
    TargetOrientationNav(ESC *_esc, MotionTask *_motion) : GenericNavTask(_esc, _motion) {}
    esc_objective_attr nextFrame(TIME_INT_t intervalMillis);

    void holdOrientation() { targetOrientation = currentOrientation; }

    void setControlMode(bool yawDirect, bool pitchDirect, bool rollDirect) {
        yawDirectMode = yawDirect;
        pitchDirectMode = pitchDirect;
        rollDirectMode = rollDirect;
    }

  private:
    esc_objective_attr esc;

    bool yawDirectMode = false, pitchDirectMode = false, rollDirectMode = false;

    void activateESCEvent();

    PID yawPID = PID(0.05, .02, 0.02, 1000, __UINT16_MAX__, 500);
    // PID yawPID = PID(0.0,0.0,0.0,1000,__UINT16_MAX__,400);
    PID rollPID = PID(0.08, 0.01, 0.005, 1000, __UINT16_MAX__, 250);
    // PID rollPID = PID(0.0,0.0,0.0 ,1000,__UINT16_MAX__,400);
    PID pitchPID = PID(0.05, 0.02, 0.02, 1000, __UINT16_MAX__, 500);
    // PID pitchPID = PID(0.0,0.0,0.0,1000,__UINT16_MAX__,400);
};

#endif