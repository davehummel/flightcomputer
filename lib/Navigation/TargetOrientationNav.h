
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
        FDOS_LOG.printf("Setting direct control on Y:%i,P:%i,R:%i\n",yawDirect,pitchDirect,rollDirect);
    }

  protected:
     void activateESCEvent(bool activated);
  private:
    esc_objective_attr esc;

    bool yawDirectMode = false, pitchDirectMode = false, rollDirectMode = false;
    bool yawAssistRollMode = false;

 

    PID yawPID = PID(PID_YAW_KP,PID_YAW_KI, PID_YAW_KD, PID_YAW_MAXOUT, PID_YAW_INPERIOD, PID_YAW_MAXINTEGRAL);
    // PID yawPID = PID(0.0,0.0,0.0,1000,__UINT16_MAX__,400);
    PID rollPID = PID(PID_ROLL_KP,PID_ROLL_KI, PID_ROLL_KD, PID_ROLL_MAXOUT, PID_ROLL_INPERIOD, PID_ROLL_MAXINTEGRAL);
    // PID rollPID = PID(0.0,0.0,0.0 ,1000,__UINT16_MAX__,400);
    PID pitchPID = PID(PID_PITCH_KP,PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_MAXOUT, PID_PITCH_INPERIOD, PID_PITCH_MAXINTEGRAL);
    // PID pitchPID = PID(0.0,0.0,0.0,1000,__UINT16_MAX__,400);
};

#endif