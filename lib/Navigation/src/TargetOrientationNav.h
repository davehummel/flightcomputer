
#ifndef TARGET_ORIENTATION_NAV_H__
#define TARGET_ORIENTATION_NAV_H__

#include "Navigation.h"
#include "PID.h"

class TargetOrientationNav : public GenericNavTask {
  public:
    TargetOrientationNav(ESC *_esc, MotionTask *_motion) : GenericNavTask(_esc, _motion);
    esc_objective_attr nextFrame(TIME_INT_t intervalMillis);

    void holdOrientation(bool yaw, bool pitch, bool roll) {
        if (yaw)
            targetOrientation.yaw = currentOrientation.yaw;
        if (pitch)
            targetOrientation.pitch = currentOrientation.pitch;
        if (roll)
            targetOrientation.roll = currentOrientation.roll;
    }

    void setControlMode(bool yawDirect, bool pitchDirect, bool rollDirect) {
        bool resetYawOrientation = yawDirect != yawDirectMode;
        bool resetPitchOrientation = pitchDirect != pitchDirectMode;
        bool resetRollOrientation = rollDirect != rollDirectMode;

        if (!(resetYawOrientation || resetPitchOrientation || resetRollOrientation))
            return;

        yawDirectMode = yawDirect;
        pitchDirectMode = pitchDirect;
        rollDirectMode = rollDirect;

        holdOrientation(resetYawOrientation, resetPitchOrientation, resetRollOrientation);

        FDOS_LOG.printf("Setting direct control on Y:%i,P:%i,R:%i\n", yawDirect, pitchDirect, rollDirect);
    }

    void updatePIDConfiguration(double yaw_KP, double yaw_KI, double yaw_KD, int32_t yaw_MAX_I, double roll_KP, double roll_KI, double roll_KD,
                                int32_t roll_MAX_I, double pitch_KP, double pitch_KI, double pitch_KD, int32_t pitch_MAX_I);

    void setTelemCapture(bool enabled) {
        telemCaptureEnabled = enabled;
        telemSampleIndex = 0;
    }

  protected:
    void activateESCEvent(bool activated);

  private:
    esc_objective_attr esc;

    bool yawDirectMode = false, pitchDirectMode = false, rollDirectMode = false;
    bool yawAssistRollMode = false;

    bool telemCaptureEnabled = false;

    PID yawPID;
    PID rollPID;
    PID pitchPID;


    pid_state_t *telemSamples;
    uint32_t telemSampleIndex;

};

#endif