
#ifndef TARGET_ORIENTATION_NAV_H__
#define TARGET_ORIENTATION_NAV_H__

#include "Navigation.h"
#include "PID.h"

class TargetOrientationNav : public GenericNavTask {
  public:
    TargetOrientationNav(ESC *_esc, MotionTask *_motion);
    esc_objective_attr nextFrame(TIME_INT_t intervalMillis);

    void holdOrientation(bool yaw, bool pitch, bool roll);

    void setControlMode(bool yawDirect, bool pitchDirect, bool rollDirect);

    void updatePIDConfiguration(double yaw_KP, double yaw_KI, double yaw_KD, int32_t yaw_MAX_I, double roll_KP, double roll_KI, double roll_KD,
                                int32_t roll_MAX_I, double pitch_KP, double pitch_KI, double pitch_KD, int32_t pitch_MAX_I);

    void setTelemCapture(bool enabled) ;
    
    uint16_t getTelemCount() ;
    uint32_t getTelemSampleDurationMicros() ;

    bool getTelemSample(int32_t index, uint8_t ypr, pid_state_t &state); 


  protected:
    void activateESCEvent(bool activated);

        
    int32_t telemSampleIndex; // Protected For unit testing
    time_t captureStart,captureEnd;

  private:
    esc_objective_attr esc;

    bool yawDirectMode = false, pitchDirectMode = false, rollDirectMode = false;
    bool yawAssistRollMode = false;

    bool telemCaptureEnabled = false;

    PID yawPID;
    PID rollPID;
    PID pitchPID;


};


#endif