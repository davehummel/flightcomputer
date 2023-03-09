#include <TargetOrientationNav.h>

#ifdef NAV_TRACE
uint16_t nav_trace_counter = 0;
#endif
#ifdef NAV_TRACE_YAW_PID
pid_state_t *yawState = new pid_state_t();
#else
pid_state_t *yawState = 0;
#endif
#ifdef NAV_TRACE_ROLL_PID
pid_state_t *rollState = new pid_state_t();
#else
pid_state_t *rollState = 0;
#endif
#ifdef NAV_TRACE_PITCH_PID
pid_state_t *pitchState = new pid_state_t();
#else
pid_state_t *pitchState = 0;
#endif

void TargetOrientationNav::updatePIDConfiguration(double yaw_KP, double yaw_KI, double yaw_KD, int32_t yaw_MAX_I, double roll_KP, double roll_KI,
                                                  double roll_KD, int32_t roll_MAX_I, double pitch_KP, double pitch_KI, double pitch_KD, int32_t pitch_MAX_I) {
    yawPID = PID(yaw_KP, yaw_KI, yaw_KD, PID_YAW_MAXOUT, PID_YAW_INPERIOD, yaw_MAX_I);
    rollPID = PID(roll_KP, roll_KI, roll_KD, PID_ROLL_MAXOUT, PID_ROLL_INPERIOD, roll_MAX_I); // PID rollPID = PID(0.0,0.0,0.0 ,1000,__UINT16_MAX__,400);
    pitchPID = PID(pitch_KP, pitch_KI, pitch_KD, PID_PITCH_MAXOUT, PID_PITCH_INPERIOD, pitch_MAX_I);

    FDOS_LOG.println("UPDATE|  KP     KI     KD     MAXI  ");
    FDOS_LOG.printf("  yaw| %5f %5f %5f %5i\n", yaw_KP, yaw_KI, yaw_KD, yaw_MAX_I);
    FDOS_LOG.printf(" roll| %5f %5f %5f %5i\n", roll_KP, roll_KI, roll_KD, roll_MAX_I);
    FDOS_LOG.printf("pitch| %5f %5f %5f %5i\n", pitch_KP, pitch_KI, pitch_KD, pitch_MAX_I);

    yawPID.reset();
    pitchPID.reset();
    rollPID.reset();

    holdOrientation(true, true, true);
}

esc_objective_attr TargetOrientationNav::nextFrame(TIME_INT_t intervalMicros) {

    int16_t rollInput = joy2H;

    if (rollDirectMode) {
        esc.roll = (rollInput * 1000) / 127;
    } else {
        bool hardRollInput = abs(rollInput < NAV_INPUT_ROLL_DIRECT_THRESHOLD);
        rollInput = rollInput * NAV_INPUT_ROLL_SCALE / 255;
        targetOrientation.roll = (hardRollInput ? targetOrientation.roll : currentOrientation.roll) + rollInput;
        esc.roll = rollPID.apply(currentOrientation.roll, targetOrientation.roll, intervalMicros / MICROS_PER_MILLI, rollState);
    }

    int16_t yawInput = joy1H;

    if (yawDirectMode) {
        esc.yaw = yawInput * 1000 / 127;
    } else {
        bool hardYawInput = abs(yawInput < NAV_INPUT_YAW_DIRECT_THRESHOLD);
        yawInput = yawInput * NAV_INPUT_YAW_SCALE / 255;
        targetOrientation.yaw = (hardYawInput ? targetOrientation.yaw : currentOrientation.yaw) + yawInput;
        esc.yaw = yawPID.apply(currentOrientation.yaw, targetOrientation.yaw, intervalMicros / MICROS_PER_MILLI, yawState);
    }

    int16_t pitchInput = joy1V;

    if (pitchDirectMode) {
        esc.pitch = pitchInput * 1000 / 127;
    } else {
        bool hardPitchInput = abs(pitchInput < NAV_INPUT_PITCH_DIRECT_THRESHOLD);
        pitchInput = pitchInput * NAV_INPUT_PITCH_SCALE / 255;
        targetOrientation.pitch = (hardPitchInput ? targetOrientation.pitch : currentOrientation.pitch) + pitchInput;
        esc.pitch = pitchPID.apply(currentOrientation.pitch, targetOrientation.pitch, intervalMicros / MICROS_PER_MILLI, pitchState);
    }

    esc.throttle = (joy2V * NAV_INPUT_THROTTLE_SCALE) / 255;

#ifdef NAV_TRACE
    if (nav_trace_counter % NAV_TRACE_EVERY == 0) {
#ifdef NAV_TRACE_YAW_PID
        FDOS_LOG.printf("YAW_PID S:%i T:%i O:%i I:%i D:%i pC:%i iC:%i dC:%i \n", yawState->state, yawState->target, yawState->output, yawState->integral,
                        yawState->derivative, yawState->pContribution, yawState->iContribution, yawState->dContribution);
#endif
#ifdef NAV_TRACE_ROLL_PID
        FDOS_LOG.printf("RLL_PID S:%i T:%i O:%i I:%i D:%i pC:%i iC:%i dC:%i \n", rollState->state, rollState->target, rollState->output, rollState->integral,
                        rollState->derivative, rollState->pContribution, rollState->iContribution, rollState->dContribution);
#endif
#ifdef NAV_TRACE_PITCH_PID
        FDOS_LOG.printf("RLL_PID S:%i T:%i O:%i I:%i D:%i pC:%i iC:%i dC:%i \n", pitchState->state, pitchState->target, pitchState->output,
                        pitchState->integral, pitchState->derivative, pitchState->pContribution, pitchState->iContribution, pitchState->dContribution);
#endif
    }
    nav_trace_counter++;
#endif

    return esc;
}

void TargetOrientationNav::activateESCEvent(bool activated) {
    FDOS_LOG.printf("Syncing target orientation to current : yaw %i pitch %i roll %i\n", currentOrientation.yaw, currentOrientation.pitch,
                    currentOrientation.roll);
    targetOrientation = currentOrientation;
    yawPID.reset();
    pitchPID.reset();
    rollPID.reset();

    esc.pitch = 0;
    esc.roll = 0;
    esc.throttle = 0;
    esc.yaw = 0;
}
