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

esc_objective_attr TargetOrientationNav::nextFrame(TIME_INT_t intervalMicros) {

    int16_t rollInput = slideH == 0 ? 0 : slideH - 127;

    if (rollDirectMode) {
        esc.roll = (rollInput * 1000) / 127;
    } else {
        bool hardRollInput = abs(rollInput < NAV_INPUT_ROLL_DIRECT_THRESHOLD);
        rollInput = rollInput * NAV_INPUT_ROLL_SCALE / 255;
        targetOrientation.roll = (hardRollInput ? targetOrientation.roll : currentOrientation.roll) + rollInput;
        esc.roll = (2 * esc.roll + rollPID.apply(currentOrientation.roll, targetOrientation.roll, intervalMicros / MICROS_PER_MILLI, rollState)) / 3;
    }

    int16_t yawInput = joyH;

    if (yawDirectMode) {
        esc.yaw = yawInput * 1000 / 127;
    } else {
        bool hardYawInput = abs(yawInput < NAV_INPUT_YAW_DIRECT_THRESHOLD);
        yawInput = yawInput * NAV_INPUT_YAW_SCALE / 255;
        targetOrientation.yaw = (hardYawInput ? targetOrientation.yaw : currentOrientation.yaw) + yawInput;
        esc.yaw = yawPID.apply(currentOrientation.yaw, targetOrientation.yaw, intervalMicros / MICROS_PER_MILLI, yawState);
    }

    int16_t pitchInput = joyV;

    if (pitchDirectMode) {
        esc.pitch = pitchInput * 1000 / 127;
    } else {
        bool hardPitchInput = abs(pitchInput < NAV_INPUT_PITCH_DIRECT_THRESHOLD);
        pitchInput = pitchInput * NAV_INPUT_PITCH_SCALE / 255;
        targetOrientation.pitch = (hardPitchInput ? targetOrientation.pitch : currentOrientation.pitch) + pitchInput;
        esc.pitch = pitchPID.apply(currentOrientation.pitch, targetOrientation.pitch, intervalMicros / MICROS_PER_MILLI, pitchState);
    }
    
    esc.throttle = (slideV * NAV_INPUT_THROTTLE_SCALE) / 255;

#ifdef NAV_TRACE
    if (nav_trace_counter % NAV_TRACE_EVERY == 0) {
// FDOS_LOG.printf("E P:%i Y:%i R:%i T:%i\n", esc.pitch, esc.yaw, esc.roll, esc.throttle);
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

void TargetOrientationNav::activateESCEvent() {
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
    // targetOrientation.roll = currentOrientation.roll;
    // targetOrientation.pitch = currentOrientation.pitch;
    // targetOrientation.yaw = currentOrientation.yaw;
}
