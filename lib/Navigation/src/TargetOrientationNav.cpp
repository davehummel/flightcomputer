#include <TargetOrientationNav.h>

#ifdef NAV_TRACE
uint16_t nav_trace_counter = 0;
#endif

pid_state_t *yawState = 0;
pid_state_t *rollState = 0;
pid_state_t *pitchState = 0;

uint16_t nav_telem_skip_counter = 0;

TargetOrientationNav::TargetOrientationNav(ESC *_esc, MotionTask *_motion) : GenericNavTask(_esc, _motion) {
    telemSamples = new pid_state_t[NAV_TELEM_SAMPLES * 3];
    telemSampleIndex = 0;
}

void TargetOrientationNav::updatePIDConfiguration(double yaw_KP, double yaw_KI, double yaw_KD, int32_t yaw_MAX_I, double roll_KP, double roll_KI,
                                                  double roll_KD, int32_t roll_MAX_I, double pitch_KP, double pitch_KI, double pitch_KD, int32_t pitch_MAX_I) {
    yawPID = PID(yaw_KP, yaw_KI, yaw_KD, PID_YAW_MAXOUT, PID_YAW_INPERIOD, yaw_MAX_I, 3);
    rollPID = PID(roll_KP, roll_KI, roll_KD, PID_ROLL_MAXOUT, PID_ROLL_INPERIOD, roll_MAX_I, 3);
    pitchPID = PID(pitch_KP, pitch_KI, pitch_KD, PID_PITCH_MAXOUT, PID_PITCH_INPERIOD, pitch_MAX_I, 3);

    FDOS_LOG.println("UPDATE|  KP     KI     KD     MAXI  ");
    FDOS_LOG.printf("  yaw| %5f %5f %5f %5i\n", yaw_KP, yaw_KI, yaw_KD, yaw_MAX_I);
    FDOS_LOG.printf("pitch| %5f %5f %5f %5i\n", pitch_KP, pitch_KI, pitch_KD, pitch_MAX_I);
    FDOS_LOG.printf(" roll| %5f %5f %5f %5i\n", roll_KP, roll_KI, roll_KD, roll_MAX_I);

    yawPID.reset();
    pitchPID.reset();
    rollPID.reset();

    holdOrientation(true, true, true);
}

esc_objective_attr TargetOrientationNav::nextFrame(TIME_INT_t intervalMicros) {

    uint16_t intervalMillis = intervalMicros / MICROS_PER_MILLI;

#ifdef NAV_TRACE
    bool doTrace = (nav_trace_counter % NAV_TRACE_EVERY == 0);
    if (doTrace)
        nav_trace_counter = 0;

    nav_trace_counter++;
#endif

    bool doTelem = (nav_trace_counter % NAV_TELEM_EVERY == 0);
    if (doTelem)
        nav_telem_skip_counter = 0;

    nav_telem_skip_counter++;

    if (doTelem && telemCaptureEnabled) {
        uint32_t index = telemSampleIndex % NAV_TELEM_SAMPLES;

        yawState = &telemSamples[index * 3];
        pitchState = &telemSamples[index * 3 + 1];
        rollState = &telemSamples[index * 3 + 2];

        telemSampleIndex++;
        if (telemSampleIndex == NAV_TELEM_SAMPLES * 2) {
            telemSampleIndex = telemSampleIndex;
        }
    } else {
        yawState = NULL;
        pitchState = NULL;
        rollState = NULL;
    }
#ifdef NAV_TRACE
    pid_state_t traceTempState;
    if (doTrace && yawState == NULL)
        yawState = &traceTempState;
    if (doTrace && pitchState == NULL)
        pitchState = &traceTempState;
    if (doTrace && rollState == NULL)
        rollState = &traceTempState;

#endif

    // YAW

    int16_t yawInput = joy1H;

    if (yawDirectMode) {
        esc.yaw = yawInput * 1000 / 127;
        if (yawState != NULL)
            yawState->zero();
#ifdef NAV_TRACE_YAW_PID
        if (doTrace)
            FDOS_LOG.printf("Direct YAW S:%i\n", esc.yaw);
#endif
    } else {
        bool hardYawInput = abs(yawInput < NAV_INPUT_YAW_DIRECT_THRESHOLD);
        yawInput = yawInput * NAV_INPUT_YAW_SCALE / 255;
        targetOrientation.yaw = (hardYawInput ? targetOrientation.yaw : currentOrientation.yaw) + yawInput;
        esc.yaw = yawPID.apply(currentOrientation.yaw, targetOrientation.yaw, intervalMillis, yawState);
#ifdef NAV_TRACE_YAW_PID
        if (doTrace) {
            FDOS_LOG.print("YAW_PID:");
            yawState->print(&FDOS_LOG, yawPID,intervalMillis);
        }
#endif
    }

    // PITCH

    int16_t pitchInput = -joy1V;

    if (pitchDirectMode) {
        esc.pitch = pitchInput * 1000 / 127;
        if (pitchState != NULL)
            pitchState->zero();
#ifdef NAV_TRACE_PITCH_PID
        if (doTrace)
            FDOS_LOG.printf("Direct PCH S:%i\n", esc.pitch);
#endif
    } else {
        bool hardPitchInput = abs(pitchInput < NAV_INPUT_PITCH_DIRECT_THRESHOLD);
        pitchInput = pitchInput * NAV_INPUT_PITCH_SCALE / 255;
        targetOrientation.pitch = (hardPitchInput ? targetOrientation.pitch : currentOrientation.pitch) + pitchInput;
        esc.pitch = pitchPID.apply(currentOrientation.pitch, targetOrientation.pitch, intervalMillis, pitchState);
#ifdef NAV_TRACE_PITCH_PID
        if (doTrace) {
            FDOS_LOG.print("PITCH_PID:");
            pitchState->print(&FDOS_LOG, pitchPID,intervalMillis);
        }
#endif
    }

    // ROLL

    int16_t rollInput = joy2H;

    if (rollDirectMode) {
        esc.roll = (rollInput * 1000) / 127;
        if (rollState != NULL)
            rollState->zero();
#ifdef NAV_TRACE_ROLL_PID
        if (doTrace)
            FDOS_LOG.printf("Direct RLL S:%i\n", esc.roll);
#endif
    } else {
        bool hardRollInput = abs(rollInput < NAV_INPUT_ROLL_DIRECT_THRESHOLD);
        rollInput = rollInput * NAV_INPUT_ROLL_SCALE / 255;
        targetOrientation.roll = (hardRollInput ? targetOrientation.roll : currentOrientation.roll) + rollInput;
        esc.roll = rollPID.apply(currentOrientation.roll, targetOrientation.roll, intervalMillis, rollState);
#ifdef NAV_TRACE_ROLL_PID
        if (doTrace) {
            FDOS_LOG.print("ROLL_PID:");
            rollState->print(&FDOS_LOG, rollPID,intervalMillis);
        }
#endif
    }

    esc.throttle = (joy2V * NAV_INPUT_THROTTLE_SCALE) / 255;

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

    if (telemCaptureEnabled)
        telemCaptureEnabled = false;
}

void TargetOrientationNav::holdOrientation(bool yaw, bool pitch, bool roll) {
    if (yaw)
        targetOrientation.yaw = currentOrientation.yaw;
    if (pitch)
        targetOrientation.pitch = currentOrientation.pitch;
    if (roll)
        targetOrientation.roll = currentOrientation.roll;
}

void TargetOrientationNav::setControlMode(bool yawDirect, bool pitchDirect, bool rollDirect) {
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

void TargetOrientationNav::setTelemCapture(bool enabled) {
    if (telemCaptureEnabled != enabled) {
        FDOS_LOG.print("Switching Telemetry ");
        FDOS_LOG.println(enabled ? "On" : "Off");
        telemCaptureEnabled = enabled;
        telemSampleIndex = 0;
    }
}

int32_t TargetOrientationNav::getMaxRecordedTelemIndex() {
    if (telemSampleIndex >= NAV_TELEM_SAMPLES)
        return NAV_TELEM_SAMPLES - 1;
    else
        return telemSampleIndex;
}

pid_state_t *TargetOrientationNav::getTelemSample(int32_t index) {
    int32_t maxIndex = getMaxRecordedTelemIndex();
    if (index > maxIndex)
        return NULL;

    index = telemSampleIndex - index;

    if (index < 0)
        index += NAV_TELEM_SAMPLES;

    return &telemSamples[telemSampleIndex * 3];
}