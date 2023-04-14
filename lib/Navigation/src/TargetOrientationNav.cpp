#include <TargetOrientationNav.h>

#ifdef NAV_TRACE
uint16_t nav_trace_counter = 0;
#endif

pid_state_t *yawState = 0;
pid_state_t *rollState = 0;
pid_state_t *pitchState = 0;

uint16_t nav_telem_skip_counter = 0;

pid_state_t *TELEM_BUFFER = new pid_state_t[NAV_TELEM_SAMPLES * 3];

TargetOrientationNav::TargetOrientationNav(ESC *_esc, MotionTask *_motion) : GenericNavTask(_esc, _motion) { telemSampleIndex = 0; }

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

#ifdef NAV_TRACE
    bool doTrace = (nav_trace_counter % NAV_TRACE_EVERY == 0);
    if (doTrace)
        nav_trace_counter = 0;

    nav_trace_counter++;
#endif

    bool doTelem = (nav_trace_counter % NAV_TELEM_EVERY == 0);
    if (doTelem) {
        nav_telem_skip_counter = 0;
#ifdef NAV_TRACE
        if (doTrace)
            FDOS_LOG.printf("Telem Index = %i as sample %i on time interval %i \n", telemSampleIndex, getTelemCount(), intervalMicros);
#endif
    }

    nav_telem_skip_counter++;

    if (doTelem && telemCaptureEnabled) {
        uint32_t index = telemSampleIndex % NAV_TELEM_SAMPLES;

        yawState = &TELEM_BUFFER[index * 3];
        pitchState = &TELEM_BUFFER[index * 3 + 1];
        rollState = &TELEM_BUFFER[index * 3 + 2];

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

    bool hardYawInput = abs(yawInput < NAV_INPUT_YAW_DIRECT_THRESHOLD);
    yawInput = yawInput * NAV_INPUT_YAW_SCALE / 255;
    targetOrientation.yaw = (hardYawInput ? targetOrientation.yaw : currentOrientation.yaw) + yawInput;
    esc.yaw = yawPID.apply(currentOrientation.yaw, targetOrientation.yaw, intervalMicros, yawState);
    if (yawDirectMode)
        esc.yaw = yawInput * 1000 / 127;
#ifdef NAV_TRACE_YAW_PID
    if (doTrace) {
        FDOS_LOG.print("YAW_PID:");
        yawState->print(&FDOS_LOG, yawPID);
    }
#endif

    // PITCH

    int16_t pitchInput = -joy1V;

    bool hardPitchInput = abs(pitchInput < NAV_INPUT_PITCH_DIRECT_THRESHOLD);
    pitchInput = pitchInput * NAV_INPUT_PITCH_SCALE / 255;
    targetOrientation.pitch = (hardPitchInput ? targetOrientation.pitch : currentOrientation.pitch) + pitchInput;
    esc.pitch = pitchPID.apply(currentOrientation.pitch, targetOrientation.pitch, intervalMicros, pitchState);
    if (pitchDirectMode)
        esc.pitch = pitchInput * 1000 / 127;
#ifdef NAV_TRACE_PITCH_PID
    if (doTrace) {
        FDOS_LOG.print("PITCH_PID:");
        pitchState->print(&FDOS_LOG, pitchPID);
    }
#endif

    // ROLL

    int16_t rollInput = joy2H;

    bool hardRollInput = abs(rollInput < NAV_INPUT_ROLL_DIRECT_THRESHOLD);
    rollInput = rollInput * NAV_INPUT_ROLL_SCALE / 255;
    targetOrientation.roll = (hardRollInput ? targetOrientation.roll : currentOrientation.roll) + rollInput;
    esc.roll = rollPID.apply(currentOrientation.roll, targetOrientation.roll, intervalMicros, rollState);
    if (rollDirectMode)
        esc.roll = (rollInput * 1000) / 127;
#ifdef NAV_TRACE_ROLL_PID
    if (doTrace) {
        FDOS_LOG.print("ROLL_PID:");
        rollState->print(&FDOS_LOG, rollPID);
    }
#endif

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
    if (yaw) {
        targetOrientation.yaw = currentOrientation.yaw;
        yawPID.reset();
    }
    if (pitch) {
        targetOrientation.pitch = currentOrientation.pitch;
        pitchPID.reset();
    }
    if (roll) {
        targetOrientation.roll = currentOrientation.roll;
        rollPID.reset();
    }
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
        telemCaptureEnabled = enabled;
        if (enabled) {
            telemSampleIndex = 0;
            FDOS_LOG.println("On!");
            captureStart = micros64();
        } else {
            captureEnd = micros64();
            FDOS_LOG.printf("Off after %i samples\n", getTelemCount());
        }
    }
}

uint16_t TargetOrientationNav::getTelemCount() {
    if (telemSampleIndex >= NAV_TELEM_SAMPLES)
        return NAV_TELEM_SAMPLES;
    else
        return telemSampleIndex;
}

uint32_t TargetOrientationNav::getTelemSampleDurationMicros() {
    if (telemSampleIndex == 0)
        return 0;
    else
        return (captureEnd - captureStart) / getTelemCount();
    ;
}

bool TargetOrientationNav::getTelemSample(int32_t index, uint8_t ypr, pid_state_t &state) {
    if (index > getTelemCount())
        return false;

    index = telemSampleIndex - index;

    if (index < 0)
        index += NAV_TELEM_SAMPLES;

    state = TELEM_BUFFER[index * 3 + ypr];

    return true;
}