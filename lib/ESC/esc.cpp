#include "esc.h"

void ESC::InitMotorTask::run(TIME_INT_t time) {
    FDOS_LOG.print('.');
    initStep++;
    switch (initStep) {
    case 1:
        parent->setAllSpeeds(0);
        return;
    case 2:
        parent->setAllSpeeds(500);
        return;
    case 3:
        parent->setAllSpeeds(1000);
        return;
    case 4:
        parent->setAllSpeeds(250);
        return;
    default:
        parent->setAllSpeeds(0);
        FDOS_LOG.println(" Completed!");
        return;
    }
}

TIME_INT_t ESC::InitMotorTask::getNextInterval(TIME_INT_t lastInterval) {
    switch (initStep) {
    case 1:
        return 1 * MICROS_PER_SECOND;
    case 2:
        return .25 * MICROS_PER_SECOND;
    case 3:
        return .5 * MICROS_PER_SECOND;
    case 4:
        return .25 * MICROS_PER_SECOND;
    default:
        return -1;
    }
}

void ESC::setAllSpeeds(uint16_t speed) {
    if (speed > 1000)
        speed = 1000;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        setMotorDirect(i, speed);
    }
}

void ESC::initMotors() {
    setAllSpeeds(0);
     // This will only be needed for ESC that have initialize throttle process 
    FDOS_LOG.print("PWM ESC init Started.");
    executor->schedule(&initTask, 1, 1);
    
}

uint16_t MIN_PMW = 16383/20-2;
uint16_t MAX_PMW = 16383/10+2;

void ESC::setMotorDirect(uint8_t motorNum, uint16_t speed) {
    if (motorNum >= MOTOR_COUNT)
        return;
    if (speed > 1000)
        speed = 1000;

    speeds[motorNum] = speed;
#if defined(TEENSYDUINO)
    analogWrite(motorPins[motorNum],map(speed,0,1000,MIN_PMW,MAX_PMW));
#else
    servos[motorNum].writeMicroseconds(1000 + speed);
#endif
}

ESC::ESC(VMExecutor *_executor) : executor(_executor), initTask(this) {
#if defined(TEENSYDUINO)
    analogWriteResolution(14);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motorPins[i], OUTPUT);
        analogWriteFrequency(motorPins[i], MOTOR_PWM_FREQ);
        setMotorDirect(i, 0);
    }
#else
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        servos[i].attach(motorPins[i]);
        setMotorDirect(i, 0);
    }
#endif
}

uint16_t ESC::getSpeed(uint8_t motorNum) {
    if (motorNum >= MOTOR_COUNT)
        return -1;
    return speeds[motorNum];
}

#ifdef ESC_TRACE
uint16_t esc_trace_count = 0;
#endif

void ESC::setMotorObjective(esc_objective_attr force) {
    if (force.pitch == 0 && force.roll == 0 && force.yaw == 0) {
        // Handle trivial case first (would break during scaling)
        setAllSpeeds(((uint32_t)force.throttle * THROTTLE_FACTOR) / 1000);
    } else {
        int32_t rawThrottles[MOTOR_COUNT] = {0};

        int32_t pitchOffset = (force.pitch * PITCH_FACTOR); // input expressed in the -1000 to +1000 then scaled by that input factor
        int32_t yawOffset = (force.yaw * YAW_FACTOR);       // The final results will be divided by 1000 to scale back down
        int32_t rollOffset = (force.roll * ROLL_FACTOR);

        /*
            pitch offset [-.5,-.5]
                         [ .5, .5]
        */
        rawThrottles[0] -= pitchOffset;
        rawThrottles[1] -= pitchOffset;
        rawThrottles[2] += pitchOffset;
        rawThrottles[3] += pitchOffset;

        /*
            yaw   offset [-.5, .5]
                         [-.5, .5]
        */
        rawThrottles[0] += yawOffset;
        rawThrottles[1] -= yawOffset;
        rawThrottles[2] += yawOffset;
        rawThrottles[3] -= yawOffset;

        /*
            roll  offset [-.5, .5]
                         [.5, -.5]
        */
        if (TL_CLOCKWISE == 1)
            rollOffset = -rollOffset;

        rawThrottles[0] += rollOffset;
        rawThrottles[1] -= rollOffset;
        rawThrottles[2] -= rollOffset;
        rawThrottles[3] += rollOffset;

        // The ratio of the target over the actual sum gives us a factor to rescale

        int32_t factorSum = (PITCH_FACTOR + ROLL_FACTOR + YAW_FACTOR);

        rawThrottles[0] = rawThrottles[0] / factorSum + ((uint32_t)force.throttle * THROTTLE_FACTOR) / 1000;
        rawThrottles[1] = rawThrottles[1] / factorSum + ((uint32_t)force.throttle * THROTTLE_FACTOR) / 1000;
        rawThrottles[2] = rawThrottles[2] / factorSum + ((uint32_t)force.throttle * THROTTLE_FACTOR) / 1000;
        rawThrottles[3] = rawThrottles[3] / factorSum + ((uint32_t)force.throttle * THROTTLE_FACTOR) / 1000;

        // TODO consider scaling total power output to be equivelent to throttle selection.
        //   Currently, in low throttle situations, strong esc objectives will create extra throttle
        //   This is probably good, gives command authority at low speed

        if (rawThrottles[0] < 0)
            rawThrottles[0] = 0;
        else if (rawThrottles[0] > 1000)
            rawThrottles[0] = 1000;
        if (rawThrottles[1] < 0)
            rawThrottles[1] = 0;
        else if (rawThrottles[1] > 1000)
            rawThrottles[1] = 1000;
        if (rawThrottles[2] < 0)
            rawThrottles[2] = 0;
        else if (rawThrottles[2] > 1000)
            rawThrottles[2] = 1000;
        if (rawThrottles[3] < 0)
            rawThrottles[3] = 0;
        else if (rawThrottles[3] > 1000)
            rawThrottles[3] = 1000;

        setMotorDirect(0, rawThrottles[0]);
        setMotorDirect(1, rawThrottles[1]);
        setMotorDirect(2, rawThrottles[2]);
        setMotorDirect(3, rawThrottles[3]);
    }

#ifdef ESC_TRACE
    if (esc_trace_count % ESC_TRACE_EVERY == 0) {
        FDOS_LOG.printf("ESC T:%i Y:%i P:%i R:%i -> \n [%4i,%4i]\n[%4i,%4i]\n", force.throttle, force.yaw, force.pitch, force.roll, getSpeed(0), getSpeed(1),
                        getSpeed(2), getSpeed(3));
    }
    esc_trace_count++;
#endif
}