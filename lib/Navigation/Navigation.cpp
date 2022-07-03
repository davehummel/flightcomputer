#include <Arduino.h>
#include <FDOS_LOG.h>
#include <Navigation.h>

void GenericNavTask::recordInput(int8_t _joyH, int8_t _joyV, uint8_t _slideH, uint8_t _slideV) {
    joyH = _joyH;
    joyV = _joyV;
    slideH = _slideH;
    slideV = _slideV;
}

void GenericNavTask::run(TIME_INT_t time) {
    TIME_INT_t interval = lastFrameTime > 0 ? time - lastFrameTime : 0;
    setCurrentOrientation(motion->yaw, motion->pitch, motion->roll, motion->pressureHPA);
    output = nextFrame(interval);
    lastFrameTime = time;
    if (isESCConnected()) {
        esc->setMotorObjective(output);
    } else {
        // This overides the implemenetation of navs control on disconnect
        // setting motors to 0.  Should remove this if we expect some active
        // flight actions on connection loss.
        output.throttle = 0;
        output.pitch = 0;
        output.yaw = 0;
        output.roll = 0;
        esc->setMotorObjective(output);
    }
}

void GenericNavTask::connectESC(bool connected) {
    escConnected = connected;
    activateESCEvent();
}
