#include <Arduino.h>
#include <FDOS_LOG.h>
#include <Navigation.h>

void GenericNavTask::recordInput(int8_t _joy1H, int8_t _joy1V, uint8_t _joy2H, uint8_t _joy2V) {
    joy1H = _joy1H;
    joy1V = _joy1V;
    joy2H = _joy2H;
    joy2V = _joy2V;
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
    if (connected == escConnected) return;
    escConnected = connected;
    FDOS_LOG.println("Calling activateESCEvent");
    activateESCEvent(connected);
}
