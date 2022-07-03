#include <TestNav.h>

esc_objective_attr DirectInputNav::nextFrame(TIME_INT_t intervalMicros) {
    esc_objective_attr objective;
    objective.pitch = joyV * 1000 / 128;
    objective.yaw = joyH * 1000 / 128;
    if (slideH > 0) {
        objective.roll = slideH * 2000 / 255 - 1000;
    } else {
        objective.roll = 0;
    }
    objective.throttle = slideV*1000/255;
    FDOS_LOG.printf("P:%i Y:%i R:%i T:%i\n",objective.pitch,objective.yaw,objective.roll,objective.throttle);
    return objective;
}
