#include <PID.h>

int32_t PID::apply(int32_t state, int32_t target, int32_t intervalMillis, pid_state_t *sample) {

    if (firstApply) {
        prevState = state;
        integral = 0;
        firstApply = false;
    }

    int32_t observedError = target - state;

    if (period != 0) {
        observedError = convertPeriodicError(observedError, period);
    }

    int32_t pContribution, dContribution, iContribution;

    // running average with weight on history values
    int32_t observedChange = prevState - state;
    if (period != 0) {
        observedChange = convertPeriodicError(observedChange, period);
    }

    derivative = 7 * (derivative + ((1000 * observedChange) / intervalMillis) / 7) / 8;

    int32_t output =
        (pContribution = (kP * observedError)) + (dContribution = (kD * derivative)) + (iContribution = (kI * (intervalMillis * integral) / 1000l));

    if (output > maxOutput) {
        output = maxOutput;
    } else if (output < -maxOutput) {
        output = -maxOutput;
    }
    integral += observedError;

    if (integral > maxIntegral)
        integral = maxIntegral;
    else if (integral < -maxIntegral)
        integral = -maxIntegral;

    if (sample != 0) {
        sample->target = target;
        sample->pContribution = pContribution;
        sample->dContribution = dContribution;
        sample->iContribution = iContribution;
        sample->state = state;
        sample->integral = integral;
        sample->derivative = derivative;
        sample->output = output;
    }

    prevState = state;

    return output;
}
