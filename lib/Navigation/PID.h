
#ifndef PID_H__
#define PID_H__

#include <stdint.h>

struct pid_state_t {
    int32_t pContribution, iContribution, dContribution, output;
    int32_t state;
    int32_t target;
    int32_t integral;
    int32_t derivative;
};

class PID {
  public:
    PID(double kP=1, double kI=1, double kD=1, int32_t maxOutput = 127, int32_t period = 0, int32_t maxIntegralComponent = 64)
        : kP(kP), kI(kI), kD(kD), maxOutput(maxOutput), period(period), maxIntegral(maxIntegralComponent/kI) {}


    int32_t apply(int32_t input, int32_t target, int32_t intervalMicros, pid_state_t *sample = 0);
    
    void reset() {
        integral = 0;
        derivative = 0;
        prevState = 0;
        firstApply = true;
    }

  private:
    int32_t convertPeriodicError(int32_t e, int32_t p) { return ((e + (p - 1) / 2) % p + p) % p - (p - 1) / 2; }

    bool firstApply = true;
     double kP, kI, kD;
     int32_t maxOutput, period;
     int32_t maxIntegral;
    int32_t integral;
    int32_t prevState;
    int32_t derivative;
};

#endif