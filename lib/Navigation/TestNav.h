
#ifndef TEST_NAV_H__
#define TEST_NAV_H__

#include "Navigation.h"

class DirectInputNav : public GenericNavTask {
  public:
    DirectInputNav(ESC *_esc,  MotionTask *_motion) : GenericNavTask(_esc, _motion) {}
    esc_objective_attr nextFrame(TIME_INT_t intervalMicros);
};

#endif