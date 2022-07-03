
#ifndef __ESC_H__
#define __ESC_H__

#include "Arduino.h"
#include "FDOS_LOG.h"
#include "VMExecutor.h"

#if defined(TEENSYDUINO)
#else
#include "Servo.h"
#endif

struct esc_objective_attr {
    int16_t roll;  // -1000 max roll CCW, 1000 max roll CW
    int16_t pitch; // -1000 max pitch down, 1000 max pitch up
    int16_t yaw;   // -1000 max yaw left, 1000 max yaw right
    uint16_t throttle; // 0 no throttle, 1000 max throttle
};

class ESC {
    /**
     * @brief ESC for four motors with an assumed layout of
     *                            0 CC 1 CW      0 CW 1 CC
     *                            2 CW 3 CC  or  2 CC 3 CW
     *   MOTOR_CONFIG_TL_CLOCKWISE    0              1
     *
     */
  private:
    Servo servos[MOTOR_COUNT];
    class InitMotorTask : public RunnableTask {

        uint8_t initStep = 0;
        ESC *parent;

      public:
        InitMotorTask(ESC *_parent) : parent(_parent) {}

        void run(TIME_INT_t time);

        TIME_INT_t getNextInterval(TIME_INT_t lastInterval);
    };

    VMExecutor *executor;

    InitMotorTask initTask;

    uint8_t motorPins[MOTOR_COUNT] = MOTOR_PINS_FROM_BACK_TL_TR_BL_BR;

    uint16_t speeds[MOTOR_COUNT] = {0};

    void setAllSpeeds(uint16_t speed);

  public:
    uint16_t THROTTLE_FACTOR = MOTOR_THROTTLE_FACTOR;
    uint8_t PITCH_FACTOR = MOTOR_PITCH_FACTOR;
    uint8_t YAW_FACTOR = MOTOR_YAW_FACTOR;
    uint8_t ROLL_FACTOR = MOTOR_ROLL_FACTOR;
    bool TL_CLOCKWISE = MOTOR_CONFIG_TL_CLOCKWISE;

    void initMotors();

    void setMotorDirect(uint8_t motorNum, uint16_t speed);

    void setMotorObjective(esc_objective_attr force);

    ESC(VMExecutor *_executor);

    uint16_t getSpeed(uint8_t motorNum);
};
#endif