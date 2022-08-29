// #include <Arduino.h>
// #include <unity.h>

// #include "FDOS_LOG.h"
// #include "Navigation.h"

// Logger FDOS_LOG(&Serial);

// // void setUp(void) {
// // // set stuff up here
// // }

// // void tearDown(void) {
// // // clean stuff up here
// // }

// void test_basic_throttle(void) {
//     ESC esc(NULL);
//     esc.THROTTLE_FACTOR = 255;
//     esc_objective_attr force = {0, 0, 0, 200};
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(3));
//     force.throttle = 0;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(3));
//     force.throttle = 255;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(255, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(255, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(255, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(255, esc.getSpeed(3));

//     esc.THROTTLE_FACTOR = 200;

//     force.throttle = 200;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(156, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(156, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(156, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(156, esc.getSpeed(3));
//     force.throttle = 0;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(3));
//     force.throttle = 255;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(200, esc.getSpeed(3));
// }

// void test_component_force(void) {
//     ESC esc(NULL);
//     esc.THROTTLE_FACTOR = 255;
//     esc.PITCH_FACTOR = 3;
//     esc.ROLL_FACTOR = 1;
//     esc.YAW_FACTOR = 2;
//     esc_objective_attr force = {0, 100, 0, 100};
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(227, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(227, esc.getSpeed(3));
//     force.throttle = 0;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(127, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(127, esc.getSpeed(3));
//     force.throttle = 100;
//     force.pitch=-10;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(112, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(112, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(88, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(88, esc.getSpeed(3));

//     force.pitch = 0;
//     force.yaw = 100;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(185, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(15, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(185, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(15, esc.getSpeed(3));
//     force.throttle = 0;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(85, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(85, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(3));
//     force.throttle = 100;
//     force.yaw=-10;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(92, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(108, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(92, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(108, esc.getSpeed(3));

//     force.pitch = 0;
//     force.yaw = 0;
//     force.roll = 100;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(142, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(58, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(58, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(142, esc.getSpeed(3));
//     force.throttle = 0;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(42, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(42, esc.getSpeed(3));
//     force.throttle = 100;
//     force.roll=-10;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(96, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(104, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(104, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(96, esc.getSpeed(3));
//     esc.ROLL_CONFIG = 1;
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(104, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(96, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(96, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(104, esc.getSpeed(3));
// }

// void test_combined_force(void) {
//     ESC esc(NULL);
//     esc.THROTTLE_FACTOR = 255;
//     esc.PITCH_FACTOR = 3;
//     esc.ROLL_FACTOR = 1;
//     esc.YAW_FACTOR = 2;
//     esc_objective_attr force = {10, 10, 10, 100};
//     esc.setMotorObjective(force);
//     TEST_ASSERT_EQUAL(100, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(75, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(117, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(108, esc.getSpeed(3));

//     esc_objective_attr force2 = {100, 100, 100, 100};
//     esc.setMotorObjective(force2);
//     TEST_ASSERT_EQUAL(100, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(255, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(185, esc.getSpeed(3));

//     esc_objective_attr force3 = {-100, -100, -100, 0};
//     esc.setMotorObjective(force3);
//     TEST_ASSERT_EQUAL(100, esc.getSpeed(0));
//     TEST_ASSERT_EQUAL(0, esc.getSpeed(1));
//     TEST_ASSERT_EQUAL(255, esc.getSpeed(2));
//     TEST_ASSERT_EQUAL(185, esc.getSpeed(3));
   
// }

// void setup() {
//     // NOTE!!! Wait for >2 secs
//     // if board doesn't support software reset via Serial.DTR/RTS
//     delay(2000);

//     UNITY_BEGIN();
//     RUN_TEST(test_basic_throttle);
//     RUN_TEST(test_component_force);
//     RUN_TEST(test_combined_force);
//     UNITY_END();
// }

// void loop() {
//     digitalWrite(13, HIGH);
//     delay(100);
//     digitalWrite(13, LOW);
//     delay(500);
// }
