#include <Arduino.h>
#include <PID.h>
#include "FDOS_LOG.h"
#include <unity.h>

Logger FDOS_LOG(&Serial);


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_PID_basic(void) {
    PID pid = PID(1,1,.5,127,256);
    TEST_ASSERT_EQUAL(0, pid.apply(0,0,100));
    TEST_ASSERT_EQUAL(10, pid.apply(0,10,100));
    TEST_ASSERT_EQUAL(11, pid.apply(0,10,100));
    TEST_ASSERT_EQUAL(12, pid.apply(0,10,100));
    TEST_ASSERT_EQUAL(7, pid.apply(1,10,100));
    TEST_ASSERT_EQUAL(6, pid.apply(2,10,100));
    TEST_ASSERT_EQUAL(-36, pid.apply(10,10,100));
    TEST_ASSERT_EQUAL(4, pid.apply(10,10,100));
    TEST_ASSERT_EQUAL(-2, pid.apply(11,10,100));
    TEST_ASSERT_EQUAL(9, pid.apply(10,10,100));
}



void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_PID_basic);
    UNITY_END();
}

void loop() {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}