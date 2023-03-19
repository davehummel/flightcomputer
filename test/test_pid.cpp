#include <Arduino.h>
#include "FDOS_LOG.h"
#include <RadioLib.h>

#include "VMExecutor.h"
#include "VMTime.h"


Logger FDOS_LOG(&Serial);

#include <unity.h>
#include <PID.h>

pid_state_t state;
PID  pid = PID(1, 1, .5, 127, 256, 64, 2);

void setUp(void) {
}
void tearDown(void) {
    state.print(&FDOS_LOG,pid);
}

void test_PID_basic(void) {
    TEST_ASSERT_EQUAL(0, pid.apply(0, 0, 100, &state));
    TEST_ASSERT_EQUAL(10, pid.apply(0, 10, 100, &state));
    TEST_ASSERT_EQUAL(11, pid.apply(0, 10, 100, &state));
    TEST_ASSERT_EQUAL(12, pid.apply(0, 10, 100, &state));
    TEST_ASSERT_EQUAL(10, pid.apply(1, 10, 100, &state));
    TEST_ASSERT_EQUAL(6, pid.apply(2, 10, 100, &state));
    TEST_ASSERT_EQUAL(-18, pid.apply(10, 10, 100, &state));
    TEST_ASSERT_EQUAL(-16, pid.apply(10, 10, 100, &state));
    TEST_ASSERT_EQUAL(1, pid.apply(11, 10, 100, &state));
    TEST_ASSERT_EQUAL(4, pid.apply(10, 10, 100, &state));
    TEST_ASSERT_EQUAL(6, pid.apply(10, 10, 100, &state));
    TEST_ASSERT_EQUAL(4, pid.apply(10, 10, 100, &state));
}

int main(int argc, char **argv) {

    UNITY_BEGIN();
    RUN_TEST(test_PID_basic);
    UNITY_END();

    return 0;
}
