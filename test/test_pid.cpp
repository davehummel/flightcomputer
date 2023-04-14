#include "FDOS_LOG.h"
#include <Arduino.h>
#include <RadioLib.h>

#include "VMExecutor.h"
#include "VMTime.h"


#include <PID.h>
#include <unity.h>

pid_state_t state;
PID pid = PID(1, 1, .5, PID_YAW_MAXOUT, PID_YAW_INPERIOD, 10000, 3);

void setUp(void) {}
void tearDown(void) { state.print(&FDOS_LOG, pid); }

void test_PID_basic(void) {
    pid.reset();
    TEST_ASSERT_EQUAL(0, pid.apply(0, 0, 10000, &state));
        FDOS_LOG.print(">");
    state.print(&FDOS_LOG, pid);
    TEST_ASSERT_EQUAL(10, pid.apply(0, 10, 10000, &state));
    FDOS_LOG.print(">");
    state.print(&FDOS_LOG, pid);
    TEST_ASSERT_EQUAL(11, pid.apply(0, 10, 10000, &state));
    TEST_ASSERT_EQUAL(12, pid.apply(0, 10, 10000, &state));
    TEST_ASSERT_EQUAL(10, pid.apply(1, 10, 10000, &state));
    TEST_ASSERT_EQUAL(6, pid.apply(2, 10, 10000, &state));
    TEST_ASSERT_EQUAL(-18, pid.apply(10, 10, 10000, &state));
    TEST_ASSERT_EQUAL(-16, pid.apply(10, 10, 10000, &state));
    TEST_ASSERT_EQUAL(1, pid.apply(11, 10, 10000, &state));
    TEST_ASSERT_EQUAL(4, pid.apply(10, 10, 10000, &state));
    TEST_ASSERT_EQUAL(6, pid.apply(10, 10, 10000, &state));
    TEST_ASSERT_EQUAL(4, pid.apply(10, 10, 10000, &state));
}

void test_PID_Periodic(void) {
    pid.reset();
    TEST_ASSERT_EQUAL(-1, pid.apply(0, 63855, 10000, &state));
        FDOS_LOG.print(">");
    state.print(&FDOS_LOG, pid);
    pid.reset();
    TEST_ASSERT_EQUAL(1, pid.apply(0, 65536, 10000, &state));
        FDOS_LOG.print(">");
    state.print(&FDOS_LOG, pid);
    pid.reset();
    TEST_ASSERT_EQUAL(1, pid.apply(0, 1, 10000, &state));
        FDOS_LOG.print(">");
    state.print(&FDOS_LOG, pid);
    pid.reset();
    TEST_ASSERT_EQUAL(-1, pid.apply(0, -1, 10000, &state));
        FDOS_LOG.print(">");
    state.print(&FDOS_LOG, pid);
}

int main(int argc, char **argv) {

    UNITY_BEGIN();
    RUN_TEST(test_PID_basic);
    RUN_TEST(test_PID_Periodic);
    UNITY_END();

    return 0;
}
