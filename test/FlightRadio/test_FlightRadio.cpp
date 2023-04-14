#include "FDOS_LOG.h"
#include <Arduino.h>
#include <RadioTask.h>

#include "VMExecutor.h"
#include "VMTime.h"

#include <FlightRadio.h>
#include <PID.h>
#include <unity.h>

pid_state_t state;

extern pid_state_t* TELEM_BUFFER;

void setUp(void) {}
void tearDown(void) {}

class MockNav : public TargetOrientationNav {
    public:
    MockNav() : TargetOrientationNav(NULL,NULL){
        
    }
    void setTelemSampleIndex(int32_t index){
        telemSampleIndex = index;
    }
} mockNav;

void test_onSendready_telemRequest(void) {
    uint8_t data[255];
    pid_request_telemetry_t request;

    request.index = 0;
    request.telemDimensionYPR = 0;

    msgToBytes(&request, data, sizeof(request));

    TEST_ASSERT_EQUAL(RADIO_MSG_ID::PID_TELEMETRY_REQUEST, data[0]);

    SustainConnectionAction sca;
    sca.setNavSource(&mockNav);


    sca.onReceive(sizeof(request), data, true);

    bool responseExpected = false;
    sca.onSendReady(data, responseExpected);

    TEST_ASSERT_EQUAL(false, responseExpected);
    TEST_ASSERT_EQUAL(data[0], RADIO_MSG_ID::PID_TELEMETRY_RESPONSE);

    pid_response_telemetry_t response;
    msgFromBytes(&response, data, sizeof(pid_response_telemetry_t));
    TEST_ASSERT_EQUAL(0, response.sampleStartIndex);
    TEST_ASSERT_EQUAL(0, response.totalTelemCount);
}

int main(int argc, char **argv) {

    UNITY_BEGIN();
    RUN_TEST(test_onSendready_telemRequest);
    UNITY_END();

    return 0;
}
