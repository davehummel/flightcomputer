#include "MotionTask.h"

#include "I2Cdev.h"
#include "IMU.h"
#include "Sensor_cal.h"
#include "Types.h"
#include "USFSMAX.h"
#include <FDOS_LOG.h>

float sensor_point[3];
int16_t gyroADC[2][3];
int16_t accADC[2][3];
int16_t magADC[2][3];
int32_t baroADC[2];

// Timing variables
uint32_t Begin;
uint32_t Acq_time;
uint32_t Start_time = 0;
float TimeStamp = 0.0f;
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint8_t serial_input = 0;
uint32_t last_refresh = 0;
uint32_t delt_t = 0;
uint32_t cycleTime = 0; // Main loop time (us)

// Interrupt/state flags
volatile uint8_t data_ready[2] = {1, 1};
volatile uint16_t calibratingG[2] = {0, 0};

// Calibration-related variables
full_adv_cal_t gyrocal[2];
full_adv_cal_t ellipsoid_magcal[2];
full_adv_cal_t accelcal[2];
full_adv_cal_t final_magcal[2];
uint8_t GyroCal_buff[sizeof(full_adv_cal_t)];
uint8_t EllipMagCal_buff[sizeof(full_adv_cal_t)];
uint8_t AccelCal_buff[sizeof(full_adv_cal_t)];
uint8_t FineMagCal_buff[sizeof(full_adv_cal_t)];
float mag_calData[2][3];

// float                                   Mv_Cal        = 0.0f;
// float                                   Mh_Cal        = 0.0f;
// float                                   M_Cal         = 0.0f;
// float                                   Del_Cal       = 0.0f;
uint8_t cube_face = 0;
uint8_t face_rotation = 0;

// USFSMAX-related variables
CoProcessorConfig_t Cfg[2];
uint8_t algostatus[2];
uint8_t eventStatus[2];
int16_t QT_Timestamp[2]; // USFSMAX Quaternion timestamps
uint8_t cfg_buff[sizeof(CoProcessorConfig_t)];
uint8_t EulerQuatFlag = OUTPUT_EULER_ANGLES;
uint8_t ScaledSensorDataFlag = SCALED_SENSOR_DATA;
uint8_t cal_status[2] = {0, 0};
uint8_t gyroCalActive[2] = {0, 0};
uint8_t Quat_flag[2] = {0, 0}; // USFSMAX data ready flags
uint8_t Gyro_flag[2] = {0, 0};
uint8_t Acc_flag[2] = {0, 0};
uint8_t Mag_flag[2] = {0, 0};
uint8_t Baro_flag[2] = {0, 0};
float Rsq = 0.0f;

// IMU-related variables
int16_t accLIN[2][3]; // USFSMAX linear acceleration
int16_t grav[2][3];
float acc_LIN[2][3]; // 9DOF linear acceleration
float Mx[2], My[2];  // Tilt-corrected horizontal magnetic components
float gyroData[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float accData[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float magData[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float qt[2][4] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}; // USFSMAX quaternions
// float QT[2][4] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}; // 9DOF quaternions
float angle[2][2] = {0.0f, 0.0f, 0.0f, 0.0f};                      // USFSMAX P,R Euler angles
// float ANGLE[2][2] = {0.0f, 0.0f, 0.0f, 0.0f};                      // 9DOF P,R Euler angles
float heading[2] = {0.0f, 0.0f};                                   // USFSMAX Heading Euler angle
// float HEADING[2] = {0.0f, 0.0f};                                   // 9DOF Heading Euler angle
                                                                   // Madgewick filter (RPS/s)
                                                                   // Madgewick fliter Compute zeta (Small or zero)
                                                                   // Mahony filter 2X integral gain (Ki)
float integralFBx = 0.0f;                                          // Mahony filter integral error terms
float integralFBy = 0.0f;
float integralFBz = 0.0f;

I2Cdev i2c_0(&USFSMAX_WIRE_INSTANCE);
USFSMAX USFSMAX_0(&i2c_0, 0);
IMU imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Intermediate data handling variables

// Host DRDY interrupt handler
void DRDY_handler_0() { data_ready[0] = 1; }

void MotionTask::initSensors() {
    pinMode(USFSMAX_INT_PIN, INPUT);
    USFSMAX_WIRE_INSTANCE.begin();
    USFSMAX_WIRE_INSTANCE.setClock(100000);
    USFSMAX_0.init_USFSMAX();
    USFSMAX_WIRE_INSTANCE.setClock(USFSMAX_I2C_CLOCK);
    attachInterrupt(USFSMAX_INT_PIN, DRDY_handler_0, RISING);
}

void MotionTask::calibrateGyro() { sensor_cal.GyroCal(); }

void MotionTask::fetchOrientationData() { 
    imu_0.computeIMU(); 
    yaw = heading[0];
    pitch = angle[0][1];
    roll = angle[0][0];
}

void MotionTask::fetchBarometerData() {
    USFSMAX_0.BARO_getADC();
    pressureHPA = ((float)baroADC[0]) / 4096.0f;
}

uint8_t x = 0;

void MotionTask::run(TIME_INT_t time) {
    fetchOrientationData();
    if ((barometerReady = (!barometerReady)) == true) {
        fetchBarometerData();
    }
    x++;
    if (x == 100) {
        // Euler angles
        Serial.print("USFSMAX Yaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);
    }

    if (x == 100) {
        Serial.print("Baro pressure = ");
        Serial.print(pressureHPA);
        Serial.println(" hPa");
    }

    if (x == 100)
        x = 0;
}
