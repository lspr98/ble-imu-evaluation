#ifndef IMU_SENSORS_H
#define IMU_SENSORS_H

#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <globals.h>

class IMUSensors {

private:
    sh2_SensorValue_t bno08xSensorData;
    imu::Quaternion bno055SensorData;
    sensors_event_t LSM_accel, LSM_gyro, LSM_temp, LIS_mag;
    imu::Vector<3> bno055_hrp;

    // Use calibration data for LSM and LIS sensors stored in EEPROM
    #if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
        Adafruit_Sensor_Calibration_EEPROM cal;
    #else
        Adafruit_Sensor_Calibration_SDFat cal;
    #endif

public:
    Adafruit_BNO055 bno055;
    Adafruit_BNO08x bno08x;
    Adafruit_LSM6DSO32 lsm6dso32;
    Adafruit_LIS2MDL lis2mdl;
    Adafruit_Madgwick fusion_filter;
    long t_last, t_curr, t_diff;

    IMUSensors();

    void setup_sensors();
    void setup_filter();
    void read_sensors(float* buffer);
};

#endif