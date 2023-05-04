#include <IMUSensors.h>
#include <Adafruit_BNO055.h>

IMUSensors::IMUSensors() {
    bno055 = Adafruit_BNO055(55, BNO055_ADDR);
    bno08x = Adafruit_BNO08x(-1);
    lsm6dso32 = Adafruit_LSM6DSO32();
    lis2mdl = Adafruit_LIS2MDL();
};

void IMUSensors::setup_sensors() {
    Wire.setClock(I2C_CLK);
    //-----------------------------------------
    // BNO08x Sensor setup
    while (true) {
      Serial.println("Searching BNO08X...");
      if(bno08x.begin_I2C()) {
        break;
      } else {
        delay(500);
      };
    }
    // Enable reports, every 10000us (=10ms, 100Hz)
    // SH2_GAME_ROTATION_VECTOR is a quaternion
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    delay(100);
    Serial.println("Sensor found.");
    //-----------------------------------------


    //-----------------------------------------
    // BNO055 Sensor setup
    while (true) {
      Serial.println("Searching BNO055...");
      if(bno055.begin()) {
        break;
      } else {
        delay(500);
      };
    }
    Serial.println("Sensor found.");
    //-----------------------------------------


    //-----------------------------------------
    // LSM6DSO32 Sensor setup
    while (true) {
      Serial.println("Searching LSM6DSO32...");
      if(lsm6dso32.begin_I2C(LSM6DSO32_ADDR)) {
        delay(100);
        break;
      } else {
        delay(500);
      };
    }
    lsm6dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
    lsm6dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6dso32.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6dso32.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("Sensor found.");
    //-----------------------------------------


    //-----------------------------------------
    // LIS2MDL Sensor setup
    while (true) {
      Serial.println("Searching LIS2MDL...");
      if(lis2mdl.begin(LIS2MDL_ADDR)) {
        delay(100);
        break;
      } else {
        delay(500);
      };
    }
    lis2mdl.setDataRate(LIS2MDL_RATE_100_HZ);
    lis2mdl.enableAutoRange(false);
    Serial.println("Sensor found.");
    //-----------------------------------------


    //-----------------------------------------
    // Calibration setup
    if(!cal.begin()) {
      Serial.println("Could not initiate calibration");
    }
};

void IMUSensors::setup_filter() {
  // Convert filter time difference into herz
  float filter_frequency = 1000/FILTER_TIME;
  fusion_filter.begin(filter_frequency);
  fusion_filter.setBeta(FILTER_GAIN);
  t_last = millis();
}

void IMUSensors::read_sensors(float* buffer) {
    //-----------------------------------------
    // Read BNO08x
    if( bno08x.getSensorEvent(&bno08xSensorData)) {
      switch (bno08xSensorData.sensorId)
      {
        case SH2_GAME_ROTATION_VECTOR:
          buffer[0] = bno08xSensorData.un.gameRotationVector.real;
          buffer[1] = bno08xSensorData.un.gameRotationVector.i;
          buffer[2] = bno08xSensorData.un.gameRotationVector.j;
          buffer[3] = bno08xSensorData.un.gameRotationVector.k;
          break;
        
        default:
          break;
      }
    }
    //-----------------------------------------


    //-----------------------------------------
    // Read BNO055
    bno055SensorData = bno055.getQuat();
    buffer[4] = bno055SensorData.w();
    buffer[5] = bno055SensorData.x();
    buffer[6] = bno055SensorData.y();
    buffer[7] = bno055SensorData.z();

    // bno055_hrp = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
    // // Note swapped index to conver HRP to HPR
    // buffer[4] = bno055_hrp[0];
    // buffer[5] = bno055_hrp[1];
    // buffer[6] = bno055_hrp[2];
    
    //-----------------------------------------


    //-----------------------------------------
    // Read LIS2MDL and LSM6DSO32 and fuse readings into quaternion
    lis2mdl.getEvent(&LIS_mag);
    lsm6dso32.getEvent(&LSM_accel, &LSM_gyro, &LSM_temp);
    // Adjust readings according to stored calibration data
    cal.calibrate(LIS_mag);
    cal.calibrate(LSM_accel);
    cal.calibrate(LSM_gyro);

    t_curr = millis();
    t_diff = t_curr - t_last;
    t_last = t_curr;
    float t_diff_s = ((float)t_diff)*0.001;

    // IMPORTANT: Swap X and Y axes for magnetometer
    fusion_filter.updateIMU(
      LSM_gyro.gyro.x * SENSORS_RADS_TO_DPS,
      LSM_gyro.gyro.y * SENSORS_RADS_TO_DPS,
      LSM_gyro.gyro.z * SENSORS_RADS_TO_DPS,
      LSM_accel.acceleration.x,
      LSM_accel.acceleration.y,
      LSM_accel.acceleration.z,
      t_diff_s
    );
    // fusion_filter.update(
    //   LSM_gyro.gyro.x * SENSORS_RADS_TO_DPS,
    //   LSM_gyro.gyro.y * SENSORS_RADS_TO_DPS,
    //   LSM_gyro.gyro.z * SENSORS_RADS_TO_DPS,
    //   LSM_accel.acceleration.x,
    //   LSM_accel.acceleration.y,
    //   LSM_accel.acceleration.z,
    //   -LIS_mag.magnetic.y,
    //   -LIS_mag.magnetic.x,
    //   -LIS_mag.magnetic.z
    // );
    // buffer[8] = fusion_filter.getYaw();
    // buffer[9] = -fusion_filter.getPitch();
    // buffer[10] = fusion_filter.getRoll();
    fusion_filter.getQuaternion(buffer+8, buffer+9, buffer+10, buffer+11);
    //-----------------------------------------
};