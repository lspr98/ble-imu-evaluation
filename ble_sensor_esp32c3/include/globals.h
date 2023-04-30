#ifndef GLOBALS_H
#define GLOBALS_H

// I2C sensor adresses
#define BNO08X_ADDR 0x4A
#define BNO055_ADDR 0x28
#define LSM6DSO32_ADDR 0x6A
#define LIS2MDL_ADDR 0x1E

// service and characteristic UUID. Generated with https://www.uuidgenerator.net/
#define SERVICE_UUID                    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SENSOR_VAL_CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define ESP_COMMAND_CHARACTERISTIC_UUID "e64415c6-7354-497f-b3c4-1ac4b3cdf756"

// Fusion filter parameters
// Desired time difference between two consecutive filter updates in milliseconds
#define FILTER_TIME 10

// I2C bus clock in herz
#define I2C_CLK 400000

#endif