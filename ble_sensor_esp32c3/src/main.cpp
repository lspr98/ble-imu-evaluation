#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_Sensor_Calibration.h>

#include <IMUSensors.h>
#include <globals.h>
#include <BLESensorServer.h>

BLESensorServer* server;
IMUSensors* sensors;

float* buffer;


void setup() {
  Serial.begin(9600);
  delay(1000);

  buffer = new float[12];

  // Start BLE server
  server = new BLESensorServer();
  // Connect to sensors
  sensors = new IMUSensors();
  sensors->setup_sensors();
  sensors->setup_filter();
}

void loop() {
  sensors->read_sensors(server->dataBuffer);
  if(server->isConnected) {
    server->updateData();
  } else {
    Serial.println("Waiting for connection...");
    delay(1000);
  }
}