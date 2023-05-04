#ifndef BLE_SENSOR_SERVER_H
#define BLE_SENSOR_SERVER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>

class BLESensorServer : public BLEServerCallbacks, public BLECharacteristicCallbacks {

enum SERVER_CMD {
    SYSTEM_RESTART = 0x00,
    SENSOR_RESTART = 0x01,
};

private:
    BLEServer *server;
    BLEService *service;
    BLECharacteristic *characteristic, *cmd_characteristic, *status_characteristic;
    BLEAdvertising *advertisement;
    std::string mac_address;

public:
    float* dataBuffer;
    bool isConnected = false;
    
    BLESensorServer();
    // Implementation of abstract functions for BLEServerCallbacks
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
    // Implementation of abstract functions for BLECharacteristicCallbacks
    void onWrite(BLECharacteristic* pCharacteristic);
    
    void startAdvertising();
    void updateData();
    void printInfo();

};

#endif