#include <BLESensorServer.h>
#include <globals.h>

BLESensorServer::BLESensorServer() {
    dataBuffer = new float [12];
    esp_base_mac_addr_set(ESP_MAC_ADDRESS);
    BLEDevice::init("ESP32");
    server = BLEDevice::createServer();
    server->setCallbacks(this);
    service = server->createService(SERVICE_UUID);
    // service->start();
    characteristic = service->createCharacteristic(
        SENSOR_VAL_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE | 
        BLECharacteristic::PROPERTY_INDICATE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    cmd_characteristic = service->createCharacteristic(
        ESP_COMMAND_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE
    );

    characteristic->addDescriptor(new BLE2902());
    cmd_characteristic->addDescriptor(new BLE2902());
    cmd_characteristic->setCallbacks(this);
    characteristic->setValue(std::string((char*)dataBuffer, 12*(sizeof(float))));
    service->start();
    advertisement = BLEDevice::getAdvertising();
    advertisement->addServiceUUID(SERVICE_UUID);
    advertisement->setScanResponse(false);
    advertisement->setMinPreferred(0x0);
    mac_address = BLEDevice::getAddress().toString();
    startAdvertising();

};

void BLESensorServer::onConnect(BLEServer* pServer) {
    Serial.println("Connected.");
    isConnected = true;
};

void BLESensorServer::onDisconnect(BLEServer* pServer) {
    Serial.println("Disconnected.");
    isConnected = false;
    startAdvertising();
};

void BLESensorServer::onWrite(BLECharacteristic* pCharacteristic) {
    if(pCharacteristic == cmd_characteristic) {
        Serial.print("Recieved command ");
        switch ((SERVER_CMD) *(cmd_characteristic->getData()))
        {
        case SERVER_CMD::SYSTEM_RESTART:
            Serial.println("SYSTEM_RESTART");
            delay(1000);
            ESP.restart();
            break;
        
        case SERVER_CMD::SENSOR_RESTART:
            Serial.println("SENSOR_RESTART");
            break;
        
        default:
            Serial.println("UNKNOWN");
            break;
        }
    }
}

void BLESensorServer::startAdvertising() {
    if(!isConnected) {
        Serial.println("Advertising...");
        BLEDevice::startAdvertising();
    }
};

void BLESensorServer::updateData() {
    characteristic->setValue(std::string((char*)dataBuffer, 12*(sizeof(float))));
    characteristic->notify();
}

void BLESensorServer::printInfo() {
    Serial.print("MAC-Address: ");
    Serial.println(mac_address.c_str());
};