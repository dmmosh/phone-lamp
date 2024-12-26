#include <Arduino.h>
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "BLEDevice.h"

#define SERVICE_UUID "9f46b94c-9574-4f6c-bd1b-ddc3a7a83a43"
#define CHARACTERISTIC_UUID "afe8ef56-902f-4b38-a6a2-0eade0aca572"
#define LED 2

// Variable to track connection status
bool deviceConnected = false;
String mac;

// BLE Server callback class
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param)
    {
        deviceConnected = true;
        mac = ((BLEDevice*)pServer->getPeerDevices(false)[0].peer_device)->getAddress().toString().c_str();
        
        Serial.println("Device connected.");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("Device disconnected.");
    }
};

void setup()
{
    Serial.begin(115200);
    BLEDevice::init("Phone Lamp");

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // pServer->getAdvertising()->setScanFilter(1,0);
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();

    while (!deviceConnected){
        Serial.println("Waiting for device to connect...");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

    BLEDevice::deinit();
    Serial.print(mac);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    BLEDevice::init("");
}

void loop()
{
    BLEScan *scan = BLEDevice::getScan();
    scan->setActiveScan(true);
    BLEScanResults results = scan->start(1);
    bool other_esp = false;
    for (size_t i = 0; i < results.getCount(); i++)
    {
        BLEAdvertisedDevice device = results.getDevice(i);
        if(!strcmp(mac.c_str(), device.getAddress().toString().c_str())){
            Serial.printf("PHONE FOUND STRENGTH: %i\n", 100+device.getRSSI());
        }

        Serial.printf("%i %s %s\n", device.getRSSI(),device.getName().c_str(), device.getAddress().toString().c_str());
    }
}
