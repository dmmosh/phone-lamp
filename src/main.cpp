#include <Arduino.h>
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "BLEDevice.h"
#include "BluetoothSerial.h"

#define SERVICE_UUID "9f46b94c-9574-4f6c-bd1b-ddc3a7a83a43"
#define CHARACTERISTIC_UUID "afe8ef56-902f-4b38-a6a2-0eade0aca572"
#define LED 2

// Variable to track connection status
bool deviceConnected = false;
String mac;
BLEScan *scan;

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
    Serial.println(mac);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    scan = BLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    BLEDevice::init("");
}


BluetoothSerial SerialBT; // Bluetooth serial object

void loop()
{
    // BLEScanResults results = scan->start(1);
    // for (size_t i = 0; i < results.getCount(); i++)
    // {
    //     BLEAdvertisedDevice device = results.getDevice(i);
    //     if(!strcmp(mac.c_str(), device.getAddress().toString().c_str())){
    //         Serial.printf("PHONE FOUND STRENGTH: %i\n", 100+device.getRSSI());
    //     }

    //     Serial.printf("%i %s %s %s\n", device.getRSSI(),device.getName().c_str(), device.getAddress().toString().c_str(), device.getServiceUUID().toString().c_str());
    // }


    int deviceCount = SerialBT.scanDevices(); // Start scanning for Bluetooth devices
  
  if (deviceCount == 0) {
    Serial.println("No devices found.");
  } else {
    Serial.println("Devices found:");
    for (int i = 0; i < deviceCount; i++) {
        Serial.printf("device mac: %s\n",SerialBT.getBtAddressString());
    }
  }
}
