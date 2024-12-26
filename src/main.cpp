#include <Arduino.h>
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "BLEDevice.h"
#include "BluetoothSerial.h"

#define LED 2



void setup()
{
    Serial.begin(115200);

    SerialBT.begin("ESP32_BT_Scanner");  // Start Bluetooth with a device name

}


BluetoothSerial SerialBT; // Bluetooth serial object

void loop()
{
    


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
