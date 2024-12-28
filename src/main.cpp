#include <Arduino.h>


// // BLE LIBRARIES / DATA 
// #include <BLEDevice.h>
// #include <BLEScan.h>
// #include <BLEAdvertisedDevice.h>
// #include <BLEClient.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLEAdvertising.h>
// #define SERVICE_UUID "9f46b94c-9574-4f6c-bd1b-ddc3a7a83a43"
// #define CHARACTERISTIC_UUID "afe8ef56-902f-4b38-a6a2-0eade0aca572"
// bool deviceConnected = false;
// BLEScan *scan;


// BLUETOOTH CLASSIC LIBRARIES / DATA
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2

// Variable to track connection status
String mac;
uint8_t curr_state = OFF; //led_
TaskHandle_t flash_led_task = NULL;


void flash_led(void* args){
    while(curr_state == FLASH){
        digitalWrite(LED,HIGH);
        vTaskDelay(500/portTICK_PERIOD_MS);
        digitalWrite(LED,LOW);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void led(const uint8_t new_state){  
    // state: new state
    // 
    if(new_state == FLASH && curr_state != FLASH){ 
        xTaskCreate(flash_led, "led flash", configMINIMAL_STACK_SIZE, NULL, 1, &flash_led_task);
    } else if (new_state == ON && curr_state != ON){
        if(curr_state == FLASH){
            vTaskDelete(flash_led_task);
        }
        digitalWrite(LED,HIGH);
        
    } else if (new_state == OFF && curr_state != OFF){
        if(curr_state == FLASH){
            vTaskDelete(flash_led_task);
        }
        digitalWrite(LED,LOW);
    }
    curr_state = new_state;
}

// BLE Server callback class
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param)
    {   
        deviceConnected = true;
        // std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(false);
        // for(const auto& pair: devices){
        //     Serial.println((int)((BLEClient*)pair.second.peer_device)->getConnId());
        //     Serial.println(((BLEClient*)pair.second.peer_device)->getPeerAddress().toString().c_str());
        // }

        for (int8_t i = 0; i < 5; i++)
        {
            Serial.printf("%.2x:", param->connect.remote_bda[i]);
        }
        Serial.printf("%.2x\n", param->connect.remote_bda[5]);

        
        //mac = ((BLEDevice*)pServer->getPeerDevices(false)[0].peer_device)->getAddress().toString().c_str();
        
        Serial.println("Device connected.");
    }


    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("Device disconnected.");
    }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Found device: ");
        Serial.printf("%s\n", advertisedDevice.getAddress().toString().c_str());
        
    }
};


void setup()
{
    Serial.begin(115200);
    pinMode(LED, OUTPUT);

    SerialBT.begin("Phone Lamp");


    led(FLASH);
    while(!SerialBT.connected()){
        Serial.println("Waiting for connection to begin...");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    Serial.println(SerialBT.getBtAddressString());

    /*
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
    pAdvertising->setAppearance(0x004C);
    BLEDevice::startAdvertising();
    led(FLASH);
    while (!deviceConnected){
        Serial.printf("Waiting for device to connect... %s\n", BLEDevice::getAddress().toString().c_str());
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    pServer->getPeerDevices(true);
    

    //Serial.println(pService->getUUID().toString().c_str());

    // while(1){
    //     vTaskDelay(1000/portTICK_PERIOD_MS);
    // };

    BLEDevice::deinit();

    BLEDevice::init("");
    //Serial.println(mac);
    led(ON);
    scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    scan->setActiveScan(true);  // Enable active scan for better results
    scan->setInterval(100);  // Scan interval
    scan->setWindow(99);     // Scan window
    //scan->start(3, false);   // Start scanning for 5 seconds (non-blocking)
    */
}



void loop()
{

    // BLEScanResults results = scan->start(3);
    // for (size_t i = 0; i < results.getCount(); i++)
    // {
    //     BLEAdvertisedDevice device = results.getDevice(i);
    //     // if(!strcmp(mac.c_str(), device.getAddress().toString().c_str())){
    //     //     Serial.printf("PHONE FOUND STRENGTH: %i\n", 100+device.getRSSI());
    //     // }
    //     //Serial.println(device.getManufacturerData().c_str());
    //     //Serial.println(device.getServiceData().c_str());
    //     Serial.println("---------");
    //     Serial.printf("%i %s %s %s\n", device.getRSSI(),device.getName().c_str(), device.getAddress().toString().c_str(), device.getServiceUUID().toString().c_str());
    // }

}