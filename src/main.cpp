#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

#define SERVICE_UUID "9f46b94c-9574-4f6c-bd1b-ddc3a7a83a43"
#define CHARACTERISTIC_UUID "afe8ef56-902f-4b38-a6a2-0eade0aca572"
#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2

// Variable to track connection status
bool deviceConnected = false;
String mac;
BLEScan *scan;
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

        std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(true);
        for(const auto& pair: devices){
            Serial.println((int)((BLEClient*)pair.second.peer_device)->getConnId());
            Serial.println((int)((BLEClient*)pair.second.peer_device)->getRssi());
            Serial.println(((BLEClient*)pair.second.peer_device)->getPeerAddress().toString().c_str());
        }
        //mac = ((BLEDevice*)pServer->getPeerDevices(false)[0].peer_device)->getAddress().toString().c_str();
        
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
    pinMode(LED, OUTPUT);
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
    led(FLASH);
    while (!deviceConnected){
        Serial.println("Waiting for device to connect...");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

    pServer->getPeerDevices(true);
    BLEDevice::deinit();
    //Serial.println(mac);
    led(ON);
    scan = BLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    BLEDevice::init("");
}



void loop()
{
    BLEScanResults results = scan->start(1);
    for (size_t i = 0; i < results.getCount(); i++)
    {
        BLEAdvertisedDevice device = results.getDevice(i);
        if(!strcmp(mac.c_str(), device.getAddress().toString().c_str())){
            Serial.printf("PHONE FOUND STRENGTH: %i\n", 100+device.getRSSI());
        }

        Serial.printf("%i %s %s %s\n", device.getRSSI(),device.getName().c_str(), device.getAddress().toString().c_str(), device.getServiceUUID().toString().c_str());
    }

}