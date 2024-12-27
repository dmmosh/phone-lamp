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
uint8_t led = OFF; //led_
TaskHandle_t flash_led_task = NULL;


void flash_led(void* args){
    bool curr_flash = LOW;
    while(1){
        switch(led){
            case FLASH:
                curr_flash = (curr_flash) ? LOW : HIGH;
                digitalWrite(LED,curr_flash);
            break;
            case ON:
                digitalWrite(LED,HIGH);
            break;
            case OFF:
                digitalWrite(LED,LOW);
            break;

            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
}


// BLE Server callback class
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param)
    {
        deviceConnected = true;

        std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(true);
        for(const auto& pair: devices){
            Serial.println(((BLEClient*)pair.second.peer_device)->toString().c_str());
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
    xTaskCreate(flash_led, "led flash", configMINIMAL_STACK_SIZE, NULL, 1, &flash_led_task);
    led = FLASH;
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
    //Serial.println(mac);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    led = ON;

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