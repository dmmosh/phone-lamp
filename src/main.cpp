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


#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"
#include <driver/adc.h>




#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2

// variables for the bluetooth server and hid device
BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;
BLEAdvertising *pAdvertising;
BLEServer *pServer;
BLEScan* pBLEScan;
char mac[18];


bool connected = false;


class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param){
    connected = true;
    Serial.println("Connected");
    BLE2902* desc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    
    for (int i = 0; i < 6; i++) {
        sprintf(((char*)mac)+2*i+i,"%.2x", param->connect.remote_bda[i]); // save to stack string
        mac[2*i+i+2] = ':'; // dont worry about last char - gets replaced by null termination char
        //Serial.printf("%.2x", param->srv_open.rem_bda[i]);
    }

    mac[17] = '\0';
    
    // for(const auto& pair: devices){
    //     //Serial.println((int)((BLEClient*)pair.second.peer_device)->getConnId());
    //     //Serial.println(((BLEClient*)pair.second.peer_device)->getRssi());
    //     Serial.printf("%i\n",((BLEClient*)pair.second.peer_device)->getRssi());
    // }
    Serial.println(mac);
    desc->setNotifications(true);
    // NEEDED ACTIONS
  }

  void onDisconnect(BLEServer* pServer){
    connected = false;
    Serial.println("Disconnect");
    BLE2902* desc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(false);
  }
};




// Variable to track connection status
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

inline void connect_wait(){
    uint16_t ms_5  = 0;
    uint16_t seconds = 0;
    while(!connected){

        if(ms_5 >= 200){
            Serial.printf("Waiting for device to pair... %is\n", seconds);
            seconds++;
            ms_5 = 0;
        } else {
        ms_5++;
        }
        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}




void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  BLEDevice::init("Phone Lamp");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());
  pServer->getPeerDevices(false);

  hid = new BLEHIDDevice(pServer);
  input = hid->inputReport(1); // <-- input REPORTID from report map
  output = hid->outputReport(1); // <-- output REPORTID from report map

    std::string name = "Phone Lamp";    
  hid->manufacturer()->setValue(name);

  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00,0x02);

  BLESecurity *pSecurity = new BLESecurity();
  //  pSecurity->setKeySize();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    hid->startServices();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_BARCODE);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();
    hid->setBatteryLevel(7);

    pBLEScan = BLEDevice::getScan();  
    pBLEScan->setActiveScan(true);  // Set active scan to get more information (e.g., RSSI)
    pBLEScan->setInterval(100);     // Set scan interval (in milliseconds)
    pBLEScan->setWindow(99);        // Set scan window (in milliseconds)
    

    //ESP_LOGD(LOG_TAG, "Advertising started!");
    //delay(portMAX_DELAY);
}

void loop() {

  if(!connected){
    pAdvertising->start();
    connect_wait();
  }  
  

    BLEScanResults scanResults = pBLEScan->start(1, false);

  // Print the results
  int count = scanResults.getCount();
  Serial.print("Devices found: ");
  Serial.println(count);

    // Serial.println("Device connected...");
    // std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(false);
    // BLEClient* client = (BLEClient*)devices[0].peer_device;


  // Iterate through the results and display information
  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice device = scanResults.getDevice(i);

    if(!strcmp(mac, device.getAddress().toString().c_str())){
        Serial.printf("Signal strength:", device.getRSSI());

    }
    // Serial.print("Device Name: ");
    // Serial.println(device.getName().c_str());
    // Serial.print("Device Address: ");
    // Serial.println(device.getAddress().toString().c_str());
    // Serial.print("RSSI: ");
    // Serial.println(device.getRSSI());
    // Serial.println("---------------------------");
  }


    // std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(false);

    // for(const auto& pair: devices){
    //     //Serial.println((int)((BLEClient*)pair.second.peer_device)->getConnId());
    //     //Serial.println(((BLEClient*)pair.second.peer_device)->getRssi());
    //     Serial.printf("%i\n",((BLEClient*)pair.second.peer_device)->getRssi());
    // }
}