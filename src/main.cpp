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

#define RED 4
#define GREEN 16
#define BLUE 17
#define MAX 255

// variables for the bluetooth server and hid device
BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;
BLEAdvertising *pAdvertising;
BLEServer *pServer;
BLEScan* pBLEScan;

// variable to track led lamp itself
uint8_t curr_lamp_state = OFF; //led_
TaskHandle_t lamp_rgb_task = NULL;


uint8_t led_timer = 0; // when it reaches 5 the led turns off
int8_t rssi;
char mac[18];
bool connected = false;



void rgb_lamp(void* args);
void lamp(const uint8_t new_state);
inline void led(const uint8_t new_state);
inline void connect_wait();
bool str_equals(const char* str1, const char* str2);



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
    // NEEDED ACTIONScdjknckj
  }

  void onDisconnect(BLEServer* pServer){
    connected = false;
    Serial.println("Disconnect");
    BLE2902* desc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(false);
  }
};



void rgb_lamp(void* args){

    uint8_t r = MAX;
    uint8_t g = 0;
    uint8_t b = 0;

    while(curr_lamp_state == ON){

        analogWrite(RED, r);
        analogWrite(GREEN,g);
        analogWrite(BLUE, b);

        if(r == MAX){
            if(g<20 && b<20){
                vTaskDelay(200/portTICK_PERIOD_MS);
            }

            if(b){
                b--;
            } else {
                g++;
            }
        } if (g == MAX){
            if(r){
                r--;
            } else {
                b++;
            }
        } if (b == MAX){
            if(g){
                g--;
            } else {
                r++;
            }
        }
        vTaskDelay(7/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void lamp(const uint8_t new_state){
    // why  no base case? data races
    // the time it takes to check if statements the current state could have already changed
    // and trying to create an already existing task or deleting a nonexistent one will crash the system

    if(new_state == ON && curr_lamp_state != ON){
        xTaskCreate(rgb_lamp, "rgb lamp", 2048,NULL,1,&lamp_rgb_task);
    } else if (new_state == OFF && curr_lamp_state != OFF) {
        vTaskDelete(lamp_rgb_task);
        analogWrite(RED,LOW);
        analogWrite(GREEN,LOW);
        analogWrite(BLUE,LOW);
    }
    curr_lamp_state = new_state;
}




inline void connect_wait(){
    uint16_t ms_5  = 0;
    uint16_t seconds = 0;
    pinMode(LED,OUTPUT);

    while(!connected){
        
        if(ms_5 >= 200){    
            if(seconds<21){ // note: at 20 second mark, it sets led to LOW and it stays there afterwards (to save power and not be annoying)
                digitalWrite(LED,seconds%2);
            }

            Serial.printf("Waiting for device to pair... %is\n", seconds);
            seconds++;
            ms_5 = 0;
        } else {
        ms_5++;
        }
        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

bool str_equals(const char* str1, const char* str2){
    uint8_t j = 0;
    while(str1[j] != '\0' || str2[j] != '\0'){
        if (str1[j] != str2[j]){
            return false;
        }
        j++;
    }
    return true;
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  pinMode(LED,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(BLUE,OUTPUT);

    analogWrite(LED,LOW);
    analogWrite(RED,LOW);
    analogWrite(GREEN,LOW);
    analogWrite(BLUE,LOW);


  BLEDevice::init("Phone Lamp");
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); 
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN ,ESP_PWR_LVL_P9);
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
    //delay(portMAX_DELAY);hksfhksfdssdhk
}


void loop() {

  if(!connected){
    lamp(OFF);
    pAdvertising->start();
    connect_wait();
  }  
  

    BLEScanResults scanResults = pBLEScan->start(1);

    // Print the results
    uint8_t count = scanResults.getCount();
 
    // Serial.println("Device connected...");
    // std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(false);
    // BLEClient* client = (BLEClient*)devices[0].peer_device;

  // Iterate through the results and display information
  for (uint16_t i = 0; i < count; i++) {
        BLEAdvertisedDevice device = scanResults.getDevice(i);
        

        if(str_equals(mac, device.getAddress().toString().c_str())){
            rssi = device.getRSSI();
            break;
        }

  }
    Serial.println(rssi);
    if(rssi > -90){
        digitalWrite(LED,ON);
        lamp(ON);
        led_timer = 0;
    } else{
        led_timer++;
    }

    if(led_timer>= 3){
        digitalWrite(LED,OFF);
        lamp(OFF);
        led_timer = 0;
    }


    // std::map<uint16_t, conn_status_t> devices = pServer->getPeerDevices(false);

    // for(const auto& pair: devices){
    //     //Serial.println((int)((BLEClient*)pair.second.peer_device)->getConnId());
    //     //Serial.println(((BLEClient*)pair.second.peer_device)->getRssi());
    //     Serial.printf("%i\n",((BLEClient*)pair.second.peer_device)->getRssi());
    // }
}