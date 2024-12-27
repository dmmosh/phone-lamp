#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BluetoothSerial.h"

#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2


BluetoothSerial SerialBT;
uint8_t curr_state = OFF; //led_
TaskHandle_t flash_led_task = NULL;



void flash_led(void* args){
    while(curr_state == FLASH){
        digitalWrite(LED,HIGH);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        digitalWrite(LED,LOW);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void led(const uint8_t new_state){  
    // state: new state
    // 
    if(new_state == FLASH && curr_state != FLASH){ 
        Serial.println("[ LED FLASH ]");
        xTaskCreate(flash_led, "led flash", configMINIMAL_STACK_SIZE, NULL, 1, &flash_led_task);

    } else if (new_state == ON && curr_state != ON){ // LED ON
        Serial.println("[ LED ON ]");
        if(curr_state == FLASH){
            vTaskDelete(flash_led_task);
        }
        digitalWrite(LED,HIGH);
        
    } else if (new_state == OFF && curr_state != OFF){ // LED OFF
        Serial.println("[ LED OFF ]");
        if(curr_state == FLASH){
            vTaskDelete(flash_led_task);
        }
        digitalWrite(LED,LOW);
    }
    curr_state = new_state;
}

void setup(){
    Serial.begin(115200);
    pinMode(LED,OUTPUT);

    SerialBT.begin("Phone Lamp");

    
    
    
    
}   


void loop(){
        BTScanResults* devices = SerialBT.getScanResults();
        uint16_t device_cnt = devices->getCount();
        for (uint16_t i = 0; i < device_cnt; i++)
        {
            BTAdvertisedDevice *device = devices->getDevice(i);
            Serial.printf("%i %s %s\n",device->getRSSI(), device->getAddress().toString().c_str(), device->getName().c_str());
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);

}
