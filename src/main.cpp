#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_bt.h"


#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2


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

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT); // Enable Classic Bluetooth
    

    if (esp_bluedroid_init() != ESP_OK || esp_bluedroid_enable() != ESP_OK){
        Serial.println("[ ESP BLUETOOTH FAILED ]");
        esp_restart();
    }

    esp_bt_dev_set_device_name("Phone Lamp");


    // Disable Bluetooth advertising (hides the device)
    //esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    //esp_bt_gap_cancel_discovery(); // If device is in discovery mode, cancel it

    
    
    
}   


void loop(){

}
