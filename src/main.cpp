#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_spp_api.h"


#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2


#define TAG "BT_SPP_SERVER"

// This is a callback to handle SPP events
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP Initialized");
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 1, NULL); // Start SPP service
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "Data received: %s", param->data_ind.data);
            break;
        default:
            break;
    }
}

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


    // Initialize Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT); // Enable Bluetooth Classic mode
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Register the SPP callback
    esp_spp_register_callback(spp_callback);

    // Start Bluetooth SPP service
    esp_spp_init(ESP_SPP_MODE_CB);

    // Set Bluetooth device name
    esp_bt_dev_set_device_name("ESP32_SPP_Server");

    ESP_LOGI(TAG, "Bluetooth SPP Server Initialized");


    // Disable Bluetooth advertising (hides the device)
    //esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    //esp_bt_gap_cancel_discovery(); // If device is in discovery mode, cancel it

    
    
    
}   


void loop(){

}
