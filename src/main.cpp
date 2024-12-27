#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BluetoothSerial.h"

#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2

uint8_t status = OFF; //led_
uint8_t flash_led_state = OFF;
TaskHandle_t flash_led_task = NULL;

void flash_led(void* args){
    flash_led_state = ON; 
    while(status == FLASH){
        digitalWrite(LED,HIGH);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        digitalWrite(LED,LOW);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    flash_led_state = OFF;
    vTaskDelete(NULL);
}

void led(const uint8_t state){
    status = state;

    if(state == FLASH && flash_led_state != ON){
        xTaskCreate(flash_led, "led flash", configMINIMAL_STACK_SIZE, NULL, 1, &flash_led_task);
    } else if (state == ON){
        vTaskDelete(flash_led_task);
        digitalWrite(LED,HIGH);
    } else if (state == OFF){
        vTaskDelete(flash_led_task);
        digitalWrite(LED,LOW);
    }
}

void setup(){
    Serial.begin(115200);
    pinMode(LED,OUTPUT);
    
}   


void loop(){
        led(ON);
        Serial.println("[ LED ON ]");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        led(OFF);
        Serial.println("[ LED OFF ]");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        led(FLASH);
        Serial.println("[ LED FLASH ]");
        vTaskDelay(2000/portTICK_PERIOD_MS);
}
