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


// DEBUG SERIAL INPUT
String input_string(){
    String out;


    while(1){
        if(Serial.available() >0){
            char curr = Serial.read();
            //str.trim();
            switch(curr){
                case '\n':
                    if(out.length()>1){

                        Serial.println();
                        return out;
                    }
                    out = "";
                break;
                case '\b':
                    out.remove(out.length()-1);
                    Serial.print('\b');
                    Serial.print(' ');
                    Serial.print('\b');
                break;
                default:
                    out += curr;
                    Serial.print(curr);
                break;
            }
        }
    }
}


void flash_led(void* args){
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

    if(state == FLASH && flash_led_state == OFF){ 
        flash_led_state = ON;
        xTaskCreate(flash_led, "led flash", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    } else if (state == ON){
        flash_led_state = OFF;
        while(flash_led_state){

        };
        digitalWrite(LED,HIGH);
        
    } else if (state == OFF){
        flash_led_state = OFF;
        while(flash_led_state){

        };
        digitalWrite(LED,LOW);

    }
}

void setup(){
    Serial.begin(115200);
    pinMode(LED,OUTPUT);
    
}   


void loop(){
        Serial.println("[ LED ON ]");
        led(ON);
        vTaskDelay(2000/portTICK_PERIOD_MS);
        Serial.println("[ LED OFF ]");
        led(OFF);
        vTaskDelay(2000/portTICK_PERIOD_MS);
        Serial.println("[ LED FLASH ]");
        led(FLASH);
        vTaskDelay(2000/portTICK_PERIOD_MS);
}
