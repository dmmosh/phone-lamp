#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BluetoothSerial.h"

#define LED 2
#define OFF 0
#define ON 1
#define FLASH 2

uint8_t curr_state = OFF; //led_
TaskHandle_t flash_led_task = NULL;


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
