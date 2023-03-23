#include "Arduino.h"
#include "Blinker.h"

Blinker::Blinker(){
  pinMode(LED_BUILTIN, OUTPUT);
};

void Blinker::blink(int num_blinks, int freq){
  for(int i = 0; i < num_blinks; i++){
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(freq);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(freq);                       // wait for a second
  }
}