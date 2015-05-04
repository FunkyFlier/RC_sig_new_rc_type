#include <Arduino.h>
#include "Definitions.h"

void RCSerialBegin(uint32_t baudRate, uint8_t config){
  RC_SERIAL_PORT.begin(baudRate,config);
}

void RCSerialBegin(uint32_t baudRate){
  RC_SERIAL_PORT.begin(baudRate);
}

void RCSerialWrte(uint8_t outByte){
  RC_SERIAL_PORT.write(outByte);
}

uint8_t RCSerialRead(){
  return RC_SERIAL_PORT.read();
}

uint8_t RCSerialAvailable(){
  return RC_SERIAL_PORT.available();
}
