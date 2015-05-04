#ifndef COMM_H
#define COMM_H

void RCSerialWrite(uint8_t);
uint8_t RCSerialRead();
uint8_t RCSerialAvailable(); 
void RCSerialBegin(uint32_t);
void RCSerialBegin(uint32_t,uint8_t);


#endif//#ifndef COMM_H
