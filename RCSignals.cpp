#include "RCSignals.h"

RC_t rcData[8];
boolean rcDetected = false;
volatile uint8_t rcType;
uint8_t ISRState = STAND;
volatile boolean RCFailSafe = false;
volatile boolean newRC = false;

uint8_t readState,inByte,byteCount,channelNumber;
uint32_t frameTime;

uint8_t DSMSerialBuffer[14];
uint8_t sBusData[25];

uint16_t bufferIndex=0;

uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint32_t timeDifference = 0;
uint32_t changeTime[8];

uint8_t channelCount = 0;

uint32_t timeDiff,waitTimer;

boolean DSMParser();
void DSMDetectRes();
void SBusParser();
void PWMPPMCheck();
void SBus();
void DSMSerial();


ISR(PCINT2_vect){
  switch(ISRState){
  case STAND:
    currentPinState = PINK;
    changeMask = currentPinState ^ lastPinState;
    lastPinState = currentPinState;
    currentTime = micros();
    for(uint8_t i=0;i<8;i++){
      if(changeMask & 1<<i){//has there been a change
        if(!(currentPinState & 1<<i)){//is the pin in question logic low?
          timeDifference = currentTime - changeTime[i];//if so then calculate the pulse width
          if (900 < timeDifference && timeDifference < 2200){//check to see if it is a valid length
            rcData[i].rcvd = timeDifference;
            if (rcData[i].chan == THRO && ((timeDifference ) < (rcData[i].min - 50) )){  
              RCFailSafe = true;
            }
            newRC = true;
          }
        }
        else{//the pin is logic high implying that this is the start of the pulse
          changeTime[i] = currentTime;
        }
      }
    }
    break;
  case PPM:
    currentTime = micros();
    if ((PINK & 0x80) == 0x80){//is the PPM pin high
      previousTime = currentTime;
    }
    else{
      timeDifference = currentTime - previousTime;
      if(timeDifference > 2500){
        channelCount = 0;
      }
      else{
        rcData[channelCount].rcvd = timeDifference;
        if (rcData[channelCount].chan == THRO && ((timeDifference ) < (rcData[channelCount].min - 50) )){  
          RCFailSafe = true;
        }
        newRC = true;
        channelCount++;
      }
    }
    break;
  }

}

void FeedLine(){

  switch(rcType){
  case 0:
  case 1:
    DSMParser();
    break;
  case 2:
    SBusParser();
    break;
  }

}
void SBusParser(){

  while( RCSerialAvailable() > 0){
    if (millis() - frameTime > 8){
      readState = 0;
    }
    inByte =  RCSerialRead();
    frameTime = millis();
    switch (readState){
    case 0:
      if (inByte == 0x0F){
        bufferIndex = 0;
        sBusData[bufferIndex] = inByte;
        sBusData[24] = 0xff;
        readState = 1;
      }

      break;
    case 1:
      bufferIndex ++;
      sBusData[bufferIndex] = inByte;

      if (bufferIndex == 24){
        readState = 0;
        if (sBusData[0]==0x0f && sBusData[24] == 0x00){
          newRC = true;
          rcData[0].rcvd = (sBusData[1]|sBusData[2]<< 8) & 0x07FF ;
          rcData[1].rcvd = (sBusData[2]>>3|sBusData[3]<<5) & 0x07FF;
          rcData[2].rcvd = (sBusData[3]>>6|sBusData[4]<<2|sBusData[5]<<10) & 0x07FF;
          rcData[3].rcvd = (sBusData[5]>>1|sBusData[6]<<7) & 0x07FF;
          rcData[4].rcvd = (sBusData[6]>>4|sBusData[7]<<4) & 0x07FF;
          rcData[5].rcvd = (sBusData[7]>>7|sBusData[8]<<1|sBusData[9]<<9) & 0x07FF;
          rcData[6].rcvd = (sBusData[9]>>2|sBusData[10]<<6) & 0x07FF;
          rcData[7].rcvd = (sBusData[10]>>5|sBusData[11]<<3) & 0x07FF;
          /*if (sBusData[23] & (1<<2)) {//frame loss flag
           RCFailSafe = true;
           }*/
          if (sBusData[23] & (1<<3)) {
            RCFailSafe = true;
          }
        }
      }
      break;
    }
  }


}


void DSMDetectRes(){
  rcType = DSM11;
  uint8_t channel1,channel2;
  boolean lowRes = false,fullRes= false;
  for(uint8_t i = 0; i < 2 ; i++){//make sure frame one starts with 1 then 5
    while(DSMParser() == false){
    }
    //check for second byte flag
    if (~(DSMSerialBuffer[0] & 1 << 0x80)){
      channel1 = DSMSerialBuffer[0] >> 2 & 0x0F;
      channel2 = DSMSerialBuffer[2] >> 2 & 0x0F;
      if (channel1 == 1 && channel2 == 5){
        lowRes = true;
      }
      channel1 = DSMSerialBuffer[0] >> 3 & 0x0F;
      channel2 = DSMSerialBuffer[2] >> 3 & 0x0F;
      if (channel1 == 1 && channel2 == 5){
        fullRes = true;
      }
    }
  }
  if (lowRes == true && fullRes == false){
    rcType = DSM10;
    rcDetected = true;
    return;
  }

  if (lowRes == false && fullRes == true){
    rcType = DSM11;
    rcDetected = true;
    return;
  }
  rcDetected = false;




}



boolean DSMParser(){
  boolean validFrame = false;
  while ( RCSerialAvailable() > 0){
    if (millis() - frameTime > 8){
      byteCount = 0;
      bufferIndex = 0;
    }
    inByte =  RCSerialRead();
    frameTime = millis();
    byteCount++;


    if (bufferIndex > 14){
      bufferIndex = 0;
      byteCount = 0;
    }
    if (byteCount > 2){
      DSMSerialBuffer[bufferIndex] = inByte;
      bufferIndex++;
    }
    if (byteCount == 16 && bufferIndex == 14){
      newRC = true;
      byteCount = 0;
      bufferIndex = 0;
      for (uint8_t i = 0; i < 14; i=i+2){
        validFrame = true;
        if (rcType == DSM10){
          channelNumber = (DSMSerialBuffer[i] >> 2) & 0x0F;
          if (channelNumber < 8){
            rcData[channelNumber].rcvd = ((DSMSerialBuffer[i] << 8) | (DSMSerialBuffer[i+1])) & 0x03FF;
          }
        }
        else{
          channelNumber = (DSMSerialBuffer[i] >> 3) & 0x0F;
          if (channelNumber < 8){
            rcData[channelNumber].rcvd = ((DSMSerialBuffer[i] << 8) | (DSMSerialBuffer[i+1])) & 0x07FF;
          }
        }

      }
    }
  }
  return validFrame;
}

void DetectRC(){
  readState = 0;
  RC_SSHigh();
  SBus();
  if(rcDetected == true){
    readState = 0;
    return;
  }
  RC_SSLow();
  DSMSerial();
  if (rcDetected == true){
    readState = 0;
    return;
  }
  PWMPPMCheck();




}


void PWMPPMCheck(){//checks if serial RC was incorrectly detected
  newRC = false;
  uint32_t frameCheckTimer;

  ISRState = STAND;
  rcType = RC;
  DDRK = 0;//PORTK as input
  PORTK |= 0xFF;//turn on pull ups
  PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
  PCICR = 1 << 2;
  delay(100);//wait for a few frames
  if (rcData[0].rcvd == 0 && rcData[1].rcvd == 0 && rcData[2].rcvd == 0 && rcData[3].rcvd == 0 && rcData[4].rcvd == 0 && rcData[5].rcvd == 0 && rcData[6].rcvd == 0 && rcData[7].rcvd != 0) {
    ISRState = PPM;
    PORTK |= 0x80;
    PCMSK2 |= 0x80;
  }
  newRC = false;
  frameCheckTimer =  millis();
  while (newRC == false) {
    if (millis() - frameCheckTimer > 1000) { //in case it has incorrectly detected serial RC
      rcDetected = false;
      return;

    }
  }
  rcDetected = true;
  newRC = false;


}

void SBus(){
  rcDetected = false;
  RCSerialBegin(100000,SERIAL_8E2);
  waitTimer = millis();
  while( RCSerialAvailable() > 0){
    RCSerialRead();
  }
  while ( RCSerialAvailable() == 0){
    if (millis() - waitTimer > 1000){
      return;
    }
  }

  delay(20);
  SBusParser();
  if(newRC == true){
    newRC = false;
    rcType = SBUS;
    rcDetected = true;
    frameTime = millis();
  }

}
void DSMSerial(){
  RCSerialBegin(115200);
  while( RCSerialAvailable() > 0){
    RCSerialRead();
  }
  waitTimer = millis();
  while ( RCSerialAvailable() == 0){
    if (millis() - waitTimer > 1000){
      return;
    }
  }  
  delay(23);
  DSMParser();
  if (newRC == true){
    DSMDetectRes();
    newRC = false;
    frameTime = millis();
  }



}


