#include <Streaming.h>
#include <EEPROM.h>
#include "Types.h"
#include "Definitions.h"
#include "RCSignals.h"





int16_t RC_value[8];

uint32_t printTimer;
uint32_t generalPurposeTimer;



volatile uint8_t failSafeCount;

uint8_t chanOrder[8] = {
  THRO,AILE,ELEV,RUDD,GEAR,AUX1,AUX2,AUX3
};


float_u outFloat;
int16_u outInt16;

uint8_t calibrationFlags;
/*uint8_t chanOrder[8] = {
 ELEV,RUDD,THRO,AILE,GEAR,AUX1,AUX2,AUX3
 };*/
/*uint8_t chanOrder[8] = {
 AUX3,AUX2,AUX1,GEAR,RUDD,ELEV,AILE,THRO
 };*/



void pause(){
  Serial<<"pause\r\n";
  while(digitalRead(PAUSE)==0){
  }//wait for the toggle
  Serial<<"***\r\n";
  delay(1500);
}


void WipeRom(){
  for(uint16_t i = 0; i < 600; i++){
    EEPROM.write(i,0xFF);
  }
  while(1){
  }
}
void setup(){
  //WipeRom();
  pinMode(PAUSE,INPUT);
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  Serial.begin(115200);
  Serial<<"Start\r\n";
  RC_SS_Output();

  DetectRC();
  Serial<<rcDetected<<","<<rcType<<","<<ISRState<<"\r\n";
  pause(); 

  _200HzISRConfig();
  printTimer = millis();
  generalPurposeTimer = millis();
  calibrationFlags = EEPROM.read(0);
  //if(  ((calibrationFlags & (1<<RC_FLAG)) >> RC_FLAG) == 0x01 ){
    if(1){
    Serial<<"calibration\r\n";
    pause();
    AssignChannels();
    GetMinMaxMid();
    WriteROM();
  }
  ReadROM();
  RCFailSafe = false;
  failSafeCount = 0;
}


void AssignChannels(){
  for (uint8_t i = 0;i < 8; i++){
    rcData[i].chan = chanOrder[i];
    /*if (rcData[i].chan == AILE || rcData[i].chan == RUDD){
     rcData[i].reverse = 0;
     }
     else{
     rcData[i].reverse = 0;
     }*/
  }

}
void ReadROM(){
  uint16_t j=0;//index for input buffers
  uint16_t k=0;//index for start of each channel's data in rom
  uint16_t l=0;//index for each channel
  uint16_t switchControl;
  for(uint16_t i = RC_DATA_START; i <=RC_DATA_END; i++){//index for each rom location
    switchControl = i - k;
    if (switchControl < CHAN_INDEX){//first 16 bit ints
      outInt16.buffer[j++] = EEPROM.read(i);
    }
    if (switchControl > CHAN_INDEX && i - k < REV_INDEX){//scale factor
      outFloat.buffer[j++] = EEPROM.read(i);
    }

    switch (switchControl){
    case MAX_INDEX://max
      rcData[l].max = outInt16.val;
      j=0;
      break;
    case MIN_INDEX://min
      rcData[l].min = outInt16.val;
      j=0;
      break;
    case MID_INDEX://mid
      rcData[l].mid = outInt16.val;
      j=0;
      break;
    case CHAN_INDEX://chan
      rcData[l].chan = EEPROM.read(i);
      break;
    case SCALE_INDEX://scale
      rcData[l].scale = outFloat.val;
      j=0;
      break;
    case REV_INDEX://reverse
      rcData[l].reverse = EEPROM.read(i);
      k += 12;
      l += 1;
      break;
    }
  }
}
void WriteROM(){
  uint16_t ROMIndex = RC_DATA_START;
  for(uint8_t i = 0; i < 8; i++){
    outInt16.val = rcData[i].max;
    EEPROM.write(ROMIndex++,outInt16.buffer[0]);
    EEPROM.write(ROMIndex++,outInt16.buffer[1]);

    outInt16.val = rcData[i].min;
    EEPROM.write(ROMIndex++,outInt16.buffer[0]);
    EEPROM.write(ROMIndex++,outInt16.buffer[1]);

    outInt16.val = rcData[i].mid;
    EEPROM.write(ROMIndex++,outInt16.buffer[0]);
    EEPROM.write(ROMIndex++,outInt16.buffer[1]);

    EEPROM.write(ROMIndex++,rcData[i].chan);

    outFloat.val = rcData[i].scale;
    EEPROM.write(ROMIndex++,outFloat.buffer[0]);
    EEPROM.write(ROMIndex++,outFloat.buffer[1]);
    EEPROM.write(ROMIndex++,outFloat.buffer[2]);
    EEPROM.write(ROMIndex++,outFloat.buffer[3]);

    EEPROM.write(ROMIndex++,rcData[i].reverse);
  }
  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags &= ~(1<<RC_FLAG);
  EEPROM.write(0x00,calibrationFlags);
}
void GetMinMaxMid(){
  while(digitalRead(PAUSE)==0){//center
    digitalWrite(RED,HIGH);

    if (newRC == true){
      newRC = false;
      Serial<<"Mid: "<<rcData[0].rcvd<<","<<rcData[1].rcvd<<","<<rcData[2].rcvd<<","<<rcData[3].rcvd<<","<<rcData[4].rcvd<<","<<rcData[5].rcvd<<","<<rcData[6].rcvd<<","<<rcData[7].rcvd<<"\r\n";

    }     
  }

  digitalWrite(RED,LOW);
  for (uint8_t i = THRO; i <= AUX3; i++){
    rcData[i].mid = rcData[i].rcvd ;
  }
  for (uint8_t i = THRO; i <= AUX3; i++){
    rcData[i].max =  rcData[i].mid;
    rcData[i].min =  rcData[i].mid;
  }
  Serial<<"Center vals: "<<rcData[0].mid<<","<<rcData[1].mid<<","<<rcData[2].mid<<","<<rcData[3].mid<<"\r\n";
  pause();
  while(digitalRead(PAUSE)==0){
    if (newRC == true){
      newRC = false;
      for (uint8_t i = THRO; i <= AUX3; i++){
        if (rcData[i].rcvd > rcData[i].max){
          rcData[i].max = rcData[i].rcvd;
        }
        if (rcData[i].rcvd < rcData[i].min){
          rcData[i].min = rcData[i].rcvd;
        }
      }
      Serial<<rcData[0].rcvd<<","<<rcData[1].rcvd<<","<<rcData[2].rcvd<<","<<rcData[3].rcvd<<","<<rcData[4].rcvd<<","<<rcData[5].rcvd<<","<<rcData[6].rcvd<<","<<rcData[7].rcvd<<"\r\n";
    }  
  }
  Serial<<"Max values: "<<rcData[0].max<<","<<rcData[1].max<<","<<rcData[2].max<<","<<rcData[3].max<<","<<rcData[4].max<<","<<rcData[5].max<<","<<rcData[6].max<<","<<rcData[7].max<<"\r\n";
  Serial<<"Min values: "<<rcData[0].min<<","<<rcData[1].min<<","<<rcData[2].min<<","<<rcData[3].min<<","<<rcData[4].min<<","<<rcData[5].min<<","<<rcData[6].min<<","<<rcData[7].min<<"\r\n";
  for (uint8_t i = THRO; i <= AUX3; i++){   
    if ((float)rcData[i].max - (float)rcData[i].min != 0){
      rcData[i].scale = 1000.0/( (float)rcData[i].max - (float)rcData[i].min );
    }
  }

  Serial<<"Scale values: "<<rcData[0].scale<<","<<rcData[1].scale<<","<<rcData[2].scale<<","<<rcData[3].scale<<","<<rcData[4].scale<<","<<rcData[5].scale<<","<<rcData[6].scale<<","<<rcData[7].scale<<"\r\n";
  pause();

}

void loop(){
  if (newRC == true){
    newRC = false;
    ProcessChannels();
    failSafeCount = 0;
  }    
  if (millis() - printTimer > 100){
    printTimer = millis();
    Serial<<rcData[0].rcvd<<","<<rcData[1].rcvd<<","<<rcData[2].rcvd<<","<<rcData[3].rcvd<<","<<rcData[4].rcvd<<","<<rcData[5].rcvd<<","<<rcData[6].rcvd<<","<<rcData[7].rcvd<<"\r\n";
    Serial<<RC_value[THRO]<<","<<RC_value[AILE]<<","<<RC_value[ELEV]<<","<<RC_value[RUDD]<<","<<RC_value[GEAR]<<","<<RC_value[AUX1]<<","<<RC_value[AUX2]<<","<<RC_value[AUX3]<<","<<RCFailSafe<<"\r\n";
  }
  if (failSafeCount == 200){
    RCFailSafe = true;
  }

}
void _200HzISRConfig(){
  TCCR5A = (1<<COM5A1);
  TCCR5B = (1<<CS51)|(1<<WGM52);
  TIMSK5 = (1<<OCIE5A);
  OCR5A = 10000;
}

ISR(TIMER5_COMPA_vect, ISR_NOBLOCK){
  failSafeCount++;
  if (rcType != RC){
    FeedLine();
  }
}


void ProcessChannels(){
  for (uint8_t i = 0; i < 8; i++)  {
    switch (rcData[i].chan){
    case THRO:
      if (rcData[i].reverse == 0){
        RC_value[THRO] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[THRO] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }
      break;
    case AILE:
      if (rcData[i].reverse == 0){
        RC_value[AILE] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else{
        RC_value[AILE] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }
      break;
    case ELEV:
      if (rcData[i].reverse == 0){
        RC_value[ELEV] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else{
        RC_value[ELEV] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }

      break;
    case RUDD:
      if (rcData[i].reverse == 0){
        RC_value[RUDD] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else{
        RC_value[RUDD] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }
      break;
    case GEAR:
      if (rcData[i].reverse == 0){
        RC_value[GEAR] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[GEAR] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX1:
      if (rcData[i].reverse == 0){
        RC_value[AUX1] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[AUX1] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX2:
      if (rcData[i].reverse == 0){
        RC_value[AUX2] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[AUX2] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX3:
      if (rcData[i].reverse == 0){
        RC_value[AUX3] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;      
      }
      else{
        RC_value[AUX3] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;    
      }

      break;


    }
  }



}








































































