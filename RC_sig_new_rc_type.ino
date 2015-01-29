#include <Streaming.h>
#define RED 38
#define YELLOW 40
#define GREEN 42

#define PAUSE 22
#define RC_SS 44

#define RC_SS_Output() DDRH |= 1<<7 
#define RC_SSHigh() PORTH |= 1<<7 
#define RC_SSLow() PORTH &= ~(1<<7)

enum ISR_States {
  STAND,PPM};

enum RC_Types {
  DSMX = 1, SBUS, RC};
enum RC_Chan {
  THRO, AILE, ELEV, RUDD, GEAR, AUX1, AUX2, AUX3};
//RC related vars
uint8_t readState,inByte,byteCount,channelNumber;
volatile uint8_t rcType;
uint32_t frameTime;
boolean detected = false;
volatile boolean newRC = false;
boolean frameStart = true;
boolean frameValid = false;

uint8_t spekBuffer[14];

uint16_t bufferIndex=0;




uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint32_t timeDifference = 0;
uint32_t changeTime[8];
uint8_t sBusData[25];

uint8_t ISRState = STAND;
uint8_t channelCount = 0;


typedef struct{
  int16_t max;
  int16_t min;
  int16_t mid;
  volatile int16_t rcvd;
  uint8_t chan;
  float scale;
  uint8_t reverse;
}
RC_t;

RC_t rcData[8];
int16_t RC_value[8];

uint32_t printTimer;
uint32_t generalPurposeTimer;
uint32_t timeDiff;
volatile boolean failSafe;
uint8_t chanOrder[8] = {
  THRO,AILE,ELEV,RUDD,GEAR,AUX1,AUX2,AUX3
};
/*uint8_t chanOrder[8] = {
 ELEV,RUDD,THRO,AILE,GEAR,AUX1,AUX2,AUX3
 };*/
/*uint8_t chanOrder[8] = {
 AUX3,AUX2,AUX1,GEAR,RUDD,ELEV,AILE,THRO
 };*/


void pause(){
  while(digitalRead(PAUSE)==0){
  }//wait for the toggle
  delay(1500);
}



void setup(){

  pinMode(PAUSE,INPUT);
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  Serial.begin(115200);
  Serial<<"Start\r\n";
  RC_SS_Output();

  DetectRC();
  Serial<<rcType<<","<<ISRState<<"\r\n";
  pause(); 
  AssignChannels();
  _200HzISRConfig();
  printTimer = millis();
  generalPurposeTimer = millis();

  GetMinMaxMid();
  dtd = true;
  failSafe = false;
}
void AssignChannels(){
  for (uint8_t i = 0;i < 8; i++){
    rcData[i].chan = chanOrder[i];
    /*if (rcData[i].chan == THRO){
     rcData[i].reverse == 1;
     }*/
    rcData[i].reverse = 0;
  }

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
    rcData[i].max = 1500;
    rcData[i].min = 1500;
  }
  Serial<<"Center vals: "<<rcData[0].mid<<","<<rcData[1].mid<<","<<rcData[2].mid<<","<<rcData[3].mid<<"\r\n";
  delay(500);
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
  delay(500);
  pause();
  for (uint8_t i = THRO; i <= AUX3; i++){   
    rcData[i].scale = 1000.0/( (float)rcData[i].max - (float)rcData[i].min );
  }

  Serial<<"Scale values: "<<rcData[0].scale<<","<<rcData[1].scale<<","<<rcData[2].scale<<","<<rcData[3].scale<<","<<rcData[4].scale<<","<<rcData[5].scale<<","<<rcData[6].scale<<","<<rcData[7].scale<<"\r\n";
  delay(500);
  pause();

}

void loop(){
  if (newRC == true){
    newRC = false;
    ProcessChannels();
  }    
  if (millis() - printTimer > 100){
    printTimer = millis();
    Serial<<rcData[0].rcvd<<","<<rcData[1].rcvd<<","<<rcData[2].rcvd<<","<<rcData[3].rcvd<<","<<rcData[4].rcvd<<","<<rcData[5].rcvd<<","<<rcData[6].rcvd<<","<<rcData[7].rcvd<<"\r\n";
    Serial<<RC_value[THRO]<<","<<RC_value[AILE]<<","<<RC_value[ELEV]<<","<<RC_value[RUDD]<<","<<RC_value[GEAR]<<","<<RC_value[AUX1]<<","<<RC_value[AUX2]<<","<<RC_value[AUX3]<<","<<failSafe<<"\r\n";
  }

}
void _200HzISRConfig(){
  TCCR5A = (1<<COM5A1);
  TCCR5B = (1<<CS51)|(1<<WGM52);
  TIMSK5 = (1<<OCIE5A);
  OCR5A = 10000;
}

ISR(TIMER5_COMPA_vect, ISR_NOBLOCK){
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
        RC_value[THRO] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale - 1000;
      }
      break;
    case AILE:
      if (rcData[i].reverse == 0){
        RC_value[AILE] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else{
        RC_value[AILE] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale - 1500;
      }
      break;
    case ELEV:
      if (rcData[i].reverse == 0){
        RC_value[ELEV] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else{
        RC_value[ELEV] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale - 1500;
      }

      break;
    case RUDD:
      if (rcData[i].reverse == 0){
        RC_value[RUDD] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else{
        RC_value[RUDD] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale - 1500;
      }
      break;
    case GEAR:
      if (rcData[i].reverse == 0){
        RC_value[GEAR] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[GEAR] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale - 1000;
      }

      break;
    case AUX1:
      if (rcData[i].reverse == 0){
        RC_value[AUX1] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[AUX1] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale - 1000;
      }

      break;
    case AUX2:
      if (rcData[i].reverse == 0){
        RC_value[AUX2] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else{
        RC_value[AUX2] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale - 1000;
      }

      break;
    case AUX3:
      if (rcData[i].reverse == 0){
        RC_value[AUX3] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;      
      }
      else{
        RC_value[AUX3] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale - 1000;      
      }

      break;


    }
  }



}


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
            //if (rcData[i].chan == THRO && ((timeDifference ) < 1025)){
            if (rcData[i].chan == THRO && ((timeDifference ) < (rcData[i].min - 50) )){  
              failSafe = true;
            }
            else{
              newRC = true;
            }

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
          failSafe = true;
        }
        else{
          newRC = true;
        }
        channelCount++;
      }
    }
    break;
  }

}

void FeedLine(){

  switch(rcType){
  case 1:
    DSMXParser();
    break;
  case 2:
    SBusParser();
    break;
  }

}
void SBusParser(){
  while(Serial1.available() > 0){
    if (millis() - frameTime > 8){
      readState = 0;
    }
    inByte = Serial1.read();
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
          if (sBusData[23] & (1<<2)) {
            failSafe = true;
          }
          if (sBusData[23] & (1<<3)) {
            failSafe = true;
          }
        }
      }
      break;
    }
  }


}

void DSMXParser(){

  while (Serial1.available() > 0){
    if (millis() - frameTime > 8){
      byteCount = 0;
      bufferIndex = 0;
    }
    inByte = Serial1.read();
    frameTime = millis();
    byteCount++;


    if (bufferIndex > 14){
      bufferIndex = 0;
      byteCount = 0;
    }
    if (byteCount > 2){
      spekBuffer[bufferIndex] = inByte;
      bufferIndex++;
    }
    if (byteCount == 16 && bufferIndex == 14){
      newRC = true;
      byteCount = 0;
      bufferIndex = 0;
      for (int i = 0; i < 14; i=i+2){
        channelNumber = (spekBuffer[i] >> 3) & 0x0F;
        rcData[channelNumber].rcvd = ((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF;
      }
      byteCount = 0;
    }
  }
}

void DetectRC(){
  readState = 0;
  RC_SSHigh();
  SBus();
  readState = 0;
  if (detected == true){
    FrameCheck();
    readState = 0;
    return;
  }
  readState = 0;
  RC_SSLow();
  Spektrum();
  readState = 0;

  if (detected == true){
    FrameCheck();
    readState = 0;
    return;
  }
  else{
    rcType = RC;
  }
  readState = 0;
  if (rcType == RC){//figure out the best way to handle this redundant code 
    DDRK = 0;//PORTK as input
    PORTK |= 0xFF;//turn on pull ups
    PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
    PCICR = 1<<2;//enable the pin change interrupt for K
    delay(100);//wait for a few frames
    if (rcData[0].rcvd == 0 && rcData[1].rcvd == 0 && rcData[2].rcvd == 0 && rcData[3].rcvd == 0 && rcData[4].rcvd == 0 && rcData[5].rcvd == 0 && rcData[6].rcvd == 0){
      ISRState = PPM;
      PORTK |= 0x80;
      PCMSK2 |= 0x80;
    }

  }



}


void FrameCheck(){//checks if serial RC was incorrectly detected
  newRC = false;
  uint32_t frameCheckTimer;
  frameCheckTimer =  millis();
  delay(100);
  while (newRC == false){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }

    if (millis() - frameCheckTimer > 1000){//in case it has incorrectly detected serial RC
      rcType = RC;
      DDRK = 0;//PORTK as input
      PORTK |= 0xFF;//turn on pull ups
      PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
      PCICR = 1<<2;
      delay(100);//wait for a few frames
      if (rcData[0].rcvd == 0 && rcData[1].rcvd == 0 && rcData[2].rcvd == 0 && rcData[3].rcvd == 0 && rcData[4].rcvd == 0 && rcData[5].rcvd == 0 && rcData[6].rcvd == 0){
        ISRState = PPM;
        PORTK |= 0x80;
        PCMSK2 |= 0x80;
      }
      Serial<<"*** "<<rcType<<","<<ISRState<<"\r\n";
      pause();
      generalPurposeTimer = millis();
    }
  } 
  newRC = false;

}

void SBus(){

  Serial1.begin(100000);
  generalPurposeTimer = millis();

  while (Serial1.available() == 0){
    if (millis() - generalPurposeTimer > 1000){
      return;
    }
  }

  delay(20);
  while(Serial1.available() > 0){
    inByte = Serial1.read();
    switch (readState){
    case 0:
      if (inByte == 0x0f){
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
          rcType = SBUS;
          detected = true;
        }
      }
      break;
    }
  }
  frameTime = millis();
}
void Spektrum(){
  Serial1.begin(115200);
  generalPurposeTimer = millis();
  while (Serial1.available() == 0){
    if (millis() - generalPurposeTimer > 1000){
      return;
    }
  }  
  delay(5);
  while(Serial1.available() > 0){
    Serial1.read();
  }
  frameTime = millis();
  rcType = DSMX;
  detected = true;
}




















































