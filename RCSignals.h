#ifndef RCSIGNALS_H
#define RCSIGNALS_H

#include "Types.h"
#include "Definitions.h"
#include "Comm.h"

typedef struct {
  int16_t max;
  int16_t min;
  int16_t mid;
  volatile int16_t rcvd;
  uint8_t chan;
  float scale;
  uint8_t reverse;
}
RC_t;

extern RC_t rcData[8];
extern boolean rcDetected;
extern volatile uint8_t rcType;
extern uint8_t ISRState;
extern volatile boolean RCFailSafe;
extern volatile boolean newRC;

void FeedLine();
void DetectRC();


#endif
