#ifndef RCSIGNALS_H
#define RCSIGNALS_H

#include "Types.h"
#include "Definitions.h"
#include "Comm.h"

typedef struct {
  uint16_t max;
  uint16_t min;
  uint16_t mid;
  volatile uint16_t rcvd;
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
