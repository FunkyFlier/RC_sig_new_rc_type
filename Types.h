#ifndef Types_h
#define Types_h

#include <Arduino.h>

typedef union {
  float val;
  uint8_t buffer[4];
}
float_u;

typedef union {
  int32_t val;
  uint8_t buffer[4];
}
int32_u;

typedef union {
  uint32_t val;
  uint8_t buffer[4];
}
uint32_u;

typedef union {
  int16_t val;
  uint8_t buffer[2];
}
int16_u;

typedef union {
  uint16_t val;
  uint8_t buffer[2];
}
uint16_u;

#endif//#ifndef Types_h
