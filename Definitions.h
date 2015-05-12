#ifndef Definitions_h
#define Definitions_h

#define GREEN 42
#define YELLOW 40
#define BLUE 13
#define RED 38

#define PAUSE 5
#define RC_SS 44

#define RC_SERIAL_PORT Serial1


#define RC_SS_Output() DDRH |= 1<<7 
#define RC_SSHigh() PORTH |= 1<<7 
#define RC_SSLow() PORTH &= ~(1<<7)

#define CAL_FLAGS 0


enum CalibrationFlags {
  RC_FLAG,
  ACC_FLAG,
  MAG_FLAG,
  GAINS_FLAG
};

enum ISR_States {
  STAND,PPM};

enum RC_Types {
  DSM10, DSM11, SBUS, RC};
enum RC_Chan {
  THRO, AILE, ELEV, RUDD, GEAR, AUX1, AUX2, AUX3};

#define RC_DATA_START 332
#define RC_DATA_END 427

#define MAX_INDEX 333
#define MIN_INDEX 335
#define MID_INDEX 337
#define CHAN_INDEX 338
#define SCALE_INDEX 342
#define REV_INDEX 343


#endif//#ifndef Definitions.h

