#include <stdio.h>
#include <string.h>

#define INIT                1
#define STATE_INIT          2
#define STATE_RUN           3
#define Q_WRITE             4
#define Q_READ              5
#define DEBUG_WRITE         6
#define CALLBACK            7

#define ADC0                20
#define ADC1                21
#define ADC2                22
#define ADC3                23

void dbgOutputVal(unsigned char outVal);
void dbgOutputLoc(unsigned char outVal);
void halt();

