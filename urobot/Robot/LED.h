#ifndef LED
#define LED
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

void setupLED();
void lightGreenLED();
void lightBlueLED();
void lightRedLED();
void LEDsOff();

#endif

