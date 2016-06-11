#ifndef stepper
#define stepper
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

void stepRight(int forward);
void stepLeft(int forward); //these are single steps. 

void looseMotors();
void looseLeft();
void looseRight();

#endif
