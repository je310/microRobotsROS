#include "stepper.h"
#include "Arduino.h"



void stepRight(int forward){
  static int state = 0;
  if(forward == 1){
    state++;
    if (state ==8) state =0;
  }
  else {
    if(state ==0) state = 7;
    else state--;
  }
  //state = state%8;
  
  switch(state){
    case 0: 
      SET(PORTD,7);
      SET(PORTB,0);
      CLR(PORTB,1);
      CLR(PORTB,2);
      break;
  
    case 1:
      SET(PORTD,7);
      SET(PORTB,0);
      SET(PORTB,1);
      SET(PORTB,2);
      break;
    case 2:
      CLR(PORTD,7);
      CLR(PORTB,0);
      SET(PORTB,1);
      SET(PORTB,2);
      break;
    case 3:
      CLR(PORTD,7);
      SET(PORTB,0);
      SET(PORTB,1);
      SET(PORTB,2);
      break;
    case 4:
      CLR(PORTD,7);
      SET(PORTB,0);
      CLR(PORTB,1);
      CLR(PORTB,2);
      break;    
    case 5:
      CLR(PORTD,7);
      SET(PORTB,0);
      CLR(PORTB,1);
      SET(PORTB,2);
      break;   
    case 6:
      CLR(PORTD,7);
      CLR(PORTB,0);
      CLR(PORTB,1);
      SET(PORTB,2);
      break;    
    case 7:
      SET(PORTD,7);
      SET(PORTB,0);
      CLR(PORTB,1);
      SET(PORTB,2);
      break;
  }
}
  
 void stepLeft(int forward){
  static int state = 0;
  if(forward == 1){
    state++;
    if (state ==8) state =0;
  }
  else {
    if(state ==0) state = 7;
    else state--;
  }
  
  switch(state){
    case 0: 
      SET(PORTC,0);
      SET(PORTC,2);
      CLR(PORTD,6);
      CLR(PORTD,2);
      break;
  
    case 1:
      SET(PORTC,0);
      SET(PORTC,2);
      SET(PORTD,6);
      SET(PORTD,2);
      break;
    case 2:
      CLR(PORTC,0);
      CLR(PORTC,2);
      SET(PORTD,6);
      SET(PORTD,2);
      break;
    case 3:
      CLR(PORTC,0);
      SET(PORTC,2);
      SET(PORTD,6);
      SET(PORTD,2);
      break;
    case 4:
      CLR(PORTC,0);
      SET(PORTC,2);
      CLR(PORTD,6);
      CLR(PORTD,2);
      break;    
    case 5:
      CLR(PORTC,0);
      SET(PORTC,2);
      CLR(PORTD,6);
      SET(PORTD,2);
      break;   
    case 6:
      CLR(PORTC,0);
      CLR(PORTC,2);
      CLR(PORTD,6);
      SET(PORTD,2);
      break;    
    case 7:
      SET(PORTC,0);
      SET(PORTC,2);
      CLR(PORTD,6);
      SET(PORTD,2);
      break;
  }
}

void looseMotors(){
    CLR(PORTC,0);
    CLR(PORTC,2);
    CLR(PORTD,6);
    CLR(PORTD,2);
    CLR(PORTD,7);
    CLR(PORTB,0);
    CLR(PORTB,1);
    CLR(PORTB,2);
}

void looseRight(){
    CLR(PORTD,7);
    CLR(PORTB,0);
    CLR(PORTB,1);
    CLR(PORTB,2);
}
void looseLeft(){
      CLR(PORTC,0);
    CLR(PORTC,2);
    CLR(PORTD,6);
    CLR(PORTD,2);
}
