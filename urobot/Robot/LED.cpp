#include "LED.h"
#include "Arduino.h"


int blueLED  = 0;
int redLED = 1;
int greenLED = 5;
int irLED = 3;

void setupLED(){
    pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode (greenLED, OUTPUT);
  pinMode (irLED, OUTPUT);
  LEDsOff();
}

void lightGreenLED(){
  CLR(PORTD,5) ; 
}

void lightBlueLED(){
  CLR(PORTD,0);
}

void lightRedLED(){
  CLR(PORTD,1);    // turn the LED on by making the voltage LOW
}

void LEDsOff(){
  SET(PORTD,5); //green off
  SET(PORTD,0); //blue off
  SET(PORTD,1); //red off
}

