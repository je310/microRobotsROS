#include <Event.h>
#include <Timer.h>

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#include <IRremote.h>
#include <IRremoteInt.h>
#include "TimerObject.h"
#include "LED.h"
#include "ir.h"
#include "stepper.h"


// https://arduino-info.wikispaces.com/IR-RemoteControl this is where the codes are 





int RECV_PIN = 4;
int sensorPower = 12;
int ir2 = A3;
IRrecv irrecv(RECV_PIN);

decode_results results;
Timer t;
int shouldSpin = 0;
int battery = A1;
int checkIR = 0;
//int checkBatt = 0;
int dirLeft = 1;
int dirRight = 1;

int speedLeft = 1;
int speedRight = 1;

int movingLeft = 0;
int movingRight = 0;


// event 'handlers', we are allowed 10;
int  motorLeftEvent;
int  motorRightEvent;
int  batteryEvent;
int deactivateLeft;
int deactivateRight;
int RCTimout;


void motorLeftUpdate(){
  if(movingLeft) stepLeft(dirLeft);
  //deactivateLeft = t.after(speedRight/2, looseLeft);
}

void motorRightUpdate(){
if (movingRight) stepRight(dirRight);
  //deactivateRight = t.after(speedLeft/2, looseRight);
}
void followLine();

void checkBatt(){
     int batteryReading = analogRead(battery);
   if(batteryReading < 300) {
     lightRedLED();
     lightBlueLED();
   }
   if(batteryReading >630) lightGreenLED();
   if(batteryReading > 300 && batteryReading < 480) lightRedLED();
   if(batteryReading > 480 && batteryReading < 640) lightBlueLED();
}

void RCTimoutCB(){
          movingLeft =0;
        movingRight =0;
 looseMotors(); 
}
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(A0,INPUT);
  pinMode(sensorPower,OUTPUT);
  digitalWrite(sensorPower,HIGH);
  irrecv.enableIRIn(); // Start the receiver
  // initialize digital pin 13 as an output.
  setupLED();
  //motorLeft->setOnTimer(&motorLeftUpdate);
  batteryEvent = t.every(1000, checkBatt);
  motorLeftEvent = t.every(speedLeft, motorLeftUpdate);
  motorRightEvent = t.every(speedRight, motorRightUpdate);
  RCTimout = t.after(100,RCTimoutCB);
}
int savedLeft = 0;
// the loop function runs over and over again forever
void loop() {
  if (irrecv.decode(&results)){
    lightBlueLED();
    if(translateIR(results) == 2){
        dirLeft = 0;
        dirRight = 0;
        movingLeft = 1;
        movingRight =1;
        lightGreenLED();
    }
     if(translateIR(results) == 5){
        followLine();
    }
    if(translateIR(results) == 8){
        dirLeft =1;
        dirRight = 1;
        movingLeft = 1;
        movingRight =1;
        lightGreenLED();
    }
        if(translateIR(results) == 4){
        dirLeft = 1;
        dirRight = 0;
        movingLeft = 1;
        movingRight =1;
        lightGreenLED();
    }
    if(translateIR(results) == 6){
        dirLeft =0;
        dirRight = 1;
        movingLeft = 1;
        movingRight =1;
        lightGreenLED();
    }
        if(translateIR(results) == 1){
          speedLeft --;
          if (speedLeft ==0) speedLeft = 1;
          speedRight = speedLeft;
          t.stop(motorLeftEvent);
          t.stop(motorRightEvent);
          motorLeftEvent = t.every(speedLeft, motorLeftUpdate);
          motorRightEvent = t.every(speedRight, motorRightUpdate);
        lightGreenLED();
    }
    if(translateIR(results) == 3){
          speedLeft ++;
          speedRight = speedLeft;
          t.stop(motorLeftEvent);
          t.stop(motorRightEvent);
          motorLeftEvent = t.every(speedLeft, motorLeftUpdate);
          motorRightEvent = t.every(speedRight, motorRightUpdate);
        lightGreenLED();
    }
    t.stop(RCTimout);
    RCTimout = t.after(100,RCTimoutCB);
    irrecv.resume(); // Receive the next value
  }
  //delay(10);              // wait for a second
   t.update();

  LEDsOff();



}

void followLine(){
  LEDsOff();
  int thresh;
  int offset1 = 20;
  int offset2 = 30;
  digitalWrite(sensorPower,LOW);
  thresh = analogRead(ir2);
  
  while(1){
    LEDsOff();
    t.update();
    digitalWrite(sensorPower,LOW);
     int value  = analogRead(ir2);
     digitalWrite(sensorPower,HIGH);
     
     //forwards
     if(value > thresh - offset1 && value < thresh + offset1){
        dirLeft = 0;
        dirRight = 0;
        movingLeft = 1;
        movingRight =1;
     }
     else{
       //turn one way
       if(value > thresh + offset1 && value < thresh +offset1 + offset2){
         //turn steeply
            dirLeft = 0;
            dirRight = 0;
            movingLeft = 0;
            movingRight =1;
       }
         //turn shallowly
       if(value > thresh +offset1 + offset2){
            dirLeft = 1;
            dirRight = 0;
            movingLeft = 1;
            movingRight =1;           
       }
       
       //turn the other way
       if(value > thresh - offset1 - offset2 && value < thresh -offset1){
            dirLeft = 0;
            dirRight = 0;
            movingLeft = 1;
            movingRight =0;           
         }
         //turn shallowly
         if(value < thresh - offset1 - offset2 ){
            dirLeft = 0;
            dirRight = 1;
            movingLeft = 1;
            movingRight =1;           
         }
       }
     }
     
  }
  


