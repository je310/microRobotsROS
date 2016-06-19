#include <SoftwareSerial.h>

#include "Timer-master/Timer.h"
#include "Timer-master/Event.h"
#include "rosToInterface.hpp"

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#include "IRremote/IRremote.h"
#include "IRremote/IRremoteInt.h"

#include "LED.h"
#include "ir.h"
#include "stepper.h"


// https://arduino-info.wikispaces.com/IR-RemoteControl this is where the codes are 


const int MyRobotNumber = 0;
const int NumRobots = 1;

int RECV_PIN = 4;
SoftwareSerial serial(RECV_PIN,-1);
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

int speedLeft = 275;
int speedRight = 275;

const int maxspeed = 275;
const int minspeed = 30000;

int movingLeft = 0;
int movingRight = 0;


// event 'handlers', we are allowed 10;
int  motorLeftEvent;
int  motorRightEvent;
int  batteryEvent;
int deactivateLeft;
int deactivateRight;
int RCTimout;

using namespace std;

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
 if(batteryReading >630)
   lightGreenLED();
 if(batteryReading > 300 && batteryReading < 480)
   lightRedLED();
 if(batteryReading > 480 && batteryReading < 640) lightBlueLED();

   LEDsOff();
}

void RCTimoutCB(){
   movingLeft =0;
   movingRight =0;
   looseMotors(); 
}

enum {
  nostate,
  linang,
} state;

// the setup function runs once when you press reset or power the board
void setup() {
  serial.begin(2400);
  pinMode(A0,INPUT);
  pinMode(sensorPower,OUTPUT);
  digitalWrite(sensorPower,HIGH);
  irrecv.enableIRIn(); // Start the receiver
  // initialize digital pin 13 as an output.
  setupLED();
  //motorLeft->setOnTimer(&motorLeftUpdate);
  batteryEvent = t.every(1000000, checkBatt);
  motorLeftEvent = t.every(speedLeft, motorLeftUpdate);
  motorRightEvent = t.every(speedRight, motorRightUpdate);
  RCTimout = t.after(100000,RCTimoutCB);

  state = nostate;
}
int savedLeft = 0;
// the loop function runs over and over again forever


size_t linangpointer;
unsigned char linangbuffer[NumRobots];

void RunLinAng(){
  lightRedLED();
  
  const unsigned linang = linangbuffer[MyRobotNumber];
  const int8_t lin = (linang & 0b11110000) >> 4;
  int8_t ang = linang & 0b00001111;

  if(ang & 0b00001000){
    ang |= 0b11110000;
  }
  const float flin = abs(lin) / 127.0f;
  const float fang = abs(ang) / 127.0f;

  const int lvel = (minspeed - maxspeed)*(1-flin)*(1+fang) + maxspeed;
  const int rvel = (minspeed - maxspeed)*(1-flin)*(1-fang) + maxspeed;

  speedLeft = lvel;
  speedRight = rvel;

  t.stop(motorLeftEvent);
  t.stop(motorRightEvent);
  motorLeftEvent = t.every(speedLeft, motorLeftUpdate);
  motorRightEvent = t.every(speedRight, motorRightUpdate);
  movingLeft = 1;
  movingRight =1;
  
  dirLeft = 1;
  dirRight = 1;
}


void loop() {
  unsigned char ch;
  int n, i;

 
  n = serial.available() ;
  if (n > 0) {
    i = n ;
    while (i--) {
      ch = serial.read() ;
      switch (state){
        case nostate:
        switch(ch){
          case CmdLINANG:
            state = linang;
            linangpointer = 0;   
            break;
 
          default:
          break;
        }
        break;
        
      case linang:
        linangbuffer[linangpointer++] = ch;
        if (linangpointer == NumRobots){
          RunLinAng();
          state = nostate;
        }
        break;

      default:
        break;
      }
    }
  }
  
  //delay(10);              // wait for a second
  t.update();


}

