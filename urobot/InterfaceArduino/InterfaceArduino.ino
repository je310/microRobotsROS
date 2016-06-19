/*Copyright (c) 2016 "Joshua Elsdon,"
Micro Robots Project

This file is part of Micro Robots.

Micro Robots is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

#include <SoftwareSerial.h>

#include "rosToInterface.hpp"
#define numberOfRobots  1 //this should be set up nicly
#define myID 0
int LED = 13;
char commandQ[numberOfRobots + 1]; //one for the command type
void addEntry(instructionUnion thisUnion);
void sendIR();
int robotsRecieved = 0;
const int IROUT = 11;
const int IRRecvPin = 3;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define SYSCLOCK 16000000  // main system clock (Hz)
#define PULSECLOCK 38000  // Hz

SoftwareSerial irserial(IRRecvPin,-1);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  irserial.begin(2400);
  cbi(TCCR2A,COM2A1) ; // connect OC2A (COM2A0 = 1)
  sbi(TCCR2A,COM2A0) ;
 
  cbi(TCCR2B,WGM22) ;  // CTC mode for TIMER2
  sbi(TCCR2A,WGM21) ;
  cbi(TCCR2A,WGM20) ;
  
  TCNT2 = 0 ;
  
  cbi(ASSR,AS2) ;  // use system clock for timer 2
  
  OCR2A = 255 ;   // set TOP to 255 for now
  
  cbi(TCCR2B,CS22) ;  // TIMER2 prescale = 1
  cbi(TCCR2B,CS21) ;
  sbi(TCCR2B,CS20) ;
  
  cbi(TCCR2B,FOC2A) ;  // clear forced output compare bits
  cbi(TCCR2B,FOC2B) ;
 
  pinMode(IROUT, OUTPUT) ;  // set OC2A to OUPUT  
  OCR2A = timer2top(PULSECLOCK) ;
  sei() ;
}
instructionUnion thisUnion;
int bytesCollected = 0;
void loop() {
  if (Serial.available() > 0) {
                // read the incoming byte:
                thisUnion.bytes[bytesCollected] = Serial.read();
                bytesCollected++;
                if(bytesCollected == sizeof thisUnion){
                  bytesCollected = 0;
                  addEntry(thisUnion); 
                }

        }
}

void addEntry(instructionUnion thisUnion){
  static int currentCID = 0;
  if(thisUnion.pack.robotID == 0){
      commandQ[0] = thisUnion.pack.instructionType;
      currentCID = thisUnion.pack.instructionID;
      
  }
  commandQ[thisUnion.pack.robotID + 1] = thisUnion.pack.value1;
  if(thisUnion.pack.robotID == numberOfRobots){
    sendIR();
  }
}

void sendIR(){

  Serial.write(commandQ, sizeof(commandQ)); 

}

// return TIMER2 TOP value per given desired frequency (Hz)
uint8_t timer2top(unsigned long freq) {
 return((byte)((unsigned long)SYSCLOCK/2/freq) - 1) ;
}
