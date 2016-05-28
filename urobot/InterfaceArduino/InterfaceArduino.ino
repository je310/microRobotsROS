#include <IRremoteInt.h>
#include <IRremote.h>
IRsend irsend;
#include "/home/josh/uRobot_ws/src/urobot/src/rosToInterface.hpp"
#define numberOfRobots  1 //this should be set up nicly
#define myID 0
int LED = 13;
char commandQ[numberOfRobots + 1]; //one for the command type
void addEntry(instructionUnion thisUnion);
void sendIR();
int robotsRecieved = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
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
  switch(commandQ[0]){
    case CmdLINANG:
    digitalWrite(LED, HIGH);
    int8_t lin = commandQ[1] & 0b11110000;
    int8_t ang = commandQ[1] & 0b00001111;
    lin = lin >> 4;
    if(ang & 0b00001000){ //extend the top bit to recover 2s comp.
      ang = ang | 0b11110000;
    }
    if(lin == 1){
      irsend.sendSony(0xFF9867, 32);
    }
    if(lin == - 1){
      irsend.sendSony(0xFF38C7, 32);
    }
    if(ang == 1){
      irsend.sendSony(0xFF30CF, 32);
    }
    if(ang == - 1){
      irsend.sendSony(0xFF7A85, 32);
    }
  };
}
