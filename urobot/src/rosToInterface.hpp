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

//this is to define structures ect for communicating over the serial from the ros system to the IR distribution board.


//definitions for enumerations
//#define RqstBATTERY 0
//#define RqstRSENSOR 1
//#define RqstLSENSOR 2
//#define RqstBSENSOR 3

//#define CmdLINEAR 4
//#define CmdANGULAR 5
//#define CmdLED 6
//#define CmdSPECIAL 7
//#define CmdLINANG 8

#define ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h> //needed for memcpy

enum instructType{
RqstBATTERY=0,
RqstRSENSOR=1,
RqstLSENSOR=2,
RqstBSENSOR=3,

CmdLINEAR=4,
CmdANGULAR=5,
CmdLED=6,
CmdSPECIAL=7,
CmdLINANG=8
};


//one packet is sent over for each robot, as we do not know at any one time how many robots there are. The bandwidth is plenty to allow us to be verbose.
struct instructionPack {
    uint8_t instructionType; //for the moment we are thinking this should alaways start with 1010. giving 16 options for instruction types.
    uint8_t instructionID;
    uint8_t robotID;
    uint8_t value1;
};

union instructionUnion {
    instructionPack pack;
    char bytes[sizeof(instructionPack)];
};
