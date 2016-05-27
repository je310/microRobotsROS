#include "ir.h"
#include "Arduino.h"



int translateIR(decode_results results) // takes action based on IR code received

// describing KEYES Remote IR codes 

{
  static int latched = 1;

  switch(results.value)

  {

  case 0xFF6897: latched = 1; return 1;//Serial.println(" 1");    break;
  case 0xFF9867: latched = 2; return 2;//Serial.println(" 2");    break;
  case 0xFFB04F: latched = 3; return 3;//Serial.println(" 3");    break;
  case 0xFF30CF: latched = 4; return 4;//Serial.println(" 4");    break;
  case 0xFF18E7: latched = 5; return 5;//Serial.println(" 4");    break;
  case 0xFF7A85: latched = 6; return 6;//Serial.println(" 1");    break;
  case 0xFF10EF: latched = 7; return 7;//Serial.println(" 2");    break;
  case 0xFF38C7: latched = 8; return 8;//Serial.println(" 3");    break;
  case 0xFF5AA5: latched = 9; return 9;//Serial.println(" 4");    break;
  case 0xFF4AB5: latched = 0; return 0;//Serial.println(" 4");    break;
  case 0xFFFFFFFF: return latched;
  }


} //END translateIR
