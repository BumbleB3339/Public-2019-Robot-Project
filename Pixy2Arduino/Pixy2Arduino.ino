//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP SPI port.  For more information go here:
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
  
#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;
String toSend;
unsigned long last;
String input;

void setup(){
  Serial.begin(115200);
  pixy.init();
}

void loop(){ 
  int i; 
  // grab blocks!
  while(!Serial.available());
  input = String(Serial.read());
  if(input=="48"){
    pixy.ccc.getBlocks();
    
    // If there are detect blocks, print them!
    if (pixy.ccc.numBlocks){
      toSend = String(pixy.ccc.numBlocks)+" ";
  
      for (i=0; i<pixy.ccc.numBlocks; i++){
        toSend += String(pixy.ccc.blocks[i].m_signature)
          + "," + String(pixy.ccc.blocks[i].m_x)
          + "," + String(pixy.ccc.blocks[i].m_y)
          + "," + String(pixy.ccc.blocks[i].m_width)
          + "," + String(pixy.ccc.blocks[i].m_height)
          + "," + String(pixy.ccc.blocks[i].m_index) + ";";
      }
      Serial.println(toSend);
      Serial.flush();
    }
  }
}
