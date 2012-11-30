#include "SoftwareSerial.h"

#define TX 3
#define RX 2
//#include <NewSoftSerial.h>

// set up a new serial port
//NewSoftSerial xbee =  NewSoftSerial(RX, TX);
SoftwareSerial xbee(RX,TX);


void setup()
{
 xbee.begin(9600);
 Serial.begin(9600); 
}

void loop()
{
  char data = xbee.read();
  if(data == 'B') {
    Serial.write(data);
  }
}
