// How to send data from the LEGO Mindstorms NXT to the Arduino. For LEGO Mindstorms
// Demonstrates how to connect a LEGO MINDSTORMS to an Arduino and Send commands, receive data.
// A4 – SDA D32         A5 – SCL D33
// See our website, www.dexterindustries.com/howto for more information on the physical setup.
#include <Wire.h>
#define MOTOR1 (0b01000000)
#define MOTOR2 (0b10000000)
byte read_register = 0x00;
unsigned char var_h_hjul;         
unsigned char var_v_hjul;
unsigned char kommando_signal;


void setup()
{
  Serial.begin(115200);
  Wire.begin(0x0A);                 // join i2c bus with address #2
  Wire.onRequest(requestEvent);     // Sending information back to the NXT!
  Wire.onReceive(receiveI2C);       // Receiving information!
}

void loop()
{
  if (Serial.available())
  {
    unsigned char reading = Serial.read();
    delay(10);
    if ((reading & MOTOR2) == MOTOR2)
    {
      var_v_hjul = reading & (~(1 << 7));        //var_v_hjul sættes til reading og andes med 01111111 
    }
    else if ((reading & MOTOR1) == MOTOR1)
    {
      var_h_hjul = reading & (~(1 << 6));         //var_h_hjul sættes til reading og andes med 10111111 
    }
  }
}

// When data is received from NXT, this function is called.
void receiveI2C(int bytesIn)        //Denne funktion kaldes når NXT beder om information ved at sende information
{
  read_register = bytesIn;
  while (1 < Wire.available())          // loop through all but the last når klokken er åben
  {
    read_register = Wire.read();        // The incoming byte tells us the register we will use.
  }
  int x = Wire.read();                  // Read the incoming byte
}

void requestEvent()                         //Heri skal den data NXT skal have stå!  We’re going to send a fixed response.
{
  if (read_register == 0x01) {
    Wire.write(var_h_hjul);                // respond with message of 10 bytes
  }
  else {                                    //if (read_register == 0x00)
    Wire.write(var_v_hjul);                 // respond with message of 1 bytes as expected by master
  }
}
