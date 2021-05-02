#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 Wireless(8, 10); 
const byte WirelessAddress[6] = "00001";
float WirelessValues[2];
float y1;
void setup() {
  Serial.begin(9600);
  Wireless.begin();
  Wireless.openWritingPipe(WirelessAddress);
  Wireless.setPALevel(RF24_PA_MIN);
  Wireless.stopListening();
 
}
void loop() {
  int Joystick_Up_Down = analogRead(A1);
  int Joystick_Left_Right = analogRead(A0);

  int Map_Up_Down = map(Joystick_Up_Down, 0, 1023, -512, 512);
  float Fwd_Bck_Angle = (float) Map_Up_Down/200;

  int Map_Left_Right = map(Joystick_Left_Right, 0, 1023, -512, 512);
  float RotationValue = (float) Map_Left_Right/5;


   WirelessValues[0] = Fwd_Bck_Angle;
   WirelessValues[1] = RotationValue;
 
  Wireless.write(&WirelessValues, sizeof(WirelessValues));

}
