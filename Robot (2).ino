#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <L298N.h>
#include <Servo.h>
#include <L298N.h>


 RF24 Wireless(4, 10); 
 const byte WirelessAddress[6] = "00001"; 
 float Fwd_Bck_Angle = 0, RotationValue = 0; 
 float WirelessValues[2]; 

 const unsigned int EN_A = 3; 
 const unsigned int IN1_A = 5;
 const unsigned int IN2_A = 6;
 const unsigned int IN1_B = 7;
 const unsigned int IN2_B = 8;
 const unsigned int EN_B = 9;


 L298N MotorR(EN_A, IN1_A, IN2_A);
 L298N MotorL(EN_B, IN1_B, IN2_B);

 int Left = 0; 
 int Right = 0;
 float SpeedRotation = 1; 


 int16_t Acceleration_X, Acceleration_Y, Acceleration_Z,Gyroscope_X, Gyroscope_Y, Gyroscope_Z; 
 

 float Acc_Angle[2];
 float Gyr_Angle[2];
 float Tot_Angle[2];


 int topspeed = 300;
 float targetvalue = 0;

 float TimePassed, time, PreviousTime;
 int i;
 float RadiansToDegrees = 180/3.141592654;

 float PID, error, previous_error;
 float PID_Proportional=0;
 float PID_Intergral=0;
 float PID_Derivative=0;

 double Kp=20.55;
 double Ki=0.02;
 double kD=0.8;
 float angle_wanted = 0; 
                         


void setup() {
  
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(19200);
  time = millis(); 
   
  Wireless.begin();
  Wireless.openReadingPipe(0, WirelessAddress);
  Wireless.setPALevel(RF24_PA_MIN);
  Wireless.startListening();

}

void loop() {
 if (Wireless.available()) {
    Wireless.read(&WirelessValues, sizeof(WirelessValues)); 
    Fwd_Bck_Angle = WirelessValues[0]; 
    RotationValue = WirelessValues[1];
  } 

    PreviousTime = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    TimePassed = (time - PreviousTime) / 1000; 
  
  
if (abs(RotationValue) < 15) { 
    RotationValue = 0;
  }

  if (abs(Fwd_Bck_Angle) < .17) {
    Fwd_Bck_Angle = 0;
  }
  
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); 
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   
     Acceleration_X=Wire.read()<<8|Wire.read(); 
     Acceleration_Y=Wire.read()<<8|Wire.read();
     Acceleration_Z=Wire.read()<<8|Wire.read();

 
    
     Acc_Angle[0] = atan((Acceleration_Y/16384.0)/sqrt(pow((Acceleration_X/16384.0),2) + pow((Acceleration_Z/16384.0),2)))*RadiansToDegrees;
     
     Acc_Angle[1] = atan(-1*(Acceleration_X/16384.0)/sqrt(pow((Acceleration_Y/16384.0),2) + pow((Acceleration_Z/16384.0),2)))*RadiansToDegrees;
 
  
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); 
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); 
   
   Gyroscope_X=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyroscope_Y=Wire.read()<<8|Wire.read();
 
 

   Gyr_Angle[0] = Gyroscope_X/131.0; 
  
   Gyr_Angle[1] = Gyroscope_Y/131.0;


   Tot_Angle[0] = 0.98 *(Tot_Angle[0] + Gyr_Angle[0]*TimePassed) + 0.02*Acc_Angle[0];

   Tot_Angle[1] = 0.98 *(Tot_Angle[1] + Gyr_Angle[1]*TimePassed) + 0.02*Acc_Angle[1];
   

  
   
  
error = Tot_Angle[1] - angle_wanted - targetvalue;
    

PID_Proportional = Kp*error;


if(-3 <error <3)
{
  PID_Intergral = PID_Intergral+(Ki*error);  
}


PID_Derivative = kD*((error - previous_error)/TimePassed);


PID = PID_Proportional + PID_Intergral + PID_Derivative; // All three values give the balance.




if(PID < -topspeed)
{
  PID=-topspeed;
}
if(PID > topspeed)
{
  PID=topspeed;
}


if(angle_wanted == 0){
if(PID < 0)
targetvalue += 0.0015;
if(PID > 0)
targetvalue -=0.0015;
   }

//NEW SHIT
//Move forward by adjusting error value
if(Fwd_Bck_Angle > 0)
{
 error = error + 4;
}
if(Fwd_Bck_Angle < 0)
{
  error = error - 4;
}


Left = PID - RotationValue*SpeedRotation;  
Right =  PID + RotationValue*SpeedRotation;


 if (Tot_Angle[1] > 40 || Tot_Angle[1] < -40) { 
   MotorL.setSpeed(0);
    MotorR.setSpeed(0); 
 } else {
   MotorL.setSpeed(abs(Left));
    MotorR.setSpeed(abs(Right));
  }


  if (Left < 0) { 
    MotorL.forward();
  } else {
    MotorL.backward();
  }
  
  if (Right < 0) { 
    MotorR.forward();
  } else {
    MotorR.backward();
  }
    
previous_error = error; 


if(PID < 5 & PID > -5) PID = 0;


}
