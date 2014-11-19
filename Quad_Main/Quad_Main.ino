//  Base Quadcopter controller Script
//  Written by Matthew Hicks
//  2/10/2014

#include<Wire.h>
#include <Servo.h> 
 
Servo motor1;  // create servo object for motor1
Servo motor2;  // create servo object for motor1
Servo motor3;  // create servo object for motor1
Servo motor4;  // create servo object for motor1
               
 
int d0; //duration of channel 1 input (Alierons)
int d1; //duration of channel 2 input (Elevators)
int d2; //duration of channel 3 input (Throttle)
int d3; //duration of channel 4 input (Rudder)
int sigmin=1100;  //Minimum signal value accepted
int sigmid=1500;  //Middle signal value accepted
int sigmax=1900;  //Maximum signal value accepted
int throtout;  //Base throttle value received from receiver
int elevf=4;  //Scaling Factor for the elevators
int alief=4;  //Scaling factor for the ailerons
int throtf=1;  //Scaling factor for the throttle
int rudf=4;  //Scaling factor for the rudder
int m1t;  //Motor 1 throttle setting
int m2t;  //Motor 2 throttle setting
int m3t;  //Motor 3 throttle setting
int m4t;  //Motor 4 throttle setting
int maxthrot=130;  //Maximum throttle output value (Value may be required to be changed based on ESC calibration)
int minthrot=55;  //Mimimum throttle output value (Value may be required to be changed based on ESC calibration)

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup() 
{ 
  motor1.attach(8);  // attaches the servo on pin 8 to the servo object 
  motor1.write(0); 
  motor2.attach(9);  // attaches the servo on pin 9 to the servo object 
  motor2.write(0); 
  motor3.attach(10);  // attaches the servo on pin 10 to the servo object 
  motor3.write(0); 
  motor4.attach(11);  // attaches the servo on pin 11 to the servo object 
  motor4.write(0); 
  pinMode(2, INPUT); 
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  
  
  
  
  
  
} 
 
 
void loop() 
{ 
//measure pulses on channels 1-4, on pins D2-D5.
d0 = pulseIn(2, HIGH);
d1 = pulseIn(3, HIGH);
d2 = pulseIn(4, HIGH);
d3 = pulseIn(5, HIGH);

//Restricts throttle speed to given values.  
  if(d2<sigmin)d2=sigmin;
  if(d2>sigmax)d2=sigmax;
    throtout=(d2-sigmin)*100/(sigmax-sigmin)+50;
  if(throtout>180) throtout=180;
  
//Autobalance
Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);  
  
//Sum reqiured values for correseonding motor ESCs (Possibly use matrix for calulations)  
m1t=throtout-(d0-sigmid)*100/alief/(sigmax-sigmin)-(d1-sigmid)*100/elevf/(sigmax-sigmin);  //Front Right
m2t=throtout-(d0-sigmid)*100/alief/(sigmax-sigmin)+(d1-sigmid)*100/elevf/(sigmax-sigmin);  //Back Right
m3t=throtout+(d0-sigmid)*100/alief/(sigmax-sigmin)+(d1-sigmid)*100/elevf/(sigmax-sigmin);  //Back Left
m4t=throtout+(d0-sigmid)*100/alief/(sigmax-sigmin)-(d1-sigmid)*100/elevf/(sigmax-sigmin);  //Front Left


//Restrict motor throttle speeds to prvent negative values, and excessive values (motor controllers stop if too high value recieved.
if(m1t>maxthrot)m1t=maxthrot;
if(m1t<minthrot)m1t=minthrot;
if(m2t>maxthrot)m2t=maxthrot;
if(m2t<minthrot)m2t=minthrot;
if(m3t>maxthrot)m3t=maxthrot;
if(m3t<minthrot)m3t=minthrot;
if(m4t>maxthrot)m4t=maxthrot;
if(m4t<minthrot)m4t=minthrot;


//Send required motor speeds to the respective ESC
motor1.write(m1t); 
motor2.write(m2t);  
motor3.write(m3t);  
motor4.write(m4t);


//Serial.print(d0);
//Serial.print("  ");
//Serial.print(d1);
//Serial.print("  ");
//Serial.print(d2);
//Serial.print("  ");
//Serial.print(d3);
//Serial.print("  ");
//Serial.print(m1t);
//Serial.print("  ");
//Serial.print(m2t);
//Serial.print("  ");
//Serial.print(m3t);
//Serial.print("  ");
//Serial.println(m4t);
//Serial.print("  ");
//Serial.println(throtout);
delay(1);
} 
