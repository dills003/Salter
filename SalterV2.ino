#include <ax12.h>
#include <BioloidController.h>


float const EMAconstantFastTrackTurn = 0.2;
float const EMAconstantLeftTrackTrim = 0.2;
float const EMAconstantTrackSpeed = 0.2;
float const EMAconstantTrackTurn = 0.2;
float const EMAconstantSalterOnOff = 0.2;
float const EMAconstantSalterSpeed = 0.2;
//constants for ranges of EMA
int const AnalogWriteLow = 0;
int const AnalogWriteHigh = 255;
int const EMAmiddleLow = 45;
int const EMAmiddleHigh = 55;
int const EMAtooLow = 30;
int const EMAtooHigh = 80;

int EMAFastTrackTurn = 0;
int EMALeftTrackTrim = 0;
int EMATrackSpeed = 0;
int EMATrackTurn = 0;
int EMASalterOnOff = 0;
int EMASalterSpeed = 0;
int rcValueFastTrackTurn = 0;
int rcValueLeftTrackTrim = 0;
int rcValueTrackSpeed = 0;
int rcValueTrackTurn = 0;
int rcValueSalterOnOff = 0;
int rcValueSalterSpeed = 0;
int FastTrackTurn = 0;
int LeftTrackTrim = 0;
int TrackSpeed = 0;
int LeftTrackPWM = 0;
int RightTrackPWM = 0;
int TrackTurn = 0;
int SalterOnOff = 0;
int SalterSpeed = 0;


//track Motors
#define LeftTrackIN1    2    
#define LeftTrackIN2    1
#define LeftTrackPWMPin    3
#define RightTrackPWMPin   4
#define RightTrackIN1   5    
#define RightTrackIN2   6
//Salter Smart Servo
#define SalterWheel 17 //Smart Servo
#define Stop 0


void setup() {
  //initialize serial communication at 9600 bit/s
  Serial.begin(9600);
  //Initialize the Pins
  pinMode(A0, INPUT); //Right Stick - Left & Right
  pinMode(A1, INPUT); //Right Stick - Up & Down
  pinMode(A2, INPUT); //Left Stick - Up & Down
  pinMode(A3, INPUT); //Left Stick - Left & Right
  pinMode(A4, INPUT); //Right Top Switch - Salter On
  pinMode(A5, INPUT); //Left VRB - Speed
  pinMode(1, OUTPUT); //Track Motor Control
  pinMode(2, OUTPUT); //Track Motor Control
  pinMode(3, OUTPUT); //Track Motor Control PWM
  pinMode(4, OUTPUT); //Track Motor Control PWM
  pinMode(5, OUTPUT); //Track Motor Control
  pinMode(6, OUTPUT); //Track Motor Control
  
  
  //Sets the smart Servo's wheel mode
  ax12SetRegister2(SalterWheel, 6, 0); // write two bytes to memory (ADDR, ADDR+1) //for wheel
  ax12SetRegister2(SalterWheel, 8, 0); //or zero for rotation 1023 joint
  
  
  //Stop Everything
  SetSpeed(SalterWheel, Stop); //Sets the Salter to Zero Speed
  
}


void loop() {
   
  //Get all of the Current RC Values
  rcValueTrackTurn = analogRead(A0);
  rcValueLeftTrackTrim = analogRead(A1);
  rcValueTrackSpeed = analogRead(A2);
  rcValueFastTrackTurn = analogRead(A3);
  rcValueSalterOnOff = analogRead(A4);
  rcValueSalterSpeed = analogRead(A5);
  
  //Plug all of the values into a moving average to smooth them out
  EMAFastTrackTurn = (EMAconstantFastTrackTurn * rcValueFastTrackTurn) + ((1-EMAconstantFastTrackTurn) * EMAFastTrackTurn); //
  EMALeftTrackTrim = (EMAconstantLeftTrackTrim * rcValueLeftTrackTrim) + ((1-EMAconstantLeftTrackTrim) * EMALeftTrackTrim);
  EMATrackSpeed = (EMAconstantTrackSpeed * rcValueTrackSpeed) + ((1-EMAconstantTrackSpeed) * EMATrackSpeed);
  EMATrackTurn = (EMAconstantTrackTurn * rcValueTrackTurn) + ((1-EMAconstantTrackTurn) * EMATrackTurn);
  EMASalterOnOff = (EMAconstantSalterOnOff * rcValueSalterOnOff) + ((1-EMAconstantSalterOnOff) * EMASalterOnOff);
  EMASalterSpeed = (EMAconstantSalterSpeed * rcValueSalterSpeed) + ((1-EMAconstantSalterSpeed) * EMASalterSpeed);
  
  //Get all of the Speeds of Stuff
  
  
  //***********************Set the Salter Speed****************************
  if (EMASalterOnOff >= EMAmiddleHigh && EMASalterOnOff < EMAtooHigh)
  {
    SalterSpeed = (30.088 * EMASalterSpeed) - 992.91;
    if (SalterSpeed > 1023)
    {
      SalterSpeed = 1023;
    }
    SetSpeed(SalterWheel, SalterSpeed);
  }
  else
  {
    SetSpeed(SalterWheel, Stop);
  }
  
  
  //***********************Set the Track Stuff, PWM goes from 0-255(********************
  /*We have 9 States
  Forward w/o turn
  Forward w/left turn
  Forward w/right turn
  Reverse w/o turn
  Reverse w/left turn
  Reverse w/right turn
  Quick turn left
  Quick turn right
  Stop
 
 if ema speed says forward, we need to set both tracks forward and
 figure out a correct pwm based on the forwardness/turnness
 All the way left is no track speed on left and full on right
 All the way right is no track speed on right and full on left
 
 Turn in place is forward/back is in the middle and both left or both right
 */
 
 //Find the State
 if ((EMATrackSpeed >= EMAmiddleHigh && EMATrackSpeed < EMAtooHigh) || (EMATrackSpeed <= EMAmiddleLow && EMATrackSpeed > EMAtooLow)) //we are not stopping or quick turn
 {
   if (EMATrackTurn > EMAmiddleLow && EMATrackTurn < EMAmiddleHigh) //we are trying to just drive straight
   {
     LeftTrackPWM = GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh);//calculate the pwm for both of the motors
     RightTrackPWM = LeftTrackPWM;     
   }
   else if (EMATrackTurn <= EMAmiddleLow) //we are trying to turn left
   {
     LeftTrackPWM = GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh) - GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh);
     RightTrackPWM = GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh);
   }
   else if (EMATrackTurn >= EMAmiddleHigh) //we are trying to turn right
   {
     LeftTrackPWM = GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh);
     RightTrackPWM = GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh) - GetPWM(EMATrackSpeed, EMAmiddleLow, EMAmiddleHigh);
   }
   
   //Sets the direction of the Robot
   if (EMATrackSpeed >= EMAmiddleHigh && EMATrackSpeed < EMAtooHigh) //driving forward
   {
     DriveForward(); //sets the tracks to forward
   }
   else
   {
     DriveReverse();
   }
 }
 else if (EMATrackSpeed < EMAmiddleHigh && EMATrackSpeed > EMAmiddleLow && ((EMATrackTurn <= EMAmiddleLow && EMAFastTrackTurn <= EMAmiddleLow) || (EMATrackTurn >= EMAmiddleLow && EMAFastTrackTurn >= EMAmiddleHigh))) //Fast Trun in Place
 {
   //calculate the speed of each of the tracks
   if (EMATrackTurn <= EMAmiddleLow && EMAFastTrackTurn <= EMAmiddleLow) //fast turn to the left
   {
     QuickTurnLeft();
   }
   else if (EMATrackTurn >= EMAmiddleLow && EMAFastTrackTurn >= EMAmiddleHigh)
   {
     QuickTurnRight();
   }
   LeftTrackPWM = GetPWM(EMATrackTurn, EMAmiddleLow, EMAmiddleHigh);
   RightTrackPWM = LeftTrackPWM;
 }
 else
 {
   LeftTrackPWM = 0;
   RightTrackPWM = 0;
   DriveStop();
 }
     
   
   //check mins and maxs before setting track speeds
   if (LeftTrackPWM > 255)
   {
     LeftTrackPWM = 255;
   }
   else if (LeftTrackPWM < 0)
   {
     LeftTrackPWM = 0;
   }
   //check mins and maxs
   if (RightTrackPWM > 255)
   {
     RightTrackPWM = 255;
   }
   else if (RightTrackPWM < 0)
   {
     RightTrackPWM = 0;
   }
   
   analogWrite(LeftTrackPWMPin, LeftTrackPWM); //sets the PWM
   analogWrite(RightTrackPWMPin, RightTrackPWM);
      
  /*Serial.print("Fast Track Turn: ");
  Serial.println(FastTrackTurn);
  Serial.print("Left Track Trim: ");
  Serial.println(LeftTrackTrim);
  Serial.print("Track Speed: ");
  Serial.println(TrackSpeed);
  Serial.print("Track Turn: ");
  Serial.println(TrackTurn);
  Serial.print("Salter On Off: ");
  Serial.println(SalterOnOff);
  Serial.print("Salter Speed: ");
  Serial.println(SalterSpeed);  
  delay(2000);*/
  
  //Serial.print("Analog Channels: ");
  //Serial.println(analogRead(A7));
  //delay(200);
  
  //Serial.print("Digital Check: ");
  //Serial.println(digitalRead(6));
  //delay(200);
  
  
}



/****************************These are my functions******************************/

/*****************************Start of Drive Forward**************************/
void DriveForward()
{
  //set the drive pins
  digitalWrite(LeftTrackIN1, HIGH);
  digitalWrite(LeftTrackIN2, LOW);
  digitalWrite(RightTrackIN1, HIGH);
  digitalWrite(RightTrackIN2, LOW);
  
  //set the PWM of each track
}
/*****************************End of Drive Forward***************************/

/*****************************Start of Drive Reverse**************************/
void DriveReverse()
{
  //set the drive pins
  digitalWrite(LeftTrackIN1, LOW);
  digitalWrite(LeftTrackIN2, HIGH);
  digitalWrite(RightTrackIN1, LOW);
  digitalWrite(RightTrackIN2, HIGH);
  
  //set the PWM of each track
}
/*****************************End of Drive Reverse***************************/ 

/*****************************Start of Quick Turn Left**************************/
void QuickTurnLeft()
{
  //set the drive pins
  digitalWrite(LeftTrackIN1, LOW);
  digitalWrite(LeftTrackIN2, HIGH);
  digitalWrite(RightTrackIN1, HIGH);
  digitalWrite(RightTrackIN2, LOW);
  
  //set the PWM of each track
}
/*****************************End of Quick Turn Left***************************/ 

/*****************************Start of Quick Turn Right**************************/
void QuickTurnRight()
{
  //set the drive pins
  digitalWrite(LeftTrackIN1, HIGH);
  digitalWrite(LeftTrackIN2, LOW);
  digitalWrite(RightTrackIN1, LOW);
  digitalWrite(RightTrackIN2, HIGH);
  
  //set the PWM of each track
}
/*****************************End of Quick Turn Right***************************/

/*****************************Start of Stop**************************/
void DriveStop()
{
  //set the drive pins
  digitalWrite(LeftTrackIN1, LOW);
  digitalWrite(LeftTrackIN2, LOW);
  digitalWrite(RightTrackIN1, LOW);
  digitalWrite(RightTrackIN2, LOW);
  
  //set the PWM of each track
}
/*****************************End of Stop**************************/ 

/*****************************Start of Find PWM**************************/
//7.5x - 247.5
int GetPWM(int EMAvalue, int EMAlow, int EMAhigh)
{
  int tempValue= 0;
  if (EMAvalue >= EMAhigh)
  {
    tempValue = (21.25 * EMAvalue) - 1168.8;
  }
  else if (EMAvalue <= EMAlow)
  {
    tempValue = (-21.25 * EMAvalue) + 956.25;
  }
  else
  {
    tempValue = 0;
  }
  
  if (tempValue > 255)
  {
    tempValue = 255;
  }
  else if (tempValue < 0)
  {
    tempValue = 0;
  }
  
  return tempValue;
  
}

/*****************************End of Find PWM**************************/
