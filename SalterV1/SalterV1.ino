
#include <ax12.h>
#include <BioloidController.h>

float EMAconstantTurn = 0.2;
float EMAconstantSpeed = 0.2;
int EMAexpTurn = 0;
int EMAexpSpeed = 0;
int turnPos = 0;
int rightDriveSpeed = 0;
int leftDriveSpeed = 0;

//Smart Servo Motors
#define SalterWheel 3
#define Stop 0

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  //Serial.begin(9600);
  pinMode(A0, INPUT); //Right Stick - Left & Right
  pinMode(A1, INPUT); //Right Stick - Up & Down
  pinMode(A2, INPUT); //Left Stick - Up & Down
  pinMode(A3, INPUT); //Left Stick - Left & Right
  pinMode(A4, INPUT); //Right Top Switch - Salter On
  pinMode(A5, INPUT); //Left VRB - Speed
  
  
  //Stop the Wheels
  
  //Sets to wheel mode
  ax12SetRegister2(SalterWheel, 6, 0); // write two bytes to memory (ADDR, ADDR+1) //for wheel
  ax12SetRegister2(SalterWheel, 8, 0); //or zero for rotation 1023 joint

  
  SetSpeed(SalterWheel, 0); //Sets the Salter to Zero Speed

}

// the loop routine runs over and over again forever:
void loop() {

  //int rcValueSpeed = analogRead(0);
  //int rcValueTurn = analogRead(1);
  int rcValueSalterOn = analogRead(4);
  int rcValueSalterSpeed = analogRead(5);
  
  //EMAexpTurn = (EMAconstantTurn * rcValueTurn) + ((1-EMAconstantTurn) * EMAexpTurn);
  EMAexpSpeed = (EMAconstantSpeed * rcValueSalterSpeed) + ((1-EMAconstantSpeed) * EMAexpSpeed);
 
 
 /*/***************To Turn************************************ 
  if ((EMAexpTurn >= 48 && EMAexpTurn <= 52) || (EMAexpTurn < 30) || (EMAexpTurn > 80))
  {
    SetPosition(LeftTurnWheel, Straight);
    SetPosition(LeftFixedWheel, Straight);
    SetPosition(RightTurnWheel, Straight);
    SetPosition(RightFixedWheel, Straight);
  }
  else if (EMAexpTurn > 52)
  {
    turnPos = (39.23 * EMAexpTurn) - 1566.231;
    if (turnPos > 925)
    {
      turnPos = 1023;
    }
    SetPosition(LeftTurnWheel, turnPos);
    SetPosition(RightTurnWheel, turnPos);
  }
  else if (EMAexpTurn < 48)
  {
    turnPos = (36.5 * EMAexpTurn) - 1204.5;
    if (turnPos < 50)
    {
      turnPos = 0;
    }
    SetPosition(LeftTurnWheel, turnPos);
    SetPosition(RightTurnWheel, turnPos);
  }  
*/

//***************To Drive the Salt Wheel************************************   
  if ((EMAexpSpeed >= 48 && EMAexpSpeed <= 52) || (EMAexpSpeed < 30) || (EMAexpSpeed > 80))
  {
    SetSpeed(SalterWheel, Stop);
    
  }
  else if (EMAexpSpeed > 52)
  {
    rightDriveSpeed = (78.62 * EMAexpSpeed) - 3141.62;
    leftDriveSpeed = (78.62 * EMAexpSpeed) -4165.62;
    if (rightDriveSpeed > 2000)
    {
      rightDriveSpeed = 2047;
    }
    if (leftDriveSpeed > 950)
    {
      leftDriveSpeed = 1023;
    }
    SetSpeed(SalterWheel, leftDriveSpeed);
  }
  else if (EMAexpSpeed < 48)
  {
    rightDriveSpeed = (-73 * EMAexpSpeed) + 3432;
    leftDriveSpeed = (-73 * EMAexpSpeed) + 4465;
    if (rightDriveSpeed > 950)
    {
      rightDriveSpeed = 1023;
    }
    if (leftDriveSpeed > 2000)
    {
      leftDriveSpeed = 2047;
    }
    SetSpeed(SalterWheel, leftDriveSpeed);
  }   

/*
  Serial.print("Turn Position: ");
  Serial.println(turnPos);
  Serial.print("Left Drive Speed: ");
  Serial.println(leftDriveSpeed);
  Serial.print("Right Drive Speed: ");
  Serial.println(rightDriveSpeed);
  delay(200);
  
  
  */
}



