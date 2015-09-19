//Written by Kenton Smith
//Last Updated 12/1/2013

//Include Arduino Servo Library

#include <Servo.h>;
#include "Arduino.h";

Servo KentonServo;

//Label Each Digital to Something Pertaining to its Function 
//And Set it Equal to a Pin Number on the Arduino

const int LEDFlasherPin = 2;
const int TactileSwitchPin = 3;
const int PowerGearMotorPin = 5;
const int DirectionGearMotorPin = 4; //Ground is Backwards rotation (towards the circuit; Positive is forwards rotation (away from circuit)
const int LimitSwitchFrontPin = 6;
const int LimitSwitchBackPin = 7;
const int ElectromagnetPin = 10;
const int ServoMotorPin = 9;

const unsigned long MaxMotorTimeOn = 2500;
boolean DirectionForward = false;

//Infrared Sensor Readings
const int MinServoAngle = 0;
const int MaxServoAngle = 180;
int InfraredVoltageReadings [MaxServoAngle + 1];

// In worst case (average size N = 1), array would be as large as raw data array
int RollingAverageIrReadings [MaxServoAngle + 1];

const int ThresholdIRVoltage = 585; //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// assumes: 10 samples, each one degree apart.
const int RollingAverageSampleSize = 5; //Bob's

//const int RollingAverageWidth = 5;

//int RollingAverageSet [(MaxServoAngle/RollingAverageWidth) + 1];   //IS THIS RIGHT?!?!?!?!?

const int InvalidAngle = -1;
const int InvalidPin = -1;
boolean UsbConnectedToPc = true;
boolean startbutton = false;
const int AngleOffset = 23; //used to account for differences in position for electromagnet and infrared scanner.  also here for slight imperfections in algorithm.

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// http://playground.arduino.cc/Code/AvailableMemory
void LogFreeMemory(void)
{
    //Serial.print("freeMemory()=");
    //Serial.println(freeMemory());
    Serial.print("freeRam()=");
    Serial.println(freeRam());    
    delay(1000);
}

void ArmMove(boolean Forward)
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
  //Moves Gear Motor to Scan Position
  int LimitSwitchPin = InvalidPin;
  int Acceleration = 0;
  const unsigned long RampUpTime = 250;
  const unsigned long MaxPowerTime = 250;
  String msg;
  
  //Serial.println(F("ArmMove() - Start"));
  DebugPrint("ArmMove() - Start");
  
  int MaxMotorSpeed = 0; //scale of 0-255 analogWrite(255) is 100%
  int SteadyMotorSpeed = 0;
  
  if (Forward == true)
  {
    digitalWrite(DirectionGearMotorPin, HIGH); //Will rotate forward
    LimitSwitchPin = LimitSwitchFrontPin;  
    Acceleration = 5;
    MaxMotorSpeed = 40; 
    SteadyMotorSpeed = 20;     
  }
  else
  {
    digitalWrite(DirectionGearMotorPin, LOW); //Will rotate forward
    LimitSwitchPin = LimitSwitchBackPin;
    Acceleration = 15;
    MaxMotorSpeed = 255;  
    SteadyMotorSpeed = 175;    
  }
  
  unsigned long StartTime = 0;
  StartTime = millis();
  
  Serial.print(F("StartTime = "));
  Serial.println(StartTime);
  
  //Checks if it hits the FrontLimitSwitch or its time on exceeds MaxMotorTimeOn
  int MotorSpeed = 0;
  unsigned long ElapsedTime = 0;
  while(true)
  {
    ElapsedTime = (millis() - StartTime);
    if (ElapsedTime > MaxMotorTimeOn)
    {
     break; 
    }  
    else
    {
      if (ElapsedTime < RampUpTime)
      {
        MotorSpeed += Acceleration;
//        if (MotorSpeed > MaxMotorSpeed)
//        {
//          MotorSpeed = MaxMotorSpeed;      
//        }
        // sanity check
        MotorSpeed = constrain(MotorSpeed, 0, MaxMotorSpeed);
      }
      else 
      {  
        if (ElapsedTime < (MaxPowerTime + RampUpTime))
        {
          MotorSpeed = MaxMotorSpeed;
        }
        else
        {
          MotorSpeed = SteadyMotorSpeed;  
        }
      } 
    }
//    msg = "MotorSpeed = ";
//    msg += MotorSpeed;
    
    analogWrite(PowerGearMotorPin, MotorSpeed);
        
    if (digitalRead(LimitSwitchPin) == LOW)
    { 
      Serial.println(F("Break Out Because Limit Switch Was Pressed")); 
      break;
    }  
    if ((millis() - StartTime) > MaxMotorTimeOn)
    {
      //Serial.println(F("Break Out Because Of Timeout")) ;
      DebugPrint("Break Out Because Of Timeout") ;
      break;
    }  
    delay(50);
  } //End While Loop
    
  // Arm off
  analogWrite(PowerGearMotorPin, 0); 
  
  //Serial.println(F("ArmMove() - End"));
  DebugPrint("ArmMove() - End");
}

void ClearInfraredScanData(void)
{
  for (int i = MinServoAngle; i <= MaxServoAngle; i++)
  {
    InfraredVoltageReadings[i] = 0;
  }
}

void ClearRollingAverageIrReadings(void)
{
  for (int i = MinServoAngle; i <= MaxServoAngle; i++)
  {
    RollingAverageIrReadings[i] = 0;
  }
}

//void LogInfraredScanData(void)
//{
//  String msg;
//  //DebugPrint("LogInfraredScanData() - Start");
//  for (int i = MinServoAngle; i < MaxServoAngle; i++)
//  {
//    msg = ""; // separate from leading milliseconds time
//    msg += i; 
//    msg += " , ";
//    msg += InfraredVoltageReadings[i];
//    //DebugPrint(msg);
//    Serial.println(msg);
//    delay(500);
//  }
//}

void LogRollingAverageIrData(void)
{
  String msg;
  for (int i = MinServoAngle; i < MaxServoAngle; i++)
  {
    msg = "";  // separate from leading milliseconds time
    msg += i; 
    msg += " , ";
    msg += RollingAverageIrReadings[i];
    Serial.println(msg);
    delay(500);
  }
}

void InfraredScan()          /////////////////////////////~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{
  MoveServoSmoothly(MinServoAngle, 1);
  KentonServo.attach(ServoMotorPin);

  ClearInfraredScanData();
  for (int i = MinServoAngle; i < MaxServoAngle  + 1 ; i++)
  {
    KentonServo.write(i);
    InfraredVoltageReadings[i] = analogRead(A0);
    Serial.print(i);
    Serial.print(F(" , "));
    Serial.println(InfraredVoltageReadings[i]);
    Serial.flush();
    // this delay might not be needed
    delay(20); //takes 20 ms x 180 = 3.6 seconds to scan    
  }  
  
  KentonServo.detach();
  Serial.println(F("reading when measured above here"));
//  String msg;  
//  for (int i = MinServoAngle ; i < MaxServoAngle; i++)
//  {
//    msg = "";  // separate from leading milliseconds time
//    msg += i; 
//    msg += " , ";
//    msg += InfraredVoltageReadings[i];
//    Serial.println(msg);
//    delay(20);
//  }
}

int FindObjectLowerEdge()
{
  MoveServoSmoothly(MinServoAngle,2);
  
  int FoundAngle = InvalidAngle;
  for (int i = MinServoAngle ; i <= MaxServoAngle ; i++)
  {
    if (RollingAverageIrReadings[i] > ThresholdIRVoltage)
    {
      FoundAngle = i;
      break;
    }
  }
  return FoundAngle;
}

int FindObjectUpperEdge()
{
  MoveServoSmoothly(MaxServoAngle,2);
  int FoundAngle = InvalidAngle;
  for (int i = MaxServoAngle ; i >= MinServoAngle ; i--)
  {
    if (RollingAverageIrReadings[i] > ThresholdIRVoltage)
    {
      FoundAngle = i;
      break;
    }
  }
  return FoundAngle;
}

int FindObject()
{
  int FoundAngle = InvalidAngle;
  int LowerEdge = FindObjectLowerEdge();
  int UpperEdge = FindObjectUpperEdge();
  
  if ((LowerEdge != InvalidAngle) && (UpperEdge != InvalidAngle))
  {
    FoundAngle = (LowerEdge + UpperEdge)/2 ;          //Can tweak because of electromagnet
  }
  return FoundAngle;
}
//void InfraredScanOld()
//{
//  //DebugPrint("x");
//  DebugPrint("InfraredScan() - Start");
//  ClearInfraredScanData();
//  //DebugPrint("Raw data before scan:");
//  //LogInfraredScanData();
//  
//  //Infrared Sensor Scan While ServoMotor Swivels
//  int StartingAngle = 0;
//  //int i = 0;
//  //KentonServo.write(StartingAngle);  //Servo to 0 degrees
//  
//  // Go to the starting position of the IR sensor scan
//  int AngleMove = StartingAngle;
//  int AngleIncrementPerMove = 2;
//  MoveServoSmoothly(AngleMove, AngleIncrementPerMove);
//
//  for (int i = StartingAngle; i < MaxServoAngle; i++)
//  {
//    KentonServo.write(i);
//    InfraredVoltageReadings[i] = analogRead(A0);
//    
//    // this delay might not be needed
//    delay(20); //takes 20 ms x 180 = 3.6 seconds to scan
//  }
  //Prints Raw Data for debug purposes (see if there any glitches or spikes in reading values)
  //for (i=StartingAngle; i < MaxServoAngle; i++)
  //{ 
//    String msg;
//    msg = i;
//    msg += " ";
//    msg += InfraredVoltageReadings[i];
//    DebugPrint(msg);
  //}
  
//  DebugPrint("Raw data after scan:");
//  LogInfraredScanData();
  
//Gets Rolling Average Set
/*void KentonRollingAverageSet()  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{
  int SumRollingAverage;

  for (int i = MinServoAngle; i < MaxServoAngle; i+= RollingAverageWidth)
  {
    SumRollingAverage = 0;
    for (int j = i; j < RollingAverageWidth ; j++)
    {  
      SumRollingAverage += InfraredVoltageReadings[j];      
    } 
    
    if (i < RollingAverageWidth)
    {
      RollingAverageSet[i] = SumRollingAverage;
    }     
    else
    {
      RollingAverageSet[(i/RollingAverageWidth)] = SumRollingAverage;  
    }
  }
  
}
*/
// KENTON - this stuff was previously in InfraredScan()
//  CalculateRollingAverageIrReadings();
//  
//  LogRollingAverageIrData();
//  
//  DebugPrint("InfraredScan() - End");
//  //DebugPrint("x");

//int KentonAngleFilter();
//  int Threshold = 0;
//  for (int i = 0; i < (MaxServoAngle/RollingAverageWidth); i++)
//  {
//    if (RollingAverageSet[i] > Threshold)
//   {
//     
//   } 
//    else
//   
//  }

void CalculateRollingAverageIrReadings(void)
{
  Serial.println(F("CalculateRollingAverageIrReadings() - Start"));
  ClearRollingAverageIrReadings();
  
  // for each rolling average, this is the position that we assign the average
  int Offset = RollingAverageSampleSize / 2;
  
  int CurrentAngle = MinServoAngle;
  // Logic should work in extreme case (one data point): MinServoAngle = 0, MaxServoAngle = 0, RollingAverageSampleSize = 1
  while ((CurrentAngle + RollingAverageSampleSize - 1) <= MaxServoAngle)
  {
    int Sum = 0;
    for (int i = CurrentAngle; i < (CurrentAngle + RollingAverageSampleSize); i++)
    {
      Sum += InfraredVoltageReadings[i];
    }
    int Average = Sum / RollingAverageSampleSize;
    
    int IndexRolling = CurrentAngle + Offset;
    
    // Sanity check
//    if (IndexRolling < MinServoAngle)
//    {
//      IndexRolling = MinServoAngle;
//    }
//    if (IndexRolling > MaxServoAngle)
//    {
//      IndexRolling = MaxServoAngle;
//    } 
    IndexRolling = constrain(IndexRolling, MinServoAngle, MaxServoAngle);
    
    RollingAverageIrReadings[IndexRolling] = Average;
    
    
    Serial.print(IndexRolling);
    Serial.print(F(" , "));
    Serial.println(RollingAverageIrReadings[IndexRolling]); 
    Serial.flush();
    
    CurrentAngle += 1;
    
  } // end while
  //DebugPrint("CalculateRollingAverageIrReadings() - End");
  Serial.println(F("CalculateRollingAverageIrReadings() - End"));

}

//Pickup at correct infrared reading range
//  if  
//  {
//    
//  }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void DebugPrint(String msg)
{
    if (UsbConnectedToPc == true)
    {
      String msg2;
      long timeSinceStartup;
      timeSinceStartup = millis();
      msg2 += timeSinceStartup;
      msg2 += "  ";
      msg2 += msg;
      //Serial.println(msg);
      Serial.println(msg2);
      
      // experiment 12-1-2013
      Serial.flush();
    }
}

void WaitForButtonPush()
{
    DebugPrint("WaitForButtonPush(): Start");
    startbutton=false;
    

//-------------Watchdog code
  while (! startbutton) {        // stay in this loop until button pressed
    digitalWrite(LEDFlasherPin,HIGH); // LED on
    for(int i=0;i<10;i++) {
      delay(25);                 // wait a bit
      if (digitalRead(TactileSwitchPin) == LOW)  // check button
        startbutton=true;        // button is pressed
    }
    digitalWrite(LEDFlasherPin,LOW);  // LED off
    for(int i=0;i<30;i++) {
      delay(25);
      if (digitalRead(TactileSwitchPin) == LOW)
        startbutton=true;
    }
  } // end of while loop
  
  DebugPrint("WaitForButtonPush(): Le Button was pushed");
}  

void HomeArm(void)
{
  delay(2000);
  //DebugPrint("HomeArm() - Arm is Moving Backward");
  ArmMove(false);  
  delay(2000);
}

void RobotCode()
{
  //DebugPrint("Arm is Moving Forward");
  ArmMove(true);  
  delay(2000);
  //DebugPrint("Arm is Moving Backward");
  ArmMove(false);
  delay(2000);
}

void LogServoReadAngle(void)
{
  String msg;
  int ReadAngle = 0;
  ReadAngle = KentonServo.read();
  msg = "Servo ReadAngle = ";
  msg += ReadAngle;
  DebugPrint(msg);
}

void HomeServo(void)
{
  //DebugPrint("HomeServo() - Start");
  int Angle = 90;
  int AngleIncrement = 2;
  MoveServoSmoothly(Angle, AngleIncrement);
  //DebugPrint("HomeServo() - End");
}

// Move servo smoothly to angle
void MoveServoSmoothly(int angle, int angleIncrement)
{
  KentonServo.attach(ServoMotorPin);
  String msg;
  //DebugPrint("MoveServo() - Start");
  if ((angle < 0) || (angle > 180))
  {
    angle = 90;
  }
  if ((angleIncrement < 0) || (angleIncrement > 30))
  {
    angleIncrement = 2;
    
  }
  
  const unsigned long LoopDelayMilliSec = 100;
  
//  msg = "angle = ";
//  msg += angle;
//  msg += ", angleIncrement = ";
//  msg += angleIncrement;
//  DebugPrint(msg);
  int CurrentAngle = 0;
  CurrentAngle = KentonServo.read();
  //DebugPrint("Initial angle:");
  //LogServoReadAngle();
  
  int sign = 1;
  if (angle < CurrentAngle)
  {
    sign = -1;
  }
  
  while (true)
  {
    KentonServo.write(CurrentAngle);
    
    if (CurrentAngle == angle)
    {
      // reached final position
      break;
    }
    
    if (sign > 0)
    {
      CurrentAngle += angleIncrement;
      if (CurrentAngle > angle)
      {
        // Will end at next loop iteration
        CurrentAngle = angle;
      }
    }
    else
    {
      CurrentAngle -= angleIncrement;
      if (CurrentAngle < angle)
      {
        // Will end at next loop iteration
        CurrentAngle = angle;
      }      
    }
    
    delay(LoopDelayMilliSec);
  } // end while
  
  //DebugPrint("Final angle:");
  //LogServoReadAngle();
  //DebugPrint("MoveServo() - End");
  KentonServo.detach();
}

void UnitTestArm1(void)
{
  Serial.println(F("UnitTestArm1() - Start"));
  WaitForButtonPush();
  //Serial.println(F("HomeArm"));
  //HomeArm();
  LogFreeMemory();
  delay(5000);
  Serial.println(F("Arm Backward"));
  ArmMove(false);
  delay(5000);
  Serial.println(F("Arm Forward"));
  ArmMove(true);
  delay(5000);
  Serial.println(F("Arm Backward"));
  ArmMove(false);
  delay(5000);
  Serial.println(F("Arm Forward"));
  ArmMove(true);
  delay(5000);  
  LogFreeMemory();
  Serial.println(F("UnitTestArm1() - End"));
}

void UnitTestServo1(void)
{
  Serial.println(F("UnitTestServo1() - Start"));
  WaitForButtonPush();
  LogFreeMemory();
  MoveServoSmoothly(90,2);
  delay(1000);
  MoveServoSmoothly(30,2);
  delay(1000);
  MoveServoSmoothly(120,2);
  delay(1000);
  HomeServo();
  LogFreeMemory();
  Serial.println(F("UnitTestServo1() - End"));

}

void Electromagnet(void)
{
  Serial.println(F("Electromagnet() - Start - magnet ON in 5 seconds ..."));
  delay(5000); //gives time for paper clip to be positioned
  digitalWrite(ElectromagnetPin, HIGH);
  delay(2000); //gives time for paper clip to be attracted
  ArmMove(false);
  delay(1000); //waits for any bounce in the arm to subside so that 
  digitalWrite(ElectromagnetPin, LOW);
  delay(2000); // time to drop, else booger shake required.
  Serial.println(F("Electromagnet() - End - magnet OFF"));
}

void setup()
{
  Serial.begin(9600);
  delay(1000);
  //DebugPrint("setup() - Start");

  //Assign Digital Pins

  pinMode(LEDFlasherPin, OUTPUT); //Pin 2
  pinMode(TactileSwitchPin, INPUT); //Pin 3
  pinMode(PowerGearMotorPin, OUTPUT); //Pin 4
  pinMode(DirectionGearMotorPin, OUTPUT); //Pin 5 
  pinMode(LimitSwitchFrontPin, INPUT); //Pin 6
  pinMode(LimitSwitchBackPin, INPUT); //Pin 7
  pinMode(ElectromagnetPin, OUTPUT);  //Pin 8
  pinMode(ServoMotorPin, OUTPUT);  //Pin 9
  
  // Assign Analog Pin for Infrared Sensor
  pinMode(A0, INPUT); // Infrared Sensor
  
  //HomeArm();
  
  //Set up Servo Motor
  //DebugPrint("KentonServo.attach(ServoMotorPin); - Does this attach command force a move also?");
  //KentonServo.attach(ServoMotorPin);
  delay(1000);
  //KentonServo.write(90); //Sets the servo's position mid-way through its range (0-180 degrees)
  //HomeServo();
  
  LogFreeMemory();
  
  DebugPrint("setup() - End");
}

void loop()
{
//  UnitTestArm1();
//  return; // debug exit early
//  UnitTestServo1();
//  return;

//  WaitForButtonPush();
//  Electromagnet();
//  return;
  
  
  WaitForButtonPush();
  LogFreeMemory();
  //HomeArm();
  ArmMove(false);
  delay(2000);
  HomeServo();
  ArmMove(true);
  delay(2000);
  LogFreeMemory();
  InfraredScan();
  LogFreeMemory();
  CalculateRollingAverageIrReadings();
  int FoundAngle = FindObject();
  
  if (FoundAngle != InvalidAngle)
  {
    MoveServoSmoothly((FoundAngle - AngleOffset), 2);  //Can change speed
    Serial.print(F("Found Object At Angle = "));
    Serial.println(FoundAngle);
  }
  else 
  {
    Serial.println(F("Could Not Find Object"));
  }
  delay(5000);
  
  Electromagnet();

  
//  pinMode(8, HIGH);
//  delay(1000000000000000);   ELECTROMAGNET FUNCTION YO  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ArmMove(false);
  return;
  
//  delay(1000);
//  MoveServoSmoothly(0, 2);
//  delay(1000);
//  KentonInfraredScan();
//  delay(1000);
//  //KentonRollingAverageSet();
//  delay(2000);
//  DebugPrint("arm backward");
//  ArmMove(false);
//  
  
    
    
    
//  String msg;
//  WaitForButtonPush();
//  DebugPrint("loop() - start");
//  delay(2000);
//  int ReadDegrees;
//  ReadDegrees = KentonServo.read();
//  msg = "ReadDegrees = ";
//  msg += ReadDegrees;
//  DebugPrint(msg);
//  for (int i = 30; i < 150 ; i++)
//  {
//    KentonServo.write(i);
//    delay(100);
//  }
  
  
//  if (false)
//  {
//    // Should already be here, some code during initialization set to 180
//    int AngleIncrementPerMove = 0;
//    AngleIncrementPerMove = 3; // faster;
//    MoveServoSmoothly(90, AngleIncrementPerMove);
//    delay(1000);
//    
//    MoveServoSmoothly(30, AngleIncrementPerMove);
//    delay(1000);
//    AngleIncrementPerMove = 1; // slower;
//    MoveServoSmoothly(150, 1);
//    delay(1000);
//  }
//  
//  InfraredScan();
//  
//  //HomeServo();
//  
//  DebugPrint("loop() - end");
//  return;
  
  //delay(200);
  //int InfraredValue = analogRead(0);
  //DebugPrint(InfraredValue);
  /*DebugPrint("Magnet On");
  digitalWrite(ElectromagnetPin, HIGH);
  delay(5000);
  DebugPrint("Magnet Off");
  digitalWrite(ElectromagnetPin, LOW);
  delay(5000);
  return;
  */
  
  
//  delay(2000);
//  WaitForButtonPush();
//  RobotCode();
    
  LogFreeMemory();
}
