#include "Wire.h"

//#define DEBUG

//////////////// Optical Encoder variables ////////////////// 
// Using interrupts, this program will measure the ticks from the optical encoder 
// attached to the robot wheel. It will then generate a speed reading "cm/sec"

// Wheel radius = 3.5cm
// Wheel circonference = 2 * pi * r = 21.991148cm or 219.91148mm
const double circonference = 219.91148;
const double ticksPerRotation = 21.00;
// 1 tick length in mm
const double singleTickLength = 10.4719752381;


//////////////// Wheel variables //////////////////  
// INT0 and INT1, and they are mapped to Arduino pins 2 and 3.
// The encoder takes 5V+ and GND as well, the pin order as seen from top or robot: (GND)|(+5V)|(SIGNAL)
// Right encoder input on PIN2 "D02" of arduino 
// Right distance is in cm
const byte rightIntChannel= 02;
signed long rightTicks;
double rightDistance;
double rightSpeed;
signed long rightPreviousTicks;
// Left encoder input on PIN3 "D03" of arduino 
// Left distance is in cm
const byte leftIntChannel= 03;
signed long leftTicks;
double leftDistance;
double leftSpeed;
signed long leftPreviousTicks;

bool verifyDistance = false;


//////////////// Timing variables //////////////////  
unsigned long previousMillis = 0;
long interval = 100;


//////////////// Motor variables ////////////////// 
// Motor A, Left Side
const uint8_t pwmLeftPin = 9;      // ENA - Enable and PWM (PWMA)
const uint8_t leftForwardPin = 8;  // IN1 - Forward Drive (INA1)
const uint8_t leftReversePin = 7;  // IN2 - Reverse Drive (INA2)
// Right MotorB
const uint8_t pwmRightPin = 6;     // ENB - Enable and PWM (PWMB)
const uint8_t rightForwardPin = 5; // IN3 - Forward Drive (INB1)
const uint8_t rightReversePin = 4; // IN4 - Reverse Drive (INB2)
// Max motor speed on 12V battery is about 190cm/s
// But 190cm/s is too quick and unstable.
// 150cm/s was found after some calibration
const int maxSpeed = 50;


//////////////// I2C variables ////////////////// 
#define SLAVE_ADDRESS 0x04
int motion = 0;
unsigned short desiredSpeed = 0;
unsigned short desiredDistance = 0;


//////////////// Setup and Loop ////////////////// 
void setup() 
{
  // I2C Setup
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // Motor Setup
   // Left Motor A Setup
  pinMode(pwmLeftPin , OUTPUT);
  pinMode(leftForwardPin, OUTPUT);
  pinMode(leftReversePin, OUTPUT);
  // Right Motor B Setup
  pinMode(pwmRightPin, OUTPUT);
  pinMode(rightForwardPin, OUTPUT);
  pinMode(rightReversePin, OUTPUT);

  // Interrupt Setup
  // Right Optical Encoder Interrupt setup
  pinMode(rightIntChannel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rightIntChannel), RightInterruptTickUpdater, RISING);
  // Left Optical Encoder Interrupt setup
  pinMode(leftIntChannel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftIntChannel), LeftInterruptTickUpdater, RISING);
  interrupts();

  // READY
  #ifdef DEBUG
  Serial.println("Ready!");
  #endif
}

void loop() {  
    timeDelay();  //we are not using Arduino's delay() function
}


//////////////// I2C Functions ////////////////// 
// callback for received data
void receiveData(int byteCount)
{
  Reset(); 
  #ifdef DEBUG
  Serial.print("data received, byte count: ");
  Serial.println(byteCount);    
  #endif

  int opcode = 0;
  motion = 0;
  desiredSpeed = 0;
  desiredDistance = 0;

  int dataIterator = 0;
  while(Wire.available())
  {
      int data = Wire.read();
      #ifdef DEBUG
      Serial.print("byte: ");
      Serial.println(data);
      #endif

      switch (dataIterator) 
      {
        case 0:  
          opcode = data;
          break;

        case 1:
          motion = data;
          break;

        case 2:
          desiredSpeed = MapDesiredSpeed(data);
          #ifdef DEBUG
          Serial.print("Desired Speed = ");
          Serial.println(desiredSpeed);
          #endif
          break;

        case 3:
          desiredDistance = data;
          break; 

        case 4:
          desiredDistance = ( ( data & 0xFF ) << 8 ) | ( desiredDistance & 0xFF );
          #ifdef DEBUG
          Serial.print("Full Desired Distance(cm) = ");
          Serial.println(desiredDistance);
          Serial.println();
          #endif
          break; 
      }

      dataIterator++;        
  }

  switch (opcode) 
  {
      // Opcode 0 (Represents: Motion and Speed)
      case 0:
        verifyDistance = false; 
        //Serial.println("Opcode = 0"); 
        Activate();    
        break;

      // Opcode 1 (Represents: Motion, Speed and Distance)
      case 1: 
        verifyDistance = true;         
        //Serial.println("Opcode = 1"); 
        Activate();  
        break;
  }
}

// callback for sending data
void sendData()
{
    // Send back 1 for acknowledgement
    Wire.write(1);
}


//////////////// Utility Functions ////////////////// 
void Activate()
{
  switch (motion)
  {
    case 0:
      allStop();
      break;

    case 1:
      allReverse(); 
      break;

    case 2:
      allForward(); 
      break;

    case 3:
      skidsteerLeft();
      break;

    case 4:
      skidsteerRight(); 
      break;
  }
}

void allStop() {
  digitalWrite(leftForwardPin, LOW);
  digitalWrite(leftReversePin, LOW);
  digitalWrite(rightForwardPin, LOW);
  digitalWrite(rightReversePin, LOW);
  analogWrite(pwmLeftPin , 0);
  analogWrite(pwmRightPin, 0);
}
 
void allForward() {
  digitalWrite(leftForwardPin, HIGH);
  digitalWrite(leftReversePin, LOW);
  digitalWrite(rightForwardPin, HIGH);
  digitalWrite(rightReversePin, LOW);
}
 
void allReverse() {
  digitalWrite(leftForwardPin, LOW);
  digitalWrite(leftReversePin, HIGH);
  digitalWrite(rightForwardPin, LOW);
  digitalWrite(rightReversePin, HIGH);
}
 
void skidsteerLeft() {
  digitalWrite(leftForwardPin, LOW);
  digitalWrite(leftReversePin, HIGH);
  digitalWrite(rightForwardPin, HIGH);
  digitalWrite(rightReversePin, LOW);
}
 
void skidsteerRight() {
  digitalWrite(leftForwardPin, HIGH);
  digitalWrite(leftReversePin, LOW);
  digitalWrite(rightForwardPin, LOW);
  digitalWrite(rightReversePin, HIGH);
}

int rightPWM;
int leftPWM;
void AdjustPWM(bool isRight) 
{
  if (isRight)
  {
    rightPWM = abs(rightPWM + GetPWMfromGainFormula(abs(rightSpeed / 10), abs(desiredSpeed)));  
    #ifdef DEBUG    
    Serial.print("New Right PWM value = ");
    Serial.println(rightPWM);
    #endif
    analogWrite(pwmRightPin, rightPWM);
  }
  else
  {
    leftPWM = abs(leftPWM + GetPWMfromGainFormula(abs(leftSpeed / 10), abs(desiredSpeed)));
    #ifdef DEBUG
    Serial.print("New Left PWM value = ");
    Serial.println(leftPWM);
    #endif
    analogWrite(pwmLeftPin , leftPWM);
  }
}

void Reset() 
{
  rightPWM = 0;
  rightDistance = 0;

  leftPWM = 0;
  leftDistance = 0;
}


//////////////// Optical Encoder Time Delay and Interrupt Functions //////////////////
void RightInterruptTickUpdater() 
{
    if (motion == 1)
    {
        rightTicks -= 1;
    }
    else if (motion == 2)
    {
        rightTicks += 1;
    }
    else 
    {
        // Manual intervention test
        rightTicks += 1;      
    }
}

void LeftInterruptTickUpdater() 
{
    if (motion == 1)
    {
        leftTicks -= 1;
    }
    else if (motion == 2)
    {
        leftTicks += 1;
    }
    else 
    {
        // Manual intervention test
        leftTicks += 1;      
    }
}

long  iterationCount = 0;
void timeDelay()  
{
  unsigned long currentMillis = millis();

  // Generting 1-sec time delay using millis
  if (currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;

    #ifdef DEBUG
    Serial.println();
    
    Serial.print("Desired Speed = ");
    Serial.print(desiredSpeed);
    Serial.println("cm/s");
    #endif
    
    CalculateMotorSpeed(true);
    AdjustPWM(true);

    CalculateMotorSpeed(false);
    AdjustPWM(false);

    if (verifyDistance)
    {
      VerifyIfDistanceIsReached();
    }

    #ifdef DEBUG
    Serial.println();
    #endif
  }
}

void CalculateMotorSpeed(bool isRight)
{
  double newTickCount = isRight 
                      ? rightTicks 
                      : leftTicks;

  double tickDelta = isRight 
                    ? (rightTicks - rightPreviousTicks) 
                    : (leftTicks - leftPreviousTicks) ;

  if (isRight)
  {
    rightSpeed = (double)(tickDelta * singleTickLength) * (1000 / interval);
    rightDistance += rightSpeed;
    rightPreviousTicks = newTickCount;
  }
  else
  {
    leftSpeed = (double)(tickDelta * singleTickLength) * (1000 / interval);
    leftDistance += leftSpeed;
    leftPreviousTicks = newTickCount;
  }
  
  #ifdef DEBUG
  Serial.print(iterationCount++);
  Serial.print("- Motion = ");
  
  if (motion == 0)
  {
      Serial.print("STOP");
  }
  else if (motion == 1)
  {
      Serial.print("REVERSE");
  }
  else if (motion == 2)
  {
      Serial.print("FWD");
  }

  Serial.print(isRight ? ", RIGHT Motor " : ", LEFT Motor " );
  Serial.print("Ticks = ");
  Serial.print((long) isRight ? rightTicks : leftTicks );
  Serial.print(isRight ? ", RIGHT Motor " : ", LEFT Motor " );
  Serial.print("Distance = ");
  Serial.print((double) isRight ? (rightDistance / 10) : (leftDistance / 10));
  Serial.print("cm");
  Serial.print(isRight ? ", RIGHT Motor " : ", LEFT Motor " );
  Serial.print("Speed = ");
  Serial.print((double) isRight ? (rightSpeed / 10) : (leftSpeed / 10));
  Serial.println("cm/sec");
  #endif
}

void VerifyIfDistanceIsReached()
{
  #ifdef DEBUG
  Serial.println();
  Serial.print("Desired Distance(cm) = ");
  Serial.println(desiredDistance);
  Serial.print("Right Distance(cm) = ");
  Serial.println(rightDistance/100);
  Serial.print("Left Distance(cm) = ");
  Serial.println(leftDistance/100);
  Serial.println();
  #endif

  if ((abs(rightDistance/100) > desiredDistance) || 
      (abs(leftDistance/100) > desiredDistance))
  {
    allStop();
  }
}

float MapDesiredSpeed(int input)
{
  // This method maps 0-10 speed steps into 0-maxSpeed cm/s speed steps
  return (float)map(input, 1, 10, 15, maxSpeed);
}

//////////////// Gain Feedback Formula //////////////////
// Input
// y_des: the desired speed (cm/s)
// y_act: actual speed (cm/s)
// Return
// u: the PWM value as input to my system
const float K = 0.9;
int GetPWMfromGainFormula(float y_act, float y_des)
{
    // y: Output (is the output speed in my case)
    // error : y_des – y_act
  
    // u: Input (PWM Pulse Width Modulation signal in my case)
    // u : - K * error

    // y_des: constant
    // y: varies by my system
    // K: constant that I adjust with a start value of 1
    // u: This is what I need to update and adjust on each iteration Loop using the following formula:
    
    int u;
    u = -K * (y_act - y_des);
    //u = -K * (y_des - y_act);

    return u;
}







 
