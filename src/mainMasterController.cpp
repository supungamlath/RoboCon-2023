#include <Arduino.h>
#include <PS4Controller.h>
#include <AccelStepperWithDistance.h>

const int dirPin = 14;
const int stepPin = 12;

const int FdFrontLeft = 22;
const int BkFrontLeft = 23;
// const int PWMFrontLeft = 14;

const int FdBackLeft = 25;
const int BkBackLeft = 26;
// const int PWMBackLeft = 12;

const int FdFrontRight = 32;
const int BkFrontRight = 27;
// const int PWMFrontRight = 21;

const int FdBackRight = 18;
const int BkBackRight = 19;
// const int PWMBackRight = 16;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepperWithDistance myStepper(motorInterfaceType, stepPin, dirPin);
float upPosition = 200;
float downPosition = 20;

void onConnect() // when connected ps4
{
  Serial.println("Connected!.");
}

void setup()
{
  pinMode(FdFrontLeft, OUTPUT);
  pinMode(BkFrontLeft, OUTPUT);
  // pinMode(PWMFrontLeft, OUTPUT);
  pinMode(FdBackLeft, OUTPUT);
  pinMode(BkBackLeft, OUTPUT);
  // pinMode(PWMBackLeft, OUTPUT);
  pinMode(FdFrontRight, OUTPUT);
  pinMode(BkFrontRight, OUTPUT);
  // pinMode(PWMFrontRight, OUTPUT);
  pinMode(FdBackRight, OUTPUT);
  pinMode(BkBackRight, OUTPUT);
  // pinMode(PWMBackRight, OUTPUT);

  PS4.begin();
  Serial.begin(115200);
  PS4.attachOnConnect(onConnect);

  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  // myStepper.setStepsPerRotation(200);
  myStepper.setDistancePerRotation(2);
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(500);
  myStepper.setSpeed(500);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void SetMotor(int Cordinate, int pin1, int pin2)
{
  int pwmValue;
  if (Cordinate >= 1)
  {
    digitalWrite(pin2, LOW);
    pwmValue = (Cordinate - 1) * 2.12;
    analogWrite(pin1, pwmValue);
    Serial.print("coordinate = ");
    Serial.print(Cordinate);
    Serial.print("pin 1 = ");
    Serial.print(pin1);
    Serial.print("    Forwad = ");
    Serial.print(pin2);
    Serial.print("     pwmValue = ");
    Serial.println(pwmValue);
  }
  else if (Cordinate <= -1)
  {
    digitalWrite(pin1, LOW);
    pwmValue = (-Cordinate - 1) * 2.12;
    analogWrite(pin2, pwmValue);
    Serial.print("coordinate = ");
    Serial.print(Cordinate);
    Serial.print("pin 1 = ");
    Serial.print(pin1);
    Serial.print("    Backword = ");
    Serial.print(pin2);
    Serial.print("     pwmValue = ");
    Serial.println(pwmValue);
  }
  else
  {
    // Serial.print("coordinate = ");
    // Serial.print(Cordinate);
    // Serial.print("pin 1 = ");
    // Serial.print(pin1);
    // Serial.print("    pin2 = ");
    // Serial.print(pin2);
    // Serial.print("     pwmValue = ");
    // Serial.println(pwmValue);
    // digitalWrite(pin1, LOW);
    // digitalWrite(pin2, LOW);
    pwmValue = 0;
  }

  // Serial.println("...................................................................");
}

void debugBtnRead(int x, int y, int dir_up, int dir_down)
{
  Serial.print("x = ");
  Serial.print(x);
  Serial.print("   ");
  Serial.print("y = ");
  Serial.print(y);
  Serial.print("   ");
  Serial.print("dir_up = ");
  Serial.print(dir_up);
  Serial.print("   ");
  Serial.print("dir_down = ");
  Serial.print(dir_down);
}

void setFrontLeft(int jValue)
{
  SetMotor(jValue, FdFrontLeft, BkFrontLeft);
}

void setBackLeft(int jValue)
{
  SetMotor(jValue, FdBackLeft, BkBackLeft);
}
void setFrontRight(int jValue)
{
  SetMotor(jValue, FdFrontRight, BkFrontRight);
}
void setBackRight(int jValue)
{
  SetMotor(jValue, FdBackRight, BkBackRight);
}

void driveStepper(int up, int down)
{

  if (up)
  {
    myStepper.moveToDistance(upPosition);
    while (myStepper.getCurrentPositionDistance() == upPosition)
    {
      myStepper.run();
    }
  }
  else if (down)
  {
    myStepper.moveToDistance(downPosition);
    while (myStepper.getCurrentPositionDistance() == downPosition)
    {
      myStepper.run();
    }
  }
  else
  {
    Serial.print("initial position = ");
    Serial.print(" ");
    Serial.println(myStepper.getCurrentPositionDistance());
  }
}

void driveMotors(int x, int y)
{
  if (x == 0)
  {
    x = 1;
  }
  float m = y / x;

  if (y >= 0 && x > 0 && m > 1)
  {                       // 1st quadrant
    setFrontLeft(y);      // y
    setBackLeft(y - x);   // y-x
    setFrontRight(y - x); // y-x
    setBackRight(y);      // y
  }

  else if (y >= 0 && x >= 0 && m <= 1)
  {
    setFrontLeft(x);
    setBackLeft(y - x);
    setFrontRight(y - x);
    setBackRight(x);
  }

  else if (y < 0 && x > 0 && m >= -1)
  { // 2nd quadrant
    setFrontLeft(x + y);
    setBackLeft(-x);
    setFrontRight(-x);
    setBackRight(x + y);
  }
  else if (y <= 0 && x > 0 && m < -1)
  {
    setFrontLeft(x + y);
    setBackLeft(y);
    setFrontRight(y);
    setBackRight(x + y);
  }
  else if (y < 0 && x < 0 && m >= 1)
  { // 3rd quadrant
    setFrontLeft(y);
    setBackLeft(y - x);
    setFrontRight(y - x);
    setBackRight(y);
  }
  else if (y <= 0 && x <= 0 && m < 1)
  {
    setFrontLeft(x);
    setBackLeft(y - x);
    setFrontRight(y - x);
    setBackRight(x);
  }
  else if (y > 0 && x < 0 && m >= -1)
  { // 4th quadrant
    setFrontLeft(y + x);
    setBackLeft(-x);
    setFrontRight(-x);
    setBackRight(y + x);
  }
  else if (y > 0 && x < 0 && m < -1)
  {
    setFrontLeft(y + x);
    setBackLeft(y);
    setFrontRight(y);
    setBackRight(y + x);
  }
}

void loop()
{

  int y = PS4.RStickY();
  int x = PS4.RStickX();
  int down = PS4.Down();
  int up = PS4.Up();

  driveMotors(x, y);
  driveStepper(up, down);
  debugBtnRead(x, y, up, down);
  delay(100);
}