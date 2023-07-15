#include <Arduino.h>
#include <ESP32Servo.h>
#include <VarSpeedServo.h>

const int ArmServoPin = 19;
const int WristServoPin = 18;
const int GripServoPin = 21;
const int ArmUpPin = 22;
const int ArmDownPin = 23;
const int LimitSwitchTopPin = 25;
const int LimitSwitchBottomPin = 26;

int l_2 = 0, r_2 = 0, l_stick_Y = 0, up_btn = 0, down_btn = 0;
int arm_motor = 0, arm_servo_val, wrist_servo_val = 0, grip_servo_val = 0;
int servo_speed = 100;
VarSpeedServo arm_servo, wrist_servo, grip_servo;

void readValues();
void calculateValues();
void driveMotors();

void setup()
{
  pinMode(ArmServoPin, OUTPUT);
  pinMode(WristServoPin, OUTPUT);
  pinMode(GripServoPin, OUTPUT);
  pinMode(ArmUpPin, OUTPUT);
  pinMode(ArmDownPin, OUTPUT);
  pinMode(LimitSwitchTopPin, INPUT);
  pinMode(LimitSwitchBottomPin, INPUT);
  arm_servo.attach(ArmServoPin);
  wrist_servo.attach(WristServoPin);
  grip_servo.attach(GripServoPin);

  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop()
{
  readValues();      // Get values from Master ESP32
  calculateValues(); // Calculate direction and PWM of each motor
  driveMotors();     // Driver each motor
}

void readValues()
{
  if (Serial2.read() == 6)
  {
    l_2 = Serial2.parseInt();
    r_2 = Serial2.parseInt();
    l_stick_Y = Serial2.parseInt();
    up_btn = Serial2.parseInt();
    down_btn = Serial2.parseInt();
  }
}

void calculateValues()
{
  // Serial.print("l_2: ");
  // Serial.print(l_2);
  // Serial.print("   r_2: ");
  // Serial.print(r_2);
  // Serial.print("   l_stick_Y: ");
  // Serial.println(l_stick_Y);

  arm_servo_val = 50;
  wrist_servo_val = abs(l_2);
  grip_servo_val = abs(r_2);
  if (up_btn == 1)
  {
    arm_motor = 150;
  }
  else if (down_btn == 1)
  {
    arm_motor = -150;
  }
  else
  {
    arm_motor = 0;
  }
}

void driveMotors()
{
  arm_servo.write(arm_servo_val, servo_speed, false);
  wrist_servo.write(wrist_servo_val, servo_speed, false);
  grip_servo.write(grip_servo_val, servo_speed, false);
  if (arm_motor > 0 and digitalRead(LimitSwitchTopPin) == LOW)
  {
    analogWrite(ArmUpPin, arm_motor);
    analogWrite(ArmDownPin, LOW);
  }
  else if (arm_motor < 0 and digitalRead(LimitSwitchBottomPin) == LOW)
  {
    analogWrite(ArmUpPin, LOW);
    analogWrite(ArmDownPin, -arm_motor);
  }
  else
  {
    analogWrite(ArmUpPin, LOW);
    analogWrite(ArmDownPin, LOW);
  }
}