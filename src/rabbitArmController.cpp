#include <Arduino.h>
#include <ESP32Servo.h>

const int ArmServoPin = 19;
const int WristServoPin = 18;
const int GripServoPin = 21;

int l_2 = 0, r_2 = 0, l_stick_Y = 0;
int arm_motor = 0, arm_servo_val, wrist_servo_val = 0, grip_servo_val = 0;
Servo arm_servo, wrist_servo, grip_servo;

void readValues();
void calculateValues();
void driveMotors();

void setup()
{
  pinMode(ArmServoPin, OUTPUT);
  pinMode(WristServoPin, OUTPUT);
  pinMode(GripServoPin, OUTPUT);
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
  }
}

void calculateValues()
{
  Serial.print("l_2: ");
  Serial.print(l_2);
  Serial.print("   r_2: ");
  Serial.print(r_2);
  Serial.print("   l_stick_Y: ");
  Serial.println(l_stick_Y);

  // arm_motor = abs(l_2) / 2;
  arm_servo_val = 50 + 0.2 * abs(l_stick_Y);
  wrist_servo_val = 0.2 * abs(l_2);
  grip_servo_val = 0.2 * abs(r_2);
}

void driveMotors()
{
  arm_servo.write(arm_servo_val);
  wrist_servo.write(wrist_servo_val);
  grip_servo.write(grip_servo_val);
  // delay(15); // waits 15ms to reach the position
}