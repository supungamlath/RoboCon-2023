#include <Arduino.h>
#include <ESP32Servo.h>

const int RingLiftSeroPin = 15;
const int ArmServoPin = 19;
const int WristServoPin = 18;
const int GripServoPin = 21;
const int ArmUpPin = 22;
const int ArmDownPin = 23;
const int LimitSwitchTopPin = 25;
const int LimitSwitchBottomPin = 26;

int l_stick_Y = 0, up_down_btns = 0, left_right_btns = 0, cmd_btns = 0, l_2_value = 0, r_2_value = 0;
int arm_motor_speed = 70;
float arm_servo_val = 40.0, wrist_servo_val = 0.0, grip_servo_val = 0.0, ring_lift_servo_val = 0.0;
Servo arm_servo, wrist_servo, grip_servo, ring_lift_servo;

void readValues();
void calculateValues();
void calculatePresetMotion();
void driveActuators();
float slowIncrement(float, float);

void setup()
{
  pinMode(ArmServoPin, OUTPUT);
  pinMode(WristServoPin, OUTPUT);
  pinMode(GripServoPin, OUTPUT);
  pinMode(RingLiftSeroPin, OUTPUT);
  pinMode(ArmUpPin, OUTPUT);
  pinMode(ArmDownPin, OUTPUT);
  pinMode(LimitSwitchTopPin, INPUT);
  pinMode(LimitSwitchBottomPin, INPUT);
  arm_servo.attach(ArmServoPin);
  wrist_servo.attach(WristServoPin);
  grip_servo.attach(GripServoPin);
  ring_lift_servo.attach(RingLiftSeroPin);

  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop()
{
  readValues();      // Get values from Master ESP32
  if (cmd_btns == 0) // If a command button is not pressed
  {
    calculateValues();
  }
  else // If a command button is pressed
  {
    calculatePresetMotion();
  }
  driveActuators(); // Drive each actuator
}

void readValues()
{
  if (Serial2.read() == 6)
  {
    l_2_value = Serial2.parseInt();
    r_2_value = 127 - Serial2.parseInt();
    l_stick_Y = Serial2.parseInt();
    up_down_btns = Serial2.parseInt();
    left_right_btns = Serial2.parseInt();
    cmd_btns = Serial2.parseInt();
  }
}

void calculateValues()
{
  l_stick_Y = (l_stick_Y < -10 ? l_stick_Y : (l_stick_Y > 10 ? l_stick_Y : 0));

  if (l_stick_Y >= 0)
    arm_servo_val += l_stick_Y;
  else
  {
    arm_servo_val += (0.0001 * l_stick_Y);
  }
  wrist_servo_val += (0.01 * left_right_btns - 0.00005 * l_stick_Y);
  grip_servo_val += (0.0005 * r_2_value);
  ring_lift_servo_val = map(l_2_value, 0, 255, 0, 180);

  if (up_down_btns == 1)
  {
    arm_motor_speed = 70;
  }
  else if (up_down_btns == -1)
  {
    arm_motor_speed = -70;
  }
  else
  {
    arm_motor_speed = 0;
  }
}

void calculatePresetMotion()
{
  if (cmd_btns == 1)
  {
    arm_servo_val = slowIncrement(arm_servo_val, 40.0);
    wrist_servo_val = slowIncrement(wrist_servo_val, 37.0);
    grip_servo_val = slowIncrement(grip_servo_val, 25.0);
  }
  else if (cmd_btns == 2)
  {
    arm_servo_val = slowIncrement(arm_servo_val, 125.0);
    wrist_servo_val = slowIncrement(wrist_servo_val, 10.0);
    grip_servo_val = slowIncrement(grip_servo_val, 0.0);
  }
  else if (cmd_btns == 3)
  {
    arm_servo_val = slowIncrement(arm_servo_val, 125.0);
    wrist_servo_val = slowIncrement(wrist_servo_val, 65.0);
    grip_servo_val = slowIncrement(grip_servo_val, 0.0);
  }
}

void driveActuators()
{
  arm_servo_val = constrain(arm_servo_val, 60, 250);
  wrist_servo_val = constrain(wrist_servo_val, 10, 80);
  grip_servo_val = constrain(grip_servo_val, 0, 25);
  ring_lift_servo_val = constrain(ring_lift_servo_val, 0, 170);

  arm_servo.write(arm_servo_val);
  wrist_servo.write(wrist_servo_val);
  grip_servo.write(grip_servo_val);
  ring_lift_servo.write(ring_lift_servo_val);

  // Serial.print("Arm Servo: ");
  // Serial.print(arm_servo_val);
  // Serial.print("\tWrist Servo: ");
  // Serial.print(wrist_servo_val);
  // Serial.print("\tGrip Servo: ");
  // Serial.println(grip_servo_val);

  if (arm_motor_speed >= 0)
  {
    analogWrite(ArmUpPin, arm_motor_speed);
    analogWrite(ArmDownPin, LOW);
  }
  else
  {
    analogWrite(ArmUpPin, LOW);
    analogWrite(ArmDownPin, -arm_motor_speed);
  }
}

float slowIncrement(float current, float target)
{
  if (current < target)
  {
    current += 0.005;
  }
  else if (current > target)
  {
    current -= 0.005;
  }
  return current;
}