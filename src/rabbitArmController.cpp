#include <Arduino.h>
#include <ESP32Servo.h>
#include <AccelStepperWithDistance.h>

// Servo Pinouts
const int RingLiftServoPin = 15;
const int ArmServoPin = 19;
const int WristServoPin = 18;
const int GripServoPin = 21;

// Stepper Pinouts
const int ArmStepperDir = 26;
const int ArmStepperStep = 27;

// Limit Switch Pinouts
const int ArmLimitSwitchPin = 23;

int l_stick_Y = 0, l1_r1_btns = 0, up_down_btns = 0, left_right_btns = 0, cmd_btns = 0, l_2_value = 0, r_2_value = 0;
float arm_stepper_position = 0.0;
float arm_servo_val = 0.0, wrist_servo_val = 0.0, grip_servo_val = 0.0, ring_lift_servo_val = 0.0;

Servo arm_servo, wrist_servo, grip_servo, ring_lift_servo;
AccelStepperWithDistance arm_stepper(AccelStepperWithDistance::DRIVER, ArmStepperStep, ArmStepperDir);
const float arm_top_position = 230.0; // 250 mm
const float arm_bottom_position = 0.0;
const float arm_stepper_speed = 1500;
const float arm_stepper_acceleration = 5000;

void readValues();
void calculateValues();
void calculatePresetMotion();
void driveActuators();
float slowIncrement(float, float, float);
void IRAM_ATTR armBottomLimitHit();

void setup()
{
  pinMode(ArmServoPin, OUTPUT);
  pinMode(WristServoPin, OUTPUT);
  pinMode(GripServoPin, OUTPUT);
  pinMode(RingLiftServoPin, OUTPUT);
  pinMode(ArmStepperDir, OUTPUT);
  pinMode(ArmStepperStep, OUTPUT);
  pinMode(ArmLimitSwitchPin, INPUT);

  arm_servo.attach(ArmServoPin);
  wrist_servo.attach(WristServoPin);
  grip_servo.attach(GripServoPin);
  ring_lift_servo.attach(RingLiftServoPin);

  arm_stepper.setAcceleration(arm_stepper_acceleration);
  arm_stepper.setMaxSpeed(arm_stepper_speed);
  arm_stepper.setStepsPerRotation(400);
  arm_stepper.setDistancePerRotation(8.0);
  attachInterrupt(ArmLimitSwitchPin, armBottomLimitHit, FALLING);

  Serial.begin(115200);
  Serial2.begin(115200);
}

void IRAM_ATTR armBottomLimitHit()
{
  arm_stepper.setCurrentPosition(0);
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
    l1_r1_btns = Serial2.parseInt();
    up_down_btns = Serial2.parseInt();
    left_right_btns = Serial2.parseInt();
    cmd_btns = Serial2.parseInt();
  }
}

void calculateValues()
{
  l_stick_Y = (l_stick_Y < -10 ? l_stick_Y : (l_stick_Y > 10 ? l_stick_Y : 0));

  arm_servo_val = map(l_stick_Y, -127, 127, 0, 220);
  // if (l_stick_Y >= 0)
  //   arm_servo_val += l_stick_Y;
  // else
  // {
  //   arm_servo_val += (0.0001 * l_stick_Y);
  // }

  // wrist_servo_val += (0.01 * left_right_btns - 0.00005 * l_stick_Y);
  wrist_servo_val += (0.05 * left_right_btns);
  grip_servo_val = map(r_2_value, 0, 255, 25, 0);
  ring_lift_servo_val = map(l_2_value, 0, 255, 0, 180);

  if (!arm_stepper.isRunning())
    arm_stepper_position += (up_down_btns * 10.0);

  if (l1_r1_btns == -1)
    arm_stepper_position = arm_top_position;
  else if (l1_r1_btns == 1)
    arm_stepper_position = arm_bottom_position;
}

float slowIncrement(float current, float target, float speed = 0.005)
{
  if (current < target)
  {
    current += speed;
  }
  else if (current > target)
  {
    current -= speed;
  }
  return current;
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
  else if (cmd_btns == 4)
  {
    arm_servo_val = 120.0;
    wrist_servo_val = 0.0;
    grip_servo_val = 0.0;

    arm_servo.write(100);
    wrist_servo.write(0);
    grip_servo.write(0);
  }
}

void driveActuators()
{
  arm_servo_val = constrain(arm_servo_val, 0, 270);
  wrist_servo_val = constrain(wrist_servo_val, 0, 80);
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

  arm_stepper.moveToDistance(arm_stepper_position);
  arm_stepper.run();
}
