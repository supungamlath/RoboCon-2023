#include <Arduino.h>
#include <AccelStepper.h>
#include <AccelStepperWithDistance.h>
#include <PS4Controller.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include "esp_gap_bt_api.h"
#include "esp_err.h"

// Define motor interface type
#define motorInterfaceType 1

// Shooter pinouts
const int LoaderStepPin = 2;
const int LoaderDirPin = 4;

// Stack pinouts
const int StackStepPin = 12;
const int StackDirPin = 13;

// Shooter Motor
const int FdShooterMotor = 26;
const int BkShooterMotor = 25;

int l_2 = 0, r_2 = 0, l_stick_Y = 0;
int left = 0, right = 0;
int arm_motor = 0, shooter_motor_val = 0, stack_motor_val = 0, loader_motor_val = 0;
float stack_position = 0, loader_position = 0;

void readValues();

// Stack Stepper instance - distance
AccelStepperWithDistance stack_stepper(motorInterfaceType, StackStepPin, StackDirPin);
float stack_max_position = 700;
float stack_min_position = 0;
float stack_speed = 1000;
float stack_acceleration = 1000;

// Stack loader stepper
AccelStepperWithDistance loader_stepper(motorInterfaceType, LoaderStepPin, LoaderDirPin);
float loader_max_position = 15;
float loader_min_position = 0;
float loader_speed = 500;
float loader_acceleration = 500;

// put function declarations here:
void readValues();
void calculateValues();
void driveMotors();

void setup()
{
    // put your setup code here, to run once:

    // Shooter Motor Initialization
    pinMode(FdShooterMotor, OUTPUT);
    pinMode(BkShooterMotor, OUTPUT);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setSpeed(stack_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(2);

    // Load Trigger initialization
    loader_stepper.setAcceleration(loader_acceleration);
    loader_stepper.setSpeed(loader_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(2);

    Serial.begin(115200);
    Serial2.begin(115200);
}

void loop()
{
    // put your main code here, to run repeatedly:
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
        left = Serial2.parseInt();
        right = Serial2.parseInt();
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
    Serial.print("   left: ");
    Serial.print(left);
    Serial.print("   right: ");
    Serial.println(right);

    // set shooter value
    if (abs(l_2 > 0))
        shooter_motor_val = 0.2 * abs(l_2);
    else if (abs(r_2 < 0))
        shooter_motor_val = 0.2 * abs(r_2);
    else
        shooter_motor_val = 0;

    // set stack value
    stack_motor_val = 0.2 * l_stick_Y;
    stack_position += stack_motor_val;
    if (stack_position > stack_max_position)
        stack_position = stack_max_position;
    else if (stack_position < stack_min_position)
        stack_position = stack_min_position;

    // set loader value
    if (abs(left) > 0)
        loader_motor_val = abs(left);
    else if (abs(right) > 0)
        loader_motor_val = -1 * abs(right);
    else
        loader_motor_val = 0;
}

void driveMotors()
{
    // Shooter Motor
    if (shooter_motor_val > 0)
    {
        digitalWrite(FdShooterMotor, shooter_motor_val);
        digitalWrite(BkShooterMotor, LOW);
    }
    else if (shooter_motor_val < 0)
    {
        digitalWrite(FdShooterMotor, -shooter_motor_val);
        digitalWrite(BkShooterMotor, HIGH);
    }
    else
    {
        digitalWrite(FdShooterMotor, LOW);
        digitalWrite(BkShooterMotor, LOW);
    }

    // Stack Motor
    if (stack_motor_val != 0)
    {
        stack_stepper.moveToDistance(stack_position);
        stack_stepper.run();
    }

    // Loader Motor
    if (loader_motor_val > 0)
    {
        loader_stepper.moveToDistance(loader_max_position);
        loader_stepper.run();
    }
    else if (loader_motor_val < 0)
    {
        loader_stepper.moveToDistance(loader_min_position);
        loader_stepper.run();
    }
}