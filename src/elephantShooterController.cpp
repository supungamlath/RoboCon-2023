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
const int ShooterStepPin = 4;
const int ShooterDirPin = 5;

// Stack pinouts
const int StackStepPin = 27;
const int StackDirPin = 28;

// Loader pinouts
const int LoaderStepPin = 21;
const int LoaderDirPin = 22;

// PS4 Controller
const int deadzone = 10;
bool isPS4Connected = true;
bool isUp = true;

// Shooter Stepper instance
AccelStepper shooter_stepper(motorInterfaceType, ShooterStepPin, ShooterDirPin);
float stepper_max_speed = 1000;
float stepper_acceleration = 500;
float stepper_speed = 500;

// Stack Stepper instance - distance
AccelStepperWithDistance stack_stepper(motorInterfaceType, StackStepPin, StackDirPin);
float stack_max_position = 100;
float stack_min_position = 0;
float stack_speed = 500;
float stack_acceleration = 500;

// put function declarations here:
int testStepper(int, int);
int calculateSpeed(int);

void moveStack(bool);

void onConnect();
void reconnect();

void setup()
{
    // put your setup code here, to run once:
    // Shooter Stepper initialization
    shooter_stepper.setMaxSpeed(stepper_max_speed);
    shooter_stepper.setAcceleration(stepper_acceleration);
    shooter_stepper.setSpeed(stepper_speed);
    Serial.begin(9600);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setSpeed(stack_speed);

    // Begin PS4 controller connection with MAC address
    // PS4.begin();
    // const uint8_t *address = esp_bt_dev_get_address();
    // char str[100];
    // sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", address[0], address[1], address[2], address[3], address[4], address[5]);
    // Serial.println(str);

    // PS4.attachOnConnect(onConnect);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // if Ps4 is not connected, try to connect
    if (isPS4Connected)
    {
        // Set the speed of the stepper
        int speed = calculateSpeed(100);
        shooter_stepper.setSpeed(speed);

        // Move the stack
        moveStack(isUp);
        isUp != isUp;
    }
    // else
    // {
    //     reconnect();
    // }
}

void reconnect()
{
    // if Ps4 is not connected, try to connect
    while (!PS4.isConnected())
    {
        Serial.println("Connecting...");
        PS4.begin();
        delay(1000);
    }
}

void onConnect()
// when connected ps4
{
    Serial.println("Connected!.");
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++)
    {
        char str[100];
        const uint8_t *address = pairedDeviceBtAddr[i];
        sprintf(str, "%d Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", i, address[0], address[1], address[2], address[3], address[4], address[5]);
        Serial.println(str);
    }

    isPS4Connected = true;
}

int calculateSpeed(int input)
{
    Serial.print("Speed: ");
    Serial.println(input);
    int speed = map(speed, -128, 127, -1000, 1000);
    return speed;
}

void moveStack(bool isUp)
{
    // Get Up and Down button of PS4 and move the stack stepper
    if (isUp)
    {
        stack_stepper.moveTo(stack_max_position);
    }
    else
    {
        stack_stepper.moveTo(stack_min_position);
    }
}