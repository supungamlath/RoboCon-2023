#include <Arduino.h>
#include <AccelStepper.h>
#include <PS4Controller.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include "esp_gap_bt_api.h"
#include "esp_err.h"

// Define motor interface type
#define motorInterfaceType 1

// Shooter pinouts
const int StepperStepPin = 4;
const int StepperDirPin = 5;

// PS4 Controller
const int deadzone = 10;
bool isPS4Connected = false;

// Creates an instance
AccelStepper shooter_stepper(motorInterfaceType, StepperStepPin, StepperDirPin);
float maxposition = 100;
float minposition = 0;

// put function declarations here:
int testStepper(int, int);
int calculateSpeed(int);

void onConnect();
void reconnect();

void setup()
{
    // put your setup code here, to run once:
    shooter_stepper.setMaxSpeed(1000);
    shooter_stepper.setAcceleration(500);
    shooter_stepper.setSpeed(500);
    Serial.begin(9600);

    // Begin PS4 controller connection with MAC address
    PS4.begin();
    const uint8_t *address = esp_bt_dev_get_address();
    char str[100];
    sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", address[0], address[1], address[2], address[3], address[4], address[5]);
    Serial.println(str);

    PS4.attachOnConnect(onConnect);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // if Ps4 is not connected, try to connect
    if (!isPS4Connected)
    {
        reconnect();
    }

    // Get the Lstick of PS4 and map it to the stepper speed
    int val = PS4.LStickY();
    // Set the speed of the stepper
    int speed = calculateSpeed(val);
    shooter_stepper.setSpeed(speed);
}

void reconnect()
{
    // if Ps4 is not connected, try to connect
    while (!isPS4Connected)
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