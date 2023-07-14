#include <Arduino.h>

const int FdFrontLeft = 2;
const int BkFrontLeft = 23;

const int FdBackLeft = 25;
const int BkBackLeft = 26;

const int FdFrontRight = 32;
const int BkFrontRight = 27;

const int FdBackRight = 18;
const int BkBackRight = 19;

void SetMotor(int, int);

void setup()
{
    pinMode(FdFrontLeft, OUTPUT);
    pinMode(BkFrontLeft, OUTPUT);
    pinMode(FdBackLeft, OUTPUT);
    pinMode(BkBackLeft, OUTPUT);
    pinMode(FdFrontRight, OUTPUT);
    pinMode(BkFrontRight, OUTPUT);
    pinMode(FdBackRight, OUTPUT);
    pinMode(BkBackRight, OUTPUT);
}

void loop()
{
    // SetMotor(4, 55);
}

// Function to test motors individually
// Inputs - Motor, Speed
void SetMotor(int motor, int speed)
{
    switch (motor)
    {
    case 1:
        analogWrite(FdFrontLeft, speed);
        digitalWrite(BkFrontLeft, LOW);
        break;
    case 2:
        analogWrite(FdBackLeft, speed);
        digitalWrite(BkBackLeft, LOW);
        break;
    case 3:
        analogWrite(FdFrontRight, speed);
        digitalWrite(BkFrontRight, LOW);
        break;
    case 4:
        analogWrite(FdBackRight, speed);
        digitalWrite(BkBackRight, LOW);
        break;
    default:
        break;
    }
}
