#include <Arduino.h>

const int StackLimitPin = 35;
const int LoaderTriggerLimitPin = 32;
const int LoaderRingLimitPin = 33;

void checkPinHigh(int pin);

void setup()
{
    pinMode(StackLimitPin, INPUT);
    pinMode(LoaderTriggerLimitPin, INPUT);
    pinMode(LoaderRingLimitPin, INPUT);
    Serial.begin(115200);
}

void checkPinHigh(int pin)
{
    if (!digitalRead(pin))
        Serial.println(pin);
}

void loop()
{
    checkPinHigh(StackLimitPin);
    checkPinHigh(LoaderTriggerLimitPin);
    checkPinHigh(LoaderRingLimitPin);

    delay(100);
}