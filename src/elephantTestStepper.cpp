// Include the AccelStepper Library
#include <AccelStepperWithDistance.h>

// Define pin connections
const int dirPin = 4;
const int stepPin = 2;

// Creates an instance
AccelStepperWithDistance myStepper(AccelStepperWithDistance::DRIVER, stepPin, dirPin);
float position = 1.0;

void setup()
{
    // set the maximum speed, acceleration factor,
    // initial speed and the target position
    myStepper.setStepsPerRotation(200);
    myStepper.setDistancePerRotation(1.0);
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(500);
    myStepper.setSpeed(500);
    Serial.begin(9600);
}

void loop()
{
    // Change direction once the motor reaches target position
    myStepper.moveToDistance(position);
    myStepper.run();
    delay(1000);
}