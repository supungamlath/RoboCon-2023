// Include the AccelStepper Library
#include <AccelStepperWithDistance.h>

// Define pin connections
const int dirPin = 19;
const int stepPin = 21;

// Creates an instance
AccelStepperWithDistance myStepper(AccelStepperWithDistance::DRIVER, stepPin, dirPin);
float position = 5.0;

void setup()
{
    // set the maximum speed, acceleration factor,
    // initial speed and the target position
    myStepper.setStepsPerRotation(200);
    myStepper.setDistancePerRotation(2);
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
}