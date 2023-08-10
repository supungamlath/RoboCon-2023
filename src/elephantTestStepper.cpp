// Include the AccelStepper Library
#include <Arduino.h>
#include <AccelStepperWithDistance.h>

// Shooter pinouts
const int LoaderStepPin = 2;
const int LoaderDirPin = 4;

// Stack pinouts
const int StackStepPin = 12;
const int StackDirPin = 13;

// Shooter adjuster
const int ShooterAdjusterStepPin = 21;
const int ShooterAdjusterDirPin = 19;

// Stack Stepper instance - distance
AccelStepperWithDistance stack_stepper(AccelStepperWithDistance::DRIVER, StackStepPin, StackDirPin);
float stack_bottom_position = -60.0;
float stack_top_position = 0.0;
float stack_speed = 600;
float stack_acceleration = 500;

// Stack loader stepper
AccelStepperWithDistance loader_stepper(AccelStepperWithDistance::DRIVER, LoaderStepPin, LoaderDirPin);
float loader_left_position = 52.0;
float loader_right_position = 0.0;
float loader_speed = 800;
float loader_acceleration = 800;

// Shooter adjuster stepper
AccelStepperWithDistance shooter_adjuster_stepper(AccelStepperWithDistance::DRIVER, ShooterAdjusterStepPin, ShooterAdjusterDirPin);
float shooter_adjuster_stepper_top_position = -35.0;
float shooter_adjuster_stepper_bottom_position = 0.0;
float shooter_adjuster_stepper_speed = 500;
float shooter_adjuster_stepper_acceleration = 500;

void setup()
{
    // set the maximum speed, acceleration factor,
    // Shooter Motor Initialization

    // Loader initialization
    loader_stepper.setAcceleration(loader_acceleration);
    loader_stepper.setMaxSpeed(loader_speed);
    loader_stepper.setStepsPerRotation(200);
    loader_stepper.setDistancePerRotation(4.8);
    loader_stepper.moveToDistance(50.0);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setMaxSpeed(stack_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(4.3);
    stack_stepper.moveToDistance(100.0);

    // Shooter Adjuster initialization
    shooter_adjuster_stepper.setAcceleration(shooter_adjuster_stepper_acceleration);
    shooter_adjuster_stepper.setMaxSpeed(shooter_adjuster_stepper_speed);
    shooter_adjuster_stepper.setStepsPerRotation(200);
    shooter_adjuster_stepper.setDistancePerRotation(1.0);
    shooter_adjuster_stepper.runToNewDistance(50.0);
}

void loop()
{
}