#include <Arduino.h>
#include <AccelStepperWithDistance.h>
#include <ESP32Servo.h>

// Shooter pinouts
const int LoaderStepPin = 2;
const int LoaderDirPin = 4;

// Stack pinouts
const int StackStepPin = 12;
const int StackDirPin = 13;

// Shooter Motor
const int FdShooterMotor = 26;
const int BkShooterMotor = 25;

const int ShooterStopServoPin = 22;

// Shooter adjuster
const int ShooterAdjusterStepPin = 21;
const int ShooterAdjusterDirPin = 19;
const float stepsize = 10;

const int deadzone = 10;

int l_2 = 0, r_2 = 0, l_stick_Y = 0;
int left = 0, right = 0, up = 0, down = 0;
int cross = 0;
int arm_motor = 0, shooter_motor_val = 0, stack_motor_val = 0, loader_position = 0;
float stack_position = 0;
long lastMillis = 0;

// Stack Stepper instance - distance
AccelStepperWithDistance stack_stepper(AccelStepperWithDistance::DRIVER, StackStepPin, StackDirPin);
float stack_max_position = 70;
float stack_min_position = 0;
float stack_speed = 1000;
float stack_acceleration = 1000;

// Stack loader stepper
AccelStepperWithDistance loader_stepper(AccelStepperWithDistance::DRIVER, LoaderStepPin, LoaderDirPin);
float loader_max_position = 0;
float loader_min_position = 12;
float loader_speed = 1000;
float loader_acceleration = 800;

// Shooter adjuster stepper
AccelStepperWithDistance shooter_adjuster_stepper(AccelStepperWithDistance::DRIVER, LoaderStepPin, LoaderDirPin);
float shooter_adjuster_stepper_max_position = 30;
float shooter_adjuster_stepper_min_position = 0;
float shooter_adjuster_stepper_speed = 1000;
float shooter_adjuster_stepper_acceleration = 1000;

Servo shooter_stop_servo;

// put function declarations here:
void readValues();
void calculateValues();
void driveActuators();

void setup()
{
    // put your setup code here, to run once:

    // Shooter Motor Initialization
    pinMode(FdShooterMotor, OUTPUT);
    pinMode(BkShooterMotor, OUTPUT);
    pinMode(ShooterStopServoPin, OUTPUT);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setMaxSpeed(stack_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(2);

    // Load Trigger initialization
    loader_stepper.setAcceleration(loader_acceleration);
    loader_stepper.setMaxSpeed(loader_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(2);

    // Load Trigger initialization
    shooter_adjuster_stepper.setAcceleration(shooter_adjuster_stepper_acceleration);
    shooter_adjuster_stepper.setMaxSpeed(shooter_adjuster_stepper_speed);
    shooter_adjuster_stepper.setStepsPerRotation(200);
    shooter_adjuster_stepper.setDistancePerRotation(2);

    // Servo initialization
    shooter_stop_servo.attach(ShooterStopServoPin);

    Serial.begin(115200);
    Serial2.begin(115200);
}

void loop()
{
    readValues();      // Get values from Master ESP32
    calculateValues(); // Calculate direction and PWM of each motor
    driveActuators();  // Drive each motor
}

void readValues()
{
    if (Serial2.read() == 6)
    {
        l_stick_Y = Serial2.parseInt();
        l_2 = Serial2.parseInt();
        r_2 = Serial2.parseInt();
        left = Serial2.parseInt();
        right = Serial2.parseInt();
        up = Serial2.parseInt();
        down = Serial2.parseInt();
        cross = Serial2.parseInt();
    }
}

void calculateValues()
{
    // set shooter value
    if (abs(l_2) > 0)
        shooter_motor_val = -1 * abs(l_2);
    else if (abs(r_2) > 0)
        shooter_motor_val = abs(r_2);
    else
        shooter_motor_val = 0;

    // set stack value
    stack_motor_val = 0.1 * l_stick_Y;

    // set loader value
    if (left == 1)
    {
        loader_position = loader_min_position;
    }
    else if (right == 1)
    {
        loader_position = loader_max_position;
    }

    if (abs(loader_stepper.getCurrentPositionDistance() - loader_max_position) < 5)
    {
        shooter_stop_servo.write(0);
    }
    else
    {
        shooter_stop_servo.write(90);
    }
}

void driveActuators()
{
    // Shooter Motor
    if (shooter_motor_val > 0)
    {
        digitalWrite(FdShooterMotor, shooter_motor_val);
        digitalWrite(BkShooterMotor, LOW);
    }
    else if (shooter_motor_val < 0)
    {
        digitalWrite(FdShooterMotor, LOW);
        digitalWrite(BkShooterMotor, shooter_motor_val);
    }
    else
    {
        digitalWrite(FdShooterMotor, LOW);
        digitalWrite(BkShooterMotor, LOW);
    }

    // Stack Motor
    stack_stepper.moveRelative(stack_motor_val);
    stack_stepper.run();

    // Loader Motor
    loader_stepper.moveToDistance(loader_position);
    loader_stepper.run();

    // Shooter Adjuster Motor
    if (up == 1)
        shooter_adjuster_stepper.move(stepsize);
    else if (down == 1)
        shooter_adjuster_stepper.move(-stepsize);

    // Reload operation
    if (cross == 1)
    {
        shooter_adjuster_stepper.runToNewDistance(loader_max_position);
        delay(1000);
        // engage servo
        shooter_adjuster_stepper.runToNewDistance(loader_min_position);
    }
}