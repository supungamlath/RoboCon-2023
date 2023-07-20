#include <Arduino.h>
#include <AccelStepperWithDistance.h>
#include <ESP32Servo.h>

// Shooter pinouts
const int LoaderStepPin = 2;
const int LoaderDirPin = 4;
const int LoaderLimitSwitchPin = 5;

// Stack pinouts
const int StackStepPin = 12;
const int StackDirPin = 13;
const int StackLimitSwitchPin = 14;
const float stack_position_step = 1.0;

// Shooter Motor
const int FdShooterMotor = 26;
const int BkShooterMotor = 25;

const int ShooterStopServoPin = 22;

// Shooter adjuster
const int ShooterAdjusterStepPin = 21;
const int ShooterAdjusterDirPin = 19;
const float adjuster_step_size = 0.5;

const int deadzone = 10;

int l_2 = 0, r_2 = 0, l_stick_Y = 0;
int left_right_btns = 0, up_down_btns = 0, l1_r1_btns = 0, cmd_btns = 0;
int shooter_motor_val = 0;
float loader_position = 0.0;
int adjuster_move = 0, stack_move = 0;
long lastMillis = 0;

// Stack Stepper instance - distance
AccelStepperWithDistance stack_stepper(AccelStepperWithDistance::DRIVER, StackStepPin, StackDirPin);
float stack_bottom_position = -60.0;
float stack_top_position = 0.0;
float stack_speed = 1000;
float stack_acceleration = 1000;

// Stack loader stepper
AccelStepperWithDistance loader_stepper(AccelStepperWithDistance::DRIVER, LoaderStepPin, LoaderDirPin);
float loader_left_position = -50.0;
float loader_right_position = 0.0;
float loader_speed = 1000;
float loader_acceleration = 800;

// Shooter adjuster stepper
AccelStepperWithDistance shooter_adjuster_stepper(AccelStepperWithDistance::DRIVER, ShooterAdjusterStepPin, ShooterAdjusterDirPin);
float shooter_adjuster_stepper_max_position = 30;
float shooter_adjuster_stepper_min_position = 0;
float shooter_adjuster_stepper_speed = 500;
float shooter_adjuster_stepper_acceleration = 500;

Servo shooter_stop_servo;

// put function declarations here:
void readValues();
void calculateValues();
void driveActuators();
void IRAM_ATTR stackLimitHit();
void IRAM_ATTR loaderLimitHit();

void setup()
{
    // Shooter Motor Initialization
    pinMode(FdShooterMotor, OUTPUT);
    pinMode(BkShooterMotor, OUTPUT);
    pinMode(ShooterStopServoPin, OUTPUT);
    pinMode(LoaderLimitSwitchPin, INPUT_PULLUP);
    pinMode(StackLimitSwitchPin, INPUT_PULLUP);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setMaxSpeed(stack_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(4.3);
    // attachInterrupt(StackLimitSwitchPin, stackLimitHit, FALLING);

    // Loader initialization
    loader_stepper.setAcceleration(loader_acceleration);
    loader_stepper.setMaxSpeed(loader_speed);
    loader_stepper.setStepsPerRotation(200);
    loader_stepper.setDistancePerRotation(1.0);
    // attachInterrupt(LoaderLimitSwitchPin, loaderLimitHit, FALLING);

    // Shooter Adjuster initialization
    shooter_adjuster_stepper.setAcceleration(shooter_adjuster_stepper_acceleration);
    shooter_adjuster_stepper.setMaxSpeed(shooter_adjuster_stepper_speed);
    shooter_adjuster_stepper.setStepsPerRotation(200);
    shooter_adjuster_stepper.setDistancePerRotation(1.0);

    // Servo initialization
    shooter_stop_servo.attach(ShooterStopServoPin);

    Serial.begin(115200);
    Serial2.begin(115200);
}

void IRAM_ATTR stackLimitHit()
{
    stack_stepper.setCurrentPosition(0);
    stack_stepper.stop();
}

void IRAM_ATTR loaderLimitHit()
{
    loader_stepper.setCurrentPosition(0);
    loader_stepper.stop();
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
        up_down_btns = Serial2.parseInt();
        left_right_btns = Serial2.parseInt();
        l1_r1_btns = Serial2.parseInt();
        cmd_btns = Serial2.parseInt();
    }
}

void calculateValues()
{
    // set shooter and adjuster value
    if (l_2 > 0)
        shooter_motor_val = l_2;
    else if (r_2 > 0)
        shooter_motor_val = -1 * r_2;
    else if (up_down_btns == 1)
    {
        adjuster_move = -1;
        shooter_motor_val = 0.8;
    }
    else if (up_down_btns == -1)
    {
        adjuster_move = 1;
    }
    else
    {
        adjuster_move = 0;
        shooter_motor_val = 0;
    }

    // set stack move
    if (stack_stepper.currentPosition() == stack_stepper.targetPosition())
        stack_move = l1_r1_btns;
    else
        stack_move = 0;

    // l_stick_Y = (l_stick_Y < -10 ? l_stick_Y : (l_stick_Y > 10 ? l_stick_Y : 0));
    // stack_position_step = 0.01 * l_stick_Y;

    // set loader value
    if (left_right_btns == 1)
        loader_position = loader_left_position;
    else if (left_right_btns == -1)
        loader_position = loader_right_position;
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
    if (stack_move == 1)
    {
        stack_stepper.moveRelative(stack_position_step);
    }
    else if (stack_move == -1)
    {
        stack_stepper.moveRelative(-stack_position_step);
    }

    // Loader Motor
    loader_stepper.moveToDistance(loader_position);

    // Shooter Adjuster Motor
    if (adjuster_move == 1)
    {
        shooter_adjuster_stepper.moveRelative(adjuster_step_size);
    }
    else if (adjuster_move == -1)
    {
        shooter_adjuster_stepper.moveRelative(-adjuster_step_size);
    }

    // Reload operation
    if (cmd_btns == 1)
    {
        if (stack_stepper.targetPosition() == stack_top_position)
            stack_stepper.moveTo(stack_bottom_position);
        else
            stack_stepper.moveTo(stack_top_position - 14.0);
    }

    if (cmd_btns == 3)
    {
        shooter_adjuster_stepper.runToNewDistance(loader_left_position);
        delay(1000);
        // engage servo
        shooter_adjuster_stepper.runToNewDistance(loader_left_position);
    }

    if (cmd_btns == 4)
    {
        shooter_adjuster_stepper.moveTo(shooter_adjuster_stepper_min_position);
    }

    if (abs(shooter_adjuster_stepper.getCurrentPositionDistance() - shooter_adjuster_stepper_max_position) < 5)
    {
        shooter_stop_servo.write(0);
    }
    else
    {
        shooter_stop_servo.write(90);
    }

    stack_stepper.run();
    loader_stepper.run();
    shooter_adjuster_stepper.run();
}