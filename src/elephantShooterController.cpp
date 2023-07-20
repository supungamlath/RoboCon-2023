#include <Arduino.h>
#include <AccelStepperWithDistance.h>

// Shooter pinouts
const int LoaderStepPin = 2;
const int LoaderDirPin = 4;
const int LoaderLimitSwitchPin = 22;
const float loader_position_step = 1.0;

// Stack pinouts
const int StackStepPin = 12;
const int StackDirPin = 13;
const int StackLimitSwitchPin = 23;
const float stack_ring_height_step = 1.0;

// Shooter Motor
const int FdShooterMotor = 26;
const int BkShooterMotor = 25;
const int ShooterStopLimitPin = 32;

// Shooter adjuster
const int ShooterAdjusterStepPin = 21;
const int ShooterAdjusterDirPin = 19;
const int ShooterAdjusterLimitSwitchPin = 35;
const float adjuster_step_size = 0.5;

const int deadzone = 10;

int l_2 = 0, r_2 = 0, l_stick_Y = 0;
int left_right_btns = 0, up_down_btns = 0, l1_r1_btns = 0, cmd_btns = 0;
int shooter_motor_val = 0;
float loader_position = 0.0;
float adjuster_position = 0.0;
float saved_adjuster_position = 0.0;
float stack_fine_step = 0.0;
int loader_move = 0;
int stack_move = 0;
long lastMillis = 0;

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

void readValues();
void calculateFreeMotion();
void calculatePresetMotion();
void driveActuators();
void IRAM_ATTR stackLimitHit();
void IRAM_ATTR loaderLimitHit();
void IRAM_ATTR adjusterHitLimit();

void setup()
{
    // Shooter Motor Initialization
    pinMode(FdShooterMotor, OUTPUT);
    pinMode(BkShooterMotor, OUTPUT);
    pinMode(ShooterStopLimitPin, INPUT_PULLUP);
    pinMode(LoaderLimitSwitchPin, INPUT_PULLUP);
    pinMode(StackLimitSwitchPin, INPUT_PULLUP);
    pinMode(ShooterAdjusterLimitSwitchPin, INPUT_PULLUP);

    // Loader initialization
    loader_stepper.setAcceleration(loader_acceleration);
    loader_stepper.setMaxSpeed(loader_speed);
    loader_stepper.setStepsPerRotation(200);
    loader_stepper.setDistancePerRotation(4.8);
    attachInterrupt(LoaderLimitSwitchPin, loaderLimitHit, FALLING);
    if (digitalRead(LoaderLimitSwitchPin))
        loader_stepper.moveToDistance(-52.0);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setMaxSpeed(stack_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(4.3);
    attachInterrupt(StackLimitSwitchPin, stackLimitHit, FALLING);
    if (digitalRead(StackLimitSwitchPin))
        stack_stepper.moveToDistance(100.0);

    // Shooter Adjuster initialization
    shooter_adjuster_stepper.setAcceleration(shooter_adjuster_stepper_acceleration);
    shooter_adjuster_stepper.setMaxSpeed(shooter_adjuster_stepper_speed);
    shooter_adjuster_stepper.setStepsPerRotation(200);
    shooter_adjuster_stepper.setDistancePerRotation(1.0);
    attachInterrupt(ShooterAdjusterLimitSwitchPin, adjusterHitLimit, FALLING);
    if (digitalRead(ShooterAdjusterLimitSwitchPin))
        shooter_adjuster_stepper.runToNewDistance(50.0);

    Serial.begin(115200);
    Serial2.begin(115200);
}

void IRAM_ATTR stackLimitHit()
{
    stack_stepper.setCurrentPosition(0);
}
void IRAM_ATTR loaderLimitHit()
{
    loader_stepper.setCurrentPosition(0);
}
void IRAM_ATTR adjusterHitLimit()
{
    shooter_adjuster_stepper.setCurrentPosition(0);
}

void loop()
{
    readValues();      // Get values from Master ESP32
    if (cmd_btns == 0) // If a command button is not pressed
    {
        calculateFreeMotion();
    }
    else // If a command button is pressed
    {
        calculatePresetMotion();
    }
    driveActuators(); // Drive each motor
}

void readValues()
{
    if (Serial2.read() == 6)
    {
        l_stick_Y = Serial2.parseInt();
        l_2 = Serial2.parseInt();
        r_2 = Serial2.parseInt();
        l1_r1_btns = Serial2.parseInt();
        up_down_btns = Serial2.parseInt();
        left_right_btns = Serial2.parseInt();
        cmd_btns = Serial2.parseInt();
    }
}

void calculateFreeMotion()
{
    // set shooter and adjuster value
    if (shooter_adjuster_stepper.currentPosition() == shooter_adjuster_stepper.targetPosition())
    {
        if (up_down_btns == 1)
        {
            adjuster_position += -1 * adjuster_step_size;
            shooter_motor_val = 0.8;
        }
        else if (up_down_btns == -1)
        {
            adjuster_position += adjuster_step_size;
        }
        else
        {
            shooter_motor_val = 0;
        }
    }

    if (l_2 > 0)
    {
        shooter_motor_val = l_2;
    }
    else if (r_2 > 0 && digitalRead(ShooterStopLimitPin))
    {
        shooter_motor_val = -1 * r_2;
    }

    // set stack move
    if (!stack_stepper.run())
        stack_move = l1_r1_btns;
    else
        stack_move = 0;

    l_stick_Y = (l_stick_Y < -10 ? l_stick_Y : (l_stick_Y > 10 ? l_stick_Y : 0));
    stack_fine_step = 0.001 * l_stick_Y;

    // set loader move
    if (loader_stepper.currentPosition() == loader_stepper.targetPosition())
        loader_move = left_right_btns;
    else
        loader_move = 0;
}

void calculatePresetMotion()
{
    // Ring pickup operation
    if (cmd_btns == 1)
    {
        if (stack_stepper.getCurrentPositionDistance() > stack_bottom_position)
        {
            stack_stepper.moveToDistance(stack_bottom_position);
        }
        else
        {
            stack_stepper.moveToDistance(stack_top_position - 14.0);
        }
    }

    // Ring plate reload operation
    if (cmd_btns == 2)
    {
        // saved_adjuster_position = adjuster_position;
        if (shooter_adjuster_stepper.getCurrentPositionDistance() > shooter_adjuster_stepper_top_position)
        {
            shooter_adjuster_stepper.moveToDistance(shooter_adjuster_stepper_top_position);
        }
        else
        {
            shooter_adjuster_stepper.moveToDistance(adjuster_position);
        }
    }

    // if (cmd_btns == 4)
    // {
    //     shooter_adjuster_stepper.moveTo(shooter_adjuster_stepper_bottom_position);
    // }
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
        stack_stepper.moveRelative(stack_ring_height_step);
    }
    else if (stack_move == -1)
    {
        stack_stepper.moveRelative(-stack_ring_height_step);
    }

    if (stack_fine_step != 0)
    {
        stack_stepper.moveRelative(stack_fine_step);
    }

    // Loader Motor
    if (loader_move == 1)
    {
        loader_stepper.moveToDistance(loader_left_position);
    }
    else if (loader_move == -1)
    {
        loader_stepper.moveToDistance(loader_right_position);
    }

    // Shooter Adjuster Motor
    if (up_down_btns != 0)
    {
        shooter_adjuster_stepper.moveToDistance(adjuster_position);
    }

    stack_stepper.run();
    loader_stepper.run();
    shooter_adjuster_stepper.run();
}
