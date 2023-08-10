#include <Arduino.h>
#include <AccelStepperWithDistance.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Shooter pinouts
const int LoaderStepPin = 2;
const int LoaderDirPin = 4;
const int LoaderLimitSwitchPin = 32;
const int LoaderAlignLimitSwitchPin = 33;
const float loader_position_step = 1.0;

// Stack pinouts
const int StackStepPin = 12;
const int StackDirPin = 13;
const int StackLimitSwitchPin = 35;
const float stack_ring_height_step = 1.0;

// Shooter Motor
const int FdShooterMotor = 26;
const int BkShooterMotor = 25;
const int ShooterStopLimitPin = 18;
const int ShooterAngleLimitPin = 22;

// Shooter adjuster
const int ShooterAdjusterStepPin = 21;
const int ShooterAdjusterDirPin = 19;
const int ShooterAdjusterLimitSwitchPin = 14;
const float adjuster_step_size = 0.5;

const int deadzone = 10;

int l_2 = 0, r_2 = 0, l_stick_Y = 0;
int left_right_btns = 0, up_down_btns = 0, l1_r1_btns = 0, cmd_btns = 0;
int shooter_motor_val = 0;
float loader_position = 0.0;
float adjuster_position = 0.0;
float saved_adjuster_position = 0.0;
float stack_fine_step = 0.0;
int loader_move = 0, loader_move_state = 0;
int stack_move = 0;

// Stack Stepper instance - distance
AccelStepperWithDistance stack_stepper(AccelStepperWithDistance::DRIVER, StackStepPin, StackDirPin);
float stack_bottom_position = -60.0;
float stack_top_position = 0.0;
float stack_speed = 600;
float stack_acceleration = 500;

// Stack loader stepper
AccelStepperWithDistance loader_stepper(AccelStepperWithDistance::DRIVER, LoaderStepPin, LoaderDirPin);
float loader_left_position = 48.0;
float loader_right_position = 2.0;
float loader_speed = 800;
float loader_acceleration = 800;

// Shooter adjuster stepper
AccelStepperWithDistance shooter_adjuster_stepper(AccelStepperWithDistance::DRIVER, ShooterAdjusterStepPin, ShooterAdjusterDirPin);
float shooter_adjuster_stepper_top_position = -35.0;
float shooter_adjuster_stepper_bottom_position = 0.0;
float shooter_adjuster_stepper_speed = 500;
float shooter_adjuster_stepper_acceleration = 500;

long current_time = 0, prev_time = 0;

void readValues();
void calculateFreeMotion();
void calculatePresetMotion();
void driveActuators();
void debug();
void IRAM_ATTR stackLimitHit();
void IRAM_ATTR loaderLimitHit();
void IRAM_ATTR adjusterHitLimit();
void IRAM_ATTR loaderAlignLimitHit();

void setup()
{
    // Shooter Motor Initialization
    pinMode(FdShooterMotor, OUTPUT);
    pinMode(BkShooterMotor, OUTPUT);
    pinMode(ShooterStopLimitPin, INPUT);
    pinMode(ShooterAngleLimitPin, INPUT);
    pinMode(LoaderLimitSwitchPin, INPUT);
    pinMode(LoaderAlignLimitSwitchPin, INPUT);
    pinMode(StackLimitSwitchPin, INPUT);
    pinMode(ShooterAdjusterLimitSwitchPin, INPUT);

    // Loader initialization
    loader_stepper.setAcceleration(loader_acceleration);
    loader_stepper.setMaxSpeed(loader_speed);
    loader_stepper.setStepsPerRotation(200);
    loader_stepper.setDistancePerRotation(4.8);
    attachInterrupt(digitalPinToInterrupt(LoaderAlignLimitSwitchPin), loaderAlignLimitHit, FALLING);
    attachInterrupt(digitalPinToInterrupt(LoaderLimitSwitchPin), loaderLimitHit, FALLING);
    if (digitalRead(LoaderLimitSwitchPin))
        loader_stepper.moveToDistance(-200.0);

    // Stack Stepper initialization
    stack_stepper.setAcceleration(stack_acceleration);
    stack_stepper.setMaxSpeed(stack_speed);
    stack_stepper.setStepsPerRotation(200);
    stack_stepper.setDistancePerRotation(4.3);
    attachInterrupt(digitalPinToInterrupt(StackLimitSwitchPin), stackLimitHit, FALLING);
    if (digitalRead(StackLimitSwitchPin))
        stack_stepper.moveToDistance(100.0);

    // Shooter Adjuster initialization
    shooter_adjuster_stepper.setAcceleration(shooter_adjuster_stepper_acceleration);
    shooter_adjuster_stepper.setMaxSpeed(shooter_adjuster_stepper_speed);
    shooter_adjuster_stepper.setStepsPerRotation(200);
    shooter_adjuster_stepper.setDistancePerRotation(1.0);
    attachInterrupt(digitalPinToInterrupt(ShooterAdjusterLimitSwitchPin), adjusterHitLimit, FALLING);
    if (digitalRead(ShooterAdjusterLimitSwitchPin))
        shooter_adjuster_stepper.runToNewDistance(50.0);

    Serial.begin(115200);
    Serial2.begin(115200);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    delay(2000);
    display.clearDisplay();
    display.setTextSize(1.2);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    // Display static text
    display.println("Elephant Robot V2.0");
    display.display();
    delay(1000);
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
void IRAM_ATTR loaderAlignLimitHit()
{
    loader_move_state = 1;
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

    debug();
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
            adjuster_position -= adjuster_step_size;
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
        stack_stepper.moveRelative(-stack_ring_height_step);
        loader_stepper.moveToDistance(loader_left_position);
    }
    else if (loader_move == -1)
    {
        loader_stepper.moveRelative(-loader_position_step);
    }

    // Shooter Adjuster Motor
    if (up_down_btns != 0)
    {
        shooter_adjuster_stepper.moveToDistance(adjuster_position);
    }

    // Align loader
    if (loader_move_state == 1)
    {
        if (loader_stepper.getCurrentPositionDistance() > loader_right_position)
            loader_stepper.moveToDistance(loader_right_position);
        else
        {
            stack_stepper.moveRelative(-stack_ring_height_step);
            loader_stepper.moveToDistance(loader_left_position);
            loader_move_state = 0;
        }
    }

    stack_stepper.run();
    loader_stepper.run();
    shooter_adjuster_stepper.run();
}

void debug()
{
    // Runs every 20ms
    current_time = millis();
    if (current_time - prev_time > 500)
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Loader: ");
        display.println(loader_stepper.getCurrentPositionDistance());
        display.print("Stack: ");
        display.println(stack_stepper.getCurrentPositionDistance());
        display.print("Adjuster: ");
        display.println(shooter_adjuster_stepper.getCurrentPositionDistance());
        display.display();
        prev_time = current_time;
    }
}