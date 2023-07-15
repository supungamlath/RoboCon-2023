#include <Arduino.h>
#include <PS4Controller.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include "esp_gap_bt_api.h"
#include "esp_err.h"

const int ArmServoPin = 2;
const int WristServoPin = 23;

const int GripServoPin = 25;
const int BkBackLeft = 26;

const int FdFrontRight = 32;
const int BkFrontRight = 27;

const int FdBackRight = 18;
const int BkBackRight = 19;

const int deadzone = 10;

bool isPS4Connected = false;

int x = 0, y = 0, z = 0;
int m1_pow = 0, m2_pow = 0, m3_pow = 0, m4_pow = 0;
long data_rate = 20, prev_time = 0, current_time = 0;

void onConnect()
// when connected ps4
{
  Serial.println("Connected !");
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

void setup()
{
  pinMode(ArmServoPin, OUTPUT);
  pinMode(WristServoPin, OUTPUT);
  pinMode(GripServoPin, OUTPUT);
  pinMode(BkBackLeft, OUTPUT);
  pinMode(FdFrontRight, OUTPUT);
  pinMode(BkFrontRight, OUTPUT);
  pinMode(FdBackRight, OUTPUT);
  pinMode(BkBackRight, OUTPUT);
  // Print the Bluetooth MAC address
  Serial.begin(115200);
  Serial2.begin(115200);
  // PS4.begin("d0:bc:c1:3b:2e:99");
  PS4.begin();
  const uint8_t *address = esp_bt_dev_get_address();
  char str[100];
  sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", address[0], address[1], address[2], address[3], address[4], address[5]);
  Serial.println(str);

  PS4.attachOnConnect(onConnect);
}

void CalculateMotorSpeeds();
void SetMotorSpeeds();
void SendValuesToArm();

void loop()
{
  if (isPS4Connected)
  {
    y = PS4.RStickY();
    x = PS4.RStickX();

    y = (y < -deadzone ? y : (y > deadzone ? y : 0));
    x = (x < -deadzone ? x : (x > deadzone ? x : 0));

    CalculateMotorSpeeds();
    SetMotorSpeeds();

    current_time = millis();
    if (current_time - prev_time > data_rate)
    {
      SendValuesToArm();
      prev_time = current_time;
    }
  }
  else
  {
    Serial.println("PS4 Controller not connected");
    delay(1000);
  }
}

void CalculateMotorSpeeds()
{
  // Should be multiplied by 2 for full power utilization
  if (x == 0 && y == 0 && PS4.R3() == 1)
  {
    m1_pow = -255;
    m2_pow = -255;
    m3_pow = 255;
    m4_pow = 255;
  }
  else
  {
    m1_pow = 2 * (y + x);
    m2_pow = 2 * (y - x);
    m3_pow = 2 * (y - x);
    m4_pow = 2 * (y + x);
    m1_pow = constrain(m1_pow, -255, 255);
    m2_pow = constrain(m2_pow, -255, 255);
    m3_pow = constrain(m3_pow, -255, 255);
    m4_pow = constrain(m4_pow, -255, 255);
  }
}

void SetMotorSpeeds()
{
  if (m1_pow > 0) // forward
  {
    analogWrite(ArmServoPin, m1_pow);
    analogWrite(WristServoPin, LOW);
  }
  else if (m1_pow < 0) // backward
  {
    analogWrite(WristServoPin, -m1_pow);
    analogWrite(ArmServoPin, LOW);
  }
  else
  {
    analogWrite(ArmServoPin, HIGH);
    analogWrite(WristServoPin, HIGH);
  }

  if (m2_pow > 0) // forward
  {
    analogWrite(GripServoPin, m2_pow);
    analogWrite(BkBackLeft, LOW);
  }
  else if (m2_pow < 0) // backward
  {
    analogWrite(BkBackLeft, -m2_pow);
    analogWrite(GripServoPin, LOW);
  }
  else
  {
    analogWrite(GripServoPin, HIGH);
    analogWrite(BkBackLeft, HIGH);
  }

  if (m3_pow > 0) // forward
  {
    analogWrite(FdFrontRight, m3_pow);
    analogWrite(BkFrontRight, LOW);
  }
  else if (m3_pow < 0) // backward
  {
    analogWrite(BkFrontRight, -m3_pow);
    analogWrite(FdFrontRight, LOW);
  }
  else
  {
    analogWrite(FdFrontRight, HIGH);
    analogWrite(BkFrontRight, HIGH);
  }

  if (m4_pow > 0) // forward
  {
    analogWrite(FdBackRight, m4_pow);
    analogWrite(BkBackRight, LOW);
  }
  else if (m4_pow < 0) // backward
  {
    analogWrite(BkBackRight, -m4_pow);
    analogWrite(FdBackRight, LOW);
  }
  else
  {
    analogWrite(FdBackRight, HIGH);
    analogWrite(BkBackRight, HIGH);
  }
}

void SendValuesToArm()
{
  char ack = 6;
  Serial2.print(ack);
  Serial2.println(PS4.L2Value());
  Serial2.println(PS4.R2Value());
  Serial2.println(PS4.LStickY());
}
