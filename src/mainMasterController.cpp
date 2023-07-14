#include <Arduino.h>
#include <PS4Controller.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include "esp_gap_bt_api.h"
#include "esp_err.h"

const int FdFrontLeft = 2;
const int BkFrontLeft = 23;

const int FdBackLeft = 25;
const int BkBackLeft = 26;

const int FdFrontRight = 32;
const int BkFrontRight = 27;

const int FdBackRight = 18;
const int BkBackRight = 19;

const int deadzone = 10;

bool isPS4Connected = false;

int x = 0, y = 0, z = 0;
int m1_pow = 0, m2_pow = 0, m3_pow = 0, m4_pow = 0;

void onConnect()
// when connected ps4
{
  Serial.println("Connected!.");
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
  pinMode(FdFrontLeft, OUTPUT);
  pinMode(BkFrontLeft, OUTPUT);
  pinMode(FdBackLeft, OUTPUT);
  pinMode(BkBackLeft, OUTPUT);
  pinMode(FdFrontRight, OUTPUT);
  pinMode(BkFrontRight, OUTPUT);
  pinMode(FdBackRight, OUTPUT);
  pinMode(BkBackRight, OUTPUT);
  // Print the Bluetooth MAC address
  Serial.begin(115200);
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

void loop()
{
  if (isPS4Connected)
  {
    y = PS4.RStickY();
    x = PS4.RStickX();
    z = PS4.LStickX();

    y = (y < -deadzone ? y : (y > deadzone ? y : 0));
    x = (x < -deadzone ? x : (x > deadzone ? x : 0));
    z = (z < -deadzone ? z : (z > deadzone ? z : 0));

    // Serial.print("x = ");
    // Serial.print(x);
    // Serial.print("   ");
    // Serial.print("y = ");
    // Serial.print(y);
    // Serial.print("   ");
    // Serial.print("z = ");
    // Serial.println(z);

    CalculateMotorSpeeds();
    SetMotorSpeeds();

    // Serial.print("m1pow = ");
    // Serial.print(m1_pow);
    // Serial.print("   ");
    // Serial.print("m2pow = ");
    // Serial.print(m2_pow);
    // Serial.print("   ");
    // Serial.print("m3pow = ");
    // Serial.print(m3_pow);
    // Serial.print("   ");
    // Serial.print("m4pow = ");
    // Serial.println(m4_pow);
  }
  else
  {
    Serial.println("PS4 not connected");
    delay(1000);
  }
}

void CalculateMotorSpeeds()
{
  // should be multiplied by 2 for full power utilization
  // m1_pow = (y - x - z);
  // m2_pow = (y + x - z);
  // m3_pow = (y + x + z);
  // m4_pow = (y - x + z);
  m1_pow = 2 * (y + x);
  m2_pow = 2 * (y - x);
  m3_pow = 2 * (y - x);
  m4_pow = 2 * (y + x);
  m1_pow = constrain(m1_pow, -255, 255);
  m2_pow = constrain(m2_pow, -255, 255);
  m3_pow = constrain(m3_pow, -255, 255);
  m4_pow = constrain(m4_pow, -255, 255);
}

void SetMotorSpeeds()
{
  if (m1_pow > 0) // forward
  {
    analogWrite(FdFrontLeft, m1_pow);
    analogWrite(BkFrontLeft, LOW);
  }
  else if (m1_pow < 0) // backward
  {
    analogWrite(BkFrontLeft, -m1_pow);
    analogWrite(FdFrontLeft, LOW);
  }
  else
  {
    analogWrite(FdFrontLeft, HIGH);
    analogWrite(BkFrontLeft, HIGH);
  }

  if (m2_pow > 0) // forward
  {
    analogWrite(FdBackLeft, m2_pow);
    analogWrite(BkBackLeft, LOW);
  }
  else if (m2_pow < 0) // backward
  {
    analogWrite(BkBackLeft, -m2_pow);
    analogWrite(FdBackLeft, LOW);
  }
  else
  {
    analogWrite(FdBackLeft, HIGH);
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
