#include <Arduino.h>
#include <PS4Controller.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include "esp_gap_bt_api.h"
#include "esp_err.h"

const int FdMotor = 25;
const int BkMotor = 26;

const int deadzone = 10;

bool isPS4Connected = false;

int x = 0, z = 0;
int m1_pow = 0;
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

void CalculateMotorSpeeds();
void SetMotorSpeeds();
void SendValuesToShooter();

void setup()
{
    pinMode(FdMotor, OUTPUT);
    pinMode(BkMotor, OUTPUT);

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

void loop()
{
    if (isPS4Connected)
    {
        x = PS4.RStickX();
        z = PS4.LStickY();
        z = (z < -deadzone ? z : (z > deadzone ? z : 0));
        x = (x < -deadzone ? x : (x > deadzone ? x : 0));

        CalculateMotorSpeeds();
        SetMotorSpeeds();

        current_time = millis();
        if (current_time - prev_time > data_rate)
        {
            SendValuesToShooter();
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
    m1_pow = 2 * x;
    m1_pow = constrain(m1_pow, -255, 255);
}

void SetMotorSpeeds()
{
    if (m1_pow > 0) // forward
    {
        analogWrite(FdMotor, m1_pow);
        analogWrite(BkMotor, LOW);
    }
    else
    {
        analogWrite(FdMotor, LOW);
        analogWrite(BkMotor, -m1_pow);
    }
}

void SendValuesToShooter()
{
    char ack = 6;
    Serial2.print(ack);

    // Stack controller values
    Serial2.println(z);

    // Loader controller values
    Serial2.println(PS4.L2Value());
    Serial2.println(PS4.R2Value());

    Serial2.println(PS4.Left());
    Serial2.println(PS4.Right());
}
