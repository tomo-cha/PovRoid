#include <Preferences.h>
#include <SimpleFOC.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncUDP.h>

const int RX_PIN = 16;
const int TX_PIN = 17;
bool dataReceived = false;
void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}
void loop()
{
    String str;
    while (Serial.available())
    {
        char c = Serial.read();
        if (c != '\n')
        {
            str += c;
        }
        dataReceived = true;
    }
    if (dataReceived)
    {
        Serial2.println(str);
        dataReceived = false;
    }
}