#include <Preferences.h>
#include <SimpleFOC.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncUDP.h>

void setup()
{
    Serial.begin(115200);
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
        Serial.println(str);
    }
}