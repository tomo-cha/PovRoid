#include <Preferences.h>
#include <SimpleFOC.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncUDP.h>
const int RX_PIN = 16;
const int TX_PIN = 17;
bool recording = false;
char chararrayDiv[4] = {'0', '0', '0', '0'};                       // フレームインデックス用の一時変数
char chararrayColor[8] = {'0', '0', '0', '0', '0', '0', '0', '0'}; // カラー値用の一時変数

const int NUMPIXELS = 25 * 2;
const int Div = 60;
unsigned long pic[Div][NUMPIXELS] = {
    0,
};
void handleSerialData(String str)
{
    // フレームインデックスを設定
    chararrayDiv[2] = str[0];
    chararrayDiv[3] = str[1];
    int frameIndex = int(strtoul(chararrayDiv, NULL, 16)); // インデックス取得

    // 各ピクセルの色データを設定
    for (int i = 0; i < NUMPIXELS; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            chararrayColor[j + 2] = str[2 + i * 6 + j]; // カラー値を抽出
        }
        pic[frameIndex][i] = strtoul(chararrayColor, NULL, 16); // pic に格納
    }
    // debug
    for (int k = 0; k < Div; k++)
    {
        for (int i = 0; i < NUMPIXELS; i++)
        {
            Serial2.print(pic[k][i]);
            Serial2.print(",");
        }
    }
}

void setup()
{
    Serial.begin(115200); // Baud rateは使用環境に合わせて設定
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop()
{
    String str;
    // データがシリアルから利用可能な場合に実行
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read(); // シリアルから1文字を読み取る
        // Serial2.println(receivedChar);

        if (receivedChar == 'b')
        {                     // 始点の文字 'b' の検出
            str = "";         // 新しいデータのため変数をリセット
            recording = true; // データの読み取りを開始
        }
        else if (receivedChar == '/' && recording)
        {                                             // 終点の文字 '/' の検出
            recording = false;                        // 読み取りを終了
            Serial2.println("Received Data: " + str); // 読み取ったデータを表示
            handleSerialData(str);
        }
        else if (recording)
        {                        // 読み取り中の場合
            str += receivedChar; // データを格納
        }
        delay(1);
    }
}
