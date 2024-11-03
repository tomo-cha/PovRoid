/*
serialとserial2
*/

#include <Preferences.h>
#include <SimpleFOC.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncUDP.h>

/*
debug部分の切り替え
*/
// #define DEBUG_MOTOR

#ifdef DEBUG_MOTOR
constexpr bool kMotorDebug = true;
#else
constexpr bool kMotorDebug = false;
#endif

/*
動作モードの切り替え
*/
enum ControlMode
{
    RotationMode,
    AngleMode
};
ControlMode currentMode = RotationMode; // 起動時のモード
// モードを切り替える関数
void setMode(ControlMode mode)
{
    currentMode = mode;
}

/*
Serial
*/
bool dataReceived = false;
// floatに変換可能なStringかどうかを確認する関数
bool isValidFloat(String str)
{
    char *end;
    str.toFloat(); // 一度toFloatで確認する手法でも使えますがより確実なのはatofもしくはstrtod
    strtod(str.c_str(), &end);
    return (*end == '\0'); // endが\0であれば正しく数値として解釈された証拠
}

/*
simpleFOC
*/
// TODO: motor instance
BLDCMotor motor = BLDCMotor(11, 5.57 / 2.0);

// TODO: driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);

// TODO: sensor instance
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5); // cs=ss=IO5
const float buffer = 0.1;
unsigned long rotTime, timeOld;
float real_vel = 0.00;
bool isZeroPositionPassed = true;

/*
preference
*/
Preferences preferences;
float zero_position;

/*
LED
*/
// #include "graphics.h"
const int NUMPIXELS = 50 * 2;
const int Div = 60;
#define DATAPIN 13
#define CLOCKPIN 14
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
unsigned long pic[Div][NUMPIXELS] = {
    0,
};
// char chararrayDiv[] = "0x00";
// char chararrayColor[] = "0xffffff";
const int RX_PIN = 16;
const int TX_PIN = 17;
bool recording = false;
char chararrayDiv[4] = {'0', '0', '0', '0'};                       // フレームインデックス用の一時変数
char chararrayColor[8] = {'0', '0', '0', '0', '0', '0', '0', '0'}; // カラー値用の一時変数
unsigned int numDiv = 0;
int stateRot = 0;

/*
WIFI
*/
#include "security.h"
// wifi udp
AsyncUDP udp_pic;
AsyncUDP udp_motor;
const int port_pic = 1234;
const int port_motor = 9090;

/*
タイマー
*/

int past_time = 0;

void getZeroPosition()
{
    preferences.begin("setPosition", false);
    zero_position = preferences.getFloat("zeroPosition", 0);
    Serial2.print("Current zeroPosition value: ");
    Serial2.println(zero_position);
    zero_position = fmod(zero_position + 3.14, 2.0 * PI);
    preferences.end();
}

void motorSetUp()

{
    // init the sensor
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // TODO: driver config
    // init the driver
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 12;
    if (!driver.init())
    {
        Serial2.println("Driver init failed!");
        return;
    }
    // link driver
    motor.linkDriver(&driver);

    // set motion control loop to be used
    motor.torque_controller = TorqueControlType::voltage;
    getZeroPosition();
    // set the initial motor target
    if (currentMode == RotationMode)
    {
        // motor.controller = MotionControlType::velocity_openloop;
        // motor.target = 10;
        // motor.voltage_limit = 4;
        motor.controller = MotionControlType::velocity;
        motor.target = 100;
        motor.voltage_limit = 1;
    }
    else if (currentMode == AngleMode)
    {
        motor.controller = MotionControlType::angle;
        motor.target = zero_position;
        motor.voltage_limit = 1;
    }

    // use monitoring with serial. SimpleFOCDebug::enable(&Serial);の役割を含んでいる
    if (kMotorDebug)
    {
        motor.useMonitoring(Serial2);
        motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
        motor.monitor_downsample = 1000; // default 10
    }

    // initialize motor
    if (!motor.init())
    {
        Serial2.println("Motor init failed!");
        return;
    }
    // align sensor and start FOC
    if (!motor.initFOC())
    {
        Serial2.println("FOC init failed!");
        return;
    }

    Serial2.println(F("Motor ready."));
}

void motorControlTask(void *pvParameters)
{
    Serial2.print("motorControlTask exec core: ");
    Serial2.println(xPortGetCoreID()); // 動作確認用出力

    motorSetUp();
    _delay(1000);

    while (1)
    {
        // main FOC algorithm function
        if (currentMode == RotationMode)
        {
            motor.loopFOC();
            // sensor.update();
            float currentSensorValue = -1.0 * sensor.getAngle();
            float normalizedSensorValue = fmod(currentSensorValue, 2.0 * PI);
            if (abs(zero_position - normalizedSensorValue) <= buffer) // zeropositionを通過したかどうか
            {
                if (isZeroPositionPassed)
                {
                    unsigned long timeNow = micros();
                    rotTime = timeNow - timeOld;
                    // Serial2.print("timeNow:");
                    // Serial2.print(timeNow);
                    // Serial2.print(",rotTime:");
                    // Serial2.print(rotTime);
                    // Serial2.print(",timeOld:");
                    // Serial2.println(timeOld);
                    timeOld = timeNow;

                    real_vel = 2 * PI * 1000000 / rotTime;
                    // Serial2.println(real_vel);
                    isZeroPositionPassed = false;
                    // numDiv = 35; // 画像の傾き調整
                }
            }
            else
            {
                isZeroPositionPassed = true;
            }
        }
        else if (currentMode == AngleMode)
        {
            motor.loopFOC();
        }
        motor.move();
        if (kMotorDebug)
        {
            motor.monitor();
        }
        /*
        時間による演出
        10秒後にvelocity modeに変更。
        */
        // int current_time = millis();
        // if (current_time - past_time > 10000)
        // {
        //     motor.controller = MotionControlType::velocity;
        //     motor.target = 80;
        //     // past_time = current_time;
        // }
    }
}
void handleUDPInput(String str)
{
    if (str == "r")
    {
        setMode(RotationMode);
        motor.controller = MotionControlType::velocity;
        motor.target = 100;
        motor.voltage_limit = 1;
        Serial2.println("Switched to Rotation Mode");
    }
    else if (str == "a")
    {
        setMode(AngleMode);
        motor.controller = MotionControlType::angle;
        motor.target = zero_position;
        motor.voltage_limit = 1;
        Serial2.println("Switched to Angle Mode");
    }
    else if (isValidFloat(str))
    {
        float input = str.toFloat();
        Serial2.println("Valid float value: " + String(input));
        if (currentMode == RotationMode)
        {
            motor.voltage_limit = input;
        }
        else if (currentMode == AngleMode)
        {
            motor.target = zero_position + input;
        }
    }
    else
    {
        Serial2.println("Invalid input! Enter 'r' for Rotation Mode or 'a' for Angle Mode");
    }
}

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
    // for (int k = 0; k < Div; k++)
    // {
    //     for (int i = 0; i < NUMPIXELS; i++)
    //     {
    //         Serial2.print(pic[k][i]);
    //         Serial2.print(",");
    //     }
    // }
}

void serialInputTask(void *pvParameters)
{
    Serial2.print("SerialInputTask exec core: ");
    Serial2.println(xPortGetCoreID()); // 動作確認用出力
    while (1)
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
        delay(1);
    }
}

void wifiTask(void *pvParameters)
{
    Serial2.print("wifiTask exec core: ");
    Serial2.println(xPortGetCoreID()); // 動作確認用出力
    // wifi
    WiFi.mode(WIFI_STA);
    if (!WiFi.config(ip, gateway, subnet, dns1)) // 固定ipの設定 https://www.farmsoft.jp/113/
    {
        Serial2.println("Failed to configure!");
    }
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial2.print(".");
    }

    Serial2.println("");
    Serial2.println("WiFi connected");
    Serial2.println("IP address: ");
    Serial2.println(WiFi.localIP());

    // UDP受信
    if (udp_pic.listen(port_pic)) // python側とポートを合わせる。自由な数字で良い
    {
        Serial2.print("UDP Listening on IP: ");
        Serial2.println(WiFi.localIP());
        udp_pic.onPacket([](AsyncUDPPacket packet)
                         {
                             //  Serial2.println("recv udp packet!");
                             chararrayDiv[2] = packet.data()[0];
                             chararrayDiv[3] = packet.data()[1];
                             //  Serial2.print("strtoul=");
                             //  Serial2.println(int(strtoul(chararrayDiv, NULL, 16))); // パケットロスをしらべる
                             // for (int k = 0; k < Frame; k++) { //gif用
                             for (int i = 0; i < NUMPIXELS; i++)
                             {
                                 for (int j = 0; j < 6; j++)
                                 {
                                     chararrayColor[j + 2] = packet.data()[2 + i * 6 + j];
                                 }
                                 pic[int(strtoul(chararrayDiv, NULL, 16))][i] = strtoul(chararrayColor, NULL, 16);
                             }
                             //      }
                         });
    }
    // ポート9090でリッスン
    if (udp_motor.listen(port_motor))
    {
        Serial2.printf("UDP Listening on IP: %s, Port: %d\n", WiFi.localIP().toString().c_str(), port_motor);
        udp_motor.onPacket([](AsyncUDPPacket packet)
                           {
                               //    Serial2.print("packet.data():");
                               //    Serial2.println((char *)packet.data());
                               String str = (char *)packet.data();
                               str.replace("\r", ""); // キャリッジリターン（\r）を削除
                               str.replace("\n", ""); // 改行（\n）を削除
                               str.trim();            // 入力のトリミング
                                                      //    Serial2.print("str:");
                                                      //    Serial2.println(str);
                               handleUDPInput(str);   // シリアルタスクの処理を呼び出し
                           });
    }

    while (1)
    {
        delay(1); // 1以上にするとウォッチドッグのリセットがなくなる
    }
}

void checkRotationTask(void *pvParameters)
{
    Serial2.println(xPortGetCoreID()); // 動作確認用出力
    while (1)
    {
        if (currentMode == RotationMode)
        {
            // sensor.update();
            float currentSensorValue = -1.0 * sensor.getAngle();
            float normalizedSensorValue = fmod(currentSensorValue, 2.0 * PI);
            if (abs(zero_position - normalizedSensorValue) <= buffer) // zeropositionを通過したかどうか
            {
                if (isZeroPositionPassed)
                {
                    unsigned long timeNow = micros();
                    rotTime = timeNow - timeOld;
                    // Serial2.print("timeNow:");
                    // Serial2.print(timeNow);
                    // Serial2.print(",rotTime:");
                    // Serial2.print(rotTime);
                    // Serial2.print(",timeOld:");
                    // Serial2.println(timeOld);
                    timeOld = timeNow;

                    real_vel = 2 * PI * 1000000 / rotTime;
                    // Serial2.println(real_vel);
                    isZeroPositionPassed = false;
                    // numDiv = 35; // 画像の傾き調整
                }
            }
            else
            {
                isZeroPositionPassed = true;
            }
        }
        delay(1);
    }
}

void ledTask(void *pvParameters)
{
    Serial2.print("ledTask exec core: ");
    Serial2.println(xPortGetCoreID()); // 動作確認用出力
    strip.begin();
    strip.setBrightness(50); // max 255
    strip.clear();
    strip.show();
    int stateDiv = 0;
    Serial2.println("clear");
    delay(5000);

    while (1)
    {
        if (currentMode == RotationMode)
        {
            // strip.clear(); // 一つ前の点灯パターンを消さないとそのまま残る。これがないと画像が回転しているように見える
            // for (int i = 0; i < NUMPIXELS; i++)
            // {
            //     strip.setPixelColor(i, pic[numDiv][i]);
            // }
            // // setした通りにLEDを光らせる
            // strip.show();
            // if (micros() - timeOld > rotTime / Div * (numDiv + 1))
            // {
            //     numDiv++;

            //     if (numDiv >= Div)
            //     {
            //         numDiv = 0;
            //     }
            // }

            if (stateDiv == 1 && micros() - timeOld > rotTime / Div * (numDiv))
            {
                stateDiv = 0;
            }

            if (stateDiv == 0 && micros() - timeOld < rotTime / Div * (numDiv + 1))
            {
                stateDiv = 1;

                // strip.clear();

                for (int i = 0; i < NUMPIXELS; i++)
                {
                    strip.setPixelColor(i, pic[numDiv][i]);
                }

                strip.show();
                // Serial2.print("numDiv:");
                // Serial2.println(numDiv);

                numDiv++;

                if (numDiv >= Div)
                    numDiv = 0;
            }
        }
        delay(1);
    }
}

/*
dual core
*/
TaskHandle_t taskHandle[5];

void setup()
{
    Serial.begin(230400);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    xTaskCreatePinnedToCore(
        motorControlTask,   // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
        "motorControlTask", // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
        4096,               // スタックサイズ(Byte)
        NULL,               // 作成タスクのパラメータのポインタ
        1,                  // 作成タスクの優先順位(0:低 - 25:高)
        &taskHandle[0],     // 作成タスクのHandleへのポインタ
        1                   // 利用するCPUコア(0-1)
    );
    xTaskCreatePinnedToCore(
        serialInputTask,   // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
        "serialInputTask", // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
        2048,              // スタックサイズ(Byte)
        NULL,              // 作成タスクのパラメータのポインタ
        1,                 // 作成タスクの優先順位(0:低 - 25:高)
        &taskHandle[1],    // 作成タスクのHandleへのポインタ
        0                  // 利用するCPUコア(0-1)
    );
    // xTaskCreatePinnedToCore(
    //     wifiTask,       // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
    //     "wifiTask",     // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
    //     8192,           // スタックサイズ(Byte)
    //     NULL,           // 作成タスクのパラメータのポインタ
    //     1,              // 作成タスクの優先順位(0:低 - 25:高)
    //     &taskHandle[2], // 作成タスクのHandleへのポインタ
    //     0               // 利用するCPUコア(0-1)
    // );
    // xTaskCreatePinnedToCore(
    //     checkRotationTask,   // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
    //     "checkRotationTask", // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
    //     4096,                // スタックサイズ(Byte)
    //     NULL,                // 作成タスクのパラメータのポインタ
    //     1,                   // 作成タスクの優先順位(0:低 - 25:高)
    //     &taskHandle[3],      // 作成タスクのHandleへのポインタ
    //     0                    // 利用するCPUコア(0-1)
    // );

    xTaskCreatePinnedToCore(
        ledTask,
        "ledTask",
        4096,
        NULL,
        1,
        &taskHandle[4],
        1);

    // ウォッチドッグ停止
    // disableCore0WDT();
    // disableCore1WDT();  // 起動直後は有効化されていないのでエラーがでる

    // ウォッチドッグ起動
    // enableCore0WDT();
    // enableCore1WDT();
}

void loop()
{
    delay(1);
}