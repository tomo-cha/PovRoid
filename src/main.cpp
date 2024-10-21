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
ControlMode currentMode = AngleMode; // 起動時のモード
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
const int NUMPIXELS = 25 * 1;
const int Div = 60;
#define DATAPIN 16
#define CLOCKPIN 4
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
unsigned long pic[Div][NUMPIXELS] = {
    0,
};
char chararrayDiv[] = "0x00";
char chararrayColor[] = "0xffffff";
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
    Serial.print("Current zeroPosition value: ");
    Serial.println(zero_position);
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
        Serial.println("Driver init failed!");
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
        motor.useMonitoring(Serial);
        motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
        motor.monitor_downsample = 1000; // default 10
    }

    // initialize motor
    if (!motor.init())
    {
        Serial.println("Motor init failed!");
        return;
    }
    // align sensor and start FOC
    if (!motor.initFOC())
    {
        Serial.println("FOC init failed!");
        return;
    }

    Serial.println(F("Motor ready."));
}

void motorControlTask(void *pvParameters)
{
    Serial.print("motorControlTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力

    motorSetUp();
    _delay(1000);

    while (1)
    {
        // main FOC algorithm function
        if (currentMode == RotationMode)
        {
            motor.loopFOC();
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
        Serial.println("Switched to Rotation Mode");
    }
    else if (str == "a")
    {
        setMode(AngleMode);
        motor.controller = MotionControlType::angle;
        motor.target = zero_position;
        motor.voltage_limit = 1;
        Serial.println("Switched to Angle Mode");
    }
    else if (isValidFloat(str))
    {
        float input = str.toFloat();
        Serial.println("Valid float value: " + String(input));

        if (currentMode == RotationMode)
        {
            if (input < 5.0)
            {
                motor.voltage_limit = input;
            }
        }
        else if (currentMode == AngleMode)
        {
            motor.target = zero_position + input;
        }
    }

    else
    {
        Serial.println("Invalid input! Enter 'r' for Rotation Mode or 'a' for Angle Mode");
    }
}

void serialInputTask(void *pvParameters)
{
    Serial.print("serialInputTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
    while (1)
    {
        // serial input string to float
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
            // モード切り替え用の入力 'r' か 'a' を確認
            if (str == "r")
            {
                setMode(RotationMode);
                // motor.controller = MotionControlType::velocity_openloop;
                // motor.target = 10;
                // motor.voltage_limit = 4;
                motor.controller = MotionControlType::velocity;
                motor.target = 100;
                motor.voltage_limit = 1;
                Serial.println("Switched to Rotation Mode");
            }
            else if (str == "a")
            {
                setMode(AngleMode);
                motor.controller = MotionControlType::angle;
                motor.target = zero_position;
                motor.voltage_limit = 1;
                Serial.println("Switched to Angle Mode");
            }
            // 文字列が数字のみを含むかどうかを確認
            else if (isValidFloat(str))
            {
                float input = str.toFloat();
                Serial.println("Valid float value: " + String(input));

                if (currentMode == RotationMode)
                {
                    // motor.target = input;
                    motor.voltage_limit = input;
                }
                else if (currentMode == AngleMode)
                {
                    motor.target = zero_position + input;
                }
            }
            else
            {
                Serial.println("Invalid input! Enter 'r' for Rotation Mode or 'a' for Angle Mode");
            }
            // 次の入力のためにリセット
            str = "";
            dataReceived = false;
        }
        delay(1);
    }
}

void wifiTask(void *pvParameters)
{
    Serial.print("wifiTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
    // wifi
    WiFi.mode(WIFI_STA);
    if (!WiFi.config(ip, gateway, subnet, dns1)) // 固定ipの設定 https://www.farmsoft.jp/113/
    {
        Serial.println("Failed to configure!");
    }
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // UDP受信
    if (udp_pic.listen(port_pic)) // python側とポートを合わせる。自由な数字で良い
    {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp_pic.onPacket([](AsyncUDPPacket packet)
                         {
                             //  Serial.println("recv udp packet!");
                             chararrayDiv[2] = packet.data()[0];
                             chararrayDiv[3] = packet.data()[1];
                             //  Serial.print("strtoul=");
                             //  Serial.println(int(strtoul(chararrayDiv, NULL, 16))); // パケットロスをしらべる
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
        Serial.printf("UDP Listening on IP: %s, Port: %d\n", WiFi.localIP().toString().c_str(), port_motor);
        udp_motor.onPacket([](AsyncUDPPacket packet)
                           {
                               //    Serial.print("packet.data():");
                               //    Serial.println((char *)packet.data());
                               String str = (char *)packet.data();
                               str.replace("\r", ""); // キャリッジリターン（\r）を削除
                               str.replace("\n", ""); // 改行（\n）を削除
                               str.trim();            // 入力のトリミング
                                                      //    Serial.print("str:");
                                                      //    Serial.println(str);
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
    Serial.println(xPortGetCoreID()); // 動作確認用出力
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
                    Serial.print("timeNow:");
                    Serial.print(timeNow);
                    Serial.print(",rotTime:");
                    Serial.print(rotTime);
                    Serial.print(",timeOld:");
                    Serial.println(timeOld);
                    timeOld = timeNow;

                    real_vel = 2 * PI * 1000000 / rotTime;
                    // Serial.println(real_vel);
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
    Serial.print("ledTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
    strip.begin();
    int stateDiv = 0;
    delay(5000);
    Serial.println("clear");
    strip.setBrightness(100); // max 255
    strip.clear();
    strip.show();
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

                strip.clear();

                for (int i = 0; i < NUMPIXELS; i++)
                {
                    strip.setPixelColor(i, pic[numDiv][i]);
                }

                strip.show();
                // Serial.print("numDiv:");
                // Serial.println(numDiv);

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
    Serial.begin(115200);

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
        1                  // 利用するCPUコア(0-1)
    );
    xTaskCreatePinnedToCore(
        wifiTask,       // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
        "wifiTask",     // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
        8192,           // スタックサイズ(Byte)
        NULL,           // 作成タスクのパラメータのポインタ
        1,              // 作成タスクの優先順位(0:低 - 25:高)
        &taskHandle[2], // 作成タスクのHandleへのポインタ
        0               // 利用するCPUコア(0-1)
    );
    xTaskCreatePinnedToCore(
        checkRotationTask,   // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
        "checkRotationTask", // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
        4096,                // スタックサイズ(Byte)
        NULL,                // 作成タスクのパラメータのポインタ
        1,                   // 作成タスクの優先順位(0:低 - 25:高)
        &taskHandle[3],      // 作成タスクのHandleへのポインタ
        0                    // 利用するCPUコア(0-1)
    );

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
