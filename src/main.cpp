#include <Preferences.h>
#include <SimpleFOC.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>

/*
debug部分の切り替え
*/
#define DEBUG_MOTOR

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
const int NUMPIXELS = 25 * 2;
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

/*
タイマー
*/
// hw_timer_t *timer = NULL;
// volatile SemaphoreHandle_t timerSemaphore;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// int timer_core;

// void ARDUINO_ISR_ATTR onTimer()
// {
//     // Increment the counter and set the time of ISR
//     portENTER_CRITICAL_ISR(&timerMux);
//     // It is safe to use digitalRead/Write here if you want to toggle an output
//     sensor.update();
//     float currentSensorValue = -1.0 * sensor.getAngle();
//     float normalizedSensorValue = fmod(currentSensorValue, 2.0 * PI);
//     if (abs(zero_position - normalizedSensorValue) <= buffer)
//     {
//         unsigned long timeNow = micros();
//         rotTime = timeNow - timeOld;
//         timeOld = timeNow;
//     }
//     portEXIT_CRITICAL_ISR(&timerMux);
//     // Give a semaphore that we can check in the loop
//     xSemaphoreGiveFromISR(timerSemaphore, NULL);
// }
int past_time = 0;

/*
dual core
*/
TaskHandle_t taskHandle[2];

void getZeroPosition()
{
    preferences.begin("setPosition", false);
    zero_position = preferences.getFloat("zeroPosition", 0);
    Serial.print("Current zeroPosition value: ");
    Serial.println(zero_position);
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
        motor.controller = MotionControlType::velocity_openloop;
        motor.target = 10;
        motor.voltage_limit = 4;
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

    // // タイマー https://docs.espressif.com/projects/arduino-esp32/en/latest/api/timer.html
    // timerSemaphore = xSemaphoreCreateBinary();
    // // Set timer frequency to 1Mhz
    // timer = timerBegin(1000000);

    // // Attach onTimer function to our timer.
    // timerAttachInterrupt(timer, &onTimer);

    // // Set alarm to call onTimer function every second (value in microseconds).
    // // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
    // timerAlarm(timer, 1000000, true, 0);
    Serial.print("motorControlTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力

    motorSetUp();
    _delay(1000);

    while (1)
    {
        // main FOC algorithm function
        if (currentMode == RotationMode)
        {
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
                motor.controller = MotionControlType::velocity_openloop;
                motor.target = 10;
                motor.voltage_limit = 4;
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
                    motor.target = input;
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
    }
}

void testTask(void *pvParameters)
{
    Serial.print("testTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
    strip.begin();

    delay(10000);
    while (1)
    {
        if (currentMode == RotationMode)
        {
            sensor.update();
            float currentSensorValue = -1.0 * sensor.getAngle();
            float normalizedSensorValue = fmod(currentSensorValue, 2.0 * PI);
            if (abs(zero_position - normalizedSensorValue) <= buffer) // zeropositionを通過したかどうか
            {
                unsigned long timeNow = micros();
                rotTime = timeNow - timeOld;
                timeOld = timeNow;
                if (isZeroPositionPassed)
                {
                    real_vel = 2 * PI * 1000000 / rotTime;
                    Serial.println(real_vel);
                    isZeroPositionPassed = false;
                }
            }
            if (micros() - timeOld > rotTime / Div * (numDiv + 1)) // LEDの切り替えタイミング
            {
                isZeroPositionPassed = true;
                strip.clear(); // 一つ前の点灯パターンを消さないとそのまま残る。これがないと画像が回転しているように見える
                for (int i = 0; i < NUMPIXELS; i++)
                {
                    strip.setPixelColor(i, 0xFF0000);
                }
                // setした通りにLEDを光らせる
                strip.show();

                numDiv++;
                if (numDiv >= Div - 1)
                {
                    numDiv = 0;
                }
            }
        }
        else if (currentMode == AngleMode)
        {
        }
        delay(1); // 1以上にするとウォッチドッグのリセットがなくなる
    }
}

void setup()
{
    Serial.begin(115200);

    // Core1でタスク起動
    xTaskCreatePinnedToCore(
        motorControlTask, // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
        "testTask1",      // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
        8192,             // スタックサイズ(Byte)
        NULL,             // 作成タスクのパラメータのポインタ
        1,                // 作成タスクの優先順位(0:低 - 25:高)
        &taskHandle[0],   // 作成タスクのHandleへのポインタ
        1                 // 利用するCPUコア(0-1)
    );

    // Core0でタスク起動
    xTaskCreatePinnedToCore(
        testTask,
        "testTask2",
        8192,
        NULL,
        1,
        &taskHandle[1],
        0);

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