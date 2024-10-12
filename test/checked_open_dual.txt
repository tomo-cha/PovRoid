/*
esp32のdual coreを用いてsimpleFOCを動かす
motor.loopFOC():を1kHz以上で可能な限り高頻度で呼び出す必要があるため、モータ駆動に1つのcoreを専有させたい
WDTが無効化されているcore1をモータに当てる
core0でデータのudp送受信やLEDの制御をする
commanderを使わずにmotor.targetを変更
*/

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
simpleFOC
*/
// TODO: motor instance
BLDCMotor motor = BLDCMotor(11, 5.57 / 2.0);

// TODO: driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);

// TODO: sensor instance
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5); // cs=ss=IO5

Commander command = Commander(Serial);
void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }
const float buffer = 0.1;

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
unsigned long rotTime, timeOld;
float real_vel = 0.00;
bool isZeroPositionPassed = true;

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
Serial
*/
bool dataReceived = false;

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
    motor.controller = MotionControlType::velocity_openloop;

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
    getZeroPosition();
    // set the initial motor target
    // motor.target = zero_position; // rad/s
    motor.target = 10;       // rad/s
    motor.voltage_limit = 4; // V

    // add target command M
    command.add('M', doMotor, "Motor");
    command.add('L', doLimit, "voltage limit");

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
        // motor.loopFOC();

        motor.move();

        command.run();
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
        // String str;
        // while (Serial.available())
        // {
        //     char c = Serial.read();
        //     if (c != '\n')
        //     {
        //         str += c;
        //     }
        //     dataReceived = true;
        // }
        // if (dataReceived)
        // {
        //     if (str == "v")
        //     {
        //         motor.controller = MotionControlType::angle;
        //     }
        //     float input = str.toFloat();

        //     motor.target = zero_position + input;
        //     dataReceived = false;
        // }
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
        sensor.update();
        float currentSensorValue = -1.0 * sensor.getAngle();
        float normalizedSensorValue = fmod(currentSensorValue, 2.0 * PI);
        if (abs(zero_position - normalizedSensorValue) <= buffer)
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
        // if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
        // {
        if (micros() - timeOld > rotTime / Div * (numDiv + 1))
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
        // }
        // strip.clear(); // 一つ前の点灯パターンを消さないとそのまま残る。これがないと画像が回転しているように見える
        // for (int i = 0; i < NUMPIXELS; i++)
        // {
        //     strip.setPixelColor(i, 0xFF0000);
        // }
        // setした通りにLEDを光らせる
        // Serial.println("before show");
        // strip.show();
        // Serial.println("after show"); // 160ms
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