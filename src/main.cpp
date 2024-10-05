/*
esp32のdual coreを用いてsimpleFOCを動かす
motor.loopFOC():を1kHz以上で可能な限り高頻度で呼び出す必要があるため、モータ駆動に1つのcoreを専有させたい
WDTが無効化されているcore1をモータに当てること
そのため、もう一つのcoreでデータの送受信やLEDの制御をする
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

// instantiate the commander
// Commander command = Commander(Serial);
// void doMotor(char *cmd) { command.motor(&motor, cmd); }
// void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

/*
preference
*/
Preferences preferences;
float zero_position;
int count = 0;
int past_time = 0;
bool v = false;

/*
timer
*/
// タイマー
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int timer_core;

void ARDUINO_ISR_ATTR onTimer()
{
    // Increment the counter and set the time of ISR
    portENTER_CRITICAL_ISR(&timerMux);
    // It is safe to use digitalRead/Write here if you want to toggle an output
    timer_core = xPortGetCoreID();
    if (v)
    {
        motor.target = 3.14 * count;
        count++;
        if(motor.target > 100){
            motor.target = 100;
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

/*
dual core
*/
TaskHandle_t taskHandle[2];

void motorControlTask(void *pvParameters)
{
    // タイマー
    timerSemaphore = xSemaphoreCreateBinary();
    // Set timer frequency to 1Mhz
    timer = timerBegin(1000000);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
    timerAlarm(timer, 1000000, true, 0);

    Serial.print("motorControlTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
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
    motor.controller = MotionControlType::angle;

    // use monitoring with serial SimpleFOCDebug::enable(&Serial);の役割を含んでいる
    // comment out if not needed
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

    preferences.begin("setPosition", false);
    zero_position = preferences.getFloat("zeroPosition", 0);
    Serial.print("Current zeroPosition value: ");
    Serial.println(zero_position);
    preferences.end();

    // set the initial motor target
    motor.target = zero_position; // rad/s
    motor.voltage_limit = 1;      // V

    // add target command M
    // command.add('M', doMotor, "Motor");
    // command.add('L', doLimit, "voltage limit");

    Serial.println(F("Motor ready."));
    // Serial.println(F("Set the target using serial terminal and command M:"));
    _delay(1000);

    while (1)
    {
        // main FOC algorithm function
        motor.loopFOC();
        // 10000ms後にvelocityモード
        int current_time = millis();
        if (current_time - past_time > 10000)
        {
            v = true;
            past_time = current_time;
        }
        if (v)
        {
            motor.controller = MotionControlType::velocity;
        }
        motor.monitor();

        // Motion control function
        motor.move();

        // user communication
        // command.run();
        // delay(1); // 1以上にするとウォッチドッグのリセットがなくなる
    }
}

void testTask(void *pvParameters)
{
    Serial.print("testTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
    while (1)
    {
        // If Timer has fired
        if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
        {
            Serial.print("onTimer exec core: ");
            Serial.println(timer_core); // 動作確認用出力
        }
        if (Serial.available())
        {
            String str = Serial.readString();
            Serial.print(str);
            float str_float = str.toFloat();

            motor.target = str_float;
        }
        vPortYield(); // vPortYield()ではウォッチドッグに影響しない
        yield();      // yield()ではウォッチドッグに影響しない
        delay(1);     // 1以上にするとウォッチドッグのリセットがなくなる
                      // delayMicroseconds(1000000);     // delayMicroseconds()はウォッチドッグとは関係ない単なるウエイト
    }
}

void setup()
{
    Serial.begin(115200);

    // Core0でタスク起動
    xTaskCreatePinnedToCore(
        motorControlTask, // タスク関数へのポインタ。無限ループで終了しないよう関数を指定します
        "testTask1",      // タスクの説明用名前。重複しても動きますがデバッグ用途。最大16文字まで
        8192,             // スタックサイズ(Byte)
        NULL,             // 作成タスクのパラメータのポインタ
        2,                // 作成タスクの優先順位(0:低 - 25:高)
        &taskHandle[0],   // 作成タスクのHandleへのポインタ
        1                 // 利用するCPUコア(0-1)
    );

    // Core1でタスク起動
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