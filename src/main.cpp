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

/*
タイマー
*/
int past_time = 0;

/*
preference
*/
Preferences preferences;
float zero_position;

/*
Serial
*/
bool dataReceived = false;

/*
dual core
*/
TaskHandle_t taskHandle[2];

void motorControlTask(void *pvParameters)
{
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

    preferences.begin("setPosition", false);
    zero_position = preferences.getFloat("zeroPosition", 0);
    Serial.print("Current zeroPosition value: ");
    Serial.println(zero_position);
    preferences.end();

    // set the initial motor target
    motor.target = zero_position; // rad/s
    motor.voltage_limit = 1;      // V

    Serial.println(F("Motor ready."));
    _delay(1000);

    while (1)
    {
        // main FOC algorithm function
        motor.loopFOC();
        if (kMotorDebug)
        {
            motor.monitor();
        }
        motor.move();

        /*
        時間による演出
        10秒後にvelocity modeに変更。初期値として30rad/sを設定
        */
        int current_time = millis();
        if (current_time - past_time > 10000)
        {
            motor.controller = MotionControlType::velocity;
            motor.target = 30;
            past_time = current_time;
        }

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
            float input = str.toFloat();
            motor.target = zero_position + input;
            dataReceived = false;
        }
    }
}

void testTask(void *pvParameters)
{
    Serial.print("testTask exec core: ");
    Serial.println(xPortGetCoreID()); // 動作確認用出力
    while (1)
    {
        vPortYield(); // vPortYield()ではウォッチドッグに影響しない
        yield();      // yield()ではウォッチドッグに影響しない
        delay(1);     // 1以上にするとウォッチドッグのリセットがなくなる
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