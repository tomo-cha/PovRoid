// 1周の間にupdateCycle回、LEDの頂点位置を変更
//ラズパイと同じ同じ数字にする

#include <SimpleFOC.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>

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
const int RX_PIN = 16;
const int TX_PIN = 17;
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
const float buffer = 0.01;
unsigned long rotTime, timeOld;
unsigned long old_time = 0;
unsigned long old_time1 = 0;
float real_vel = 0.00;
const int updateCycle = 4;
bool isZeroPositionPassed[updateCycle] = {true};
// bool isZeroPositionPassed0 = true;
// bool isZeroPositionPassed1 = true;
unsigned int count = 0;

/*
preference
*/
float zero_position = 0;

// instantiate the commander
Commander command = Commander(Serial2);
void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

/*
filter
*/
const int SAMPLE_COUNT = 10; // サンプル数
unsigned long rotTimeSamples[SAMPLE_COUNT];
int sampleIndex = 0;
bool isSampleBufferFull = false;
unsigned long prevSample = 360000;      // 前回のrotTime
const float THRESHOLD_MULTIPLIER = 1.5; // 閾値倍率
void addSample(unsigned long newSample, int flag)
{
    // 異常値チェック
    if (newSample > prevSample * THRESHOLD_MULTIPLIER)
    {
        // 異常値の場合は無視して次のサンプリングを待つ
        // Serial.println("Outlier detected, skipping this value.");
        // return;
        newSample = prevSample; // 異常値が検出されたら、一つ前の正常な値を足す
    }

    // サンプルを追加
    rotTimeSamples[sampleIndex] = newSample;
    sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;

    prevSample = newSample;

    if (sampleIndex == 0)
    {
        isSampleBufferFull = true;
    }

    // サンプルが揃った場合、中央値フィルタと移動平均フィルタを適用
    if (isSampleBufferFull)
    {
        // 1. 中央値フィルタ
        unsigned long sortedSamples[SAMPLE_COUNT];
        memcpy(sortedSamples, rotTimeSamples, sizeof(rotTimeSamples));
        std::sort(sortedSamples, sortedSamples + SAMPLE_COUNT);
        unsigned long medianRotTime = sortedSamples[SAMPLE_COUNT / 2];

        // 2. 移動平均フィルタ
        static float movingAverage = 0;
        movingAverage = (movingAverage * (SAMPLE_COUNT - 1) + medianRotTime) / SAMPLE_COUNT;
        // 平滑化した値を送信
        String sendData = String(flag) + "\t" + String(movingAverage); // 0 372911 や、 1 417914みたいなのを送る

        Serial2.print(sendData);
        // count++;
        // if (millis() - old_time > 1000)
        // {
        //     Serial.print("count: ");
        //     Serial.println(count);
        //     count = 0;
        //     old_time = millis();
        // }
    }
}

void setup()
{
    // use monitoring with serial
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    // enable more verbose output for debugging
    // comment out if not needed
    // SimpleFOCDebug::enable(&Serial);

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
    motor.controller = MotionControlType::velocity;

    // use monitoring with serial
    // comment out if not needed
    motor.useMonitoring(Serial);
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    motor.monitor_downsample = 1000; // default 10

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

    // set the initial motor target
    motor.target = 100; // rad/s
    motor.voltage_limit = 1;

    // add target command M
    command.add('M', doMotor, "Motor");
    command.add('L', doLimit, "voltage limit");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target using serial terminal and command M:"));
    _delay(1000);
}

void loop()
{
    // main FOC algorithm function
    motor.loopFOC();
    // motor.monitor();

    // Motion control function
    motor.move();

    // user communication
    command.run();

    float currentSensorValue = -1.0 * sensor.getAngle();
    float normalizedSensorValue = fmod(currentSensorValue, 2.0 * PI);

    for (int i = 0; i < updateCycle; i++) // nは通過点の数
    {
        float targetPosition = fmod(zero_position + i * 6.28 / updateCycle, 2.0 * PI); // i番目の通過点の位置
        if (abs(targetPosition - normalizedSensorValue) <= buffer)                     // 通過したかどうか
        {
            if (isZeroPositionPassed[i])
            {
                unsigned long timeNow = micros();
                rotTime = timeNow - timeOld;
                String data = String(i) + " " + String(rotTime); // フラグ値と回転時間を送信
                Serial.println(data);

                addSample(rotTime, i); // i 番目のサンプルを追加
                timeOld = timeNow;
                isZeroPositionPassed[i] = false;
            }
        }
        else
        {
            isZeroPositionPassed[i] = true;
        }
    }
}