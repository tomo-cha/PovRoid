//LEDをラズパイに任せる場合のコード

#include <SimpleFOC.h>
// #include <Adafruit_DotStar.h>
// #include <SPI.h>

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
const float buffer = 0.00;
unsigned long rotTime, timeOld;
unsigned long old_time = 0;
float real_vel = 0.00;
bool isZeroPositionPassed = true;

/*
preference
*/
float zero_position = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup()
{

    // use monitoring with serial
    Serial.begin(115200);
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
    motor.target = 30; // rad/s
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
    if (abs(zero_position - normalizedSensorValue) <= buffer) // zeropositionを通過したかどうか
    {
        if (isZeroPositionPassed)
        {
            unsigned long timeNow = micros();
            rotTime = timeNow - timeOld;
            if (millis() - old_time > 1000)
            {
                // Serial.print("timeNow:");
                // Serial.print(timeNow);
                Serial.print(",rotTime:");
                Serial.print(rotTime);
                Serial.print(",timeOld:");
                Serial.println(timeOld);
                old_time = millis();
            }
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