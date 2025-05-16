#include "sepic_hardware_def.h"
#include "sepic.h"
#include "stm32ADC.h"
#include "HardwareTimer.h"
#include "stm32f405xx.h"
#include "SimpleFOC.h"

TIM_TypeDef *pwmTimer;
HardwareTimer *sepicPWM;
uint32_t channel;
uint16_t tick = 0;
LowPassFilter filterVout = LowPassFilter(.1);      // Tf = 1ms
LowPassFilter filterBoartTemp = LowPassFilter(.1); // Tf = 1ms
// int duty[7] = {0, 10, 20, 30, 40, 50, 60};
SepicData sepicData;
SepicSafety sepicSafety;
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
// double setpoint = 75.00;
Commander command = Commander(Serial);

LowPassFilter LPFv{.5};
void doFilter(char *cmd) { command.lpf(&LPFv, cmd); }

void initCmd()
{
    command.add('s', SetPoint, "set point setting");
    command.add('f', frequencySetting, "frequency setting");
    command.add('i', info, "information of sepic");
    command.add('d', duty, "duty cycle setting");
    command.add('F', doFilter, "tune LPF");
}

void duty(char *str)
{
    char cmd = str[0];
    bool GET = command.isSentinel(str[1]);
    float value = atoi(&str[1]);
    Serial.print(F("duty: "));
    Serial.println(value);
    for (int i = 0; i < 50; i++)
    {
        setDuty(i);
        delay(100);
    }
    setDuty(0);
}
void SetPoint(char *str)
{
    char cmd = str[0];
    bool GET = command.isSentinel(str[1]);
    float value = atoi(&str[1]);
    Serial.print(F("duty: "));
    Serial.println(value);
    sepicData.setPoint = value;
}
void info(char *str)
{
    printInfo();
}
void frequencySetting(char *str)
{
    char cmd = str[0];
    bool GET = command.isSentinel(str[1]);
    float value = atoi(&str[1]);
    Serial.print(F("freq: "));
    Serial.println(value);
    if ((value >= 50000) && (value <= 200000))
    {
        setSpeed(value);
    }
    else
    {
        Serial.println(F("Frequency out of range!, valid range 50000-200000"));
    }
}
void setSpeed(int speed)
{

    Serial.print(F("setSpeed = "));
    Serial.println(speed);

    // Create new instance for new speed
    if (sepicPWM)
        delete sepicPWM;

    sepicPWM = new HardwareTimer(pwmTimer);

    if (speed == 0)
    {
        // Use DC = 0 to stop sepicPWM
        sepicPWM->setPWM(channel, PWM_PIN, 500, 0, nullptr);
    }
    else
    {
        //  Set the frequency of the PWM output and a duty cycle of 50%
        // digitalWrite(DIR_PIN, (speed < 0));
        sepicPWM->setPWM(channel, PWM_PIN, abs(speed), 25, nullptr);
    }
}
void setDuty(uint32_t dutycycle)
{
    sepicPWM->setCaptureCompare(channel, dutycycle, PERCENT_COMPARE_FORMAT);
}
void initPWM(uint32_t pwmPin)
{
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);

    PinName pinNameToUse = digitalPinToPinName(pwmPin);
    pwmTimer = (TIM_TypeDef *)pinmap_peripheral(pinNameToUse, PinMap_PWM);

    if (pwmTimer != nullptr)
    {
        uint8_t timerIndex = get_timer_index(pwmTimer);

        // pin => 0, 1, etc
        channel = STM_PIN_CHANNEL(pinmap_function(pinNameToUse, PinMap_PWM));
        Serial.print("pwmTimer = 0x");
        Serial.print((uint32_t)pwmTimer, HEX);
        Serial.print(", channel = ");
        Serial.print(channel);
        Serial.print(", TimerIndex = ");
        Serial.print(get_timer_index(pwmTimer));
        Serial.print(", PinName = ");
        Serial.println(pinNameToUse);

        sepicPWM = new HardwareTimer(pwmTimer);
        sepicPWM->setPWM(channel, pwmPin, 0, 0, nullptr);
    }
    else
    {
        Serial.println("ERROR => Wrong pin, You have to select another one. Skip NULL pwmTimer");
    }
}
double pid(double error)
{
    double proportional = error;
    integral += error * dt;
    double derivative = (error - previous) / dt;
    previous = error;
    double output = (kp * proportional) + (ki * integral) + (kd * derivative);
    return output;
}
void vPrintTask(void *pvParameters)
{
    UNUSED(pvParameters);
    while (1)
    {
        // Sleep for one second.
        vTaskDelay(configTICK_RATE_HZ);
        Serial.print(F("Count: "));
        Serial.println(tick++);
    }
}
/*

create task for protection feature
1. over voltage protection (OVP)
2. under voltage protection (UVP)
3. over current protection (OCP)
4. over temperature protection (OTP)
*/

void vMainTask(void *pvParameters)
{
    UNUSED(pvParameters);
    initPWM(PWM_PIN);
    // setSpeed(170000);
    setSpeed(100000);
    initADC();
    initCmd();

    while (1)
    {

        int32_t VRef = readVref();
        // Serial.print(F(">VIN:"));
        // Serial.print(sepicData.voltageInput);
        // Serial.print(F(",BoardTemperature:"));
        // Serial.print(sepicData.boardTemperature);
        // Serial.print(F("\r\n"));

        // checking for temperature protection
        int32_t TemperatureOut = readVoltage(VRef, TEMP_PIN);
        sepicData.boardTemperature = map(TemperatureOut, 0, VRef, 0, 80);
        // sepicData.boardTemperature = filterBoartTemp(sepicData.boardTemperature);
        if (sepicData.boardTemperature > TEMP_MAX)
        {
            sepicSafety.isBoardOverTemperature = true;
        }
        else if (sepicData.boardTemperature < TEMP_MIN)
        {
            sepicSafety.isBoardOverTemperature = false;
        }

        // checking for voltage input protection
        int32_t temp = readVoltage(VRef, VIN_PIN);
        sepicData.voltageInput = map(temp, 0, VRef, 0, 95.7);
        if (sepicData.voltageInput > VIN_MAX)
        {
            sepicSafety.isVinOverVoltage = true;
        }
        else if (sepicData.voltageInput < VIN_MAX)
        {
            sepicSafety.isVinOverVoltage = false;
        }
        if (sepicData.voltageInput < VIN_MIN)
        {
            sepicSafety.isVinUnderVoltage = true;
        }
        else if (sepicData.voltageInput > VIN_MIN)
        {
            sepicSafety.isVinUnderVoltage = false;
        }

        // checking for current input protection
        int32_t currentInput = readVoltage(VRef, IIN_PIN);
        // sepicData.currentInput = map(currentInput, 0, VRef, 0, 5);
        sepicData.currentInput = currentInput;
        if (sepicData.currentInput > IIN_MAX)
        {
            sepicSafety.isOverCurrent = true;
        }
        else if (sepicData.currentInput < IIN_MAX)
        {
            sepicSafety.isOverCurrent = false;
        }

        // checking for voltage output protection
        // ======== reading voltage output ========
        int32_t Vout = readVoltage(VRef, VOUT_PIN);
        sepicData.voltageOutput = map(Vout, 0, VRef, 0, 100);
        // sepicData.voltageOutput = filterVout(sepicData.voltageOutput);
        if (sepicData.voltageOutput > VOUT_MAX)
        {
            sepicSafety.isOutputOverVoltage = true;
        }
        else if (sepicData.voltageOutput < VOUT_MAX)
        {
            sepicSafety.isOutputOverVoltage = false;
        }

        if ((sepicSafety.isBoardOverTemperature) || (sepicSafety.isVinOverVoltage) || (sepicSafety.isOutputOverVoltage) || (sepicSafety.isVinUnderVoltage))
        {
            setDuty(0);
            Serial.print(F("Protection Triggered by :"));
            if (sepicSafety.isBoardOverTemperature)
            {
                Serial.print(F("Board Over Temperature: "));
                Serial.print(sepicData.boardTemperature);
            }
            if (sepicSafety.isVinOverVoltage)
            {
                Serial.print(F("Vin Over Voltage: "));
                Serial.print(sepicData.voltageInput);
            }
            if (sepicSafety.isVinUnderVoltage)
            {
                Serial.print(F("Vin under Voltage: "));
                Serial.print(sepicData.voltageInput);
            }
            // if (sepicSafety.isOverCurrent)
            // {
            //     Serial.print(F("Over Current,"));
            // }
            if (sepicSafety.isOutputOverVoltage)
            {
                Serial.print(F("Output Over Voltage: "));
                Serial.print(sepicData.voltageOutput);
            }
            Serial.println();
        }
        command.run();

        vTaskDelay(50);
    }
}
void vPlottingTask(void *pvParameters)
{
    UNUSED(pvParameters);
    initADC();
    // uint16_t tic = 0;
    while (1)
    {
        int32_t VRef = readVref();

        // Serial.print(F(">VSumber:"));
        // Serial.print(sepicData.voltageInput);
        // Serial.print(F(",VOUT_Terbaca:"));
        // Serial.print(sepicData.voltageOutput);
        // Serial.print(F(",BoardTemperature:"));
        // Serial.print(sepicData.boardTemperature);
        Serial.print(F(">IIN_Terbaca:"));
        Serial.print(sepicData.currentInput);
        Serial.print(F(",SetDutyCycle:"));
        Serial.print(sepicData.dutyCycle);
        Serial.print(F("\r\n"));
        vTaskDelay(100);
    }
}

void vStairSetpointTask(void *pvParameters)
{
    UNUSED(pvParameters);
    sepicData.setPoint = 20;
    while (1)
    {
        sepicData.setPoint += 5;
        if (sepicData.setPoint > 50) // Reset if setpoint exceeds 100
        {
            sepicData.setPoint = 20;
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
}
void vPIDTask(void *pvParameters)
{
    UNUSED(pvParameters);
    kp = 3;
    ki = 4;
    kd = 0.01;
    last_time = 0;
    while (1)
    {
        double now = millis();
        dt = (now - last_time) / 1000.00;
        last_time = now;

        double actual = LPFv(sepicData.voltageOutput);
        double error = sepicData.setPoint - actual;
        output = pid(error);
        setDuty(output);

        Serial.print(F(">SetPoint:"));
        Serial.print(sepicData.setPoint);
        Serial.print(",Actual:");
        Serial.print(actual);
        Serial.print(F("\r\n"));

        delay(10);
    }
}

void printInfo()
{
    Serial.print(F("Vout:"));
    Serial.println(sepicData.voltageOutput);
    Serial.print(F("Vin:"));
    Serial.println(sepicData.voltageInput);
    Serial.print(F("Iin:"));
    Serial.println(sepicData.currentInput);
    Serial.print(F("BoardTemperature:"));
    Serial.println(sepicData.boardTemperature);
}
