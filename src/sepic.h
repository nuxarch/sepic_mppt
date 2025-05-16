#ifndef SEPIC_H
#define SEPIC_H

#include "sepic_hardware_def.h"
#include "SimpleFOC.h"
// uint16_t tick = 0;

void initCmd();
void duty(char *str);
void SetPoint(char *str);
void frequencySetting(char *str);
void info(char *str);
void setSpeed(int speed);
void setDuty(uint32_t dutycycle);
void initPWM(uint32_t pwmPin);
void vPrintTask(void *pvParameters);
void vPlottingTask(void *pvParameters);
void vMainTask(void *pvParameters);
void vPIDTask(void *pvParameters);
void vStairSetpointTask(void *pvParameters);
void printInfo();
typedef struct
{
    float voltageInput;
    float currentInput;
    float voltageOutput;
    float boardTemperature;
    float MCUTemperature;
    float setPoint;
    int32_t dutyCycle;
} SepicData;
typedef struct
{
    bool isBoardOverTemperature = true;
    bool isVinOverVoltage = true;
    bool isOutputOverVoltage = true;
    bool isOverCurrent = true;
    bool isVinUnderVoltage = true;
    bool isOutputUnderVoltage = true;
} SepicSafety;

#endif // THREAD_H