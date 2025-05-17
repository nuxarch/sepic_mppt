#ifndef SEPIC_HARDWARE_DEF_H
#define SEPIC_HARDWARE_DEF_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "STM32_PWM.h"
#include "pins_arduino.h"

#define PWM_PIN PB13

// definisi pin sensor
#define VIN_PIN PC0
#define VOUT_PIN PB0
#define IIN_PIN PA2
#define TEMP_PIN PA3

// define parameter value for safety
#define TEMP_MAX 60
#define TEMP_MIN 50
#define VIN_MAX 25
#define VIN_MIN 8
#define IIN_MAX 5
#define VOUT_MAX 100
#define VOUT_MIN 0
#endif // SEPIC_HARDWARE_DEF_H