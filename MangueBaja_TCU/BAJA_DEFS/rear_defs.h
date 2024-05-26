#ifndef REAR_DEFS_H
#define REAR_DEFS_H

#include "mbed.h"
#include "defs.h"
#include "FIR.h"

#define CAN_BPS_1000K       1000000

/* Moving Average Definitions */
#define ADCVoltageLimit     3.3
#define R_TERM              1000
#define sample              25 
//#define DENSITY             1.3565

/* Wheel Definitions */
#define PI                        3.1416
#define WHEEL_DIAMETER            0.5842      // m
//#define WHEEL_HOLES_NUMBER_MB1  24
#define WHEEL_HOLES_NUMBER_REAR   12
#define WHEEL_HOLES_NUMBER_FRONT  24

/* Servo definitions */
#define MID_MODE            0x00
#define RUN_MODE            0x01
#define CHOKE_MODE          0x02
#define SERVO_MID           1180
#define SERVO_RUN           800
#define SERVO_CHOKE         1550

typedef enum
{
    IDLE_ST,            // Wait
    TEMP_MOTOR_ST,      // Measure temperature of motor
    TEMP_CVT_ST,        // Measure temperature of CVT
    FUEL_ST,            // Proccess fuel data sampling
    SPEED_ST,           // Calculate speed
    VOLTAGE_ST,         // Calculate State of Charge and battery voltage
    SYSTEM_CURRENT_ST,  // Measure the current of the system
    THROTTLE_ST,        // Write throttle position (PWM)
    DEBUG_ST            // Send data for debug

} state_t;

#endif
