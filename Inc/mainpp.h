/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/************************************* INCLUDE FILES ******************************************/
#include "main.h"

#include <stdio.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>


/*************************************** CONSTANTS ********************************************/
//#define DEBUG_ENABLE
#define BUFFER_SIZE 128


/************************************ TYPE DEFINITIONS ****************************************/
typedef struct {
    bool  drive_pid_en;
    int   drive_setpoint;
    bool  servo_pid_en;
    int   servo_setpoint;
    //
    int   heading;
    float speed;
} ugv_t;

/*************************************** EXTERNALS ********************************************/
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim14;

/*************************************** FUNCTIONS ********************************************/
void setup(void);
void loop(void);

#endif /* MAINPP_H_ */
