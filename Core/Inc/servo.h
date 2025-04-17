/*
 * servo.h
 *
 *  Created on: 2025/03/16
 *      Author: fukuj
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f7xx_hal.h"

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

#define Servo_low 25			// ds3218 , FS5115M			0.5ms - 2.5ms , 50 [Hz] * 0.5
#define Servo_high 125			// 50 [Hz] * 2.5

#define servo1_init 90
#define servo2_init 90
#define servo3_init 90
#define servo4_init 90
#define servo5_init 90
#define servo6_init 0
#define servo7_init 0
#define servo8_init 0


long map(float x , float in_min , float in_max , float out_min , float out_max);

void servo_1(uint16_t pulse);
void servo_2(uint16_t pulse);
void servo_3(uint16_t pulse);
void servo_4(uint16_t pulse);
void servo_5(uint16_t pulse);
void servo_6(uint16_t pulse);
void servo_7(uint16_t pulse);
void servo_8(uint16_t pulse);

void servo_init();
void servo_move(uint8_t command , uint16_t degree);

#endif /* INC_SERVO_H_ */
