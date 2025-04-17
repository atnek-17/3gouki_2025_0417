/*
 * servo.c
 *
 *  Created on: 2025/03/16
 *      Author: fukuj
 */
#include "servo.h"

long map(float x , float in_min , float in_max , float out_min , float out_max)
{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_1(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 180 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pulse_1);
}
void servo_2(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 180 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pulse_1);
}
void servo_3(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pulse_1);
}
void servo_4(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pulse_1);
}
void servo_5(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 180 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pulse_1);
}

void servo_6(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,pulse_1);
}
void servo_7(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 180 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,pulse_1);
}

void servo_8(uint16_t pulse){
	int pulse_1 = map(pulse , 0 , 180 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,pulse_1);
}

void servo_init(){
	servo_1(servo1_init);
	servo_2(servo2_init);
	servo_3(servo3_init);
	servo_4(servo4_init);
	servo_5(servo5_init);
	servo_6(servo6_init);
	servo_7(servo7_init);
	servo_8(servo8_init);
}
void servo_move(uint8_t command , uint16_t degree){
	switch(command){
	case 1:
		servo_1(degree);

	  break;

	case 2:
		servo_2(degree);

	  break;

	case 3:
		servo_3(degree);

	  break;

	case 4:				//shift to left
		servo_4(degree);

		break;
	case 5:				//turn right
		servo_5(degree);

		break;

	case 6:				//turn left
		servo_6(degree);

		break;

	case 7:
		servo_7(degree);

		break;

	case 8:
		servo_8(degree);

		break;

	}
}
