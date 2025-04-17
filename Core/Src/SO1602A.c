/*
 * SO1602A.c
 *
 *  Created on: 2024/02/14
 *      Author: aiueo
 */

#include "SO1602A.h"

void oled_Init(I2C_HandleTypeDef *handler){
	oled_I2C = handler;
}

int writeData(uint8_t data){
 	uint8_t buf[] = { DATA , data };
  	int status = HAL_I2C_Master_Transmit(oled_I2C , SO1602_ADDR , buf , 2 , 100);
  	HAL_Delay(1);
  	return status == HAL_OK;
}

void writeCommand(uint8_t command){
	uint8_t buf[] = { CMD , command };
  	HAL_I2C_Master_Transmit(oled_I2C , SO1602_ADDR , buf , 2 , 100);
  	HAL_Delay(1);
}

void OLED_Init(){
  	HAL_Delay(100);
  	writeCommand(0x01); //clear Display
  	writeCommand(0x02); //Return Home
  	writeCommand(0x0c); //Send Display on command
  	writeCommand(0x01); //Clear Display
}

void set_cursor(uint8_t tate , uint8_t yoko){
	if(tate == 1){
		tate = 0x00;
	}else{
		tate = 0x20;
	}
	writeCommand(tate + yoko + 0x80);
}

void Puts(const char *p){
	for( ; *p ; p++){
		writeData(*p);
	}
}


