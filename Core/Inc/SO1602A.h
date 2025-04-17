/*
 * SO1602A.h
 *
 *  Created on: 2024/02/14
 *      Author:aiueo
 */

#ifndef INC_SO1602A_H_
#define INC_SO1602A_H_

#include "stm32f7xx_hal.h"

I2C_HandleTypeDef *oled_I2C;
#define SO1602_ADDR 0x78
#define CMD 0x00
#define DATA 0x40

int writeData(uint8_t data);
void writeCommand(uint8_t command);
void OLED_Init();
void oled_Init(I2C_HandleTypeDef *handler);
void set_cursor(uint8_t tate , uint8_t yoko);
void Puts(const char *p);

#endif /* INC_SO1602A_H_ */
