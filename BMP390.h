/*
 * BMP390.h
 *
 *  Created on: Oct 20, 2025
 *      Author: bugra
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"
#include <math.h>


uint8_t BMP380_chip_address_al();
uint8_t BMP380_default_address_al();

typedef struct{
	uint16_t Calib_T1;
	uint16_t Calib_T2;
	int8_t Calib_T3;
	float PAR_Temp[3];
}BMP390_TempCalib;

typedef struct{
	int16_t Calib_P1;
	int16_t Calib_P2;
	int8_t Calib_P3;
	int8_t Calib_P4;
	uint16_t Calib_P5;
	uint16_t Calib_P6;
	int8_t Calib_P7;
	int8_t Calib_P8;
	int8_t Calib_P9;
	int8_t Calib_P10;
	int8_t Calib_P11;
	float PAR_Pres[11];
}BMP390_PresCalibData;


uint8_t BMP390_default_address_al(I2C_HandleTypeDef *hi2c);
uint8_t BMP390_chip_address_al(I2C_HandleTypeDef *hi2c);

void BMP390_Yapilandir(I2C_HandleTypeDef *hi2c);
float BMP390_Sicaklik_oku(I2C_HandleTypeDef *hi2c);
float BMP390_Irtifa_Hesapla(I2C_HandleTypeDef *hi2c);
#endif /* INC_BMP390_H_ */
