/*
 * BMP390.c
 *
 *  Created on: Oct 20, 2025
 *      Author: bugrahan-turkmen
 *
 *      BMP390_Yapilandir(I2C_HandleTypeDef *hi2c)   fonksiyonunu init olarak calistir
 *      irtifa ve sicaklik verisini float olarak fonksiyondan cekebilirsiniz
 *
 */

#include "BMP390.h"


#define BMP390_I2C_ADDRESS  (0x77 << 1)

BMP390_TempCalib Temp;
BMP390_PresCalibData Pres;

uint8_t BMP390_chip_address_al(I2C_HandleTypeDef *hi2c){
	  uint8_t chip_id_reg = 0x00;
	  uint8_t chip_id;
	  HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, &chip_id_reg, 1, 100);
	  HAL_I2C_Master_Receive(hi2c, BMP390_I2C_ADDRESS, &chip_id, 1, 100);
	  return chip_id;
}


//  Sensörün default adresini alir yani bir nevi haberleşme var mi kontrolü. 0x77 döndürmeli

uint8_t BMP390_default_address_al(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef res;
    uint8_t i;
    uint8_t address;
    for (i = 1; i < 127; i++) {
        res = HAL_I2C_IsDeviceReady(hi2c, i << 1, 1, HAL_MAX_DELAY);
        if (res == HAL_OK) {
            address=i;
        }
    }
    return address;
}

static void BMP390_sicaklik_okuma_kalibrasyon(I2C_HandleTypeDef *hi2c) {
    uint8_t calib_data[5];
    uint8_t reg_temp = 0x31;

    HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, &reg_temp, 1, 100);
    HAL_I2C_Master_Receive(hi2c, BMP390_I2C_ADDRESS, calib_data, 5, 100);

    Temp.Calib_T1 = (uint16_t)(calib_data[0] | (calib_data[1] << 8));
    Temp.Calib_T2 = (uint16_t)(calib_data[2] | (calib_data[3] << 8));
    Temp.Calib_T3 = (int8_t)calib_data[4];

    Temp.PAR_Temp[0] = (float)(Temp.Calib_T1)*256;
    Temp.PAR_Temp[1] = (float)(Temp.Calib_T2)/ (exp2(30));
    Temp.PAR_Temp[2] = (float)(Temp.Calib_T3)/ (exp2(48));
}

static void BMP390_basinc_okuma_kalibrasyon(I2C_HandleTypeDef *hi2c){
	uint8_t temp_data[16];
	uint8_t reg_pres = 0x36;

    HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, &reg_pres, 1, 100);
    HAL_I2C_Master_Receive(hi2c, BMP390_I2C_ADDRESS, temp_data, 16, 100);

    Pres.Calib_P1 = (int16_t)(temp_data[1] << 8) | temp_data[0];
    Pres.Calib_P2 = (int16_t)(temp_data[3] << 8) | temp_data[2];
    Pres.Calib_P3 = (int8_t)temp_data[4];
    Pres.Calib_P4 = (int8_t)temp_data[5];
    Pres.Calib_P5 = (uint16_t)(temp_data[7] << 8) | temp_data[6];
    Pres.Calib_P6 = (uint16_t)(temp_data[9] << 8) | temp_data[8];
    Pres.Calib_P7 = (int8_t)temp_data[10];
    Pres.Calib_P8 = (int8_t)temp_data[11];
    Pres.Calib_P9 = (int16_t)(temp_data[13] << 8) | temp_data[12];
    Pres.Calib_P10 = (int8_t)temp_data[14];
    Pres.Calib_P11 = (int8_t)temp_data[15];

    Pres.PAR_Pres[0] = ((float)(Pres.Calib_P1) - exp2(14))/ exp2(20);
    Pres.PAR_Pres[1] = ((float)(Pres.Calib_P2) - exp2(14))/ exp2(29);
    Pres.PAR_Pres[2] = (float)(Pres.Calib_P3) / exp2(32);
    Pres.PAR_Pres[3] = (float)(Pres.Calib_P4) / exp2(37);
    Pres.PAR_Pres[4] = (float)(Pres.Calib_P5) * 8;
    Pres.PAR_Pres[5] = (float)(Pres.Calib_P6) / 64;
    Pres.PAR_Pres[6] = (float)(Pres.Calib_P7) / 256;
    Pres.PAR_Pres[7] = (float)(Pres.Calib_P8) / exp2(15);
    Pres.PAR_Pres[8] = (float)(Pres.Calib_P9) / exp2(48);
    Pres.PAR_Pres[9] = (float)(Pres.Calib_P10) / exp2(48);
    Pres.PAR_Pres[10] = (float)(Pres.Calib_P11) / exp2(65);

}
void BMP390_Yapilandir(I2C_HandleTypeDef *hi2c){
	uint8_t configData[2];

	configData[0] = 0x1F;
	configData[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, configData, 2, 100);
	configData[0] = 0x1B;
	configData[1] = 0x33;
	HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, configData, 2, 100);

	BMP390_sicaklik_okuma_kalibrasyon(hi2c);
	BMP390_basinc_okuma_kalibrasyon(hi2c);

}

static uint32_t BMP390_Raw_Temp_Oku(I2C_HandleTypeDef *hi2c){
	uint8_t Raw_Temp[3];
	uint8_t reg_Temp = 0x07;

    HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, &reg_Temp, 1, 100);
    HAL_I2C_Master_Receive(hi2c, BMP390_I2C_ADDRESS, Raw_Temp, 3, 100);

    return ((Raw_Temp[2] << 16) | (Raw_Temp[1] << 8) | (Raw_Temp[0]));
}


float BMP390_Sicaklik_oku(I2C_HandleTypeDef *hi2c){
	uint32_t uncomp_temp;
	float partial_data1;
	float partial_data2;

	uncomp_temp = BMP390_Raw_Temp_Oku(hi2c);
	partial_data1 = ((float)(uncomp_temp)) - Temp.PAR_Temp[0];
	partial_data2 = partial_data1 * Temp.PAR_Temp[1];
	float temp_c = partial_data2 + (partial_data1 * partial_data1)*Temp.PAR_Temp[2];
	return temp_c;
}

static uint32_t BMP390_Raw_Pres_Oku(I2C_HandleTypeDef *hi2c){
	uint8_t Raw_Pres[3];
	uint8_t reg_Pres = 0x04;

    HAL_I2C_Master_Transmit(hi2c, BMP390_I2C_ADDRESS, &reg_Pres, 1, 100);
    HAL_I2C_Master_Receive(hi2c, BMP390_I2C_ADDRESS, Raw_Pres, 3, 100);

    return ((Raw_Pres[2] << 16) | (Raw_Pres[1] << 8) | (Raw_Pres[0]));
}

static float BMP390_Basinc_Oku(I2C_HandleTypeDef *hi2c){
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	float temperature = BMP390_Sicaklik_oku(hi2c);
	uint32_t uncomp_press = BMP390_Raw_Pres_Oku(hi2c);

	partial_data1 = Pres.PAR_Pres[5] * temperature;
	partial_data2 = Pres.PAR_Pres[6] * (temperature * temperature);
	partial_data3 = Pres.PAR_Pres[7] * (temperature * temperature * temperature);
	partial_out1 = Pres.PAR_Pres[4] + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = Pres.PAR_Pres[1] * temperature;
	partial_data2 = Pres.PAR_Pres[2]* (temperature * temperature);
	partial_data3 = Pres.PAR_Pres[3] * (temperature * temperature * temperature);
	partial_out2 = (float)uncomp_press * (Pres.PAR_Pres[0] + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = Pres.PAR_Pres[8] + Pres.PAR_Pres[9] * temperature;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press)*Pres.PAR_Pres[10];
	return (partial_out1 + partial_out2 + partial_data4);
}

float BMP390_Irtifa_Hesapla(I2C_HandleTypeDef *hi2c){
	float pressure_pa = BMP390_Basinc_Oku(hi2c);

    const float T0 = 288.15f;
    const float lapse_rate = -0.0065f;
    const float R = 8.314f;
    const float g = 9.80665f;
    const float M = 0.0289644f;
    const float p0_pa = 101325.0f;

    // Hata kontrolü: Geçersiz basınç
    if (pressure_pa <= 0.0f) {
        return 0.0f;
    }

    // Barometrik formül: h = (T0 / lapse_rate) * (1 - (P/P0)^((R * lapse_rate) / (g * M)))
    float exponent = (R * lapse_rate) / (g * M);
    float pressure_ratio = pressure_pa / p0_pa;
    float altitude = (T0 / lapse_rate) * (1.0f - powf(pressure_ratio, exponent));

    return altitude;
}






