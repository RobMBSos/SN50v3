/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains all hardware driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    i2c_a.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    01-June-2017
  * @brief   contains all hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_SENSOR_H__
#define __I2C_SENSOR_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h> 
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  
 *
 * @note
 * @retval None
 */
	 
#include "bsp.h"

// MLX90614 Definitions
#define MLX90614_ADDR     0x5A        // Default I2C address
#define MLX90614_TA_REG   0x06        // Ambient temperature register
#define MLX90614_TOBJ1_REG 0x07       // Object temperature register

// ADS1115 definitions 
#define ADS1115_ADDR              0x48    // Default I2C address

// Registers
#define ADS1115_REG_POINTER_CONVERT   0x00
#define ADS1115_REG_POINTER_CONFIG    0x01

// Config register bit definitions
#define ADS1115_CONFIG_OS_MASK        0x8000
#define ADS1115_CONFIG_MUX_MASK       0x7000
#define ADS1115_CONFIG_PGA_MASK       0x0E00
#define ADS1115_CONFIG_MODE_MASK      0x0100

// Config values
#define ADS1115_CONFIG_MODE_SINGLE    0x0100  // Single-shot mode
#define ADS1115_CONFIG_PGA_6_144V     0x0000  // +/-6.144V range = Gain 2/3
#define ADS1115_CONFIG_MUX_DIFF_0_1   0x0000  // Differential P = AIN0, N = AIN1
#define ADS1115_CONFIG_DR_128SPS      0x0080  // 128 samples per second
#define ADS1115_CONFIG_OS_SINGLE      0x8000  // Write: Start a single conversion



#define ADS122C04_ADDR  0x40 

	 
typedef struct{
	
	float temp_sht;
	
	float hum_sht;
  /**more may be added*/
} sht31_t;

typedef struct{
	
	float temp_sht;
	
	float hum_sht;
  /**more may be added*/
} sht20_t;

uint8_t SHT31_CheckSum_CRC8(uint8_t* Result,uint8_t num);
void SHT31_Read(sht31_t *sht31_data);
uint8_t check_sht31_connect(void);
uint8_t SHT20_CheckSum_CRC8(uint8_t* Result);
float SHT20_RH(void);
float SHT20_RT(void);
uint8_t check_sht20_connect(void);
uint16_t bh1750_read(void);
void I2C_read_data(sensor_t *sensor_data,uint8_t flag_temp, uint8_t message);
void LidarLite_init(void);
uint16_t LidarLite(void);
uint16_t waitbusy(uint8_t mode);
// MLX90614 Function Prototypes
uint8_t check_mlx90614_connect(uint8_t devAddr);
float MLX90614_ReadTemp(uint8_t devAddr, uint8_t regAddr);
uint8_t check_ads1115_connect(void);
int16_t ads1115_read_differential(void);
//float ads1115_convert_to_mv(int16_t raw_adc);
float ads1115_convert_to_mm(int16_t raw_adc);
uint8_t check_ads122c04_connect(void); 
int32_t ads122c04_read_raw(void); 
float ads122c04_to_mm(int32_t raw_adc);
void ads122c04_read_displacement(int32_t* raw_out, float* voltage_mv_out, float* filtered_disp_out);


// External variable declaration
extern uint8_t mlx_flag;

#ifdef __cplusplus
}
#endif

#endif /* __I2C_A_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
