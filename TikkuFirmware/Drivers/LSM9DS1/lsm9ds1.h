/**
  ******************************************************************************
  * @file    lsm303dlhc.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    20-November-2014
  * @brief   This file contains all the functions prototypes for the lsm303dlhc.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM9DS1_H
#define __LSM9DS1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
   
/** @addtogroup LSM9DS1
  * @{
  */
  
/** @defgroup LSM9DS1_Exported_Types
  * @{
  */
 typedef struct
 {
   uint8_t Full_Scale;                         	/* Full Scale selection */
   uint8_t Output_DataRate;                 	/* OUT data rate */
   uint8_t BandWidth_Mode;						/* Antialiasing filter bandwidth mode */
   uint8_t BandWidth;							/* Antialiasing filter bandwidth */
   uint8_t Axes_Enable;                        	/* Axes enable */
   uint8_t Output_Decimation;					/* Decimation of acceleration data on OUT REG and FIFO */
 }LSM9DS1_A_InitTypeDef;

 typedef struct
 {
   uint8_t Full_Scale;                         	/* Full Scale selection */
   uint8_t Output_DataRate;                 	/* OUT data rate */
   uint8_t Axes_Enable;                        	/* Axes enable */
 }LSM9DS1_G_InitTypeDef;

 typedef struct
 {
   uint8_t Full_Scale;                         	/* Full Scale selection */
   uint8_t Output_DataRate;                 	/* OUT data rate */
   uint8_t Operating_Mode;
   uint8_t I2C_Interface;
   uint8_t SPI_Interface_Read;
   uint8_t Low_Power_Mode;
   uint8_t Temperature_Compensation;
   uint8_t Self_Test;
   uint8_t XY_Performance;
   uint8_t Z_Performance;
 }LSM9DS1_M_InitTypeDef;

/**
  * @}
  */
 
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
/* Exported constant IO ------------------------------------------------------*/
#define LSM9DS1_AG_I2C_ADDRESS   			(0xD4)
#define LSM9DS1_M_I2C_ADDRESS				(0x38)

/* Accelerometer and gyroscope register definitions */
#define LSM9DS1_AG_ACT_THS 					(0x04) /* Activity threshold register */
#define LSM9DS1_AG_ACT_DUR 					(0x05) /* Inactivity duration register */
#define LSM9DS1_AG_INT_GEN_CFG_XL 			(0x06) /* Linear acceleration sensor interrupt generator configuration register. */
#define LSM9DS1_AG_INT_GEN_THS_X_XL 		(0x07) /* Linear acceleration sensor interrupt threshold register. */
#define LSM9DS1_AG_INT_GEN_THS_Y_XL 		(0x08) /* Linear acceleration sensor interrupt threshold register. */
#define LSM9DS1_AG_INT_GEN_THS_Z_XL 		(0x09) /* Linear acceleration sensor interrupt threshold register. */
#define LSM9DS1_AG_INT_GEN_DUR_XL 			(0x0A) /* Linear acceleration sensor interrupt duration register. */
#define LSM9DS1_AG_REFERENCE_G 				(0x0B) /* Angular rate sensor reference value register for digital high-pass filter (r/w). */
#define LSM9DS1_AG_INT1_CTRL 				(0x0C) /* INT1_A/G pin control register. */
#define LSM9DS1_AG_INT2_CTRL 				(0x0D) /* INT2_A/G pin control register */
#define LSM9DS1_AG_WHO_AM_I 				(0x0F) /* Who_AM_I register */
#define LSM9DS1_AG_CTRL_REG1_G 				(0x10) /* Angular rate sensor Control Register 1. */
#define LSM9DS1_AG_CTRL_REG2_G 				(0x11) /* Angular rate sensor Control Register 2 */
#define LSM9DS1_AG_CTRL_REG3_G 				(0x12) /* Angular rate sensor Control Register 3 */
#define LSM9DS1_AG_ORIENT_CFG_G 			(0x13) /* Angular rate sensor sign and orientation register. */
#define LSM9DS1_AG_INT_GEN_SRC_G 			(0x14) /* Angular rate sensor interrupt source register. */
#define LSM9DS1_AG_OUT_TEMP_L 				(0x15) /* Temperature data output register (LSB). */
#define LSM9DS1_AG_OUT_TEMP_H 				(0x16) /* Temperature data output register (MSB). */
#define LSM9DS1_AG_STATUS_REG 				(0x17) /* Status register. */
#define LSM9DS1_AG_OUT_X_L_G 				(0x18) /* Angular rate sensor pitch axis (X) angular rate output register (LSB). */
#define LSM9DS1_AG_OUT_X_H_G 				(0x19) /* Angular rate sensor pitch axis (X) angular rate output register (MSB). */
#define LSM9DS1_AG_OUT_Y_L_G 				(0x1A) /* Angular rate sensor pitch axis (Y) angular rate output register (LSB). */
#define LSM9DS1_AG_OUT_Y_H_G 				(0x1B) /* Angular rate sensor pitch axis (Y) angular rate output register (MSB). */
#define LSM9DS1_AG_OUT_Z_L_G 				(0x1C) /* Angular rate sensor pitch axis (Z) angular rate output register (LSB). */
#define LSM9DS1_AG_OUT_Z_H_G 				(0x1D) /* Angular rate sensor pitch axis (Z) angular rate output register (MSB). */
#define LSM9DS1_AG_CTRL_REG4 				(0x1E) /* Control register 4. */
#define LSM9DS1_AG_CTRL_REG5_XL 			(0x1F) /* Linear acceleration sensor Control Register 5. */
#define LSM9DS1_AG_CTRL_REG6_XL 			(0x20) /* Linear acceleration sensor Control Register 6. */
#define LSM9DS1_AG_CTRL_REG7_XL 			(0x21) /* Linear acceleration sensor Control Register 7 */
#define LSM9DS1_AG_CTRL_REG8 				(0x22) /* Control register 8. */
#define LSM9DS1_AG_CTRL_REG9 				(0x23) /* Control register 9. */
#define LSM9DS1_AG_CTRL_REG10 				(0x24) /* Control register 10. */
#define LSM9DS1_AG_INT_GEN_SRC_XL 			(0x26) /* Linear acceleration sensor interrupt source register. */
#define LSM9DS1_AG_STATUS_REG_2				(0x27) /* Status register. */
#define LSM9DS1_AG_OUT_X_L_XL 				(0x28) /* Linear acceleration sensor X-axis output register (LSB). */
#define LSM9DS1_AG_OUT_X_H_XL 				(0x29) /* Linear acceleration sensor X-axis output register (MSB). */
#define LSM9DS1_AG_OUT_Y_L_XL 				(0x2A) /* Linear acceleration sensor Y-axis output register (LSB). */
#define LSM9DS1_AG_OUT_Y_H_XL 				(0x2B) /* Linear acceleration sensor Y-axis output register (MSB). */
#define LSM9DS1_AG_OUT_Z_L_XL 				(0x2C) /* Linear acceleration sensor Z-axis output register (LSB). */
#define LSM9DS1_AG_OUT_Z_H_XL 				(0x2D) /* Linear acceleration sensor Z-axis output register (MSB). */
#define LSM9DS1_AG_FIFO_CTRL 				(0x2E) /* FIFO control register */
#define LSM9DS1_AG_FIFO_SRC 				(0x2F) /* FIFO status control register */
#define LSM9DS1_AG_INT_GEN_CFG_G 			(0x30) /* Angular rate sensor interrupt generator configuration register. */
#define LSM9DS1_AG_INT_GEN_THS_XH_G 		(0x31) /* Angular rate sensor (X) interrupt generator threshold registers (MSB). */
#define LSM9DS1_AG_INT_GEN_THS_XL_G 		(0x32) /* Angular rate sensor (X) interrupt generator threshold registers (LSB). */
#define LSM9DS1_AG_INT_GEN_THS_YH_G 		(0x33) /* Angular rate sensor (Y) interrupt generator threshold registers (MSB). */
#define LSM9DS1_AG_INT_GEN_THS_YL_G 		(0x34) /* Angular rate sensor (Y) interrupt generator threshold registers (LSB). */
#define LSM9DS1_AG_INT_GEN_THS_ZH_G 		(0x35) /* Angular rate sensor (Z) interrupt generator threshold registers (MSB). */
#define LSM9DS1_AG_INT_GEN_THS_ZL_G 		(0x36) /* Angular rate sensor (Z) interrupt generator threshold registers (LSB). */
#define LSM9DS1_AG_INT_GEN_DUR_G 			(0x37) /* Angular rate sensor interrupt generator duration register. */

/* Magnetometer register definitions */ 
#define LSM9DS1_M_OFFSET_X_REG_L_M 			(0x05) /* Enviromental effects compensation (X) offset registers (LSB). */
#define LSM9DS1_M_OFFSET_X_REG_H_M 			(0x06) /* Enviromental effects compensation (X) offset registers (MSB). */
#define LSM9DS1_M_OFFSET_Y_REG_L_M 			(0x07) /* Enviromental effects compensation (Y) offset registers (LSB). */
#define LSM9DS1_M_OFFSET_Y_REG_H_M 			(0x08) /* Enviromental effects compensation (Y) offset registers (MSB). */
#define LSM9DS1_M_OFFSET_Z_REG_L_M 			(0x09) /* Enviromental effects compensation (Z) offset registers (LSB). */
#define LSM9DS1_M_OFFSET_Z_REG_H_M 			(0x0A) /* Enviromental effects compensation (Z) offset registers (MSB). */
#define LSM9DS1_M_WHO_AM_I_M 				(0x0F) /* Device identification register */
#define LSM9DS1_M_CTRL_REG1_M 				(0x20) /* Control register 1. */
#define LSM9DS1_M_CTRL_REG2_M 				(0x21) /* Control register 2. */
#define LSM9DS1_M_CTRL_REG3_M 				(0x22) /* Control register 3. */
#define LSM9DS1_M_CTRL_REG4_M 				(0x23) /* Control register 4. */
#define LSM9DS1_M_CTRL_REG5_M 				(0x24) /* Control register 5. */
#define LSM9DS1_M_STATUS_REG_M 				(0x27) /* Status register. */
#define LSM9DS1_M_OUT_X_L_M 				(0x28) /* Magnetometer X-axis data output (LSB). */
#define LSM9DS1_M_OUT_X_H_M 				(0x29) /* Magnetometer X-axis data output (MSB). */
#define LSM9DS1_M_OUT_Y_L_M 				(0x2A) /* Magnetometer Y-axis data output (LSB). */
#define LSM9DS1_M_OUT_Y_H_M 				(0x2B) /* Magnetometer Y-axis data output (MSB). */
#define LSM9DS1_M_OUT_Z_L_M 				(0x2C) /* Magnetometer Z-axis data output (LSB). */
#define LSM9DS1_M_OUT_Z_H_M 				(0x2D) /* Magnetometer Z-axis data output (MSB). */
#define LSM9DS1_M_INT_CFG_M 				(0x30) /* Interrupt configuration register. */
#define LSM9DS1_M_INT_SRC_M 				(0x31) /* Interrupt source configuration register. */
#define LSM9DS1_M_INT_THS_L_M 				(0x32) /* Interrupt threshold (LSB). */
#define LSM9DS1_M_INT_THS_H_M 				(0x33) /* Interrupt threshold (MSB). */


/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_LMS9DS1_AG                   	((uint8_t)0x68) /* Accelerometer & Gyroscope Who Am I register content. */
#define I_AM_LMS9DS1_M	                  	((uint8_t)0x3D) /* Magnetometer Who Am I register content. */

 /** @defgroup Accelerometer defines
   * @{
   */

/* Accelerometer Sensor Full Scale */
#define LSM9DS1_A_FS_MASK		(0x18)
#define LSM9DS1_A_FS_2G 		(0x00)
#define LSM9DS1_A_FS_4G 		(0x10)
#define LSM9DS1_A_FS_8G 		(0x18)
#define LSM9DS1_A_FS_16G 		(0x08)

/* Accelerometer sensitivity factors */
#define LSM9DS1_A_SENSITIVITY_2G     ((float)0.000061)  /*!< accelerometer sensitivity with 2 g full scale [g/LSB] */
#define LSM9DS1_A_SENSITIVITY_4G     ((float)0.000122)  /*!< accelerometer sensitivity with 4 g full scale [g/LSB] */
#define LSM9DS1_A_SENSITIVITY_8G     ((float)0.000244)  /*!< accelerometer sensitivity with 8 g full scale [g/LSB] */
#define LSM9DS1_A_SENSITIVITY_16G    ((float)0.000732)  /*!< accelerometer sensitivity with 16 g full scale [g/LSB] */

/* Accelerometer Anti-Aliasing Filter */
#define LSM9DS1_A_BW_408		(0X00)
#define LSM9DS1_A_BW_211		(0X01)
#define LSM9DS1_A_BW_105		(0X02)
#define LSM9DS1_A_BW_50			(0X03)
#define LSM9DS1_A_BW_MASK		(0X03)

#define LSM9DS1_A_BW_MODE_MASK	(0x04)
#define LSM9DS1_A_BW_MODE_AUTO	(0x00)
#define LSM9DS1_A_BW_MODE_MAN	(0x04)

/* Accelerometer Output Data Rate */
#define LSM9DS1_A_ODR_MASK		(0xE0)
#define LSM9DS1_A_ODR_OFF		(0x00)
#define LSM9DS1_A_ODR_10		(0x20)
#define LSM9DS1_A_ODR_50		(0x40)
#define LSM9DS1_A_ODR_119		(0x60)
#define LSM9DS1_A_ODR_238		(0x80)
#define LSM9DS1_A_ODR_476		(0xA0)
#define LSM9DS1_A_ODR_952		(0xC0)

/* Accelerometer Axes Output Enable */
#define LSM9DS1_A_AXE_MASK	(0x38)
#define LSM9DS1_A_AXE_X		(0x08)
#define LSM9DS1_A_AXE_Y		(0x10)
#define LSM9DS1_A_AXE_Z		(0x20)

/* Accelerometer Output Decimation */
#define LSM9DS1_A_OUT_DEC_MASK	(0xC0)
#define LSM9DS1_A_OUT_DEC_1		(0x00)
#define LSM9DS1_A_OUT_DEC_2		(0x40)
#define LSM9DS1_A_OUT_DEC_4		(0x80)
#define LSM9DS1_A_OUT_DEC_8		(0xC0)

 /**
   * @}
   */

/** @defgroup Gyroscope defines
  * @{
  */

/* Gyroscope Sensor Full Scale */
#define LSM9DS1_G_FS_MASK		(0x18)
#define LSM9DS1_G_FS_245DPS		(0x00)
#define LSM9DS1_G_FS_500DPS		(0x08)
#define LSM9DS1_G_FS_2000DPS	(0x18)

/* Gyroscope sensitivity factors */
#define LSM9DS1_G_SENSITIVITY_245DPS     ((float)0.00875)  /*!< gyroscope sensitivity with 245 DPS full scale [dps/LSB] */
#define LSM9DS1_G_SENSITIVITY_500DPS     ((float)0.01750)  /*!< gyroscope sensitivity with 500 DPS full scale [dps/LSB] */
#define LSM9DS1_G_SENSITIVITY_2000DPS 	 ((float)0.07000)  /*!< gyroscope sensitivity with 2000 DPS full scale [dps/LSB] */

/* Gyroscope Output Data Rate */
#define LSM9DS1_G_ODR_MASK		(0xE0)
#define LSM9DS1_G_ODR_OFF		(0x00)
#define LSM9DS1_G_ODR_14_9		(0x20)
#define LSM9DS1_G_ODR_59_5		(0x40)
#define LSM9DS1_G_ODR_119		(0x60)
#define LSM9DS1_G_ODR_238		(0x80)
#define LSM9DS1_G_ODR_476		(0xA0)
#define LSM9DS1_G_ODR_952		(0xC0)

/* Gyroscope Axes Output Enable */
#define LSM9DS1_G_AXE_MASK	(0x38)
#define LSM9DS1_G_AXE_X		(0x08)
#define LSM9DS1_G_AXE_Y		(0x10)
#define LSM9DS1_G_AXE_Z		(0x20)

/**
  * @}
  */

/** @defgroup Magnetometer defines
* @{
*/

/*  Magnetometer Sensor Full Scale */
#define LSM9DS1_M_FS_MASK		(0x60)
#define LSM9DS1_M_FS_4GAUSS		(0x00)
#define LSM9DS1_M_FS_8GAUSS		(0x20)
#define LSM9DS1_M_FS_12GAUSS	(0x40)
#define LSM9DS1_M_FS_16GAUSS	(0x60)

/* Magnetometer sensitivity factors */
#define LSM9DS1_M_SENSITIVITY_4GS     ((float)0.00014)  /*!< magnetometer sensitivity with 4 Gauss full scale [gauss/LSB] */
#define LSM9DS1_M_SENSITIVITY_8GS     ((float)0.00029)  /*!< magnetometer sensitivity with 8 Gauss full scale [gauss/LSB] */
#define LSM9DS1_M_SENSITIVITY_12GS    ((float)0.00043)  /*!< magnetometer sensitivity with 12 Gauss full scale [gauss/LSB] */
#define LSM9DS1_M_SENSITIVITY_16GS    ((float)0.00058)  /*!< magnetometer sensitivity with 16 Gauss full scale [gauss/LSB] */

/* Magnetometer Output Data Rate */
#define LSM9DS1_M_ODR_MASK		(0x1C)
#define LSM9DS1_M_ODR_0_625		(0x00)
#define LSM9DS1_M_ODR_1_25		(0x04)
#define LSM9DS1_M_ODR_2_5		(0x08)
#define LSM9DS1_M_ODR_5			(0x0C)
#define LSM9DS1_M_ODR_10		(0x10)
#define LSM9DS1_M_ODR_20		(0x14)
#define LSM9DS1_M_ODR_40		(0x18)
#define LSM9DS1_M_ODR_80		(0x1C)

/* Magnetometer operation mode XY axes*/
#define LSM9DS1_M_XY_PERF_MASK			(0x60)
#define LSM9DS1_M_XY_PERF_LOW			(0x00)
#define LSM9DS1_M_XY_PERF_MEDIUM		(0x20)
#define LSM9DS1_M_XY_PERF_HIGH			(0x40)
#define LSM9DS1_M_XY_PERF_ULTRAHIGH		(0x60)

/* Magnetometer operation mode Z axes*/
#define LSM9DS1_M_Z_PERF_MASK			(0x0C)
#define LSM9DS1_M_Z_PERF_LOW			(0x00)
#define LSM9DS1_M_Z_PERF_MEDIUM			(0x04)
#define LSM9DS1_M_Z_PERF_HIGH			(0x08)
#define LSM9DS1_M_Z_PERF_ULTRAHIGH		(0x0C)

/* Magnetometer temperature compensation */
#define LSM9DS1_M_TEMPCOMP_MASK		(0x80)
#define LSM9DS1_M_TEMPCOMP_ENABLE	(0x80)
#define LSM9DS1_M_TEMPCOMP_DISABLE	(0x00)

/* Magnetometer self-test */
#define LSM9DS1_M_SELFTEST_MASK		(0x01)
#define LSM9DS1_M_SELFTEST_ENABLE	(0x01)
#define LSM9DS1_M_SELFTEST_DISABLE	(0x00)

/* Magnetometer I2C interface */
#define LSM9DS1_M_I2C_MASK		(0x80)
#define LSM9DS1_M_I2C_ENABLE	(0x00)
#define LSM9DS1_M_I2C_DISABLE	(0x80)

/* Magnetometer SPI read interface */
#define LSM9DS1_M_SPI_R_MASK		(0x04)
#define LSM9DS1_M_SPI_R_ENABLE		(0x04)
#define LSM9DS1_M_SPI_R_DISABLE		(0x00)

/* Magnetometer Low-Power mode */
#define LSM9DS1_M_LOWPOWER_MASK			(0x20)
#define LSM9DS1_M_LOWPOWER_ENABLE		(0x20)
#define LSM9DS1_M_LOWPOWER_DISABLE		(0x00)

/* Magnetometer Operating Mode selection */
#define LSM9DS1_M_OPMODE_MASK			(0x03)
#define LSM9DS1_M_OPMODE_CONTINUOS		(0x00)
#define LSM9DS1_M_OPMODE_SINGLE			(0x01)
#define LSM9DS1_M_OPMODE_POWERDOWN		(0x03)

/**
* @}
*/

/** @defgroup Exported functions
  * @{
  */

void LSM9DS1_A_Init(LSM9DS1_A_InitTypeDef* initStruct);
void LSM9DS1_A_SetFullScale(uint8_t value);
void LSM9DS1_A_SetOutputDataRate(uint8_t value);
void LSM9DS1_A_SetBandWidth(uint8_t value);
void LSM9DS1_A_SetBandWidthMode(uint8_t value);
void LSM9DS1_A_ReadAccelerationXYZ(float* pData);

void LSM9DS1_G_Init(LSM9DS1_G_InitTypeDef* initStruct);
void LSM9DS1_G_ReadGyroXYZ(float* pData);

void LSM9DS1_M_Init(LSM9DS1_M_InitTypeDef* initStruct);
void LSM9DS1_M_ReadMagnetometerXYZ(float* pData);
 
/**
  * @}
  */

  


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __LSM9DS1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
