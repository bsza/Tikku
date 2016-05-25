/**
  ******************************************************************************
  * @file    lsm9ds1.c
  * @author  Szabolcs Balási
  * @version V0.1.
  * @date    05-May-2016
  * @brief   This file provides a set of functions needed to manage the lsm9ds1
  *          MEMS 9-DOF IMU.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "lsm9ds1.h"
#include "mxconstants.h"
#include "spi.h"
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

/** @defgroup LSM9DS1_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup LSM9DS1_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup LSM9DS1_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup LSM9DS1_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup LSM9DS1_Private_Functions
  * @{
  */

/**
  * @brief  Writes one byte to the ACCELEROMETER / GYROSCOPE.
  * @param  RegisterAddr specifies the ACCELEROMETER / GYROSCOPE register to be written.
  * @param  Value Data to be written
  * @retval   None
 */
void LSM9DS1_AG_IO_Write(uint8_t RegisterAddr, uint8_t Value)
{
	HAL_GPIO_WritePin(SENS_SPI_CSn_A_G_GPIO_Port, SENS_SPI_CSn_A_G_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &RegisterAddr, 1, 5);

	HAL_SPI_Transmit(&hspi1, &Value, 1, 5);

	HAL_GPIO_WritePin(SENS_SPI_CSn_A_G_GPIO_Port, SENS_SPI_CSn_A_G_Pin, GPIO_PIN_SET);

	// TODO: Optimize for multiple byte message
}

/**
  * @brief  Writes one byte to the MAGNETOMETER.
  * @param  RegisterAddr specifies the MAGNETOMETER register to be written.
  * @param  Value Data to be written
  * @retval   None
 */
void LSM9DS1_M_IO_Write(uint8_t RegisterAddr, uint8_t Value)
{
	HAL_GPIO_WritePin(SENS_SPI_CSn_M_GPIO_Port, SENS_SPI_CSn_M_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &RegisterAddr, 1, 5);

	HAL_SPI_Transmit(&hspi1, &Value, 1, 5);

	HAL_GPIO_WritePin(SENS_SPI_CSn_M_GPIO_Port, SENS_SPI_CSn_M_Pin, GPIO_PIN_SET);

	// TODO: Optimize for multiple byte message
}

/**
  * @brief  Reads a block of data from the ACCELEROMETER / GYROSCOPE.
  * @param  RegisterAddr : specifies the ACCELEROMETER / GYROSCOPE internal address register to read from
  * @retval ACCELEROMETER / GYROSCOPE register value
  */
uint8_t LSM9DS1_AG_IO_Read(uint8_t RegisterAddr)
{
	uint8_t result = 0;

	RegisterAddr |= 0x80; /* Set RW bit to read */

	HAL_GPIO_WritePin(SENS_SPI_CSn_A_G_GPIO_Port, SENS_SPI_CSn_A_G_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &RegisterAddr, 1, 5);

	HAL_SPI_Receive(&hspi1, &result, 1, 5);

	HAL_GPIO_WritePin(SENS_SPI_CSn_A_G_GPIO_Port, SENS_SPI_CSn_A_G_Pin, GPIO_PIN_SET);

	return result;

	// TODO: Optimize for multiple byte message
}

/**
  * @brief  Reads a block of data from the MAGNETOMETER.
  * @param  RegisterAddr : specifies the MAGNETOMETER internal address register to read from
  * @retval MAGNETOMETER register value
  */
uint8_t LSM9DS1_M_IO_Read(uint8_t RegisterAddr)
{
	uint8_t result = 0;

	RegisterAddr |= 0x80; /* Set RW bit to read */

	HAL_GPIO_WritePin(SENS_SPI_CSn_M_GPIO_Port, SENS_SPI_CSn_M_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &RegisterAddr, 1, 5);

	HAL_SPI_Receive(&hspi1, &result, 1, 5);

	HAL_GPIO_WritePin(SENS_SPI_CSn_M_GPIO_Port, SENS_SPI_CSn_M_Pin, GPIO_PIN_SET);

	return result;

	// TODO: Optimize for multiple byte message
}

/**
  * @brief  Verifies the content of the Who Am I register in the ACCELEROMETER / GYROSCOPE.
  * @retval 1 if correct, 0 otherwise.
  */
uint8_t LSM9DS1_AG_WhoAmIVerification(void)
{
	if (I_AM_LMS9DS1_AG == LSM9DS1_AG_IO_Read(LSM9DS1_AG_WHO_AM_I))
		return 1;
	else
		return 0;
}

/**
  * @brief  Verifies the content of the Who Am I register in the MAGNETOMETER.
  * @retval 1 if correct, 0 otherwise.
  */
uint8_t LSM9DS1_M_WhoAmIVerification(void)
{
	if (I_AM_LMS9DS1_M == LSM9DS1_M_IO_Read(LSM9DS1_M_WHO_AM_I_M))
		return 1;
	else
		return 0;
}

/**
  * @}
  */

/** @defgroup LSM9DS1_Exported_Functions
  * @{
  */

/**
  * @brief  Set full-scale value of ACCELEROMETER.
  * @param  Value Data to be written
  * @retval None
 */
void LSM9DS1_A_SetFullScale(uint8_t value)
{
	uint8_t ctrl = LSM9DS1_AG_IO_Read(LSM9DS1_AG_CTRL_REG6_XL);

	ctrl |= (value & LSM9DS1_A_FS_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG6_XL, ctrl);
}

/**
  * @brief  Set Output Data Rate of ACCELEROMETER.
  * @param  Value Data to be written
  * @retval None
 */
void LSM9DS1_A_SetOutputDataRate(uint8_t value)
{
	uint8_t ctrl = LSM9DS1_AG_IO_Read(LSM9DS1_AG_CTRL_REG6_XL);

	ctrl |= (value & LSM9DS1_A_ODR_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG6_XL, ctrl);
}

/**
  * @brief  Set Anti Aliasing Filter Bandwidth of ACCELEROMETER.
  * @param  Value Data to be written
  * @retval None
 */
void LSM9DS1_A_SetBandWidth(uint8_t value)
{
	uint8_t ctrl = LSM9DS1_AG_IO_Read(LSM9DS1_AG_CTRL_REG6_XL);

	ctrl |= (value & LSM9DS1_A_BW_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG6_XL, ctrl);
}

/**
  * @brief  Select Anti Aliasing Filter Bandwidth Mode of ACCELEROMETER.
  * @param  Value Data to be written
  * @retval None
 */
void LSM9DS1_A_SetBandWidthMode(uint8_t value)
{
	uint8_t ctrl = LSM9DS1_AG_IO_Read(LSM9DS1_AG_CTRL_REG6_XL);

	ctrl |= (value & LSM9DS1_A_BW_MODE_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG6_XL, ctrl);
}

/**
  * @brief  Initialize ACCELEROMETER.
  * @param  initStruct Initialization data structure.
  * @retval None
 */
void LSM9DS1_A_Init(LSM9DS1_A_InitTypeDef* initStruct)
{
	uint8_t ctrl;

	ctrl = (initStruct->Axes_Enable & LSM9DS1_A_AXE_MASK) | (initStruct->Output_Decimation & LSM9DS1_A_OUT_DEC_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG5_XL, ctrl);

	ctrl = (initStruct->Full_Scale & LSM9DS1_A_FS_MASK) | (initStruct->Output_DataRate & LSM9DS1_A_ODR_MASK) | (initStruct->BandWidth_Mode & LSM9DS1_A_BW_MASK) | (initStruct->BandWidth & LSM9DS1_A_BW_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG6_XL, ctrl);

}

/**
  * @brief  Initialize GYROSCOPE.
  * @param  initStruct Initialization data structure.
  * @retval None
 */
void LSM9DS1_G_Init(LSM9DS1_G_InitTypeDef* initStruct)
{
	uint8_t ctrl;

	ctrl = (initStruct->Axes_Enable & LSM9DS1_G_AXE_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG4, ctrl);

	ctrl = (initStruct->Full_Scale & LSM9DS1_G_FS_MASK) | (initStruct->Output_DataRate & LSM9DS1_G_ODR_MASK);

	LSM9DS1_AG_IO_Write(LSM9DS1_AG_CTRL_REG1_G, ctrl);

}

/**
  * @brief  Initialize MAGNETOMETER.
  * @param  initStruct Initialization data structure.
  * @retval None
 */
void LSM9DS1_M_Init(LSM9DS1_M_InitTypeDef* initStruct)
{
	uint8_t ctrl;

	ctrl = (initStruct->Full_Scale & LSM9DS1_M_FS_MASK);

	LSM9DS1_M_IO_Write(LSM9DS1_M_CTRL_REG2_M, ctrl);

	ctrl = (initStruct->Output_DataRate & LSM9DS1_M_ODR_MASK) | (initStruct->Self_Test & LSM9DS1_M_SELFTEST_MASK) | (initStruct->Temperature_Compensation & LSM9DS1_M_TEMPCOMP_MASK) | (initStruct->XY_Performance & LSM9DS1_M_XY_PERF_MASK);

	LSM9DS1_M_IO_Write(LSM9DS1_M_CTRL_REG1_M, ctrl);

	ctrl = (initStruct->I2C_Interface & LSM9DS1_M_I2C_MASK) | (initStruct->SPI_Interface_Read & LSM9DS1_M_SPI_R_MASK) | (initStruct->Operating_Mode & LSM9DS1_M_OPMODE_MASK) | (initStruct->Low_Power_Mode & LSM9DS1_M_LOWPOWER_MASK);

	LSM9DS1_M_IO_Write(LSM9DS1_M_CTRL_REG3_M, ctrl);

	ctrl = (initStruct->Z_Performance & LSM9DS1_M_Z_PERF_MASK);

	LSM9DS1_M_IO_Write(LSM9DS1_M_CTRL_REG4_M, ctrl);


}

/* @brief  Read X, Y & Z Acceleration values
* @param  pData: Data out pointer
* @retval None
*/
void LSM9DS1_A_ReadAccelerationXYZ(float* pData){

	uint8_t buffer[6];
	uint8_t ctrl;
	float sensitivity = 0;

	/* Read content of control register to obtain sensitivity */
	ctrl = LSM9DS1_AG_IO_Read(LSM9DS1_AG_CTRL_REG6_XL);

	/* Read output register of linear acceleration */
	buffer[0] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_X_L_XL);
	buffer[1] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_X_H_XL);
	buffer[2] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Y_L_XL);
	buffer[3] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Y_H_XL);
	buffer[4] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Z_L_XL);
	buffer[5] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Z_H_XL);

	/* Select sensitivity correction factor */
	switch(ctrl & LSM9DS1_A_FS_MASK) {
		case LSM9DS1_A_FS_2G:
			sensitivity = LSM9DS1_A_SENSITIVITY_2G;
			break;
		case LSM9DS1_A_FS_4G:
			sensitivity = LSM9DS1_A_SENSITIVITY_4G;
			break;
		case LSM9DS1_A_FS_8G:
			sensitivity = LSM9DS1_A_SENSITIVITY_8G;
			break;
		case LSM9DS1_A_FS_16G:
			sensitivity = LSM9DS1_A_SENSITIVITY_16G;
			break;
	}

	pData[0] = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0]) * sensitivity;
	pData[1] = ((int16_t)((uint16_t)buffer[3] << 8) + buffer[2]) * sensitivity;
	pData[2] = ((int16_t)((uint16_t)buffer[5] << 8) + buffer[4]) * sensitivity;
}

/* @brief  Read X, Y & Z gyroscope values
* @param  pData: Data out pointer
* @retval None
*/
void LSM9DS1_G_ReadGyroXYZ(float* pData){

	uint8_t buffer[6];
	uint8_t ctrl;
	float sensitivity = 0;

	/* Read content of control register to obtain sensitivity */
	ctrl = LSM9DS1_AG_IO_Read(LSM9DS1_AG_CTRL_REG1_G);

	/* Read output registers of gyroscope */
	buffer[0] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_X_L_G);
	buffer[1] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_X_H_G);
	buffer[2] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Y_L_G);
	buffer[3] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Y_H_G);
	buffer[4] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Z_L_G);
	buffer[5] = LSM9DS1_AG_IO_Read(LSM9DS1_AG_OUT_Z_H_G);

	/* Select sensitivity correction factor */
	switch(ctrl & LSM9DS1_G_FS_MASK) {
		case LSM9DS1_G_FS_245DPS:
			sensitivity = LSM9DS1_G_SENSITIVITY_245DPS;
			break;
		case LSM9DS1_G_FS_500DPS:
			sensitivity = LSM9DS1_G_SENSITIVITY_500DPS;
			break;
		case LSM9DS1_G_FS_2000DPS:
			sensitivity = LSM9DS1_G_SENSITIVITY_2000DPS;
			break;
	}

	pData[0] = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0]) * sensitivity;
	pData[1] = ((int16_t)((uint16_t)buffer[3] << 8) + buffer[2]) * sensitivity;
	pData[2] = ((int16_t)((uint16_t)buffer[5] << 8) + buffer[4]) * sensitivity;
}

/* @brief  Read X, Y & Z gyroscope values
* @param  pData: Data out pointer
* @retval None
*/
void LSM9DS1_M_ReadMagnetometerXYZ(float* pData){

	uint8_t buffer[6];
	uint8_t ctrl;
	float sensitivity = 0;

	/* Read content of control register to obtain sensitivity */
	ctrl = LSM9DS1_M_IO_Read(LSM9DS1_M_CTRL_REG2_M);

	/* Read output registers of magnetometer */
	buffer[0] = LSM9DS1_M_IO_Read(LSM9DS1_M_OUT_X_L_M);
	buffer[1] = LSM9DS1_M_IO_Read(LSM9DS1_M_OUT_X_H_M);
	buffer[2] = LSM9DS1_M_IO_Read(LSM9DS1_M_OUT_Y_L_M);
	buffer[3] = LSM9DS1_M_IO_Read(LSM9DS1_M_OUT_Y_H_M);
	buffer[4] = LSM9DS1_M_IO_Read(LSM9DS1_M_OUT_Z_L_M);
	buffer[5] = LSM9DS1_M_IO_Read(LSM9DS1_M_OUT_Z_H_M);

	/* Select sensitivity correction factor */
	switch(ctrl & LSM9DS1_M_FS_MASK) {
		case LSM9DS1_M_FS_4GAUSS:
			sensitivity = LSM9DS1_M_SENSITIVITY_4GS;
			break;
		case LSM9DS1_M_FS_8GAUSS:
			sensitivity = LSM9DS1_M_SENSITIVITY_8GS;
			break;
		case LSM9DS1_M_FS_12GAUSS:
			sensitivity = LSM9DS1_M_SENSITIVITY_12GS;
			break;
		case LSM9DS1_M_FS_16GAUSS:
			sensitivity = LSM9DS1_M_SENSITIVITY_16GS;
			break;
	}

	pData[0] = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0]) * sensitivity;
	pData[1] = ((int16_t)((uint16_t)buffer[3] << 8) + buffer[2]) * sensitivity;
	pData[2] = ((int16_t)((uint16_t)buffer[5] << 8) + buffer[4]) * sensitivity;
}



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

