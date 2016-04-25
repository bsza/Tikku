/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "stm32_bluenrg_ble.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi1);

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi3);

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|SPI1_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else  if(hspi->Instance==BNRG_SPI_INSTANCE)
  {
      /* Enable peripherals clock */

      /* Enable GPIO Ports Clock */
      BNRG_SPI_RESET_CLK_ENABLE();
      BNRG_SPI_SCLK_CLK_ENABLE();
      BNRG_SPI_MISO_CLK_ENABLE();
      BNRG_SPI_MOSI_CLK_ENABLE();
      BNRG_SPI_CS_CLK_ENABLE();
      BNRG_SPI_IRQ_CLK_ENABLE();

      /* Enable SPI clock */
      BNRG_SPI_CLK_ENABLE();

      /* Reset */
      GPIO_InitStruct.Pin = BNRG_SPI_RESET_PIN;
      GPIO_InitStruct.Mode = BNRG_SPI_RESET_MODE;
      GPIO_InitStruct.Pull = BNRG_SPI_RESET_PULL;
      GPIO_InitStruct.Speed = BNRG_SPI_RESET_SPEED;
      GPIO_InitStruct.Alternate = BNRG_SPI_RESET_ALTERNATE;
      HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);
      HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);	/*Added to avoid spurious interrupt from the BlueNRG */

      /* SCLK */
      GPIO_InitStruct.Pin = BNRG_SPI_SCLK_PIN;
      GPIO_InitStruct.Mode = BNRG_SPI_SCLK_MODE;
      GPIO_InitStruct.Pull = BNRG_SPI_SCLK_PULL;
      GPIO_InitStruct.Speed = BNRG_SPI_SCLK_SPEED;
      GPIO_InitStruct.Alternate = BNRG_SPI_SCLK_ALTERNATE;
      HAL_GPIO_Init(BNRG_SPI_SCLK_PORT, &GPIO_InitStruct);

      /* MISO */
      GPIO_InitStruct.Pin = BNRG_SPI_MISO_PIN;
      GPIO_InitStruct.Mode = BNRG_SPI_MISO_MODE;
      GPIO_InitStruct.Pull = BNRG_SPI_MISO_PULL;
      GPIO_InitStruct.Speed = BNRG_SPI_MISO_SPEED;
      GPIO_InitStruct.Alternate = BNRG_SPI_MISO_ALTERNATE;
      HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);

      /* MOSI */
      GPIO_InitStruct.Pin = BNRG_SPI_MOSI_PIN;
      GPIO_InitStruct.Mode = BNRG_SPI_MOSI_MODE;
      GPIO_InitStruct.Pull = BNRG_SPI_MOSI_PULL;
      GPIO_InitStruct.Speed = BNRG_SPI_MOSI_SPEED;
      GPIO_InitStruct.Alternate = BNRG_SPI_MOSI_ALTERNATE;
      HAL_GPIO_Init(BNRG_SPI_MOSI_PORT, &GPIO_InitStruct);

      /* NSS/CSN/CS */
      GPIO_InitStruct.Pin = BNRG_SPI_CS_PIN;
      GPIO_InitStruct.Mode = BNRG_SPI_CS_MODE;
      GPIO_InitStruct.Pull = BNRG_SPI_CS_PULL;
      GPIO_InitStruct.Speed = BNRG_SPI_CS_SPEED;
      GPIO_InitStruct.Alternate = BNRG_SPI_CS_ALTERNATE;
      HAL_GPIO_Init(BNRG_SPI_CS_PORT, &GPIO_InitStruct);
      HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);

      /* IRQ -- INPUT */
      GPIO_InitStruct.Pin = BNRG_SPI_IRQ_PIN;
      GPIO_InitStruct.Mode = BNRG_SPI_IRQ_MODE;
      GPIO_InitStruct.Pull = BNRG_SPI_IRQ_PULL;
      GPIO_InitStruct.Speed = BNRG_SPI_IRQ_SPEED;
      GPIO_InitStruct.Alternate = BNRG_SPI_IRQ_ALTERNATE;
      HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStruct);

      /* Configure the NVIC for SPI */
      HAL_NVIC_SetPriority(BNRG_SPI_EXTI_IRQn, 3, 0);
      HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);
  }
  else if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __SPI3_CLK_ENABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|SPI1_MISO_Pin);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
