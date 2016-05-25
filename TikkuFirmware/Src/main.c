/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "cube_hal.h"
#include "osal.h"
#include "sensor_service.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include "lsm9ds1.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define BDADDR_SIZE 6

extern volatile uint8_t set_connectable;
extern volatile int connected;
extern AxesRaw_t axes_data;
uint8_t bnrg_expansion_board = IDB04A1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void User_Process(AxesRaw_t* p_axes);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void configureLSM9DS1(void){

	LSM9DS1_A_InitTypeDef AccInitStructure = {};
	LSM9DS1_G_InitTypeDef GyroInitStructure = {};
	LSM9DS1_M_InitTypeDef MagInitStructure = {};

	AccInitStructure.Output_DataRate = LSM9DS1_A_ODR_119;
	AccInitStructure.Axes_Enable = LSM9DS1_A_AXE_X | LSM9DS1_A_AXE_Y | LSM9DS1_A_AXE_Z;
	AccInitStructure.BandWidth_Mode = LSM9DS1_A_BW_MODE_AUTO;
	AccInitStructure.Full_Scale = LSM9DS1_A_FS_8G;

	LSM9DS1_A_Init(&AccInitStructure);

	GyroInitStructure.Output_DataRate = LSM9DS1_G_ODR_119;
	GyroInitStructure.Axes_Enable = LSM9DS1_G_AXE_X | LSM9DS1_G_AXE_Y | LSM9DS1_G_AXE_Z;
	GyroInitStructure.Full_Scale = LSM9DS1_G_FS_2000DPS;

	LSM9DS1_G_Init(&GyroInitStructure);

	//TODO: Change interface to SPI

	MagInitStructure.Full_Scale = LSM9DS1_M_FS_8GAUSS;
	MagInitStructure.Output_DataRate = LSM9DS1_M_ODR_10;
	MagInitStructure.Operating_Mode = LSM9DS1_M_OPMODE_CONTINUOS;
	MagInitStructure.I2C_Interface = LSM9DS1_M_I2C_ENABLE;
	MagInitStructure.SPI_Interface_Read = LSM9DS1_M_SPI_R_DISABLE;
	MagInitStructure.Low_Power_Mode = LSM9DS1_M_LOWPOWER_DISABLE;
	MagInitStructure.Self_Test = LSM9DS1_M_SELFTEST_DISABLE;
	MagInitStructure.Temperature_Compensation = LSM9DS1_M_TEMPCOMP_ENABLE;
	MagInitStructure.XY_Performance = LSM9DS1_M_XY_PERF_HIGH;
	MagInitStructure.Z_Performance = LSM9DS1_M_Z_PERF_HIGH;

	LSM9DS1_M_Init(&MagInitStructure);

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	const char *name = "MyTikku";
		  uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};
		  uint8_t bdaddr[BDADDR_SIZE];
		  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

		  uint8_t  hwVersion;
		  uint16_t fwVersion;

		  int ret;

		  int16_t AccXYZData[3];
		  float GyroXYZData[3];
		  float MagXYZData[3];
		  volatile float test_acc[3]={0,0,0};
		  volatile float test_gyro[3]={0,0,0};
		  volatile float test_magn[3]={0,0,0};
		  volatile int i = -5;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ADC1_Init();
  //MX_I2C1_Init();
  MX_SPI1_Init();
  //MX_SPI2_Init();
  //MX_TIM2_Init();
  //MX_TIM3_Init();
  //MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  BNRG_SPI_Init();
  HCI_Init();
  BlueNRG_RST();
  getBlueNRGVersion(&hwVersion, &fwVersion);
  BlueNRG_RST();

  HAL_Delay(1000);
  configureLSM9DS1();
  HAL_Delay(1000);

  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
      bnrg_expansion_board = IDB05A1;
      /*
       * Change the MAC address to avoid issues with Android cache:
       * if different boards have the same MAC address, Android
       * applications unless you restart Bluetooth on tablet/phone
       */
      SERVER_BDADDR[5] = 0x02;
    }

  /* The Nucleo board must be configured as SERVER */
  Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  ret = aci_gatt_init();

  if (bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(name), (uint8_t *)name);


  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);

  ret = Add_Acc_Service();

  ret = Add_Environmental_Sensor_Service();

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);


  AccXYZData[0] = 100;
  AccXYZData[1] = 100;
  AccXYZData[2] = 100;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  i = LSM9DS1_AG_WhoAmIVerification();

	  LSM9DS1_A_ReadAccelerationXYZ(test_acc);
	  LSM9DS1_G_ReadGyroXYZ(test_gyro);
	  LSM9DS1_M_ReadMagnetometerXYZ(test_magn);

	  AccXYZData[0] += 100;
	  AccXYZData[1] += 100;
	  AccXYZData[2] += 100;



	  axes_data.AXIS_X = -((int)(test_acc[0]*1024));
	  axes_data.AXIS_Y = -((int)(test_acc[1]*1024));
	  axes_data.AXIS_Z = -((int)(test_acc[2]*1024));

	  HCI_Process();
	  User_Process(&axes_data);

	  HAL_Delay(10);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void User_Process(AxesRaw_t* p_axes)
{
  if(set_connectable){
    setConnectable();
    set_connectable = FALSE;
  }

  /* Check if the user has pushed the button */
  if(1)
  {
    //while (BSP_PB_GetState(BUTTON_KEY) == RESET);

    //BSP_LED_Toggle(LED2); //used for debugging (BSP_LED_Init() above must be also enabled)

    if(connected)
    {
      /* Update acceleration data */
      p_axes->AXIS_X += 100;
      p_axes->AXIS_Y += 100;
      p_axes->AXIS_Z += 100;
      //PRINTF("ACC: X=%6d Y=%6d Z=%6d\r\n", p_axes->AXIS_X, p_axes->AXIS_Y, p_axes->AXIS_Z);
      Acc_Update(p_axes);
    }
  }
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
