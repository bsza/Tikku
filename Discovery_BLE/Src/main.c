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

/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#include "usbd_cdc_if.h"

#include "cube_hal.h"
#include "osal.h"
#include "sensor_service.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
/* USER CODE END Includes */

#define BDADDR_SIZE 6

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t set_connectable;
extern volatile int connected;
extern AxesRaw_t axes_data;
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void User_Process(AxesRaw_t* p_axes);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	  const char *name = "BlueNRG";
	  uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};
	  uint8_t bdaddr[BDADDR_SIZE];
	  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

	  uint8_t  hwVersion;
	  uint16_t fwVersion;

	  int ret;

	  int16_t AccXYZData[3];
	  float GyroXYZData[3];
	  float MagXYZData[3];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN 2 */
#if NEW_SERVICES
  /* Configure LED2 */
  BSP_LED_Init(LED2);
#endif

  /* Configure the User Button in GPIO Mode */
  //BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); /* GPIO ports */
  MX_I2C1_Init(); /* L3GD20 */
  MX_SPI1_Init(); /* LS303DH */
  MX_USB_DEVICE_Init(); /* USB serial debug */

  BSP_ACCELERO_Init();
  BSP_GYRO_Init();

  /* Initialize the BlueNRG SPI driver */
   BNRG_SPI_Init();

   /* Initialize the BlueNRG HCI */
   HCI_Init();

   /* Reset BlueNRG hardware */
   BlueNRG_RST();

   /* get the BlueNRG HW and FW versions */
   getBlueNRGVersion(&hwVersion, &fwVersion);

   /*
    * Reset BlueNRG again otherwise we won't
    * be able to change its MAC address.
    * aci_hal_write_config_data() must be the first
    * command after reset otherwise it will fail.
    */
   BlueNRG_RST();

   PRINTF("HWver %d, FWver %d", hwVersion, fwVersion);

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
     if(ret){
       PRINTF("Setting BD_ADDR failed.\n");
     }

     ret = aci_gatt_init();
     if(ret){
       PRINTF("GATT_Init failed.\n");
     }

     if (bnrg_expansion_board == IDB05A1) {
       ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
     }
     else {
       ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
     }

     if(ret != BLE_STATUS_SUCCESS){
       PRINTF("GAP_Init failed.\n");
     }

     ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                      strlen(name), (uint8_t *)name);

     if(ret){
       PRINTF("aci_gatt_update_char_value failed.\n");
       while(1);
     }

     ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                        OOB_AUTH_DATA_ABSENT,
                                        NULL,
                                        7,
                                        16,
                                        USE_FIXED_PIN_FOR_PAIRING,
                                        123456,
                                        BONDING);
     if (ret == BLE_STATUS_SUCCESS) {
       PRINTF("BLE Stack Initialized.\n");
     }

     PRINTF("SERVER: BLE Stack Initialized\n");

     ret = Add_Acc_Service();

     if(ret == BLE_STATUS_SUCCESS)
       PRINTF("Acc service added successfully.\n");
     else
       PRINTF("Error while adding Acc service.\n");

     ret = Add_Environmental_Sensor_Service();

     if(ret == BLE_STATUS_SUCCESS)
       PRINTF("Environmental Sensor service added successfully.\n");
     else
       PRINTF("Error while adding Environmental Sensor service.\n");

   #if NEW_SERVICES
     /* Instantiate Timer Service with two characteristics:
      * - seconds characteristic (Readable only)
      * - minutes characteristics (Readable and Notifiable )
      */
     ret = Add_Time_Service();

     if(ret == BLE_STATUS_SUCCESS)
       PRINTF("Time service added successfully.\n");
     else
       PRINTF("Error while adding Time service.\n");

     /* Instantiate LED Button Service with one characteristic:
      * - LED characteristic (Readable and Writable)
      */
     ret = Add_LED_Service();

     if(ret == BLE_STATUS_SUCCESS)
       PRINTF("LED service added successfully.\n");
     else
       PRINTF("Error while adding LED service.\n");
   #endif

     /* Set output power level */
     ret = aci_hal_set_tx_power_level(1,4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  BSP_ACCELERO_AccGetXYZ(AccXYZData);
	  BSP_ACCELERO_MagGetXYZ(MagXYZData);
	  BSP_GYRO_GetXYZ(GyroXYZData);

	  axes_data.AXIS_X = -(AccXYZData[0] >> 4);
	  axes_data.AXIS_Y = -(AccXYZData[1] >> 4);
	  axes_data.AXIS_Z = -(AccXYZData[2] >> 4);

	  HCI_Process();
	  User_Process(&axes_data);

	  HAL_Delay(25);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* USER CODE BEGIN 4 */

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
