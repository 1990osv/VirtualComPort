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
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include "protocol.h"
#include "i2c_lcd.h"
#include "can.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

CAN_FilterConfTypeDef hcan1filter;
CanRxMsgTypeDef canRxMsg;
CanTxMsgTypeDef canTxMsg;


uint32_t printDelay;


char str[20];
uint16_t speed=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
* @brief Преобразоывает число в строку
* @param n: число
* @param s: строка
* @retval None
*/
void itoa(int n, char s[])
{
	char c;
	int i,j, sign;

	if ((sign = n) < 0)
	n = -n;        
	i = 0;
	do {      
		s[i++] = n % 10 + '0'; 
	} while ((n /= 10) > 0);   
	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';
	
	j = i - 1;
	i = 0; 
	while (i < j) {
		c = s[i];
		s[i] = s[j];
		s[j] = c;
		i++;
		j--;
	}
}

void lcd_PrintXY(char *str, unsigned char x, unsigned char y)
{
        lcd_Goto(y,x);
        lcd_PrintC(str);        
}

void lcd_PrintSpase(unsigned char n)
{
        while(n--)
        lcd_Data(' ');
        
}

//uint16_t pinToggleReadSSI ( void )
//{
//        uint8_t bit_count;
//        uint16_t u16result = 0;
//        GPIO_PinState pinState;
//        
//        u16result = 0;
//        
//        for (bit_count = 0; bit_count <= 16; bit_count++)
//        {
//                GPIOD->BSRR = GPIO_PIN_11; //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,0);
//                u16result = (u16result << 1);
//                pinState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10); 
//                GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16; //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,1);
//                if ( pinState != GPIO_PIN_RESET)
//                {
//                        u16result = u16result | 0x01;
//                }
//        }
//        
//        azPosition = u16result;
//        
//        return u16result;
//}

uint16_t pinToggleReadSSI ( void )
{
        uint8_t bit_count;
        uint16_t u16result = 0;
        GPIO_PinState pinState;
        
        u16result = 0;
        
        for (bit_count = 0; bit_count <= 16; bit_count++)
        {
                // falling edge on clock port
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,0);//SSI_CLK_PORT &= ~(1 << SSI_CLK_BIT);
                
                // left-shift the current result
                u16result = (u16result << 1);
                // read the port data
                pinState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10); //u8portdata = SSI_DTA_PORT;
                // rising edge on clock port, data changes
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,1);//SSI_CLK_PORT |= (1 << SSI_CLK_BIT);
                // evaluate the port data (port set or clear)
                if ( pinState != GPIO_PIN_RESET)
                {
                        // bit is set, set LSB of result
                        u16result = u16result | 0x01;
                } // if
        } // for
        
        azPosition = u16result;
        
        return u16result;
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);

        lcd_initialisation();
				lcd_PrintXY("count  = ",0,0); 
				lcd_PrintXY("azimut = ",0,1);
				lcd_PrintXY("angle  = ",0,2);
				lcd_PrintXY("phase  = ",0,3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

        //encryptTxMsg(&canTxMsg,100,10,21,0,0,0);
        
        printDelay=0;

//        HAL_CAN_Transmit_IT(&hcan1);
//        HAL_NVIC_SetPriority(CAN1_TX_IRQn,7,7);
//        HAL_NVIC_SetPriority(CAN1_RX0_IRQn,7,7);
//        HAL_NVIC_SetPriority(CAN1_RX1_IRQn,7,7);
        while (1)
        {
//                speed++;
                pinToggleReadSSI();
                //HAL_Delay(50);  
                //canStatus = hcan1.State;
//                switch(printDelay++)
//                {
//                        case 1: 
//                        {
                                itoa(lastCiclCount,str);
                                lcd_PrintSpase(5);
                                lcd_PrintXY(str,10,0); 
//                        } break;
//                        case 2: 
//                        {
                                itoa(azPosition,str);
                                lcd_PrintSpase(5);
                                lcd_PrintXY(str,10,1);
//                        } break;
//                        case 3: 
//                        {
                                itoa(azPosition*360/0xFFFF,str);
                                lcd_PrintSpase(3);
                                lcd_PrintXY(str,10,2);
//                                itoa(canTxMsg.Data[0],str);
//                                lcd_PrintSpase(5);
//                                lcd_PrintXY(str,10,2);
//                        } break;
//                        case 4: 
//                        {
                                itoa(canTxMsg.Data[0],str);
                                lcd_PrintSpase(5);
                                lcd_PrintXY(str,10,3);
                                printDelay=1; 
                                
//                        } break;
//                }										
        }
        
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_SILENT_LOOPBACK;//CAN_MODE_LOOPBACK;
	hcan1.Init.Prescaler = 3; //2400000/1000000/3 = 8
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_4TQ;
	hcan1.Init.BS2 = CAN_BS2_3TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = ENABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;

        hcan1.pRxMsg = &canRxMsg;
        hcan1.pTxMsg = &canTxMsg;
        
	HAL_CAN_Init(&hcan1);

        hcan1filter.FilterNumber = 0;        
        hcan1filter.BankNumber = 14;
        hcan1filter.FilterMode = CAN_FILTERMODE_IDMASK;
        hcan1filter.FilterScale = CAN_FILTERSCALE_32BIT;
        hcan1filter.FilterIdHigh = 0x0000;
        hcan1filter.FilterIdLow = 0x0000;
        hcan1filter.FilterMaskIdHigh = 0x0000;
        hcan1filter.FilterMaskIdLow = 0x0000;
        hcan1filter.FilterFIFOAssignment = 0;
        hcan1filter.FilterActivation = ENABLE;
     
        HAL_CAN_ConfigFilter(&hcan1, &hcan1filter);
        
        HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM10 init function */
static void MX_TIM4_Init(void)
{
        __HAL_RCC_TIM4_CLK_ENABLE();
        
        htim4.Instance = TIM4;
        htim4.Init.Prescaler = 9600;
        htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim4.Init.Period = 20; //4..5 мс период опроса CAN
        htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
        {
                Error_Handler();
        }

        
        if(HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
        {
                /* Starting Error */
                Error_Handler();
        }        
        HAL_NVIC_EnableIRQ(TIM4_IRQn); 
        HAL_NVIC_SetPriority(TIM4_IRQn,0,7);        

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ssiClc3_GPIO_Port, ssiClc3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ssiClc2_Pin|ssiClk1_Pin|LD4_Pin|LD3_Pin 
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin ssiData3_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|ssiData3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ssiClc3_Pin */
  GPIO_InitStruct.Pin = ssiClc3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ssiClc3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ssiData2_Pin ssiData1_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = ssiData2_Pin|ssiData1_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ssiClc2_Pin ssiClk1_Pin LD4_Pin LD3_Pin 
                           LD5_Pin LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = ssiClc2_Pin|ssiClk1_Pin|LD4_Pin|LD3_Pin 
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 I2S3_SCK_Pin PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|I2S3_SCK_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
        while(1)
        {
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);                 
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13); 
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);                 
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); 

                HAL_Delay(100);
        }
  /* USER CODE END Error_Handler */ 
}

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
        
        while(1)
        {
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14); 
                HAL_Delay(100);
        }        
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
