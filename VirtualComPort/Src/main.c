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
#include <stdio.h>
/* USER CODE BEGIN Includes */

#include "protocol.h"
#include "i2c_lcd.h"
#include "can.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

CAN_HandleTypeDef hcan1;
CAN_FilterConfTypeDef hcan1filter;
CanRxMsgTypeDef canRxMsg;
CanTxMsgTypeDef canTxMsg;

uint8_t canStatus;
uint8_t data_,err_cnt;
uint32_t printDelay;

I2C_HandleTypeDef hi2c3;

char str5[5];
char str6[6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C3_Init(void);
void assert_failed(uint8_t* file, uint32_t line);

static void Error_Handler(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void lcd_PrintXY(char *str, unsigned char x, unsigned char y)
{
        lcd_Goto(y,x);
        lcd_PrintC(str);        
}



uint16_t pinToggleReadSSI ( void )
{
        uint8_t bit_count;
        uint16_t u16result = 0;
        GPIO_PinState pinState;
        
        u16result = 0;
        
        for (bit_count = 0; bit_count <= 16; bit_count++)
        {
                GPIOD->BSRR = GPIO_PIN_11; //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,0);
                u16result = (u16result << 1);
                pinState = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10); 
                GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16; //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,1);
                if ( pinState != GPIO_PIN_RESET)
                {
                        u16result = u16result | 0x01;
                }
        }
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

        encryptTxMsg(&canTxMsg,100,10,21,0,0,0);

        printDelay=100;
        err_cnt =0;
        
        HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);  
        HAL_CAN_Transmit_IT(&hcan1);

        while (1)
        {
                HAL_Delay(1);  
                //canStatus = hcan1.State;
                switch(printDelay++)
                {
                        case 1: 
                        {
                                sprintf(str6,"%d     ", lastCiclCount); 
                                lcd_PrintXY(str6,10,0); 
                                canStatus  = canRxMsg.Data[0] - data_;
                                data_ = canRxMsg.Data[0];
                                if( canStatus!=1 )
                                        err_cnt++;
                                canTxMsg.Data[0]  = canRxMsg.Data[0];                                
                        } break;
                        case 2: 
                        {
                                sprintf(str6,"%d     ", pinToggleReadSSI());//azPosition); 
                                lcd_PrintXY(str6,10,1);
                        } break;
                        case 3: 
                        {
                                sprintf(str6,"%d     ", umPosition);		
                                lcd_PrintXY(str6,10,2);
                        } break;
                        case 4: 
                        {
                                sprintf(str6,"%d     ", fvPosition);    
                                lcd_PrintXY(str6,10,3);
                                printDelay=1; 
                        } break;
                        default:
                        {
                                printDelay=1; 
                        } break;
                }										
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

	__PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
	|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

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
void MX_I2C3_Init(void)
{
        hi2c3.Instance = I2C3;
        hi2c3.Init.ClockSpeed = 80000;
        hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c3.Init.OwnAddress1 = 0xFE;
        hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
        hi2c3.Init.OwnAddress2 = 0xFE;
        hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
        hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
        if(HAL_I2C_Init(&hi2c3) != HAL_OK)
        {
                /* Initialization Error */
                Error_Handler();    
        }
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
void MX_GPIO_Init(void)
{

        GPIO_InitTypeDef GPIO_InitStruct;

        /* GPIO Ports Clock Enable */
        __GPIOE_CLK_ENABLE();
        __GPIOC_CLK_ENABLE();
        __GPIOH_CLK_ENABLE();
        __GPIOA_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();
        __GPIOD_CLK_ENABLE();

        /*Configure GPIO pin : PE3 */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /*Configure GPIO pin : PC0 */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /*Configure GPIO pin : PC3 */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /*Configure GPIO pin : PA0 */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*Configure GPIO pin : PA4 */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*Configure GPIO pins : PA5 PA6 PA7 */
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


        /*Configure GPIO pins : PB2 PB14 */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pin : PB10 */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pin : PB15 */
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pins : PD8 PD10 PD5 */
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /*Configure GPIO pins : PD9 PD11 PD12 PD13 
        PD14 PD15 PD4 */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
        |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /*Configure GPIO pins : PC7 PC10 PC12 */
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /*Configure GPIO pins : PB6 PB9 */
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pin : PE1 */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

//        /* Configure CAN RX and TX pins */
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//	GPIO_InitStruct.Pull  = GPIO_PULLUP;
//      GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

static void Error_Handler(void)
{
        while(1)
        {
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); 
                HAL_Delay(100);
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
        while(1)
        {
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14); 
                HAL_Delay(100);
        }
}

#endif

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
