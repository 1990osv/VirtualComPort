/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "protocol.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef    htim4;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  tickModel(); 
  
	transfer();
	if(needRunModel) 
		model();
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
/**
* @brief This function handles CAN1 RX0 interrupt.
*/

extern         uint32_t printDelay;
uint32_t        time1,time2,gt1,gt2,gt3;

uint8_t crc, canStatus=0;
uint8_t data_,err_cnt=0;

//HAL_GetTick
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
        canStatus=0;        
        gt3=HAL_GetTick();
        time2 = gt3 - gt2;
        HAL_CAN_IRQHandler(&hcan1);
        HAL_NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);    
        //HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE END CAN1_RX0_IRQn 0 */
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13); 

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

void CAN1_TX_IRQHandler(void)
{
        gt1=HAL_GetTick();
        time1 = gt1 - gt2;
        gt2 = gt1;
        HAL_CAN_IRQHandler(&hcan1);
        HAL_NVIC_ClearPendingIRQ(CAN1_TX_IRQn); 
        HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
        //HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */



extern CanRxMsgTypeDef canRxMsg;
extern CanTxMsgTypeDef canTxMsg;
void TIM4_IRQHandler(void)
{
        crc  = canRxMsg.Data[0] - data_;
        data_ = canRxMsg.Data[0];
        if( crc!=1 )
                err_cnt++;
        canTxMsg.Data[0]  = canRxMsg.Data[0]; 
  
        if((canStatus==0) || (canStatus>10))
        {
                //HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
                HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
                HAL_CAN_Transmit_IT(&hcan1);   
                 
        }
        canStatus++;
        HAL_TIM_IRQHandler(&htim4);
        HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);         
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        HAL_GPIO_TogglePin(GPIOD,LD4_Pin);         
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
