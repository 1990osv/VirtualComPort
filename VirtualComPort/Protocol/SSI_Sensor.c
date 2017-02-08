#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include <stdint.h>
#include "SSI_Sensor.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

GPIO_PinState pinState;

void DWT_Init(void)
{
        //разрешаем использовать счётчик
        SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        //обнуляем значение счётного регистра
        DWT_CYCCNT  = 0;
        //запускаем счётчик
        DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk; 
}

static __inline uint32_t delta(uint32_t t0, uint32_t t1)
{
        return (t1 - t0); 
}
void SSI_delay_us(uint32_t us)
{
        uint32_t t0 =  DWT->CYCCNT;
        uint32_t us_count_tic =  us * (SystemCoreClock/1000000);
        while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

void SSI_delay_01us(uint32_t us)
{
        uint32_t t0 =  DWT->CYCCNT;
        uint32_t us_count_tic =  us * (SystemCoreClock/10000000);
        while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

/**
  * @brief Read SSI sensor value
  * @param  none
  * @retval none
  */
bool readValue(SSIsensor* s)
{
        uint8_t bitCount;
        uint16_t u16result;
        GPIO_PinState pinState;

        u16result = 0;

        HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_RESET);
        SSI_delay_01us(3);
        //pinState = HAL_GPIO_ReadPin(s->gpioDataPort, s->gpioDataPin);
        HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_SET);
        SSI_delay_01us(3);
        
        if(s->needReadFaultBit)
        {
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_RESET);
                SSI_delay_01us(3);
                s->fault = HAL_GPIO_ReadPin(s->gpioDataPort, s->gpioDataPin);
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_SET);
                SSI_delay_01us(3);
        }
        for (bitCount = 0; bitCount < s->bitCount; bitCount++)
        {
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_RESET);
                SSI_delay_01us(3);
                pinState = HAL_GPIO_ReadPin(s->gpioDataPort, s->gpioDataPin);
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_SET);
                SSI_delay_01us(3);
                
                u16result = (u16result << 1);                
                if ( pinState != GPIO_PIN_RESET)
                {
                        u16result = u16result | 0x01;
                }
        }
        s->code = u16result & s->mask;
        if(s->needInvert)
                s->code = ~(s->code);
        s->angle = (double)s->code * (360.0 / (double)s->mask) - 180.0;
        return s->fault;
}
