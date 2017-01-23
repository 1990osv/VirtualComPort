#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include <stdint.h>
#include "SSI_Sensor.h"

/**
  * @brief Read SSI sensor value
  * @param  
  * @retval
  */
bool readValue(SSIsensor* s)
{
        uint8_t bitCount;
        uint16_t u16result;
        uint16_t sensorMask; // mask equal bitCount
        int16_t d;
        GPIO_PinState pinState;
        
        u16result = 0;
        sensorMask = 0;
        if(s->needReadFaultBit)
        {
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_RESET);
                d=50;while(d>0)d-=1;
                s->fault = HAL_GPIO_ReadPin(s->gpioDataPort, s->gpioDataPin);
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_SET);
        }
        for (bitCount = 0; bitCount <= s->bitCount; bitCount++)
        {
                HAL_GPIO_WritePin(s->gpioClkPort, s->gpioClkPin,GPIO_PIN_RESET);
                d=50;while(d>0)d-=1;
                pinState = HAL_GPIO_ReadPin(s->gpioDataPort, s->gpioDataPin);
                HAL_GPIO_WritePin(s->gpioDataPort, s->gpioClkPin,GPIO_PIN_SET);
                if ( pinState != GPIO_PIN_RESET)
                {
                        u16result = u16result | 0x01;
                }
                sensorMask = sensorMask | 0x01;
                sensorMask = (sensorMask << 1);
                u16result = (u16result << 1);
        }
        s->code = u16result & s->mask;
        if(s->needInvert)
                s->code = ~(s->code);
        s->angle = (double)s->code * (360.0 / (double)s->mask) - 180.0;
        return s->fault;
}
