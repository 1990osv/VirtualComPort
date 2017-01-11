#ifndef __SSI_SENSOR_H
#define __SSI_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"



typedef struct
{
//out parameters
        double angle;
        uint16_t code;
        bool fault;
        bool needInvert;
//settings
        uint8_t bitCount;
        uint16_t mask;
        bool needReadFaultBit;
        GPIO_TypeDef* gpioClkPort;
        uint16_t gpioClkPin;
        GPIO_TypeDef* gpioDataPort;
        uint16_t gpioDataPin;

} SSIsensor;

//return true or false sensor state
bool readValue(SSIsensor* s);

void DWT_Init(void);

#endif /* __SSI_SENSOR_H */
