#include <stdint.h>

#include "SSI_Sensor.h"
#include "protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define IN_MSG_SIZE             9
#define OUT_MSG_SIZE            11

#define INTERFACE_STATE         3
#define INTERFACE_STATE_CNT     4

#define MODEL_TICK_COUNT        100


uint16_t modelDelay;            //не менять тип (задержка в циклах systick)
uint8_t needRunModel;

extern  uint16_t speed;

uint8_t lastCiclCount;     //номер предыдущего сообщения

uint16_t azPosition;
uint16_t umPosition;
uint16_t fvPosition;

int8_t azVelosity;
int8_t umVelosity;
int8_t fvVelosity;

struct StructInMsg{
        unsigned char ciclCount;
        unsigned char mode;
        unsigned char azimutL;
        unsigned char azimutH;
        unsigned char angleL;
        unsigned char angleH;
        unsigned char speedL;
        unsigned char speedH;
        unsigned char crc;
} ;

struct StructOutMsg{
        unsigned char ciclCount;
        unsigned char azimutL;
        unsigned char azimutH;
        unsigned char angleL;
        unsigned char angleH;
        unsigned char phazeL;
        unsigned char phazeH;
        unsigned char stateL;
        unsigned char stateH;
        unsigned char driveState;
        unsigned char crc;
} ;

union InMsg {
        unsigned char buf[IN_MSG_SIZE];
        struct StructInMsg msg;
} in;

union OutMsg {
        unsigned char buf[OUT_MSG_SIZE];
        struct StructOutMsg msg;
} out;

SSIsensor az,um,fv;

/*Объявление статических функций*/

static void setDataFaultFlag(void);
static void resetDataFaultFlag(void);
static unsigned char crcCompute(unsigned char *data, unsigned char len);
static unsigned char crcOutCompute(unsigned char *data, unsigned char len);

void sensor_initialisation(void)
{
        az.gpioDataPort = GPIOD;
        az.gpioDataPin = GPIO_PIN_10; 
        az.gpioClkPort = GPIOD;
        az.gpioClkPin = GPIO_PIN_11;     
        az.bitCount = 16;
        az.needReadFaultBit = false;

        um.gpioDataPort = GPIOD;
        um.gpioDataPin = GPIO_PIN_8; 
        um.gpioClkPort = GPIOD;
        um.gpioClkPin = GPIO_PIN_9;     
        um.bitCount = 16;
        um.needReadFaultBit = false;
        
        fv.gpioDataPort = GPIOB;
        fv.gpioDataPin = GPIO_PIN_14; 
        fv.gpioClkPort = GPIOB;
        fv.gpioClkPin = GPIO_PIN_15;     
        fv.bitCount = 13;
        fv.needReadFaultBit = false;        

}

//скорость по интерфейсу приходит как значение [1..10]
//speed = 127 => скорость СПШ = 0 
void azModel(void)
{
int16_t azTarget;
int16_t maxVelosity;    
        readValue(&az);
        azPosition = az.code;
        if(az.fault == false)
        {
                if(in.msg.mode & 0x01)
                {
                        azTarget =  in.msg.azimutL | (in.msg.azimutH << 8);
                        maxVelosity = azTarget - azPosition;
                        maxVelosity /= 20;
                        if(maxVelosity > 100) maxVelosity = 100;
                        if(maxVelosity < -100) maxVelosity = -100; 
                        azVelosity = 127 + (uint8_t)maxVelosity;
                }
                else
                {
                        if(in.msg.mode & 0x04)
                                azVelosity = 127 + (in.msg.speedL & 0x0F) * 10;
                        else if(in.msg.mode & 0x08)
                                azVelosity = 127 - (in.msg.speedL & 0x0F) * 10;
                        else 
                                azVelosity = 127;
                }
        }
        else
        {
                azVelosity = 127;
        }
        out.msg.azimutL = azPosition & 0xFF;
        out.msg.azimutH = (azPosition >> 8) & 0xFF;
}


void umModel(void)
{
int16_t umTarget;
int16_t maxVelosity;  
        readValue(&um); 
        umPosition = um.code;
        if(in.msg.mode & 0x02)
        {
                umTarget =  in.msg.angleL | (in.msg.angleH << 8);
                maxVelosity = umTarget - umPosition;
                maxVelosity /= 20;
                if(maxVelosity > 100) maxVelosity = 100;
                if(maxVelosity < -100) maxVelosity = -100; 
                umVelosity = 127 + (uint8_t)maxVelosity;
        }
        else
        {
                if(in.msg.mode & 0x10)
                {
                        umVelosity = 127 + (in.msg.speedL >> 4) * 10;
                }
                else if(in.msg.mode & 0x20)
                {
                        umVelosity = 127 - (in.msg.speedL >> 4) * 10;
                }
                else
                        umVelosity = 127;
        }
        out.msg.angleL = umPosition & 0xFF;
        out.msg.angleH = (umPosition >> 8) & 0xFF;
}

void fvModel(void)
{
        readValue(&fv); 
        fvPosition = fv.code;
        if(in.msg.mode & 0x40)
        {
                fvVelosity = 127 + (in.msg.speedH & 0x0F) * 10;
        }
        else if(in.msg.mode & 0x80)
        {
                fvVelosity = 127 - (in.msg.speedH & 0x0F) * 10;
        }
        else
                fvVelosity = 127;
        out.msg.phazeL = fvPosition & 0xFF;
        out.msg.phazeH = (fvPosition >> 8) & 0xFF;
}

void tickModel(void)
{
        if(++modelDelay > MODEL_TICK_COUNT)
        {
                modelDelay=0;
                needRunModel = 1;               
        }
}

void model(void)
{
        needRunModel = 0;
        azModel();
        umModel();
        fvModel();
}


void setDataFaultFlag(void)
{
        out.msg.stateL |= (1<<(INTERFACE_STATE));
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);        
}

void resetDataFaultFlag(void)
{
        out.msg.stateL &= (~(1<<(INTERFACE_STATE)));
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
}

void transfer(void)
{
        uint32_t Len = IN_MSG_SIZE;
        uint8_t inMsgCrc;     //контрольная сумма принятого сообщения
        if (CDC_Receive_FS(in.buf, &Len) == USBD_OK)
        {
                inMsgCrc = crcCompute(in.buf, IN_MSG_SIZE - 1);
                if(lastCiclCount != in.msg.ciclCount)
                {
                        lastCiclCount = in.msg.ciclCount;
                        out.msg.ciclCount = in.msg.ciclCount;

                        if ( inMsgCrc == in.msg.crc )
                        {
                                resetDataFaultFlag();                     
                        }
                        else
                        {
                                setDataFaultFlag();
                        }
                        out.msg.crc = crcOutCompute(out.buf,OUT_MSG_SIZE - 1);
                        CDC_Transmit_FS(out.buf, OUT_MSG_SIZE);        
                }
        }
}

unsigned char crcCompute(unsigned char *data, unsigned char len)
{
        volatile unsigned char i, crcComp;
       
        crcComp = data[0] - 1;
        for (i = 1; i < len; i++)
        {
                crcComp ^= data[i];
        }
        return crcComp;
}

unsigned char crcOutCompute(unsigned char *data, unsigned char len)
{
        volatile unsigned char i, crcComp;
       
        crcComp = data[0];
        for (i = 1; i < len; i++)
        {
                crcComp ^= data[i];
        }
        return crcComp;
}

