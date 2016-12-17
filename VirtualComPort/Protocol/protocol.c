#include <stdint.h>
#include <stdbool.h>
#include "SSI_Sensor.h"
#include "protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define IN_MSG_SIZE             9
#define OUT_MSG_SIZE            11

#define INTERFACE_STATE         3
#define INTERFACE_STATE_CNT     4

#define AZ_MAX_ANGLE            150.0
#define AZ_MIN_ANGLE            -150.0

#define UM_MAX_ANGLE            35.0
#define UM_MIN_ANGLE            -15.0

#define FV_MAX_ANGLE            100.0
#define FV_MIN_ANGLE            100.0

uint16_t modelDelay;            //не менять тип (задержка в циклах systick)

uint8_t lastCiclCount;     //номер предыдущего сообщения

uint8_t cntErrorCRC;

uint16_t alarmStopCnt;  //счетчик времени. 
                        //если от компьютера не приходят сообщения остановить привода
bool alarmDelayStop;    //alarmStopCnt досчитал до ALARM_DELAY_MS

bool alarmStop;         //пришло 20 пакетов и CRC не совпали


struct StructInMsg{
        uint8_t ciclCount;
        uint8_t mode;
        uint8_t azimutL;
        uint8_t azimutH;
        uint8_t angleL;
        uint8_t angleH;
        uint8_t speedL;
        uint8_t speedH;
        uint8_t crc;
} ;

struct StructOutMsg{
        uint8_t ciclCount;
        uint8_t azimutL;
        uint8_t azimutH;
        uint8_t angleL;
        uint8_t angleH;
        uint8_t phazeL;
        uint8_t phazeH;
        uint8_t stateL;
        uint8_t stateH;
        uint8_t driveState;
        uint8_t crc;
} ;

union InMsg {
        uint8_t buf[IN_MSG_SIZE];
        struct StructInMsg msg;
} in;

union OutMsg {
        uint8_t buf[OUT_MSG_SIZE];
        struct StructOutMsg msg;
} out;

SSIsensor azSensor,umSensor,fvSensor;

Privod drive[DRIVE_COUNT];

/*Объявление статических функций*/

static void setDataFaultFlag(void);
static void resetDataFaultFlag(void);
static void setDataFaultCntFlag(void);
static void resetDataFaultCntFlag(void);
static uint8_t crcCompute(uint8_t *data, uint8_t len);
static uint8_t crcOutCompute(uint8_t *data, uint8_t len);

void sensor_initialisation(void)
{
        azSensor.gpioDataPort = GPIOD;
        azSensor.gpioDataPin = GPIO_PIN_10; 
        azSensor.gpioClkPort = GPIOD;
        azSensor.gpioClkPin = GPIO_PIN_11;     
        azSensor.bitCount = 16;
        azSensor.needReadFaultBit = false;

        umSensor.gpioDataPort = GPIOD;
        umSensor.gpioDataPin = GPIO_PIN_8; 
        umSensor.gpioClkPort = GPIOD;
        umSensor.gpioClkPin = GPIO_PIN_9;     
        umSensor.bitCount = 16;
        umSensor.needReadFaultBit = true;
        
        fvSensor.gpioDataPort = GPIOB;
        fvSensor.gpioDataPin = GPIO_PIN_14; 
        fvSensor.gpioClkPort = GPIOB;
        fvSensor.gpioClkPin = GPIO_PIN_15;     
        fvSensor.bitCount = 13;
        fvSensor.needReadFaultBit = true;        
}


void drive_initialisation(void)
{
        drive[AZ].canAddr = 1;
        drive[AZ].noAnswerCnt = 0;
        drive[AZ].speed = 127;
        
        drive[UM].canAddr = 2;
        drive[UM].noAnswerCnt = 0;
        drive[UM].speed = 127;
        
        drive[FV].canAddr = 3;
        drive[FV].noAnswerCnt = 0;
        drive[FV].speed = 127;
}

//скорость по интерфейсу приходит как значение [1..10]
//speed = 127 => скорость СПШ = 0 
void azModel(void)
{
int16_t velosity, maxVelosity;    
        
        readValue(&azSensor);
        drive[AZ].position = azSensor.code;

        if(in.msg.mode & 0x01)
        {
                if((in.msg.speedH & 0x20) && (azSensor.fault == false))
                {
                        drive[AZ].target =  in.msg.azimutL | (in.msg.azimutH << 8);
                        maxVelosity = drive[AZ].target - drive[AZ].position;
                        if(maxVelosity > 100) maxVelosity = 100;
                        if(maxVelosity < -100) maxVelosity = -100; 
                        velosity = 127 + (uint8_t)maxVelosity;
                }
                else
                        velosity = 127;
        }
        else
        {
                if(in.msg.mode & 0x04)
                        velosity = 127 + (in.msg.speedL & 0x0F) * 10;
                else if(in.msg.mode & 0x08)
                        velosity = 127 - (in.msg.speedL & 0x0F) * 10;
                else 
                        velosity = 127;
        }
#ifdef DEBUG_NOT_CONNECT_ENCODER
        if( (azSensor.angle >= AZ_MAX_ANGLE) || (azSensor.angle <= AZ_MIN_ANGLE) )
                velosity = 127;
#endif
        drive[AZ].speed = velosity;
        out.msg.azimutL = drive[AZ].position & 0xFF;
        out.msg.azimutH = (drive[AZ].position >> 8) & 0xFF;
        
}


void umModel(void)
{
int16_t velosity, maxVelosity;  
        readValue(&umSensor); 
        drive[UM].position = umSensor.code;
        if(in.msg.mode & 0x02)
        {
                if((in.msg.speedH & 0x40) && (umSensor.fault == false))
                {
                        drive[UM].target =  in.msg.angleL | (in.msg.angleH << 8);
                        maxVelosity = drive[UM].target - drive[UM].position;
                        if(maxVelosity > 100) maxVelosity = 100;
                        if(maxVelosity < -100) maxVelosity = -100; 
                        velosity = 127 + (uint8_t)maxVelosity;
                }
                else
                        velosity = 127;                        
        }
        else
        {
                if(in.msg.mode & 0x10)
                {
                        velosity = 127 + (in.msg.speedL >> 4) * 10;
                }
                else if(in.msg.mode & 0x20)
                {
                        velosity = 127 - (in.msg.speedL >> 4) * 10;
                }
                else
                        velosity = 127;
        }
#ifdef DEBUG_NOT_CONNECT_ENCODER        
        if( (umSensor.angle >= UM_MAX_ANGLE) || (umSensor.angle <= UM_MIN_ANGLE) )
                velosity = 127;
#endif
        drive[UM].speed = velosity;
        
        
        out.msg.angleL = drive[UM].position & 0xFF;
        out.msg.angleH = (drive[UM].position >> 8) & 0xFF;
        
       
}

void fvModel(void)
{
int16_t velosity;
        readValue(&fvSensor); 
        drive[FV].position = fvSensor.code;
        if(in.msg.mode & 0x40)
        {
                velosity = 127 + (in.msg.speedH & 0x0F) * 10;
        }
        else if(in.msg.mode & 0x80)
        {
                velosity = 127 - (in.msg.speedH & 0x0F) * 10;
        }
        else
                velosity = 127;
#ifdef DEBUG_NOT_CONNECT_ENCODER
        if( (fvSensor.angle >= FV_MAX_ANGLE) || (fvSensor.angle <= FV_MIN_ANGLE) )
                velosity = 127;
#endif
        drive[FV].speed = velosity;
        
        out.msg.phazeL = drive[FV].position & 0xFF;
        out.msg.phazeH = (drive[FV].position >> 8) & 0xFF;

}


void model(void)
{
        if((!alarmStop) && (!alarmDelayStop)){
                azModel();
                umModel();
                fvModel();
        }
        else{
                drive[AZ].speed = 127;
                drive[UM].speed = 127;
                drive[FV].speed = 127;
        }
}

void setDataFaultFlag(void)
{
        out.msg.stateL |= (1<<(INTERFACE_STATE));
}

void resetDataFaultFlag(void)
{
        out.msg.stateL &= (~(1<<(INTERFACE_STATE)));
}

void setDataFaultCntFlag(void)
{
        out.msg.stateL |= (1<<(INTERFACE_STATE_CNT));
}

void resetDataFaultCntFlag(void)
{
        out.msg.stateL &= (~(1<<(INTERFACE_STATE_CNT)));
}

void transfer(void)
{
        uint32_t Len = IN_MSG_SIZE;
        uint8_t inMsgCrc;     //контрольная сумма принятого сообщения
        if (CDC_Receive_FS(in.buf, &Len) == USBD_OK)
        {
                inMsgCrc = crcCompute(in.buf, IN_MSG_SIZE - 1);
                if(lastCiclCount != in.msg.ciclCount) // приняли новое сообщение от компьютера
                {
                        alarmStopCnt = 0;
                        lastCiclCount = in.msg.ciclCount;
                        out.msg.ciclCount = in.msg.ciclCount;

                        if ( inMsgCrc == in.msg.crc )
                        {
                                resetDataFaultFlag();
                                resetDataFaultCntFlag();
                                cntErrorCRC = 0;
                        }
                        else
                        {
                                setDataFaultFlag();
                                cntErrorCRC++;
                        }
                        if(cntErrorCRC>20){
                                alarmStop = true;
                                setDataFaultCntFlag();
                        }
                        out.msg.crc = crcOutCompute(out.buf,OUT_MSG_SIZE - 1);
                        
                        out.msg.stateL &= ~(0x07); // обнулили статус готовности всех 3-х приводов
                        
                        if(drive[AZ].status)
                                out.msg.stateL |= 0x01;
                        if(drive[UM].status)
                                out.msg.stateL |= 0x02;
                        if(drive[FV].status)
                                out.msg.stateL |= 0x04;
                        
                        out.msg.stateH = drive[AZ].limit | 
                                        (drive[UM].limit << 2)| 
                                        (drive[FV].limit << 4);
                        
                        CDC_Transmit_FS(out.buf, OUT_MSG_SIZE);        
                }
        }
}

uint8_t crcCompute(uint8_t *data, uint8_t len)
{
        volatile uint8_t i, crcComp;
       
        crcComp = data[0] - 1;
        for (i = 1; i < len; i++)
        {
                crcComp ^= data[i];
        }
        return crcComp;
}

uint8_t crcOutCompute(uint8_t *data, uint8_t len)
{
        volatile uint8_t i, crcComp;
       
        crcComp = data[0];
        for (i = 1; i < len; i++)
        {
                crcComp ^= data[i];
        }
        return crcComp;
}

