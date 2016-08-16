#include "protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define IN_MSG_SIZE             9
#define OUT_MSG_SIZE            11

#define INTERFACE_STATE         3
#define INTERFACE_STATE_CNT     4

uint8_t lastCiclCount;     //номер предыдущего сообщения
uint8_t inMsgCrc;     //контрольная сумма принятого сообщения

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


unsigned char crcCompute(unsigned char *data, unsigned char len);

void setDataFaultFlag(void)
{
        out.msg.stateL |= (1<<(INTERFACE_STATE));
}

void resetDataFaultFlag(void)
{
        out.msg.stateL &= (~(1<<(INTERFACE_STATE)));
}

void transfer(void)
{
        uint32_t Len = IN_MSG_SIZE;

        if (CDC_Receive_FS(in.buf, &Len) == USBD_OK)
        {
                inMsgCrc = crcCompute(in.buf, IN_MSG_SIZE - 1) - 1;
                if ((lastCiclCount != in.msg.ciclCount) && ( inMsgCrc == in.msg.crc))
                {
                        lastCiclCount = in.msg.ciclCount;
                        out.msg.ciclCount = in.msg.ciclCount;
                        out.msg.azimutL = in.msg.crc;
                        out.msg.azimutH = inMsgCrc;
                        out.msg.crc = crcCompute(out.buf,OUT_MSG_SIZE - 1);
                        CDC_Transmit_FS(out.buf, OUT_MSG_SIZE);
                        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);                        

//                        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
                }
//                else
//                {
//                        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
//                }
                        
        }
}

unsigned char crcCompute(unsigned char *data, unsigned char len)
{
        unsigned char i, crcComp;
        crcComp = data[0];
        for (i = 1; i < len; i++)
        {
                crcComp ^= data[i];
        }
        return crcComp;
}



