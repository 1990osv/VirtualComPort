#include "stm32f4xx_hal.h"
#include "can.h"
#include "protocol.h"

#define MY_ADDRESS              0

#define USER_COMMAND_21         21





CanRxMsgTypeDef canRxMsg;
CanTxMsgTypeDef canTxMsg;

CRYPT_MSG receiveMsg;

void encryptTxMsg(CanTxMsgTypeDef *pTxMsg, uint8_t idSender, uint8_t idRecipient, uint8_t command, 
                uint8_t commandTP, uint16_t address, uint32_t parameter);
				



void setSpeed(CanTxMsgTypeDef *pTxMsg, uint32_t idSender, uint32_t idRecipient, uint32_t speeed);



void encryptTxMsgEx(CanTxMsgTypeDef *pTxMsg, uint8_t idSender, uint8_t idRecipient, uint8_t command, 
                uint8_t commandTP, uint16_t address, uint32_t parameter)
{
        pTxMsg->IDE = 0x000;
//        pTxMsg->StdId = ((command & 0x1F) << 6) 
//                    | ((idSender & 0x07) << 3) 
//                    | ((idRecipient & 0x07) << 0);

        //pTxMsg->ExtId = 0x0000;
        pTxMsg->ExtId = ((command & 0x7F) << 14) 
                       | ((idSender & 0x7F) << 7) 
                       | ((idRecipient & 0x7F) << 0);
        
        
        
        pTxMsg->RTR = CAN_RTR_DATA;
        pTxMsg->IDE = CAN_ID_EXT;//;CAN_ID_STD
        pTxMsg->DLC = 1;
        pTxMsg->Data[0] = commandTP;
        pTxMsg->Data[1] = ((address & 0xFF00) >> 8);
        pTxMsg->Data[2] = ((address & 0x00FF));
        pTxMsg->Data[6] = ((parameter & 0xFF000000) >> 24);
        pTxMsg->Data[5] = ((parameter & 0x00FF0000) >> 16);
        pTxMsg->Data[4] = ((parameter & 0x0000FF00) >> 8);
        pTxMsg->Data[3] = ((parameter & 0x000000FF) >> 0);
        pTxMsg->Data[7] = 0;        
}

void encryptTxMsg(CanTxMsgTypeDef *pTxMsg, uint8_t idSender, uint8_t idRecipient, uint8_t command, 
                uint8_t commandTP, uint16_t address, uint32_t parameter)
{
//        pTxMsg->StdId = 0x000;
        pTxMsg->StdId = ((command & 0x1F) << 6) 
                    | ((idSender & 0x07) << 3) 
                    | ((idRecipient & 0x07) << 0);

        pTxMsg->ExtId = 0x0000;
//        pTxMsg->ExtId = ((command & 0x7F) << 14) 
//                       | ((idSender & 0x7F) << 7) 
//                       | ((idRecipient & 0x7F) << 0);
        
        
        
        pTxMsg->RTR = CAN_RTR_DATA;
        pTxMsg->IDE = CAN_ID_STD;//;CAN_ID_EXT
        pTxMsg->DLC = 1;
        pTxMsg->Data[0] = commandTP;
        pTxMsg->Data[1] = ((address & 0xFF00) >> 8);
        pTxMsg->Data[2] = ((address & 0x00FF));
        pTxMsg->Data[6] = ((parameter & 0xFF000000) >> 24);
        pTxMsg->Data[5] = ((parameter & 0x00FF0000) >> 16);
        pTxMsg->Data[4] = ((parameter & 0x0000FF00) >> 8);
        pTxMsg->Data[3] = ((parameter & 0x000000FF) >> 0);
        pTxMsg->Data[7] = 0;        
}

void setSpeed(CanTxMsgTypeDef *pTxMsg, uint32_t idSender, uint32_t idRecipient, uint32_t speeed)
{
        uint32_t command;
        command = 13;
//        pTxMsg->StdId = 0x000;
        pTxMsg->StdId = ((command & 0x1F) << 6) 
                    | ((idSender & 0x07) << 3) 
                    | ((idRecipient & 0x07) << 0);

        pTxMsg->ExtId = 0x0000;
//        pTxMsg->ExtId = ((command & 0x7F) << 14) 
//                       | ((idSender & 0x7F) << 7) 
//                       | ((idRecipient & 0x7F) << 0);
//        
        pTxMsg->RTR = CAN_RTR_DATA;
        pTxMsg->IDE = CAN_ID_EXT;
        pTxMsg->DLC = 1;
        pTxMsg->Data[0] = 7;
        pTxMsg->Data[1] = (uint8_t)((0x0409 & 0x00FF) >> 0);
        pTxMsg->Data[2] = (uint8_t)((0x0409 & 0xFF00) >> 8);
        pTxMsg->Data[3] = (uint8_t)((speeed & 0x000000FF) >> 0);
        pTxMsg->Data[4] = (uint8_t)((speeed & 0x0000FF00) >> 8);
        pTxMsg->Data[5] = (uint8_t)((speeed & 0x00FF0000) >> 16);
        pTxMsg->Data[6] = (uint8_t)((speeed & 0xFF000000) >> 24);
        pTxMsg->Data[7] = 0;        
}


void decryptRxMsg(CanRxMsgTypeDef *pRxMsg, CRYPT_MSG *pCryptMsg)
{
uint8_t i;        
        pCryptMsg->idSender    = ((pRxMsg->StdId >> 3) & 0x07);
        pCryptMsg->idRecipient = ((pRxMsg->StdId >> 0) & 0x07);
        pCryptMsg->command     = ((pRxMsg->StdId >> 6) & 0x1F);
        
        i=pCryptMsg->idSender-1; 
        //pCryptMsg->idSender-1 = AZ for 1 adress azimut drive can idSender

        if(i<=FV) // AZ = 0 , i>=AZ always TRUE because i - unsigned int
        {
                drive[i].status = (pRxMsg->Data[0] >> 4) & 0x0F;
                drive[i].limit  = (pRxMsg->Data[0] >> 0) & 0x03;
        }
}

uint8_t current=AZ;
void CANtransfer(void)
{
        encryptTxMsg(&canTxMsg,MY_ADDRESS,drive[current].canAddr,USER_COMMAND_21,drive[current].speed,0,0);
        current++;
        if(current > FV)
                current = AZ;
}

