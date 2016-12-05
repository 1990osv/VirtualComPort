#include "stm32f4xx_hal.h"
#include "can.h"

CRYPT_MSG receiveMsg;
void encryptTxMsg(CanTxMsgTypeDef *pTxMsg, uint8_t idSender, uint8_t idRecipient, uint8_t command, 
                uint8_t commandTP, uint16_t address, uint32_t parameter)
{
//        pTxMsg->IDE = 0x000;
        pTxMsg->StdId = ((command & 0x1F) << 6) 
                    | ((idSender & 0x07) << 3) 
                    | ((idRecipient & 0x07) << 0);

        //pTxMsg->StdId = 1385;
        //pTxMsg->ExtId = 0x0000;
//        pTxMsg->ExtId = ((command & 0x7F) << 14) 
//                       | ((idSender & 0x7F) << 7) 
//                       | ((idRecipient & 0x7F) << 0);
//        
        
        
        pTxMsg->RTR = CAN_RTR_DATA;
        pTxMsg->IDE = CAN_ID_STD;//CAN_ID_EXT;
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
//        pTxMsg->IDE = 0x000;
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
        pCryptMsg->idSender    = (pRxMsg->StdId & 0x00000380) >> 3;
        pCryptMsg->idRecipient = (pRxMsg->StdId & 0x00000007) >> 0;     
        pCryptMsg->command     = (pRxMsg->StdId & 0x000007C0) >> 6;     

//        pCryptMsg->idSender    = (pRxMsg->ExtId & 0x00003F80) >> 7;
//        pCryptMsg->idRecipient = (pRxMsg->ExtId & 0x0000007F) >> 0;     
//        pCryptMsg->command     = (pRxMsg->ExtId & 0x001FC000) >> 14;     

        pCryptMsg->commandTP = pRxMsg->Data[0];
        
        pCryptMsg->addr = (uint16_t)(pRxMsg->Data[1] << 8 | pRxMsg->Data[2]);
        
        
        pCryptMsg->param = (uint32_t)(pRxMsg->Data[6] << 24 | pRxMsg->Data[5] << 16 | 
                                       pRxMsg->Data[4] << 8  | pRxMsg->Data[3]);
}
