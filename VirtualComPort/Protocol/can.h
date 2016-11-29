#ifndef __CAN_H
#define __CAN_H

typedef struct CRYPT_MSG
{
    uint32_t idSender;
    uint32_t idRecipient;
    uint32_t command;
    uint8_t commandTP;
    uint16_t addr;
    uint32_t param;
} CRYPT_MSG;


extern CRYPT_MSG receiveMsg;
void encryptTxMsg(CanTxMsgTypeDef *pTxMsg, uint8_t idSender, uint8_t idRecipient, uint8_t command, 
                uint8_t commandTP, uint16_t address, uint32_t parameter);
				
void decryptRxMsg(CanRxMsgTypeDef *pRxMsg, CRYPT_MSG *pCryptMsg);


void setSpeed(CanTxMsgTypeDef *pTxMsg, uint32_t idSender, uint32_t idRecipient, uint32_t speeed);

#endif

