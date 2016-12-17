#ifndef __CAN_H
#define __CAN_H


void CANtransfer(void);
void drive_initialisation(void);

typedef struct 
{
    uint32_t idSender;
    uint32_t idRecipient;
    uint32_t command;
    uint8_t commandTP;
    uint16_t addr;
    uint32_t param;
} CRYPT_MSG;

extern CanRxMsgTypeDef canRxMsg;
extern CanTxMsgTypeDef canTxMsg;

extern CRYPT_MSG receiveMsg;

void decryptRxMsg(CanRxMsgTypeDef *pRxMsg, CRYPT_MSG *pCryptMsg);

#endif

