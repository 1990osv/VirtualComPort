#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>


enum {
        AZ=0,
        UM=1,
        FV=2  
};


void transfer(void);
void model(void);

extern uint8_t lastCiclCount;     //номер предыдущего сообщения

typedef struct {
        uint8_t canAddr;
        uint8_t noAnswerCnt;
        uint8_t speed;
        uint16_t target; 
        uint16_t position;       
        uint8_t status;
        uint8_t limit;
} Privod;

extern Privod drive[3];

void sensor_initialisation(void);
void drive_initialisation(void);
#endif
