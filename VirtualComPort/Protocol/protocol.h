#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

void transfer(void);
void tickModel(void);
void model(void);

extern uint8_t needRunModel;


extern uint8_t lastCiclCount;     //номер предыдущего сообщения

extern uint16_t azPosition;
extern uint16_t umPosition;
extern uint16_t fvPosition;


#endif
