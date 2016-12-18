#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#include "SSI_Sensor.h"

#define ALARM_DELAY_MS  2000

typedef struct {
        uint8_t canAddr;
        uint8_t noAnswerCnt;
        uint8_t speed;
        uint16_t target; 
        uint16_t position;       
        uint8_t status;
        uint8_t limit;
} Privod;

enum {
        AZ=0,
        UM=1,
        FV=2,
        AXIS_COUNT
};

extern Privod drive[AXIS_COUNT];
extern SSIsensor sensor[AXIS_COUNT];
extern uint16_t alarmStopCnt;   //счетчик времени. 
                                //если от компьютера не приходят сообщения остановить привода


extern bool alarmStop;
extern bool alarmDelayStop;

extern uint8_t lastCiclCount;     //номер предыдущего сообщения


void transfer(void);
void model(void);




void sensor_initialisation(void);
void drive_initialisation(void);
#endif
