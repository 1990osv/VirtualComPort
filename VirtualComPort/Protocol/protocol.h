#ifndef PROTOCOL_H
#define PROTOCOL_H

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

extern union InMsg inMsg;
extern union OutMsg outMsg;


void transfer(void);

#endif
