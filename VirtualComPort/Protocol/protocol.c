#include "protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

union InMsg {
        unsigned char buf[9];
        struct StructInMsg msg;
} in;

union OutMsg {
        unsigned char buf[9];
        struct StructOutMsg msg;
} out;



void transfer(void)
{
        //while(CDC_Receive_FS(in.buf, 9) != USBD_OK);
        //CDC_Transmit_FS(out.buf, 11);
}




