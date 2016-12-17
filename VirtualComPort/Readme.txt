не надо !!!!!!!!! pdev->pClassData = (void *) USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));  //pdev->pClassData = USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));
#define CDC_DATA_HS_MAX_PACKET_SIZE                 64 //512  /* Endpoint IN & OUT Packet size */


my CAN address  0
спш             1, 2, 3
команда 21



Программа СПШ:

IF(CAN_RECV(0,21,X))
  IF(X>0)
    Z=127
    up1=X-Z
    Z=up1
    X=0
    up2=X-Z
  ELSE
    up2=0
  ENDIF
  Y=dd11
  Y=Y*16
  Z=PORT&15
  Y=Y|Z
  SEND(5,21,Y)
ENDIF
IF(up2>0)
  IF(P_IN.0=0)
    up2=0
  ENDIF
ENDIF
IF(up2<0)
  IF(P_IN.1=0)
    up2=0
  ENDIF
ENDIF
W=up2
REPEAT

