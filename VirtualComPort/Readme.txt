не надо !!!!!!!!! pdev->pClassData = (void *) USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));  //pdev->pClassData = USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));
#define CDC_DATA_HS_MAX_PACKET_SIZE                 64 //512  /* Endpoint IN & OUT Packet size */


my CAN address  0
спш             1, 2, 3
команда 21



Программа СПШ:

W>0 P_IN.0=0 =

IF(Y=0)
  up0=0
ENDIF
Y=Y+1
IF(Y=10000)
  Y=up0 + 1
  up0= Y
  SEND(5,21,Y)
  Y=1
ENDIF
IF(CAN_RECV(5,21,X))
  Z=127
  up1=Z-X
  Z=up1*7
  X=0
  up2=X-Z
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


IF(Y=0)
  up0=0
ENDIF
Y=Y+1
IF(Y=10000)
  Y=up0+1
  up0=Y
  SEND(0,21,Y)
  Y=1
ENDIF
IF(CAN_RECV(0,21,X))
  Z=127
  up1=X-Z
  Z=up1
  X=0
  up2=X-Z
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

