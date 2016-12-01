не надо !!!!!!!!! pdev->pClassData = (void *) USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));  //pdev->pClassData = USBD_malloc(sizeof (USBD_CDC_HandleTypeDef));
#define CDC_DATA_HS_MAX_PACKET_SIZE                 64 //512  /* Endpoint IN & OUT Packet size */


my CAN address  100
спш             10, 20, 30
команда 21



Программа СПШ:


EVENT 0 + START
GET(100,21,X)
W=X-127
Z=dd11
IF(Z>0)
  SEND(100,21,Y)
ENDIF
REPEAT
ON_EVENT 0
W=0
X=127
Z=0
Y=0




EVENT 0 + START
EVENT 1 + CAN_RECV(2,21,Y)
Y=Y
REPEAT
ON_EVENT 0
W=0
ON_EVENT 1
W=X-127
EVENT 1 +

