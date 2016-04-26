

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "freeRTOS.h"
#include "task.h"
#include "queue.h"

#define USART_REC_LEN 64  
 /* event */
#define START_EVENT          0x01
#define STOP_EVENT           0x02
#define ALL_MESSAGE_EVENT           0x03

extern xTimerHandle Receive_Timer;
extern xTimerHandle Updata_Timer;
	 
typedef struct {
int Setting_Time;                     
int Current_Time;
int Current_duanwei;
int Setting_Temperature;
int Current_Temperature;
int Setting_Power;
int Current_Power;
} Running_Message;




#ifdef __cplusplus
}
#endif

#endif  /* _BOARD_H_ */

