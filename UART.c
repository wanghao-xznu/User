#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "bilang.h"

char USART_RX_BUF[USART_REC_LEN];
unsigned int index_receive=0;


/*
1、点击启动按钮MSGC将段位，设定时间都发送出去
2、单片机收到启动信息后，启动定时器，并每秒发送一次 第三点的信息。
3、单片机需要 保存 段位，设定时间，当前运行时间，
4、以后什么数据都是单片机发送出去的。
5、如果MSGC给单片机发送暂停信号，单片机需要停止。
6、定义一个运行界面的数据结构，没一次进入新的运行界面，这个结构图都重新更新一次，更新的时间点是接受到MSGC的启动信息。
7、数据结构中的每个元素信息应该都能转换成字符串，主要是为了接受显示。
8、keil中的字符串比较函数，或者单个单个比较吧！
9、能不能单片机一秒发送一次，那边就减一次
*/

void USART1_IRQHandler(void)//这里不确认MSGC会发来什么，所以接受的字节数不确定
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		/*增加处理流程*/
		USART_RX_BUF[index_receive++]= USART_ReceiveData(USART1);
		xTimerStop(Receive_Timer,0);
		xTimerReset(Receive_Timer, 0);
		xTimerStart(Receive_Timer, 0);	
		//printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}
}

