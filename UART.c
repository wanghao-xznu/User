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
1�����������ťMSGC����λ���趨ʱ�䶼���ͳ�ȥ
2����Ƭ���յ�������Ϣ��������ʱ������ÿ�뷢��һ�� ���������Ϣ��
3����Ƭ����Ҫ ���� ��λ���趨ʱ�䣬��ǰ����ʱ�䣬
4���Ժ�ʲô���ݶ��ǵ�Ƭ�����ͳ�ȥ�ġ�
5�����MSGC����Ƭ��������ͣ�źţ���Ƭ����Ҫֹͣ��
6������һ�����н�������ݽṹ��ûһ�ν����µ����н��棬����ṹͼ�����¸���һ�Σ����µ�ʱ����ǽ��ܵ�MSGC��������Ϣ��
7�����ݽṹ�е�ÿ��Ԫ����ϢӦ�ö���ת�����ַ�������Ҫ��Ϊ�˽�����ʾ��
8��keil�е��ַ����ȽϺ��������ߵ��������Ƚϰɣ�
9���ܲ��ܵ�Ƭ��һ�뷢��һ�Σ��Ǳ߾ͼ�һ��
*/

void USART1_IRQHandler(void)//���ﲻȷ��MSGC�ᷢ��ʲô�����Խ��ܵ��ֽ�����ȷ��
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		/*���Ӵ�������*/
		USART_RX_BUF[index_receive++]= USART_ReceiveData(USART1);
		xTimerStop(Receive_Timer,0);
		xTimerReset(Receive_Timer, 0);
		xTimerStart(Receive_Timer, 0);	
		//printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}
}

