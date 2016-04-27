/*******************************************************************************
** 文件名:      mian.c
** 版本：       2.0
** 工作环境:    RealView MDK-ARM 4.1.0
** 作者:
** 生成日期:    2013-07-25
** 功能:        USART_Printf
** 相关文件:    无
** 修改日志：   2013-07-25   创建文档
*******************************************************************************/



#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Library includes. */
#include "bilang.h"

ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void UART1Write(u8 *data, u16 len);
void UART1_SendByte(u16 Data);
void Receive_TimeOut(xTimerHandle handle);
void Updata_TimeOut(xTimerHandle handle);

xQueueHandle MsgQueue;
xQueueHandle CmdMsg;
xTimerHandle Receive_Timer = NULL;
xTimerHandle Updata_Timer = NULL;

Running_Message all_message;
static void prvSetupHardware( void );
void TaskA( void *pvParameters );
void TaskB( void *pvParameters );

extern char USART_RX_BUF[USART_REC_LEN];
char receive_buf[USART_REC_LEN];
extern unsigned int index_receive;


int main(void)
{
    /* 初始化硬件平台 */
    prvSetupHardware();

    printf("\r\n Welcome to ,Hello WH \r\n");
    printf("test = %d\n", atoi("00123"));
    printf("\r\n =========line = %d======%s====\r\n", __LINE__, __FILE__);
    /* create sw timer, 2s */
    Receive_Timer = xTimerCreate((const char *)"ReceiveTimer", \
                                 20 / portTICK_RATE_MS, pdFALSE, NULL, Receive_TimeOut);

    Updata_Timer = xTimerCreate((const char *)"UpdataTimer", \
                                1000 / portTICK_RATE_MS, pdTRUE, NULL, Updata_TimeOut);
    /* 建立队列 */
    MsgQueue = xQueueCreate( USART_REC_LEN , sizeof(char) );
    CmdMsg   = xQueueCreate( 5 , sizeof( int16_t ) );
    /* 建立任务 */
    xTaskCreate( TaskA, "TaskA", 1024,
                 NULL, tskIDLE_PRIORITY + 3, NULL );
    xTaskCreate( TaskB, "TaskB", 1024,
                 NULL, tskIDLE_PRIORITY + 4, NULL );
    /* 启动OS */
    vTaskStartScheduler();

    return 0;



}

void Receive_TimeOut(xTimerHandle handle) //20ms的timeout，时间到了将接收的字符串吐出
{
    //printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
    char event = 0;
    USART_RX_BUF[index_receive] = '\0';
    //printf("\r\n =========line = %d======%s====\r\n",__LINE__,USART_RX_BUF);

    if (strcmp(USART_RX_BUF, "start") == 0)
    {
        //printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
        event = START_EVENT;
    }
    if (strcmp(USART_RX_BUF, "stop") == 0)
    {
        //printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
        event = STOP_EVENT;
    }
    if ((USART_RX_BUF[0] == '#') && (USART_RX_BUF[0] == '#'))
    {
        //printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
        event = ALL_MESSAGE_EVENT;
        memcpy(receive_buf, USART_RX_BUF, USART_REC_LEN);
    }
    xQueueSend( CmdMsg, ( void *)&event, 0 );
    index_receive = 0;
    memset(USART_RX_BUF, 0, USART_REC_LEN * sizeof(char));
    return;
}

void Updata_TimeOut(xTimerHandle handle) //20ms的timeout，时间到了将接收的字符串吐出
{
    //printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
    return;
}










#ifdef  USE_FULL_ASSERT
/*******************************************************************************
  * @函数名称   assert_failed
  * @函数说明   报告在检查参数发生错误时的源文件名和错误行数
  * @输入参数   file: 源文件名
                line: 错误所在行数
  * @输出参数   无
  * @返回参数   无
  *****************************************************************************/
void assert_failed(uint8_t *file, uint32_t line)
{
    while (1)
    {
    }
}
#endif


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
#if 0
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        //RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
#endif
    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures the USART1.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef  USART_ClockInitStructure;
    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
    /* Configure the USART1 synchronous paramters */
    USART_ClockInit(USART1, &USART_ClockInitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl =

        USART_HardwareFlowControl_None;


    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Configure USART1 basic and asynchronous paramters */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
}

/*******************************************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART1, (u8) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }

    return ch;
}

/******************* 向串口写字符串*************/






/*------------------------这个任务1秒发送一次显示状态-----------------------------------*/
void TaskA( void *pvParameters )
{
    int16_t SendNum = 1;
    //xTimerStart(Receive_Timer, 0);
    for ( ;; )
    {
        vTaskDelay( 1000 / portTICK_RATE_MS );
        /* 向队列中填充内容 */
        xQueueSend( MsgQueue, ( void *)&SendNum, 0 );
        SendNum++;
    }
}
/*------------------------这个任务作为接受消息队列使用，类似小蜜蜂用法-----------------------------------*/
void TaskB( void *pvParameters )
{
    char event = 0;
    char fen_string[3];
    char miao_string[3];
    for ( ;; )
    {
        /* 从队列中获取内容 */
        if ( xQueueReceive( CmdMsg, &event, 100 / portTICK_RATE_MS ) == pdPASS)
        {
            if (event == START_EVENT)
            {
                printf("\r\n =========line = %d======%s====\r\n", __LINE__, __FILE__);
                xTimerStart(Updata_Timer, 0);
            }
            if (event == STOP_EVENT)
            {
                printf("\r\n =========line = %d======%s====\r\n", __LINE__, __FILE__);
                xTimerStop(Updata_Timer, 0);
            }
            if (event == ALL_MESSAGE_EVENT)
            {
                //printf("\r\n =========line = %d======%s====\r\n",__LINE__,__FILE__);
                //printf("\r\n =========line = %d======%s====\r\n",__LINE__,receive_buf);
                //将接收的一坨数据处理送给结构体
                fen_string[0] = receive_buf[2];
                fen_string[1] = receive_buf[3];
                fen_string[2] = '\0';
                miao_string[0] = receive_buf[4];
                miao_string[1] = receive_buf[5];
                miao_string[2] = '\0';

                all_message.Setting_Time = 60 * atoi(fen_string) + atoi(miao_string);
                all_message.Current_duanwei = 0;
                all_message.Setting_Power = 0;
                all_message.Setting_Temperature = 0;
                all_message.Current_Temperature = 0;
                all_message.Current_Power = 0;
                all_message.Current_Time = 0;
                printf("\r\n =========line = %d======%d====\r\n", __LINE__, all_message.Setting_Time);
                memset(receive_buf, 0, USART_REC_LEN * sizeof(char));

            }
        }
    }
}
static void prvSetupHardware( void )
{
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    USART_Configuration();
}
