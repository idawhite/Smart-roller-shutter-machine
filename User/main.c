#include "debug.h"
#include "lcd.h"
#include "AP3216C.h"
#include "IIC.h"

/* Global define */
#define RXBUF_SIZE 1024     // DMA buffer size
#define size(a)   (sizeof(a) / sizeof(*(a)))

/* Global Variable */
u8 TxBuffer[] = " ";
u8 RxBuffer[RXBUF_SIZE]={0};


/*******************************************************************************
* Function Name  : USARTx_CFG
* Description    : Initializes the USART peripheral.
* 描述    ：   串口初始化
* Input          : None
* Return         : None
*******************************************************************************/
void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    //开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART7 TX-->C2  RX-->C3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX，输入上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* USART2 TX-->A2  RX-->A3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX，输入上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;                    // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 数据位 8
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 停止位 1
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //使能 RX 和 TX

    USART_Init(UART7, &USART_InitStructure);
    DMA_Cmd(DMA2_Channel9, ENABLE);                                  //开启接收 DMA
    USART_Cmd(UART7, ENABLE);                                        //开启UART

    USART_Init(USART2, &USART_InitStructure);
    DMA_Cmd(DMA1_Channel6, ENABLE);                                  //开启接收 DMA
    USART_Cmd(USART2, ENABLE);                                        //开启UART
}

/*******************************************************************************
* Function Name  : DMA_INIT
* Description    : Configures the DMA.
* 描述    ：   DMA 初始化
* Input          : None
* Return         : None
*******************************************************************************/
void DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    // TX DMA 初始化
    DMA_DeInit(DMA2_Channel8);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART7->DATAR);        // DMA 外设基址，需指向对应的外设
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;                   // DMA 内存基址，指向发送缓冲区的首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 方向 : 外设 作为 终点，即 内存 ->  外设
    DMA_InitStructure.DMA_BufferSize = 0;                                   // 缓冲区大小,即要DMA发送的数据长度,目前没有数据可发
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址自增，禁用
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存地址自增，启用
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据位宽，8位(Byte)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据位宽，8位(Byte)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 普通模式，发完结束，不循环发送
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // M2P,禁用M2M
    DMA_Init(DMA2_Channel8, &DMA_InitStructure);

    // RX DMA 初始化，环形缓冲区自动接收
    DMA_DeInit(DMA2_Channel9);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART7->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;                   // 接收缓冲区
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 方向 : 外设 作为 源，即 内存 <- 外设
    DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;                          // 缓冲区长度为 RXBUF_SIZE
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // 循环模式，构成环形缓冲区
    DMA_Init(DMA2_Channel9, &DMA_InitStructure);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // TX DMA 初始化
    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR);        // DMA 外设基址，需指向对应的外设
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;                   // DMA 内存基址，指向发送缓冲区的首地址。未指定合法地址就启动 DMA 会进 hardfault
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                     // 方向 : 外设 作为 终点，即 内存 ->  外设
    DMA_InitStructure.DMA_BufferSize = 0;                                   // 缓冲区大小,即要 DMA 发送的数据长度,目前没有数据可发
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址自增，禁用
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存地址自增，启用
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据位宽，8位(Byte)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据位宽，8位(Byte)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 普通模式，发完结束，不循环发送
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 实际为 M2P,所以禁用 M2M
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    // RX DMA 初始化，环形缓冲区自动接收
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;                   // 接收缓冲区
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 方向 : 外设 作为 源，即 内存 <- 外设
    DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;                          // 缓冲区长度为 RXBUF_SIZE
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // 循环模式，构成环形缓冲区
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
}


/*******************************************************************************
* Function Name  : GPIO_CFG
* Description    : Initializes GPIOs.
* 描述    ：   GPIO　初始化
* Input          : None
* Return         : None
*******************************************************************************/
void GPIO_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    // CH9141 配置引脚初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    /* BLE_sleep --> C13  BLE_AT-->A7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  :  uartWriteBLE
* Description    :  send data to BLE via UART7          向蓝牙模组发送数据
* Input          :  char * data          data to send   要发送的数据的首地址
*                   uint16_t num         number of data 数据长度
* Return         :  RESET                UART7 busy,failed to send  发送失败
*                   SET                  send success               发送成功
*******************************************************************************/
FlagStatus uartWriteBLE(char * data , uint16_t num)
{
    //如上次发送未完成，返回
    if(DMA_GetCurrDataCounter(DMA2_Channel8) != 0){
        return RESET;
    }

    DMA_ClearFlag(DMA2_FLAG_TC8);
    DMA_Cmd(DMA2_Channel8, DISABLE );           // 关 DMA 后操作
    DMA2_Channel8->MADDR = (uint32_t)data;      // 发送缓冲区为 data
    DMA_SetCurrDataCounter(DMA2_Channel8,num);  // 设置缓冲区长度
    DMA_Cmd(DMA2_Channel8, ENABLE);             // 开 DMA
    return SET;
}

/*******************************************************************************
* Function Name  :  uartWriteBLEstr
* Description    :  send string to BLE via UART7    向蓝牙模组发送字符串
* Input          :  char * str          string to send
* Return         :  RESET                UART7 busy,failed to send  发送失败
*                   SET                  send success               发送成功
*******************************************************************************/
FlagStatus uartWriteBLEstr(char * str)
{
    uint16_t num = 0;
    while(str[num])num++;           // 计算字符串长度
    return uartWriteBLE(str,num);
}


/*******************************************************************************
* Function Name  :  uartReadBLE
* Description    :  read some bytes from receive buffer 从接收缓冲区读出一组数据
* Input          :  char * buffer        buffer to storage the data 用来存放读出数据的地址
*                   uint16_t num         number of data to read     要读的字节数
* Return         :  int                  number of bytes read       返回实际读出的字节数
*******************************************************************************/
uint16_t rxBufferReadPos1 = 0;       //接收缓冲区读指针
uint32_t uartReadBLE(char * buffer , uint16_t num)
{
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel9); //计算 DMA 数据尾的位置
    uint16_t i = 0;

    if (rxBufferReadPos1 == rxBufferEnd){
        // 无数据，返回
        return 0;
    }

    while (rxBufferReadPos1!=rxBufferEnd && i < num){
        buffer[i] = RxBuffer[rxBufferReadPos1];
        i++;
        rxBufferReadPos1++;
        if(rxBufferReadPos1 >= RXBUF_SIZE){
            // 超出缓冲区，回零
            rxBufferReadPos1 = 0;
        }
    }
    return i;
}

/*******************************************************************************
* Function Name  :  uartReadByteBLE
* Description    :  read one byte from UART buffer  从接收缓冲区读出 1 字节数据
* Input          :  None
* Return         :  char    read data               返回读出的数据(无数据也返回0)
*******************************************************************************/
char uartReadByteBLE()
{
    char ret;
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel9);//计算 DMA 数据尾的位置
    if (rxBufferReadPos1 == rxBufferEnd){
        // 无数据，返回
        return 0;
    }
    ret = RxBuffer[rxBufferReadPos1];
    rxBufferReadPos1++;
    if(rxBufferReadPos1 >= RXBUF_SIZE){
        // 超出缓冲区，回零
        rxBufferReadPos1 = 0;
    }
    return ret;
}
/*******************************************************************************
* Function Name  :  uartAvailableBLE
* Description    :  get number of bytes Available to read from the UART buffer  获取缓冲区中可读数据的数量
* Input          :  None
* Return         :  uint16_t    number of bytes Available to readd              返回可读数据数量
*******************************************************************************/
uint16_t uartAvailableBLE()
{
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel9);//计算 DMA 数据尾的位置
    // 计算可读字节
    if (rxBufferReadPos1 <= rxBufferEnd){
        return rxBufferEnd - rxBufferReadPos1;
    }else{
        return rxBufferEnd +RXBUF_SIZE -rxBufferReadPos1;
    }
}

/*******************************************************************************
* Function Name  :  uartWriteBlocking
* Description    :  send data via USART2            用USART2发送数据，阻塞式
* Input          :  char * data          data to send   要发送的数据的首地址
*                   uint16_t num         number of data 数据长度
* Return         :  RESET                USART2 busy,failed to send 发送失败
*                   SET                  send success               发送成功
*******************************************************************************/
FlagStatus uartWriteBlocking(char * data , uint16_t num)
{
    //等待上次发送完成
    while(DMA_GetCurrDataCounter(DMA1_Channel7) != 0){
    }

    DMA_ClearFlag(DMA2_FLAG_TC8);
    DMA_Cmd(DMA1_Channel7, DISABLE );           // 关 DMA 后操作
    DMA1_Channel7->MADDR = (uint32_t)data;      // 发送缓冲区为 data
    DMA_SetCurrDataCounter(DMA1_Channel7,num);  // 设置缓冲区长度
    DMA_Cmd(DMA1_Channel7, ENABLE);             // 开 DMA
    return SET;
}

/*******************************************************************************
* Function Name  :  uartWriteStrBlocking
* Description    :  send string via USART2  用USART2发送字符串，阻塞式
* Input          :  char * str          string to send
* Return         :  RESET                USART2 busy,failed to send 发送失败
*                   SET                  send success               发送成功
*******************************************************************************/
FlagStatus uartWriteStrBlocking(char * str)
{
    uint16_t num = 0;
    while(str[num])num++;           // 计算字符串长度
    return uartWriteBlocking(str,num);
}


/*******************************************************************************
* Function Name  :  uartRead
* Description    :  read some bytes from receive buffer 从接收缓冲区读出一组数据
* Input          :  char * buffer        buffer to storage the data 用来存放读出数据的地址
*                   uint16_t num         number of data to read     要读的字节数
* Return         :  int                  number of bytes read       返回实际读出的字节数
*******************************************************************************/
uint16_t rxBufferReadPos2 = 0;       //接收缓冲区读指针
uint32_t uartRead(char * buffer , uint16_t num)
{
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6); //计算 DMA 数据尾的位置
    uint16_t i = 0;

    if (rxBufferReadPos2 == rxBufferEnd){
        // 无数据，返回
        return 0;
    }

    while (rxBufferReadPos2!=rxBufferEnd && i < num){
        buffer[i] = RxBuffer[rxBufferReadPos2];
        i++;
        rxBufferReadPos2++;
        if(rxBufferReadPos2 >= RXBUF_SIZE){
            // 读指针超出接收缓冲区，回零
            rxBufferReadPos2 = 0;
        }
    }
    return i;
}

/*******************************************************************************
* Function Name  :  uartReadByte
* Description    :  read one byte from UART buffer  从接收缓冲区读出 1 字节数据
* Input          :  None
* Return         :  char    read data               返回读出的数据(无数据也返回0)
*******************************************************************************/
char uartReadByte()
{
    char ret;
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);//计算 DMA 数据尾的位置
    if (rxBufferReadPos2 == rxBufferEnd){
        // 无数据，返回
        return 0;
    }
    ret = RxBuffer[rxBufferReadPos2];
    rxBufferReadPos2++;
    if(rxBufferReadPos2 >= RXBUF_SIZE){
        // 读指针超出接收缓冲区，回零
        rxBufferReadPos2 = 0;
    }
    return ret;
}
/*******************************************************************************
* Function Name  :  uartAvailable
* Description    :  get number of bytes Available to read from the UART buffer  获取缓冲区中可读数据的数量
* Input          :  None
* Return         :  uint16_t    number of bytes Available to read               返回可读数据数量
*******************************************************************************/
uint16_t uartAvailable()
{
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);//计算 DMA 数据尾的位置
    // 计算可读字节
    if (rxBufferReadPos2 <= rxBufferEnd){
        return rxBufferEnd - rxBufferReadPos2;
    }else{
        return rxBufferEnd +RXBUF_SIZE -rxBufferReadPos2;
    }
}

/* Global Variable */
s16 Calibrattion_Val = 0;

/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void POWER_init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void Left(void){
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}
void Right(void){
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}
void Close(void){
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}


void ADC_Function_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    Calibrattion_Val = Get_CalibrationValue(ADC1);

    ADC_BufferCmd(ADC1, ENABLE);   //enable buffer

    ADC_TempSensorVrefintCmd(ENABLE);
}

/*********************************************************************
 * @fn      Get_ADC_Val
 *
 * @brief   Returns ADCx conversion result data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *            ADC_Channel_10 - ADC Channel10 selected.
 *            ADC_Channel_11 - ADC Channel11 selected.
 *            ADC_Channel_12 - ADC Channel12 selected.
 *            ADC_Channel_13 - ADC Channel13 selected.
 *            ADC_Channel_14 - ADC Channel14 selected.
 *            ADC_Channel_15 - ADC Channel15 selected.
 *            ADC_Channel_16 - ADC Channel16 selected.
 *            ADC_Channel_17 - ADC Channel17 selected.
 *
 * @return  none
 */
u16 Get_ADC_Val(u8 ch)
{
    u16 val;

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));

    val = ADC_GetConversionValue(ADC1);

    return val;
}

/*********************************************************************
 * @fn      Get_ADC_Average
 *
 * @brief   Returns ADCx conversion result average data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *            ADC_Channel_10 - ADC Channel10 selected.
 *            ADC_Channel_11 - ADC Channel11 selected.
 *            ADC_Channel_12 - ADC Channel12 selected.
 *            ADC_Channel_13 - ADC Channel13 selected.
 *            ADC_Channel_14 - ADC Channel14 selected.
 *            ADC_Channel_15 - ADC Channel15 selected.
 *            ADC_Channel_16 - ADC Channel16 selected.
 *            ADC_Channel_17 - ADC Channel17 selected.
 *
 * @return  val - The Data conversion value.
 */
u16 Get_ADC_Average(u8 ch,u8 times)
{
    u32 temp_val=0;
    u8 t;
    u16 val;

    for(t=0;t<times;t++)
    {
        temp_val+=Get_ADC_Val(ch);
        Delay_Ms(5);
    }

    val = temp_val/times;

    return val;
}

/*********************************************************************
 * @fn      Get_ConversionVal
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal(s16 val)
{
    if((val+Calibrattion_Val)<0) return 0;
    if((Calibrattion_Val+val)>4095||val==4095) return 4095;
    return (val+Calibrattion_Val);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    u16 ADC_val;
    s32 val_mv;
    s32 temperature;
    u16 ir, als, ps;
    SystemCoreClockUpdate();

    POWER_init();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);
    ADC_Function_Init();
    printf("CalibrattionValue:%d\n", Calibrattion_Val);
    lcd_init();

    lcd_set_color(BLACK, WHITE);
    lcd_show_string(50, 0, 32, "openCH.io");
    lcd_set_color(BLACK, WHITE);
    lcd_show_string(0, 48, 16, "AP3216C");
    while (AP3216C_Init()) // 初始化AP3216C
    {
        lcd_set_color(BLACK, RED);
        lcd_show_string(180, 48, 16, "Error");
        Delay_Ms(200);
        lcd_show_string(180, 48, 16, "     ");
        Delay_Ms(200);
    }

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();

    DMA_INIT();
    USARTx_CFG();                                                 /* USART INIT */
    USART_DMACmd(UART7,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);     //开启 UART7 DMA 接收和发送
    USART_DMACmd(USART2,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);    //开启 USART2 DMA 接收和发送

    GPIO_CFG();
    GPIO_WriteBit(GPIOC, GPIO_Pin_13,SET); //enable CH9141
    char x[10]={"0123456789"};
    Delay_Ms(1000);

    lcd_set_color(BLACK, GREEN);
    lcd_show_string(180, 48, 16, "OK");
    lcd_show_string(180, 68, 16, "OK");
    int state=0;//此时卷帘机的状态，为0时表示卷帘机此时放下了，为1时表示此时卷帘机升起了，为2时代表卷帘机正在升起，为3时代表卷帘机正在放下；
    while(1)
    {

        char s1[1024]={" T: "};
        char s2[1024]={"\0"};
        char s3[1024]={" A: "};
        char s4[1024]={"\0"};
        s2[3]='\n';
        s4[3]='\n';
        int t=temperature,a=als;
        int t0=t/10,t1=t%10,a0,a1=a%10,a3=a/100;
        if(a>=100){
            a0=(a-100)/10;
        }
        else{
            a0=a/10;
        }
        s4[0]='0';
        for(int j=0;j<10;j++){
            if(x[j]-'0'==a3)s4[0]=x[j];
            if(x[j]-'0'==t0)s2[0]=x[j];
            if(x[j]-'0'==t1)s2[1]=x[j];
            if(x[j]-'0'==a0)s4[1]=x[j];
            if(x[j]-'0'==a1)s4[2]=x[j];
        }
        char buffer1[1024];
        for(int j=0;j<4;j++){
            buffer1[j]=s1[j];
        }
        for(int j=4;j<7;j++){
            buffer1[j]=s2[j-4];
        }
        for(int j=7;j<11;j++){
            buffer1[j]=s3[j-7];
        }
        for(int j=11;j<15;j++){
            buffer1[j]=s4[j-11];
        }
        //1
        int num2 = 15;
        char bufferx[1024]={"\0"};
        uartRead(bufferx , num2); //把数据从缓冲区读到 buffer 里
        uartWriteBLE(buffer1 , num2); //把数据发送给 蓝牙
        int num1 =uartAvailableBLE();
        char buffery[1024]={"\0"};
        uartReadBLE(buffery , num1);       //读取蓝牙传输出来的数据
        uartWriteBLE(buffery , 15);

        ADC_val = Get_ADC_Average( ADC_Channel_TempSensor, 10 );
        Delay_Ms(500);
        printf( "ADC-Val:%04d\r\n", ADC_val);
        ADC_val = Get_ConversionVal(ADC_val);
        printf( "ADC-Val:%04d\r\n", ADC_val);

        val_mv = (ADC_val*3300/4096);
        temperature=TempSensor_Volt_To_Temper(val_mv);
        printf("mv-T-%d,%0d\n",val_mv ,temperature);
        AP3216C_ReadData(&ir, &ps, &als); // 读取数据
        lcd_set_color(BLACK, WHITE);
        lcd_show_string(0, 48, 16, "AP3216C");
        lcd_show_string(0, 68, 16, "Temperature Sensor");
        lcd_set_color(BLACK, GREEN);
        lcd_show_string(30, 100, 16, "temperature : %0d", temperature);
        lcd_show_string(30, 116, 16, "ALS: %5d", als);
        if(temperature>=30&&als<20){
            Delay_Ms(500);//延时若干
            if(temperature<=32&&als<=52&&state==0){
                Left();
                printf("Roller shutter machine is rising.\n");
                lcd_show_string(30, 155, 16, "Machine is rising.    ");
                Delay_Ms(4000);
                state=1;
                Close();
                lcd_show_string(30, 155, 16, "Machine is rised.     ");
            }
        }
        else if(temperature>=30){
            Delay_Ms(500);
            if(temperature>=30&&state==1){
                Right();
                printf("Roller shutter machine is lowering.\n");
                lcd_show_string(30, 155, 16, "Machine is lowering.  ");
                Delay_Ms(4000);
                state=0;
                Close();
                lcd_show_string(30, 155, 16, "Machine is lowered.   ");
            }
        }
        Delay_Ms(2);

    }
}
