#include "bsp.h"
#include "cpu.h"
#include "os.h"
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>

// ---- 타입, 변수 선언 ----
typedef enum
{
  COM1 = 0,
  COMn
} COM_TypeDef;

// ---- 함수 선언 ----
// (1) HW 초기화
static void STM_Nucleo_COMInit(COM_TypeDef com, const USART_InitTypeDef *cfg);
static void USART_Config(void);

// (2) RTC 설정
void RTC_Init_Config(void);
void RTC_CustomSetTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void RTC_GetCurrentTimeString(char *buf, size_t len);
void RTC_SetDailyAlarm(void);

// (3) USART 추가
static void send_char(uint8_t c);
static void send_string(const char *s);

// (4) Task 함수
static void TaskStart(void *p_arg);
static void TaskUsart(void *p_arg);

#define NUCLEO_COM1 USART3
#define NUCLEO_COM1_CLK RCC_APB1Periph_USART3
#define NUCLEO_COM1_TX_PIN GPIO_Pin_8
#define NUCLEO_COM1_TX_PORT GPIOD
#define NUCLEO_COM1_TX_PORT_CLK RCC_AHB1Periph_GPIOD
#define NUCLEO_COM1_TX_SRC GPIO_PinSource8
#define NUCLEO_COM1_TX_AF GPIO_AF_USART3
#define NUCLEO_COM1_RX_PIN GPIO_Pin_9
#define NUCLEO_COM1_RX_PORT GPIOD
#define NUCLEO_COM1_RX_PORT_CLK RCC_AHB1Periph_GPIOD
#define NUCLEO_COM1_RX_SRC GPIO_PinSource9
#define NUCLEO_COM1_RX_AF GPIO_AF_USART3

// bsp_int.c에서 공유
volatile uint8_t alarm_flag = 0;


// ---- HW 초기화 ----
static void STM_Nucleo_COMInit(COM_TypeDef com, const USART_InitTypeDef *cfg)
{
  if (com != COM1)
    return;
  RCC_AHB1PeriphClockCmd(NUCLEO_COM1_TX_PORT_CLK | NUCLEO_COM1_RX_PORT_CLK,
                         ENABLE);
  RCC_APB1PeriphClockCmd(NUCLEO_COM1_CLK, ENABLE);

  GPIO_PinAFConfig(NUCLEO_COM1_TX_PORT, NUCLEO_COM1_TX_SRC, NUCLEO_COM1_TX_AF);
  GPIO_PinAFConfig(NUCLEO_COM1_RX_PORT, NUCLEO_COM1_RX_SRC, NUCLEO_COM1_RX_AF);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStruct.GPIO_Pin = NUCLEO_COM1_TX_PIN;
  GPIO_Init(NUCLEO_COM1_TX_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = NUCLEO_COM1_RX_PIN;
  GPIO_Init(NUCLEO_COM1_RX_PORT, &GPIO_InitStruct);

  USART_Init(NUCLEO_COM1, (USART_InitTypeDef *)cfg);
  USART_Cmd(NUCLEO_COM1, ENABLE);
}

static void USART_Config(void)
{
  USART_InitTypeDef cfg;
  cfg.USART_BaudRate = 115200;
  cfg.USART_WordLength = USART_WordLength_8b;
  cfg.USART_StopBits = USART_StopBits_1;
  cfg.USART_Parity = USART_Parity_No;
  cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  cfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  STM_Nucleo_COMInit(COM1, &cfg);
}

static void send_char(uint8_t c)
{
  while (USART_GetFlagStatus(NUCLEO_COM1, USART_FLAG_TXE) == RESET)
  {
  }
  USART_SendData(NUCLEO_COM1, c);
}

static void send_string(const char *s)
{
  while (*s)
    send_char((uint8_t)*s++);
}


// 함수 선언
void USART_Config(void);
void send_string(const char *str);
void delay_ms(uint32_t ms);

int main(void)
{
    // PC0 GPIO 클럭 활성화
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // PC0 입력 모드 설정 (풀업)
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;  // 내부 풀업
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // USART 초기화
    USART_Config();

    send_string("== Button Test Start ==\r\n");

    while (1)
    {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == Bit_RESET)  // 버튼 눌림 (GND)
        {
            send_string("Button Pressed!\r\n");
            delay_ms(150);  // 디바운싱 지연
        }
    }
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 4000; i++)
        __NOP();
}




