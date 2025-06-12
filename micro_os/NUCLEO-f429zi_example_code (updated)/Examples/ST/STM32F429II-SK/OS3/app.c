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
#include <stdlib.h>
#include <time.h>
// 조이스틱 핀 정의 (PA0, PA1, PC13 등)
#define JOYSTICK_X_PIN    GPIO_Pin_3 // PA0
#define JOYSTICK_Y_PIN    GPIO_Pin_7 // PA7
#define JOYSTICK_SW_PIN   GPIO_Pin_12 // PC13 (SW 버튼)

// ----    엯, 蹂  닔  꽑 뼵 ----
typedef enum
{
  COM1 = 0,
  COMn
} COM_TypeDef;
volatile uint8_t alarm_flag=0;
static void STM_Nucleo_COMInit(COM_TypeDef com, const USART_InitTypeDef *cfg);
static void USART_Config(void);

static void send_char(uint8_t c);
static void send_string(const char *s);

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

#define START_PRIO 2u
#define START_STK_SIZE 512u

// ---- 蹂  닔  꽑 뼵 ----
static OS_TCB TCB_Start;
static CPU_STK STK_Start[START_STK_SIZE];
// USART 초기화 함수

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

// 문자 전송 함수
static void send_char(uint8_t c)
{
    while (USART_GetFlagStatus(NUCLEO_COM1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(NUCLEO_COM1, c);
}

// 문자열 전송 함수
static void send_string(const char *s)
{
    while (*s)
        send_char((uint8_t)*s++);
}


void ADC_InitConfig(void) {
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	    // ADC1에 대한 클럭 활성화
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	    GPIO_InitTypeDef GPIO_InitStruct;
	    // PA3, PA7 핀을 아날로그 모드로 설정
	    GPIO_InitStruct.GPIO_Pin = JOYSTICK_X_PIN | JOYSTICK_Y_PIN;  // PA3, PA7
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;  // 아날로그 모드
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  // 풀업/풀다운 없음
	    GPIO_Init(GPIOA, &GPIO_InitStruct);  // PA3, PA7 아날로그 설정

	    ADC_InitTypeDef ADC_InitStruct;
	    ADC_StructInit(&ADC_InitStruct);
	    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;  // 12비트 해상도
	    ADC_Init(ADC1, &ADC_InitStruct);  // ADC1 초기화

	    ADC_Cmd(ADC1, ENABLE);  // ADC1 활성화
	    ADC_SoftwareStartConv(ADC1);  // ADC 변환 시작
}

// 조이스틱 아날로그 값 읽기
uint16_t ReadJoystickX(void) {
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

uint16_t ReadJoystickY(void) {
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

// 조이스틱 테스트를 위한 태스크 함수
void TaskStart(void *p_arg) {
	OS_ERR err;
	BSP_Init();
	BSP_Tick_Init();
	USART_Config();
    uint16_t joystickX, joystickY;
    char buffer[50];
    int left,right,up,down;
    srand(time(NULL));
    int random_number = rand() % 5 + 1;
    int pattern[5][4]={{1, 2, 3, 4},{1, 2, 4, 3},{1, 3, 2, 4},{1, 3, 4, 2},{1, 4, 2, 3}};
    left = pattern[random_number][0];
    right = pattern[random_number][1];
    up = pattern[random_number][2];
    down = pattern[random_number][3];

    while (1) {
        joystickX = ReadJoystickX();
        joystickY = ReadJoystickY();

        // UART로 출력
        sprintf(buffer, "X: %d, Y: %d\n", joystickX*7, joystickY);

        if((joystickX*7 >300 && joystickX*7 <600) && (joystickY >1000 && joystickY<3000)){
        	send_string("center\r\n");
        }
        else if(joystickX*7 > 1000){
        	send_string("right\r\n");
        	right -=1;
        	if (right < 0){
        		right = 0;
        	}
        }
        else if(joystickX*7 < 100){
        	send_string("left\r\n");
        	left -=1;
        	if(left <0){
        		left =0;
        	}
        }
        else if( joystickY >3000){
        	send_string("Up\r\n");
        	up-=1;
        	if(up<0){
        		up =0;
        	}
        }
        else{
        	send_string("Down\r\n");
        	down-=1;
        	if(down<0){
        		down=0;
        	}
        }
        sprintf(buffer, "left: %d right: %d up: %d down: %d\n", left, right, up, down);
		send_string(buffer);

		if (left == 0 && right == 0 && up == 0 && down == 0) {
           break;
		 }
        // 조이스틱 버튼 (SW) 확인
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == SET) {
            send_string("Button Pressed\n");
        }

        OSTimeDly(200, OS_OPT_TIME_PERIODIC, &err);  // 10ms 대기
    }
}

// 메인 함수
int main(void) {
    OS_ERR err;

    CPU_Init();  // CPU 초기화
    OSInit(&err);  // RTOS 초기화

    USART_Config();  // USART 설정
    ADC_InitConfig();  // ADC 설정

    // 조이스틱 테스트 태스크 생성
    OSTaskCreate(&TCB_Start,
                 "Start",
                 TaskStart,
                 NULL,
                 START_PRIO,
                 &STK_Start[0],
                 START_STK_SIZE / 10,
                 START_STK_SIZE,
                 0, 0, 0,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSStart(&err);  // RTOS 시작

    return 0;
}
