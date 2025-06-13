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

#define START_PRIO 2u
#define START_STK_SIZE 512u

// ---- 변수 선언 ----
static OS_TCB TCB_Start;
static CPU_STK STK_Start[START_STK_SIZE];

char target_time[16] = {0};
char current_time[16] = {0};
char input_buffer[32];
uint8_t input_index = 0;
uint8_t is_target_set = 0;

// bsp_int.c에서 공유
volatile uint8_t alarm_flag = 0;

#define JOYSTICK_STK_SIZE 512u
#define JOYSTICK_PRIO     3u  // Start보다 낮거나 높게 적절히 조정

static OS_TCB TCB_Joystick;
static CPU_STK STK_Joystick[JOYSTICK_STK_SIZE];
void JoystickTaskStart(void *p_arg);

// ---- 메인 함수 ----
int main(void)
{
  OS_ERR err;
  CPU_Init();
  OSInit(&err);

  RTC_Init_Config();            // RTC 설정, LSI 확인 후 설정 후
  RTC_CustomSetTime(8, 59, 55); // 테스트용 시간
  RTC_SetDailyAlarm();          // 매일 9시 알림

  OSTaskCreate(&TCB_Start, "Start", TaskStart, 0, START_PRIO, &STK_Start[0],
               START_STK_SIZE / 10, START_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

  RTC_AlarmCmd(RTC_Alarm_A, ENABLE); // 알람 동작 시작
  NVIC_EnableIRQ(RTC_Alarm_IRQn);    // NVIC에 등록

  OSStart(&err);
  while (1)
    ; // OSStart() 호출 후 대기 중 오류 발생 시 이 부분 실행
}

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

// ---- RTC ----
void RTC_Init_Config(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_BackupAccessCmd(ENABLE); // 백업 접근 확인
  RCC_LSICmd(ENABLE);          // LSI 확인 후 초기화
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    ;                                     // LSI 확인 대기
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI); // RTC 설정 LSI
  RCC_RTCCLKCmd(ENABLE);                  // RTC 설정 확인

  RTC_WaitForSynchro(); // 초기화 대기

  RTC_InitTypeDef RTC_InitStruct;
  RTC_StructInit(&RTC_InitStruct);
  RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStruct.RTC_AsynchPrediv = 0x7F;
  RTC_InitStruct.RTC_SynchPrediv = 0x0130; // LSI(32kHz) 설정
  RTC_Init(&RTC_InitStruct);
}

// ---- 시간 설정 후 확인 ----
void RTC_CustomSetTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  RTC_TimeTypeDef RTC_Time;
  RTC_Time.RTC_Hours = hours;
  RTC_Time.RTC_Minutes = minutes;
  RTC_Time.RTC_Seconds = seconds;
  RTC_SetTime(RTC_Format_BIN, &RTC_Time);
}

// ---- 시간 확인 문자열 확인 ----
void RTC_GetCurrentTimeString(char *buf, size_t len)
{
  RTC_TimeTypeDef RTC_Time;
  RTC_GetTime(RTC_Format_BIN, &RTC_Time);
  snprintf(buf, len, "%02d:%02d:%02d", RTC_Time.RTC_Hours, RTC_Time.RTC_Minutes,
           RTC_Time.RTC_Seconds);
}

// ---- 알림 시간 설정 ----
void RTC_SetDailyAlarm(void)
{
  RTC_AlarmTypeDef alarm;
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE); // 알림 설정 확인

  // EXTI Line 17 (RTC Alarm) 확인 후 테스트 설정 후
  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStruct.EXTI_Line = EXTI_Line17;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  // 매일 9시 설정 후
  alarm.RTC_AlarmTime.RTC_Hours = 9;
  alarm.RTC_AlarmTime.RTC_Minutes = 0;
  alarm.RTC_AlarmTime.RTC_Seconds = 0;
  alarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; // 일자 확인 후 테스트 후
  alarm.RTC_AlarmDateWeekDay = 1;                  // 일자 확인
  alarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm);
  RTC_ITConfig(RTC_IT_ALRA, ENABLE); // 알림 확인
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE); // 알림 설정 확인
  NVIC_EnableIRQ(RTC_Alarm_IRQn);    // NVIC 확인
}

// ---- 메인 Task ----
static void TaskStart(void *p_arg)
{
  OS_ERR err;
  (void)p_arg;
  BSP_Init();
  BSP_Tick_Init();
  USART_Config();

  while (DEF_TRUE)
  {
    RTC_GetCurrentTimeString(current_time, sizeof(current_time));


    if (alarm_flag)
    {
      send_string("\r\n!!! ALARM !!!\r\n");
      OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
      alarm_flag = 0;
      OSTaskCreate(&TCB_Joystick, "JoystickTask", JoystickTaskStart, 0, JOYSTICK_PRIO,
				   &STK_Joystick[0], JOYSTICK_STK_SIZE / 10, JOYSTICK_STK_SIZE,
				   0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

    }else{
    	send_string(current_time);
    	send_string("\r\n");
    }

    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}



#define JOYSTICK_X_PIN    GPIO_Pin_3 // PA0
#define JOYSTICK_Y_PIN    GPIO_Pin_7 // PA7
#define JOYSTICK_SW_PIN   GPIO_Pin_12 // PC13 (SW 버튼)
int left,right,up,down;

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
void JoystickTaskStart(void *p_arg) {
	OS_ERR err;
	BSP_Init();
	BSP_Tick_Init();
	USART_Config();
	ADC_InitConfig();

    uint16_t joystickX, joystickY;
    char buffer[50];
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
        	send_string("\r\n");
        }
        else if(joystickX*7 > 1000){
        	right -=1;
        	if (right < 0){
        		right = 0;
        	}
        }
        else if(joystickX*7 < 100){
        	left -=1;
        	if(left <0){
        		left =0;
        	}
        }
        else if( joystickY >3000){
        	up-=1;
        	if(up<0){
        		up =0;
        	}
        }
        else{
        	down-=1;
        	if(down<0){
        		down=0;
        	}
        }
        sprintf(buffer, "left: %d right: %d up: %d down: %d\r\n", left, right, up, down);
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


