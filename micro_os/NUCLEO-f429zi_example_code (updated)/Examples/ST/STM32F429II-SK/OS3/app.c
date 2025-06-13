/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

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

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

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

#define APP_CFG_ApptaskCreate_PRIO       2u
#define APP_CFG_ApptaskCreate_STK_SIZE 512u

// 조이스틱 pin
#define JOYSTICK_X_PIN    GPIO_Pin_3 // PA0
#define JOYSTICK_Y_PIN    GPIO_Pin_7 // PA7
#define JOYSTICK_SW_PIN   GPIO_Pin_12 // PC13 (SW 버튼)

//조이스틱  task 등록 관련
#define JOYSTICK_STK_SIZE 512u
#define JOYSTICK_PRIO     3u

/*
*********************************************************************************************************
*                                            TYPES DEFINITIONS
*********************************************************************************************************
*/

typedef enum
{
  COM1 = 0,
  COMn
} COM_TypeDef;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

// HW init
static void USART_InitCOM(COM_TypeDef com, const USART_InitTypeDef *cfg);
static void USART_Config(void);
void RTC_CustomInit(void);
static void TouchSensor_Init(void);
static void KnockSensor_Init(void);
void Buzzer_Init(void);
void Joystick_InitConfig(void); //joystick init
void HW_ButtonInit(void); // button init

// USART utils
static void USART_SendChar(uint8_t c);
static void USART_SendString(const char *s);

// RTC utils
void RTC_CustomSetTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void RTC_GetTimeStr(char *buf, size_t len);
void RTC_SetAlarmDaily(void);

// touch sensor utils

// knock sensor utils
static uint8_t KnockSensor_Read(void);

// buzzer utils
void Buzzer_On(void);
void Buzzer_Off(void);

// joystick utils
uint16_t ReadJoystickX(void);
uint16_t ReadJoystickY(void);

// button utils
void Button_delay(uint32_t ms);

// Task
static void AppTaskStart(void *p_arg);
static void ApptaskCreate(void *p_arg);
void JoystickTaskStart(void *p_arg);
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static OS_TCB  AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_CFG_ApptaskCreate_STK_SIZE];

char target_time[16] = {0};
char current_time[16] = {0};
char input_buffer[32];
uint8_t input_index = 0;
uint8_t is_target_set = 0;
volatile uint8_t alarm_flag = 0; // bsp_int.c�뿉�꽌 怨듭쑀

// joystick task
static OS_TCB TCB_Joystick;
static CPU_STK STK_Joystick[JOYSTICK_STK_SIZE];
int left,right,up,down;

/*
*********************************************************************************************************
*                                   HARDWARE MODULE INITIALIZATION
*********************************************************************************************************
*/

// USART
static void USART_InitCOM(COM_TypeDef com, const USART_InitTypeDef *cfg)
{
  if (com != COM1)
    return;

  RCC_AHB1PeriphClockCmd(NUCLEO_COM1_TX_PORT_CLK | NUCLEO_COM1_RX_PORT_CLK, ENABLE);
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
  USART_InitCOM(COM1, &cfg);
}

// RTC
void RTC_CustomInit(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    ;
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  RCC_RTCCLKCmd(ENABLE);
  RTC_WaitForSynchro();

  RTC_InitTypeDef RTC_InitStruct;
  RTC_StructInit(&RTC_InitStruct);
  RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStruct.RTC_AsynchPrediv = 0x7F;
  RTC_InitStruct.RTC_SynchPrediv = 0x0130;
  RTC_Init(&RTC_InitStruct);
}

// touch sensor
static void TouchSensor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIOA �겢�윮 Enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // PA3 �엯�젰�쑝濡� �꽕�젙 (���떎�슫 �궗�슜)
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;      // PA3 (A0)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;   // �엯�젰 紐⑤뱶
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN; // ���떎�슫 �꽕�젙

  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// knock sensor
static void KnockSensor_Init(void)
{
  // GPIOA �겢�윮 �솢�꽦�솕
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // PA6 �엯�젰 �� �꽕�젙
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;        // PA6
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;     // �엯�젰 紐⑤뱶
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // ���뾽/���떎�슫 �뾾�쓬
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// buzzer
void Buzzer_Init(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5; // (PA5 / D13)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// joystick
void Joystick_InitConfig(void) {
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

//button init
void HW_ButtonInit(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;           // PC0 사용
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;        // 입력 모드
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // 풀업 설정
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/*
*********************************************************************************************************
*                                       USART FUNCTIONS
*********************************************************************************************************
*/

static void USART_SendChar(uint8_t c)
{
  while (USART_GetFlagStatus(NUCLEO_COM1, USART_FLAG_TXE) == RESET)
    ;
  USART_SendData(NUCLEO_COM1, c);
}

static void USART_SendString(const char *s)
{
  while (*s)
    USART_SendChar((uint8_t)*s++);
}

/*
*********************************************************************************************************
*                                       RTC FUNCTIONS
*********************************************************************************************************
*/

void RTC_CustomSetTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  RTC_TimeTypeDef RTC_Time;
  RTC_Time.RTC_Hours = hours;
  RTC_Time.RTC_Minutes = minutes;
  RTC_Time.RTC_Seconds = seconds;
  RTC_SetTime(RTC_Format_BIN, &RTC_Time);
}

void RTC_GetTimeStr(char *buf, size_t len)
{
  RTC_TimeTypeDef RTC_Time;
  RTC_GetTime(RTC_Format_BIN, &RTC_Time);
  snprintf(buf, len, "%02d:%02d:%02d", RTC_Time.RTC_Hours, RTC_Time.RTC_Minutes, RTC_Time.RTC_Seconds);
}

void RTC_SetAlarmDaily(void)
{
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStruct.EXTI_Line = EXTI_Line17;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  RTC_AlarmTypeDef alarm;
  alarm.RTC_AlarmTime.RTC_Hours = 9;
  alarm.RTC_AlarmTime.RTC_Minutes = 0;
  alarm.RTC_AlarmTime.RTC_Seconds = 0;
  alarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
  alarm.RTC_AlarmDateWeekDay = 1;
  alarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm);
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
  NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

/*
*********************************************************************************************************
*                                       KNOCK SEONSOR FUNCTIONS
*********************************************************************************************************
*/

// ---- �끂�겕 �꽱�꽌 媛� �씫湲� ----
static uint8_t KnockSensor_Read(void)
{
  return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6); // 1�씠硫� 吏꾨룞 媛먯��맖
}

/*
*********************************************************************************************************
*                                           BUZZER FUNCTIONS
*********************************************************************************************************
*/

void Buzzer_On(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void Buzzer_Off(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

/*
*********************************************************************************************************
*                                           JOYSTICK FUNCTIONS
*********************************************************************************************************
*/
// Read analog value
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
/*
*********************************************************************************************************
*                                           BUTTON FUNCTIONS
*********************************************************************************************************
*/
void Button_delay(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 4000; i++)
        __NOP();
}
/*
*********************************************************************************************************
*                                                main
*********************************************************************************************************
*/

// main()->AppTaskStart()->AppTaskCreate() �닚�쑝濡� �샇異�
// AppTaskCreate()�뿉�꽌 紐⑤뱺 �깭�뒪�겕 �젙�쓽

int main(void)
{
  OS_ERR err;

  /* Basic Init */
  RCC_DeInit();

  /* BSP Init */
  BSP_IntDisAll();

  CPU_Init();
  Mem_Init();
  Math_Init();

  // �쁽�옱 �떆媛꾧낵 �븣�엺 �슱由� �떆媛� �꽕�젙
  RTC_CustomInit();
  RTC_CustomSetTime(8, 59, 55);
  RTC_SetAlarmDaily();

  /* OS Init */
  OSInit(&err);

  OSTaskCreate((OS_TCB *)&AppTaskStartTCB,
               (CPU_CHAR *)"App Task Start",
               (OS_TASK_PTR)AppTaskStart,
               (void *)0u,
               (OS_PRIO)APP_CFG_ApptaskCreate_PRIO,
               (CPU_STK *)&AppTaskStartStk[0u],
               (CPU_STK_SIZE)AppTaskStartStk[APP_CFG_ApptaskCreate_STK_SIZE / 10u],
               (CPU_STK_SIZE)APP_CFG_ApptaskCreate_STK_SIZE,
               (OS_MSG_QTY)0u,
               (OS_TICK)0u,
               (void *)0u,
               (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);

  OSStart(&err); /* Start multitasking (i.e. give control to uC/OS-III). */

  (void)&err;

  return (0u);
}

/*
*********************************************************************************************************
*                                           TASK FUNCTION
*********************************************************************************************************
*/

static void AppTaskStart(void *p_arg)
{
  OS_ERR err;

  (void)p_arg;

  // HW 珥덇린�솕
  BSP_Init();
  BSP_Tick_Init();

  USART_Config();
  TouchSensor_Init();
  KnockSensor_Init();
  Buzzer_Init();
  // �뿬湲� 議곗씠�뒪�떛,踰꾪듉 珥덇린�솕 �븿�닔 異붽�

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  ApptaskCreate(NULL);
}

static void ApptaskCreate(void *p_arg)
{
  OS_ERR err;
  (void)p_arg;

  BSP_Init();
  BSP_Tick_Init();

  // �깭�뒪�겕 異붽�
}

//joystick task
void JoystickTaskStart(void *p_arg) {
	OS_ERR err;
	BSP_Init();
	BSP_Tick_Init();
	USART_Config();
	Joystick_InitConfig();

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
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == SET) {
            send_string("Button Pressed\n");
        }

        OSTimeDly(200, OS_OPT_TIME_PERIODIC, &err);
    }
}
