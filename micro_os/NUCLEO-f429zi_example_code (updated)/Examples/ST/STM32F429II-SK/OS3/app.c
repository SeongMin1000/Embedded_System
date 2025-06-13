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

//---------------------------PRIORITY & STACK SIZE-----------------------------
#define APP_CFG_ApptaskCreate_PRIO 2u
#define APP_CFG_ApptaskCreate_STK_SIZE 512u

// buzzer
#define BUZZER_TASK_PRIO 3u
#define BUZZER_TASK_STK_SIZE 512u

// alarm
#define ALARM_TASK_PRIO 4u
#define ALARM_TASK_STK_SIZE 512u

// touch sensor
#define TOUCH_TASK_PRIO 5u
#define TOUCH_TASK_STK_SIZE 512u

// mission
#define MISSION_TASK_PRIO 6u
#define MISSION_TASK_STK_SIZE 512u

// knock sensor
#define KNOCK_TASK_PRIO 7u
#define KNOCK_TASK_STK_SIZE 512u

// joystick
#define JOYSTICK_PRIO 7u
#define JOYSTICK_STK_SIZE 512u

// button
#define BUTTON_TASK_PRIO 7u
#define BUTTON_TAASK_STK_SIZE 512u

// USART
#define USART_TASK_PRIO 8u
#define USART_TASK_STK_SIZE 512u
//-------------------------------------------------------------------------

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
typedef struct
{
  int button_count;
  int knock_count;
  int joystick_dir; // 1 = left, 2 = right, 3 = up, 4 = down
} Mission;

Mission missions[] = {
    {2, 1, 1}, {1, 2, 4}, {2, 2, 2}, {3, 1, 3}, {1, 1, 4}};

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

//---------------------------------HW init---------------------------------
static void USART_InitCOM(COM_TypeDef com, const USART_InitTypeDef *cfg);
static void USART_Config(void);
static void RTC_CustomInit(void);
static void TouchSensor_Init(void);
static void KnockSensor_Init(void);
static void Buzzer_Init(void);
static void JoyStick_Init(void);
static void Button_Init(void);
//-------------------------------------------------------------------------

//------------------------------util function------------------------------
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
uint16_t JoyStick_ReadX(void);
uint16_t JoyStick_ReadY(void);

// button utils
void Button_delay(uint32_t ms);
//-------------------------------------------------------------------------

//-------------------------------task--------------------------------------
static void BuzzerTask(void *p_arg);      // PRIO 3
static void AlarmTask(void *p_arg);       // PRIO 4
static void TouchSensorTask(void *p_arg); // PRIO 5
static void MissionTask(void *p_arg);     // PRIO 6
static void KnockSensorTask(void *p_arg); // PRIO 7
static void JoystickTask(void *p_arg);    // PRIO 7
static void ButtonTask(void *p_arg);      // PRIO 7
static void USARTTask(void *p_arg);       // PRIO 8

// 기타 필수 Task
static void AppTaskStart(void *p_arg);
static void ApptaskCreate(void *p_arg);
//-------------------------------------------------------------------------

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static OS_TCB AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_CFG_ApptaskCreate_STK_SIZE];

char target_time[16] = {0};
char current_time[16] = {0};
char input_buffer[32];
uint8_t input_index = 0;
uint8_t is_target_set = 0;
volatile uint8_t alarm_flag = 0; // bsp_int.c

//-----------------------------TCB & STACK ---------------------------------
// buzzer
static OS_TCB TCB_Buzzer;
static CPU_STK STK_Buzzer[BUZZER_TASK_STK_SIZE];

// alram
static OS_TCB TCB_Alarm;
static CPU_STK STK_Alarm[ALARM_TASK_STK_SIZE];

// touch sensor
static OS_TCB TCB_Touch;
static CPU_STK STK_Touch[TOUCH_TASK_STK_SIZE];

// mission
static OS_TCB TCB_Mission;
static CPU_STK STK_Mission[MISSION_TASK_STK_SIZE];

// knock sensor
static OS_TCB TCB_Knock;
static CPU_STK STK_Knock[KNOCK_TASK_STK_SIZE];

// joystick
static OS_TCB TCB_Joystick;
static CPU_STK STK_Joystick[JOYSTICK_STK_SIZE];
int left, right, up, down;

// Button
static OS_TCB TCB_Button;
static CPU_STK STK_Button[USART_TASK_STK_SIZE];

// USART
static OS_TCB TCB_USART;
static CPU_STK STK_USART[USART_TASK_STK_SIZE];
//-------------------------------------------------------------------------

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
static void RTC_CustomInit(void)
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

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; // (PA3 / A0)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// knock sensor
static void KnockSensor_Init(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6; // PA6
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// buzzer
static void Buzzer_Init(void)
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
static void JoyStick_Init(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = JOYSTICK_X_PIN | JOYSTICK_Y_PIN; // PA3, PA7
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;                   // analog mode
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  ADC_InitTypeDef ADC_InitStruct;
  ADC_StructInit(&ADC_InitStruct);
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_Init(ADC1, &ADC_InitStruct);

  ADC_Cmd(ADC1, ENABLE);       // ADC1 activate
  ADC_SoftwareStartConv(ADC1); // ADC convert
}

// button init
static Button_Init(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // PC0
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
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

static uint8_t KnockSensor_Read(void)
{
  return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
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
uint16_t JoyStick_ReadX(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
  ADC_SoftwareStartConv(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    ;
  return ADC_GetConversionValue(ADC1);
}

uint16_t JoyStick_ReadY(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
  ADC_SoftwareStartConv(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    ;
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

// main()->AppTaskStart()->AppTaskCreate()

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

  // set current & alram time
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

  BSP_Init();
  BSP_Tick_Init();

  Buzzer_Init();
  TouchSensor_Init();
  KnockSensor_Init();
  JoyStick_Init();
  Button_Init();
  USART_Config();

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

  // buzzer task create (PRIO 3)
  OSTaskCreate(&TCB_Buzzer,
               "Buzzer Task",
               BuzzerTask,
               0,
               BUZZER_TASK_PRIO,
               &STK_Buzzer[0],
               BUZZER_TASK_STK_SIZE / 10,
               BUZZER_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // alarm task create (PRIO 4)
  OSTaskCreate(&TCB_Alarm,
               "Alarm Task",
               AlarmTask,
               0,
               ALARM_TASK_PRIO,
               &STK_Alarm[0],
               ALARM_TASK_STK_SIZE / 10,
               ALARM_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // touch sensor task create (PRIO 5)
  OSTaskCreate(&TCB_Touch,
               "Touch Sensor Task",
               TouchSensorTask,
               0,
               TOUCH_TASK_PRIO,
               &STK_Touch[0],
               TOUCH_TASK_STK_SIZE / 10,
               TOUCH_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // mission task create (PRIO 6)
  OSTaskCreate(&TCB_Mission,
               "Mission Task",
               MissionTask,
               0,
               MISSION_TASK_PRIO,
               &STK_Mission[0],
               MISSION_TASK_STK_SIZE / 10,
               MISSION_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // knock sensor task create (PRIO 7)
  OSTaskCreate(&TCB_Knock,
               "Knock Sensor Task",
               KnockSensorTask,
               0,
               KNOCK_TASK_PRIO,
               &STK_Knock[0],
               KNOCK_TASK_STK_SIZE / 10,
               KNOCK_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // joystick task create (PRIO 7)
  OSTaskCreate(&TCB_Joystick,
               "Joystick Task",
               JoystickTask,
               0,
               JOYSTICK_PRIO,
               &STK_Joystick[0],
               JOYSTICK_STK_SIZE / 10,
               JOYSTICK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // button task create (PRIO 7)
  OSTaskCreate(&TCB_Button,
               "Button Task",
               ButtonTask,
               0,
               BUTTON_TASK_PRIO,
               &STK_Button[0],
               BUTTON_TASK_STK_SIZE / 10,
               BUTTON_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);

  // USART task create (PRIO 8)
  OSTaskCreate(&TCB_USART,
               "USART Task",
               USARTTask,
               0,
               USART_TASK_PRIO,
               &STK_USART[0],
               USART_TASK_STK_SIZE / 10,
               USART_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);
}

// joystick task
static void JoystickTask(void *p_arg)
{
  OS_ERR err;

  uint16_t joystickX, joystickY;
  char buffer[50];
  srand(time(NULL));
  int random_number = rand() % 5 + 1;
  int pattern[5][4] = {{1, 2, 3, 4}, {1, 2, 4, 3}, {1, 3, 2, 4}, {1, 3, 4, 2}, {1, 4, 2, 3}};
  left = pattern[random_number][0];
  right = pattern[random_number][1];
  up = pattern[random_number][2];
  down = pattern[random_number][3];

  while (1)
  {
    joystickX = JoyStick_ReadX();
    joystickY = JoyStick_ReadY();

    // UART로 출력
    sprintf(buffer, "X: %d, Y: %d\n", joystickX * 7, joystickY);

    if ((joystickX * 7 > 300 && joystickX * 7 < 600) && (joystickY > 1000 && joystickY < 3000))
    {
      USART_SendString("\r\n");
    }
    else if (joystickX * 7 > 1000)
    {
      right -= 1;
      if (right < 0)
      {
        right = 0;
      }
    }
    else if (joystickX * 7 < 100)
    {
      left -= 1;
      if (left < 0)
      {
        left = 0;
      }
    }
    else if (joystickY > 3000)
    {
      up -= 1;
      if (up < 0)
      {
        up = 0;
      }
    }
    else
    {
      down -= 1;
      if (down < 0)
      {
        down = 0;
      }
    }
    sprintf(buffer, "left: %d right: %d up: %d down: %d\r\n", left, right, up, down);
    USART_SendString(buffer);

    if (left == 0 && right == 0 && up == 0 && down == 0)
    {
      break;
    }
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == SET)
    {
      USART_SendString("Button Pressed\n");
    }

    OSTimeDly(200, OS_OPT_TIME_PERIODIC, &err);
  }
}
