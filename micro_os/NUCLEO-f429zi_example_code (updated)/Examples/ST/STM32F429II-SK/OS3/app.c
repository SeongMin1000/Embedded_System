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
#include <time.h>
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

// touch sensor
#define TOUCH_TASK_PRIO 4u
#define TOUCH_TASK_STK_SIZE 512u

// mission
#define MISSION_TASK_PRIO 5u
#define MISSION_TASK_STK_SIZE 512u

// joystick
#define JOYSTICK_PRIO 6u
#define JOYSTICK_STK_SIZE 512u

// button
#define BUTTON_TASK_PRIO 6u
#define BUTTON_TASK_STK_SIZE 512u

// USART
#define USART_TASK_PRIO 7u
#define USART_TASK_STK_SIZE 512u

// time monitor
#define TIME_MONITOR_TASK_PRIO 8u
#define TIME_MONITOR_TASK_STK_SIZE 512u
//-------------------------------------------------------------------------

#define JOYSTICK_X_PIN GPIO_Pin_0   // PA0
#define JOYSTICK_Y_PIN GPIO_Pin_7   // PA7
#define JOYSTICK_SW_PIN GPIO_Pin_12 // PC13 (SW 버튼)

#define ALARM_FLAG_BUZZER 0x01u  // 0001
#define ALARM_FLAG_TOUCH 0x02u   // 0010 (터치 센서 태스크 대기용)
#define ALARM_FLAG_MISSION 0x04u // 0100 (미션 태스크 시작 신호)
#define ALARM_FLAG_BUTTON 0x08u
#define ALARM_FLAG_BUTTON_SUCCESS 0x40u

#define ALARM_FLAG_OFF 0x10u
#define ALARM_FLAG_JOYSTICK_START 0x20u

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

typedef enum
{
  SENSOR_BUTTON,
  SENSOR_JOYSTICK_LEFT,
  SENSOR_JOYSTICK_RIGHT,
  SENSOR_JOYSTICK_UP,
  SENSOR_JOYSTICK_DOWN
} SensorType;

typedef struct
{
  SensorType type;
} SensorInput;

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
static void Buzzer_Init(void);
static void JoyStick_Init(void);
static void Button_Init(void);
//-------------------------------------------------------------------------

//------------------------------util function------------------------------
// USART utils
static void USART_SendChar(uint8_t c);
static void USART_SendString(const char *s);
void USART_ReceiveString(char *buffer, uint16_t len);

// RTC utils
int is_valid_time_format(const char *str);
void RTC_CustomSetTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void RTC_GetTimeStr(char *buf, size_t len);
void RTC_SetAlarmDaily(void);

// buzzer utils
void Buzzer_On(void);
void Buzzer_Off(void);

// joystick utils
uint16_t JoyStick_ReadX(void);
uint16_t JoyStick_ReadY(void);

// button utils
uint8_t Button_Read(void);

//-------------------------------------------------------------------------

//-------------------------------task--------------------------------------
static void BuzzerTask(void *p_arg);      // PRIO 3
static void TouchSensorTask(void *p_arg); // PRIO 4
static void MissionTask(void *p_arg);     // PRIO 5
// 미션 입력 센서 2개 같은 순위
static void JoystickTask(void *p_arg); // PRIO 6
static void ButtonTask(void *p_arg);   // PRIO 6
static void USARTTask(void *p_arg);    // PRIO 7
static void TimeMonitorTask(void *p_arg);

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

// 알람 시각
RTC_TimeTypeDef g_alarmTime;

uint8_t input_index = 0;
uint8_t is_target_set = 0;

// 버튼, 조이스틱 미션
volatile uint8_t random_index;
int button_missions[3] = {1, 2, 3};
int Joystick_missions[3][3] = {
    {2, 1, 1}, {1, 2, 4}, {3, 1, 2}};

//-----------------------------TCB & STACK ---------------------------------
// buzzer
static OS_TCB TCB_Buzzer;
static CPU_STK STK_Buzzer[BUZZER_TASK_STK_SIZE];

// touch sensor
static OS_TCB TCB_Touch;
static CPU_STK STK_Touch[TOUCH_TASK_STK_SIZE];

// mission
static OS_TCB TCB_Mission;
static CPU_STK STK_Mission[MISSION_TASK_STK_SIZE];

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

// Time Monitor Task
static OS_TCB TCB_TimeMonitor;
static CPU_STK STK_TimeMonitor[TIME_MONITOR_TASK_STK_SIZE];
//-------------------------------------------------------------------------

OS_FLAG_GRP AlarmFlagGroup; // 알람 이벤트 플래그
OS_Q SensorInputQ;          // input 센서 입력 큐로 처리
OS_Q USARTMsgQ;             // usart 출력 메시지 큐로 처리

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
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6; // PA6
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

void USART_ReceiveString(char *buffer, uint16_t len)
{
  uint16_t i = 0;

  while (i < len - 1)
  {
    while (USART_GetFlagStatus(NUCLEO_COM1, USART_FLAG_RXNE) == RESET)
      ;

    char c = USART_ReceiveData(NUCLEO_COM1);

    // 엔터 입력 시 종료
    if (c == '\r' || c == '\n')
    {
      USART_SendString("\r\n");
      break;
    }

    // 백스페이스 처리
    if (c == '\b' || c == 127) // '\b' or DEL
    {
      if (i > 0)
      {
        i--;                       // 커서 뒤로
        buffer[i] = '\0';          // 마지막 문자 제거
        USART_SendString("\b \b"); // 화면에서도 지우기
      }
      continue;
    }

    buffer[i++] = c;
    USART_SendChar(c);
  }

  buffer[i] = '\0'; // 널 종료
}

/*
*********************************************************************************************************
*                                       RTC FUNCTIONS
*********************************************************************************************************
*/

int is_valid_time_format(const char *str)
{
  // 1. 길이 확인
  if (strlen(str) != 8)
    return 0;

  // 2. ':' 위치 확인
  if (str[2] != ':' || str[5] != ':')
    return 0;

  // 3. 숫자 위치 확인
  for (int i = 0; i < 8; i++)
  {
    if (i == 2 || i == 5)
      continue;

    if (str[i] < '0' || str[i] > '9')
      return 0;
  }

  // 4. 숫자로 변환 후 범위 확인
  int hours = (str[0] - '0') * 10 + (str[1] - '0');
  int minutes = (str[3] - '0') * 10 + (str[4] - '0');
  int seconds = (str[6] - '0') * 10 + (str[7] - '0');

  if (hours > 23 || minutes > 59 || seconds > 59)
    return 0;

  return 1;
}

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
  // 1. 알람 인터럽트 비활성화 & 플래그 클리어
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
  RTC_ClearITPendingBit(RTC_IT_ALRA);
  EXTI_ClearITPendingBit(EXTI_Line17);

  // 2. 외부 인터럽트 설정 (RTC 알람용 EXTI17)
  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.EXTI_Line = EXTI_Line17;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  // 3. 알람 시간 설정 ( 사용자 입력으로 변경 )
  USART_Config();
  uint8_t hours, minutes, seconds;
  char time_buffer[9]; // HH:MM:SS

  // 시간 입력 받기 (제대로 된 입력이 아닐 시 다시 입력 받도록 수정)
  while (1)
  {
    USART_SendString("알람 시각을 설정하세요 (HH:MM:SS)\r\n");
    USART_ReceiveString(time_buffer, 9);
    USART_SendString("\r\n");

    // 검증
    if (is_valid_time_format(time_buffer))
    {
      hours = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
      minutes = (time_buffer[3] - '0') * 10 + (time_buffer[4] - '0');
      seconds = (time_buffer[6] - '0') * 10 + (time_buffer[7] - '0');
      break;
    }

    USART_SendString("\r\n잘못된 시간 형식 또는 범위입니다. 다시 입력해주세요.\r\n");
  }

  RTC_AlarmTypeDef alarm;
  alarm.RTC_AlarmTime.RTC_Hours = hours;
  alarm.RTC_AlarmTime.RTC_Minutes = minutes;
  alarm.RTC_AlarmTime.RTC_Seconds = seconds;
  alarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; // 날짜 무시
  alarm.RTC_AlarmDateWeekDay = 1;                  // 무시됨
  alarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm);

  char buf[32];
  RTC_GetTimeStr(buf, sizeof(buf));
  USART_Config();
  USART_SendString("\r\n[현재 시각] ");
  USART_SendString(buf);
  USART_SendString("\r\n");

  char alarm_time_msg[64];
  snprintf(alarm_time_msg, sizeof(alarm_time_msg), "%02d:%02d:%02d 에 알람이 울립니다...\r\n\r\n", hours, minutes, seconds);
  USART_SendString(alarm_time_msg);

  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm);

  // 4. 인터럽트 설정 및 알람 켜기
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

  // 5. NVIC 등록
  NVIC_EnableIRQ(RTC_Alarm_IRQn);

  // 6. 알람 울리기 전 카운트 다운을 위한 변수 설정
  g_alarmTime.RTC_Hours = hours;
  g_alarmTime.RTC_Minutes = minutes;
  g_alarmTime.RTC_Seconds = seconds;
}

/*
*********************************************************************************************************
*                                           BUZZER FUNCTIONS
*********************************************************************************************************
*/

void Buzzer_On(void) // LOW에서 ON
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

void Buzzer_Off(void) //// HIGH에서 OFF
{
  GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

/*
*********************************************************************************************************
*                                           JOYSTICK FUNCTIONS
*********************************************************************************************************
*/

// Read analog value
uint16_t JoyStick_ReadX(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
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

uint8_t Button_Read(void)
{
  return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
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

  // 현재 시간 설정
  RTC_CustomInit();
  RTC_CustomSetTime(8, 59, 50);

  // 사용자 알람 설정
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

  // HW INIT
  BSP_Init();
  BSP_Tick_Init();
  Buzzer_Init();
  TouchSensor_Init();
  JoyStick_Init();
  Button_Init();
  USART_Config();

  Buzzer_Off();

  // Alarm Flag create
  OSFlagCreate(&AlarmFlagGroup, "Alarm Flag Group", (OS_FLAGS)0, &err);
  // Usart Message Queue create
  OSQCreate(&USARTMsgQ, "USART Msg Queue", 10, &err);
  // Sensor Input Queue create
  OSQCreate(&SensorInputQ, "Sensor Input Queue", 10, &err);

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

  // touch sensor task create (PRIO 4)
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

  // mission task create (PRIO 5)
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

  // button task create(PRIO 6)
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

  // USART task create (PRIO 7)
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

  // Time Monitor Task (PRIO 8)
  OSTaskCreate(&TCB_TimeMonitor,
               "Time Monitor Task",
               TimeMonitorTask,
               0,
               TIME_MONITOR_TASK_PRIO,
               &STK_TimeMonitor[0],
               TIME_MONITOR_TASK_STK_SIZE / 10,
               TIME_MONITOR_TASK_STK_SIZE,
               0,
               0,
               0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
               &err);
}

// Buzzer Task
static void BuzzerTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 알람 발생까지 대기 (알람 ON 신호 올 때까지 무한대기)
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_BUZZER,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    OSFlagPend(&AlarmFlagGroup,
               ALARM_FLAG_OFF,
               0,
               OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_NON_BLOCKING,
               0,
               &err);

    // 2. 부저 울림 반복 (알람 OFF 신호 올 때까지)
    static const char msg_on[] = "\r\n알람이 울리는 중...\r\n터치 센서를 눌러 미션을 수행하세요!\r\n";
    OSQPost(&USARTMsgQ, (void *)msg_on, sizeof(msg_on), OS_OPT_POST_FIFO, &err);

    while (DEF_TRUE)
    {
      // 깜빡임 효과 (비프 ON)
      Buzzer_On();
      OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); // 0.3초 ON

      // 깜빡임 효과 (비프 OFF)
      Buzzer_Off();
      OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err); // 0.2초 OFF

      // 알람 OFF 신호가 들어왔는지 체크
      if (OSFlagPend(&AlarmFlagGroup, ALARM_FLAG_OFF, 0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0, &err))
      {
        break;
      }
    }

    // 3. 부저 완전히 종료
    Buzzer_Off();
    static const char msg_off[] = "알람이 해제되었습니다!\r\n\r\n";
    OSQPost(&USARTMsgQ, (void *)msg_off, sizeof(msg_off), OS_OPT_POST_FIFO, &err);
  }
}

// Touch Sensor Task
static void TouchSensorTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  uint8_t prev, curr;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 알람 울림 신호 기다림
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_TOUCH,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 터치센서 OFF될 때까지 대기
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET)
    {
      OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    prev = Bit_RESET;

    // 3. 터치 감지(딱 한 번)
    int touched = 0;
    while (!touched)
    {
      curr = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
      if (curr == Bit_SET && prev == Bit_RESET)
      {
        static char mission_start[] = "\r\n터치 성공! 미션 시작!\r\n";
        OSQPost(&USARTMsgQ, (void *)mission_start, sizeof(mission_start), OS_OPT_POST_FIFO, &err);
        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);

        // 터치된 순간의 초를 캡쳐해 랜덤한 인덱스 생성
        RTC_TimeTypeDef rtc_time;
        RTC_GetTime(RTC_Format_BIN, &rtc_time);
        random_index = rtc_time.RTC_Seconds % 3;

        // MissionTask에 신호
        OSFlagPost(&AlarmFlagGroup,
                   ALARM_FLAG_MISSION,
                   OS_OPT_POST_FLAG_SET,
                   &err);

        touched = 1;
      }
      prev = curr;
      OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    }

    // 4. 알람이 해제될 때까지 Block (ALARM_FLAG_OFF 대기)
    OSFlagPend(&AlarmFlagGroup,
               ALARM_FLAG_OFF,
               0,
               OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
               0,
               &err);
  }
}

static void MissionTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  void *msg;
  OS_MSG_SIZE msg_size;
  SensorInput input;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 터치 감지될 때까지 대기 (미션 시작 신호)
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_MISSION,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 랜덤 인덱스 기반 미션 선택
    int required_button = button_missions[random_index];
    int *required_joy = Joystick_missions[random_index];

    // 조이스틱 방향 카운터 초기화
    int left = 0, right = 0, up = 0, down = 0;
    for (int i = 0; i < 3; i++)
    {
      if (required_joy[i] == 1)
        left++;
      else if (required_joy[i] == 2)
        right++;
      else if (required_joy[i] == 3)
        up++;
      else if (required_joy[i] == 4)
        down++;
    }

    // 미션 안내 출력
    char buf[128];
    snprintf(buf, sizeof(buf),
             "[미션 1]: 버튼 %d회를 누르세요!\r\n", required_button);
    OSQPost(&USARTMsgQ, (void *)buf, strlen(buf) + 1, OS_OPT_POST_FIFO, &err);

    // 버튼 태스크 시작 신호
    OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_BUTTON, OS_OPT_POST_FLAG_SET, &err);

    int button_cnt = 0;
    int mission1_done = 0;

    // 3. 버튼 미션 수행 루프
    while (!mission1_done)
    {
      // 강제 종료 확인
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_OFF,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0,
                     &err))
        return;

      msg = OSQPend(&SensorInputQ, 0, OS_OPT_PEND_BLOCKING, &msg_size, 0, &err);
      if (err != OS_ERR_NONE)
        continue;

      input = *(SensorInput *)msg;

      if (input.type == SENSOR_BUTTON)
      {
        button_cnt++;
        if (button_cnt >= required_button)
        {
          OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
          static const char msg[] = "미션 1 성공!\r\n\r\n[미션 2]: ";
          OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
          mission1_done = 1;

          // 버튼 태스크에게 알림
          OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_BUTTON_SUCCESS, OS_OPT_POST_FLAG_SET, &err);

          // 조이스틱 미션 안내
          char joy_msg[128] = {0};
          strcat(joy_msg, "조이스틱을 ");
          for (int i = 0; i < 3; i++)
          {
            if (required_joy[i] == 1)
              strcat(joy_msg, "← ");
            else if (required_joy[i] == 2)
              strcat(joy_msg, "→ ");
            else if (required_joy[i] == 3)
              strcat(joy_msg, "↑ ");
            else if (required_joy[i] == 4)
              strcat(joy_msg, "↓ ");
          }
          strcat(joy_msg, "방향으로 움직여주세요\r\n");
          OSQPost(&USARTMsgQ, (void *)joy_msg, strlen(joy_msg) + 1, OS_OPT_POST_FIFO, &err);
        }
      }
    }

    // 4. 조이스틱 미션 수행 루프
    while (1)
    {
      // 강제 종료 확인
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_OFF,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0,
                     &err))
        return;

      msg = OSQPend(&SensorInputQ, 0, OS_OPT_PEND_BLOCKING, &msg_size, 0, &err);
      if (err != OS_ERR_NONE)
        continue;

      input = *(SensorInput *)msg;

      if (input.type == SENSOR_JOYSTICK_LEFT && left > 0)
        left--;
      else if (input.type == SENSOR_JOYSTICK_RIGHT && right > 0)
        right--;
      else if (input.type == SENSOR_JOYSTICK_UP && up > 0)
        up--;
      else if (input.type == SENSOR_JOYSTICK_DOWN && down > 0)
        down--;

      if (left <= 0 && right <= 0 && up <= 0 && down <= 0)
      {
        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
        static const char msg[] = "\r\n미션 2 성공! 알람이 해제됩니다.\r\n\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);

        // 최종 알람 해제
        OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_OFF, OS_OPT_POST_FLAG_SET, &err);
        break;
      }
    }
  }
}

// Button Task
static void ButtonTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  uint8_t prev, curr;
  SensorInput input;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // ALARM_FLAG_BUTTON 받을 때까지 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_BUTTON,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    prev = Button_Read();
    while (1)
    {
      // 미션 성공 시 종료
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_BUTTON_SUCCESS,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0, &err))
      {
        OSFlagPost(&AlarmFlagGroup,
                   ALARM_FLAG_JOYSTICK_START,
                   OS_OPT_POST_FLAG_SET,
                   &err);
        break;
      }

      curr = Button_Read();
      if (curr == Bit_RESET && prev == Bit_SET) // 버튼 눌림 (Active Low)
      {
        input.type = SENSOR_BUTTON;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        static const char msg[] = "[버튼] 눌림 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);

        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err); // 디바운싱
      }

      prev = curr;
      OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    }
  }
}

// Joystick Task - 센서 방향 감지만 수행 (방향 입력을 SensorInput 구조체로 전송)
static void JoystickTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  SensorInput input;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 미션 시작 신호 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_JOYSTICK_START,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 조이스틱이 중앙 위치일 때까지 대기 (초기화 목적)
    uint16_t joystickX, joystickY;
    do
    {
      joystickX = JoyStick_ReadX();
      joystickY = JoyStick_ReadY();
      OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    } while (!((joystickX * 7 > 300 && joystickX * 7 < 600) && (joystickY > 1000 && joystickY < 3000)));

    // 3. 방향 감지 루프
    uint16_t prevX = joystickX;
    uint16_t prevY = joystickY;

    while (DEF_TRUE)
    {
      joystickX = JoyStick_ReadX();
      joystickY = JoyStick_ReadY();

      // → 오른쪽
      if (joystickX * 7 > 1000 && prevX * 7 <= 1000)
      {
        input.type = SENSOR_JOYSTICK_RIGHT;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        static const char msg[] = "[조이스틱] → 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }
      // ← 왼쪽
      else if (joystickX * 7 < 100 && prevX * 7 >= 100)
      {
        input.type = SENSOR_JOYSTICK_LEFT;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        static const char msg[] = "[조이스틱] ← 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }
      // ↑ 위
      else if (joystickY > 3000 && prevY <= 3000)
      {
        input.type = SENSOR_JOYSTICK_UP;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        static const char msg[] = "[조이스틱] ↑ 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }
      // ↓ 아래
      else if (joystickY < 1000 && prevY >= 1000)
      {
        input.type = SENSOR_JOYSTICK_DOWN;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        static const char msg[] = "[조이스틱] ↓ 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }

      prevX = joystickX;
      prevY = joystickY;

      // 미션 완료 확인은 MissionTask가 하므로 여기선 무한 반복
      OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
  }
}

// USART Task
static void USARTTask(void *p_arg)
{
  OS_ERR err;
  void *msg;
  OS_MSG_SIZE size;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 메시지 큐에서 문자열 수신 (blocking)
    msg = OSQPend(&USARTMsgQ,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  &size,
                  0,
                  &err);

    if (err == OS_ERR_NONE && msg != NULL)
    {
      // 문자열 출력
      USART_SendString((char *)msg);
    }
  }
}

// Timer Monitor Task
static void TimeMonitorTask(void *p_arg)
{
  OS_ERR err;
  (void)p_arg;

  while (DEF_TRUE)
  {
    RTC_TimeTypeDef now;
    RTC_GetTime(RTC_Format_BIN, &now);

    int now_sec = now.RTC_Hours * 3600 + now.RTC_Minutes * 60 + now.RTC_Seconds;
    int alarm_sec = g_alarmTime.RTC_Hours * 3600 + g_alarmTime.RTC_Minutes * 60 + g_alarmTime.RTC_Seconds;

    int remaining = alarm_sec - now_sec;

    // 알람 울리기 3초 전에 시간 표시
    if (remaining <= 3 && remaining > 0)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d\r\n", now.RTC_Hours, now.RTC_Minutes, now.RTC_Seconds);
      OSQPost(&USARTMsgQ, (void *)buf, sizeof(buf), OS_OPT_POST_FIFO, &err);
    }

    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err); // 1초 대기
  }
}
