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

// touch sensor
#define TOUCH_TASK_PRIO 4u
#define TOUCH_TASK_STK_SIZE 512u

// mission
#define MISSION_TASK_PRIO 5u
#define MISSION_TASK_STK_SIZE 512u

// knock sensor
#define KNOCK_TASK_PRIO 6u
#define KNOCK_TASK_STK_SIZE 512u

// joystick
#define JOYSTICK_PRIO 6u
#define JOYSTICK_STK_SIZE 512u

// button
#define BUTTON_TASK_PRIO 6u
#define BUTTON_TAASK_STK_SIZE 512u

// USART
#define USART_TASK_PRIO 7u
#define USART_TASK_STK_SIZE 512u
//-------------------------------------------------------------------------

#define JOYSTICK_X_PIN GPIO_Pin_0   // PA0
#define JOYSTICK_Y_PIN GPIO_Pin_7   // PA7
#define JOYSTICK_SW_PIN GPIO_Pin_12 // PC13 (SW 버튼)

#define ALARM_FLAG_BUZZER 0x01u  // 0001
#define ALARM_FLAG_TOUCH 0x02u   // 0010 (터치 센서 태스크 대기용)
#define ALARM_FLAG_MISSION 0x04u // 0100 (미션 태스크 시작 신호)
#define ALARM_FLAG_KNOCK 0x08u
#define ALARM_FLAG_KNOCK_SUCCESS    0x40u

#define ALARM_FLAG_OFF 0x10u // 1000
#define ALARM_FLAG_JOYSTICK_START  0x20u

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
 Mission missionsjoy[] = {
		 {2, 1, 1}, {1, 2, 4}, {2, 2, 2}, {3, 1, 3}, {1, 1, 4}};

// 진동 감지 센서만 test
Mission missions[] = {
    {0, 1, 0}, // 노크 1회만 수행
    {0, 2, 0}, // 노크 2회
    {0, 3, 0}  // 노크 3회
};

typedef enum
{
  SENSOR_BUTTON,
  SENSOR_KNOCK,
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
static void TouchSensorTask(void *p_arg); // PRIO 4
static void MissionTask(void *p_arg);     // PRIO 5
// 미션 입력 센서 3개 같은 순위
static void KnockSensorTask(void *p_arg); // PRIO 6
static void JoystickTask(void *p_arg);    // PRIO 6
static void ButtonTask(void *p_arg);      // PRIO 6
static void USARTTask(void *p_arg);       // PRIO 7

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
// volatile uint8_t alarm_flag = 0; // bsp_int.c

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

  // 3. 알람 시간 설정 (매일 09:00:00)
  RTC_AlarmTypeDef alarm;
  alarm.RTC_AlarmTime.RTC_Hours = 9;
  alarm.RTC_AlarmTime.RTC_Minutes = 0;
  alarm.RTC_AlarmTime.RTC_Seconds = 0;
  alarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; // 날짜 무시
  alarm.RTC_AlarmDateWeekDay = 1;                  // 무시됨
  alarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm);

  // 4. 인터럽트 설정 및 알람 켜기
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

  // 5. NVIC 등록
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
  RTC_CustomSetTime(8, 59, 58);
  RTC_SetAlarmDaily();

  // 현재 시각 UART로 출력 (디버깅용)
  char buf[32];
  RTC_GetTimeStr(buf, sizeof(buf));
  USART_Config(); // 반드시 먼저 USART 초기화하고
  USART_SendString("\r\n[현재 시각] ");
  USART_SendString(buf);
  USART_SendString("\r\n");

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
  KnockSensor_Init();
  //  JoyStick_Init();
  //  Button_Init();
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

  // knock sensor task create(PRIO 6)
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

  // button task create (PRIO 6)
  // OSTaskCreate(&TCB_Button,
  //              "Button Task",
  //              ButtonTask,
  //              0,
  //              BUTTON_TASK_PRIO,
  //              &STK_Button[0],
  //              BUTTON_TASK_STK_SIZE / 10,
  //              BUTTON_TASK_STK_SIZE,
  //              0,
  //              0,
  //              0,
  //              OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
  //              &err);

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
    static const char msg_on[] = "\r\n[부저] 알람이 울리는 중...\r\n 터치 센서를 눌러 미션을 수행하세요!\r\n";
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
    static const char msg_off[] = "[부저] 알람이 해제되었습니다.\r\n";
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

    // **여기서 break가 아니라, 자연스럽게 루프 첫 부분(1)로 돌아가야 함!**
    // break;  <<<< 쓰면 안 됨!
    // continue; 혹은 아무 처리 없이 while(DEF_TRUE) 첫 부분으로 돌아가게
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
    // 1. 터치 감지될 때까지 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_MISSION,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 미션 랜덤 선택
    int knock_missions[] = {1, 2, 3};
    int idx = rand() % (sizeof(knock_missions) / sizeof(int));
    int required_knock = knock_missions[idx];
    USART_SendString("미션태스크 진입 확인\r\n");
    static const char start_msg[] = "\r\n[미션] 테스트 모드: 노크 센서만 사용 중!\r\n";
    OSQPost(&USARTMsgQ, (void *)start_msg, sizeof(start_msg), OS_OPT_POST_FIFO, &err);

    char buf[128];
    snprintf(buf, sizeof(buf),
             "[테스트 미션] 노크 %d회를 감지하세요.\r\n", required_knock);
    OSQPost(&USARTMsgQ, (void *)buf, strlen(buf) + 1, OS_OPT_POST_FIFO, &err);

    OSFlagPost(&AlarmFlagGroup,
               ALARM_FLAG_KNOCK,
               OS_OPT_POST_FLAG_SET,
               &err);

    int knock_cnt = 0;
    int mission_fail = 0;

    while (1)
    {
      // 알람 해제 신호 오면 즉시 종료(예: 외부에서 강제 종료)
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_OFF,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0,
                     &err))
      {
        // 강제 종료(성공/실패 상관없이 태스크 종료)
        return;
      }

      msg = OSQPend(&SensorInputQ, 0, OS_OPT_PEND_BLOCKING, &msg_size, 0, &err);
      if (err != OS_ERR_NONE)
        continue;

      input = *(SensorInput *)msg;

      if (input.type == SENSOR_KNOCK)
      {
        knock_cnt++;
      }
      else
      {
        mission_fail = 1;
        static const char fail_msg[] = "[실패] 허용되지 않은 입력입니다. 다시 시도하세요.\r\n";
        OSQPost(&USARTMsgQ, (void *)fail_msg, sizeof(fail_msg), OS_OPT_POST_FIFO, &err);
        return; // 미션 실패!
      }

      if (knock_cnt >= required_knock)
      {

        OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
        USART_SendString("[미션] 테스트 모드: 조이스틱 모듈 사용 중\r\n");
        OSFlagPost(&AlarmFlagGroup,
            			  ALARM_FLAG_KNOCK_SUCCESS,
            	  					 OS_OPT_POST_FLAG_SET,
            	  					 &err);




        /*
        // 재시작 테스트
        RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
        RTC_ClearITPendingBit(RTC_IT_ALRA);
        EXTI_ClearITPendingBit(EXTI_Line17);

        RTC_CustomSetTime(8, 59, 58);
        // 꼭 50~100ms 정도 대기!
        OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);

        RTC_SetAlarmDaily();
        continue;
        ;
        */
      }
    }

    // 이 아래는 “실패” 했을 때만 실행됨
    if (mission_fail)
    {
      OSTimeDlyHMSM(0, 0, 0, 700, OS_OPT_TIME_HMSM_STRICT, &err); // 재도전 전 잠깐 쉬기
      continue;                                                   // 바깥 while(DEF_TRUE)로 돌아가서 미션 새로 안내
    }
  }
}

// Knock sensor Task
static void KnockSensorTask(void *p_arg)
{
  OS_ERR err;
  SensorInput input;
  OS_FLAGS flags;
  uint8_t prev, curr;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 미션 시작(터치 플래그)까지 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_KNOCK,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 미션 종료(ALARM_FLAG_OFF)까지 반복 감지
    prev = KnockSensor_Read();
    while (1)
    {
      // 미션 종료 플래그 오면 바로 대기 상태로 돌아감
      if (OSFlagPend(&AlarmFlagGroup,
    		  	  	  ALARM_FLAG_KNOCK_SUCCESS,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0,
                     &err))
      {
    	  OSFlagPost(&AlarmFlagGroup,
    	  					 ALARM_FLAG_JOYSTICK_START,
    	  					 OS_OPT_POST_FLAG_SET,
    	  					 &err);
    	  break;
      }

      curr = KnockSensor_Read();

      if (curr == Bit_SET && prev == Bit_RESET)
      {
        input.type = SENSOR_KNOCK;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        static const char msg[] = "[노크] 입력 감지됨\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);

        // 800ms(혹은 적절한 값) 동안 입력 무시
        OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
      }
      else
      {
        OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
      }

      prev = curr;
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


void JoystickTask(void *p_arg) {
	OS_ERR err;
	OS_FLAGS flags;

	flags = OSFlagPend(&AlarmFlagGroup,
		                       ALARM_FLAG_JOYSTICK_START,
		                       0,
		                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
		                       0,
		                       &err);

	BSP_Init();
	BSP_Tick_Init();
	USART_Config();
	JoyStick_Init();


    uint16_t joystickX, joystickY;
    char buffer[50];
    srand(time(NULL));
    int random_number = rand() % 5 + 1;
    int pattern[5][4]={{1, 2, 3, 4},{1, 2, 4, 3},{1, 3, 2, 4},{1, 3, 4, 2},{1, 4, 2, 3}};
    left = pattern[random_number][0];
    right = pattern[random_number][1];
    up = pattern[random_number][2];
    down = pattern[random_number][3];

    USART_SendString("[테스트 미션] 조이스틱 모듈을 움직이세요\r\n");
    while (1) {
        joystickX = JoyStick_ReadX();
        joystickY = JoyStick_ReadY();


        // 조이스틱 중앙값 범위 내에 있으면 방향 잔여값 출력
        if((joystickX*7 >300 && joystickX*7 <600) && (joystickY >1000 && joystickY<3000)){
            sprintf(buffer, "left: %d right: %d up: %d down: %d\r\n", left, right, up, down);
            USART_SendString(buffer);
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

        if (left == 0 && right == 0 && up == 0 && down == 0) {
            OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_OFF, OS_OPT_POST_FLAG_SET, &err);
            break;
        }

        OSTimeDly(100, OS_OPT_TIME_PERIODIC, &err);
    }
}
