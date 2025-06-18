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
#define JOYSTICK_SW_PIN GPIO_Pin_12 // PC13

#define ALARM_FLAG_BUZZER 0x01u
#define ALARM_FLAG_TOUCH 0x02u
#define ALARM_FLAG_MISSION 0x04u
#define ALARM_FLAG_BUTTON 0x08u
#define ALARM_FLAG_OFF 0x10u
#define ALARM_FLAG_JOYSTICK_START 0x20u
#define ALARM_FLAG_BUTTON_SUCCESS 0x40u

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

// 센서 종류를 정의
// 각 항목은 미션 수행을 위한 센서 입력 종류를 나타냄
typedef enum
{
  SENSOR_BUTTON,         // 버튼 입력 감지
  SENSOR_JOYSTICK_LEFT,  // 조이스틱 왼쪽 방향 입력
  SENSOR_JOYSTICK_RIGHT, // 조이스틱 오른쪽 방향 입력
  SENSOR_JOYSTICK_UP,    // 조이스틱 위 방향 입력
  SENSOR_JOYSTICK_DOWN   // 조이스틱 아래 방향 입력
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

char target_time[16] = {0};
char current_time[16] = {0};
char input_buffer[32];

// 알람 시각
RTC_TimeTypeDef g_alarmTime;

// 무작위 미션 선택용 인덱스 (0~2)
volatile uint8_t random_index;

// 버튼미션 (눌러야 하는 횟수 1~3회)
int button_missions[3] = {1, 2, 3};

// 조이스틱 미션 (방향 코드 3개로 구성된 미션 3종)
// 방향: 1=LEFT, 2=UP, 3=RIGHT, 4=DOWN
int Joystick_missions[3][3] = {
    {2, 1, 1}, // 미션 1: UP → LEFT → LEFT
    {1, 2, 4}, // 미션 2: LEFT → UP → DOWN
    {3, 1, 2}  // 미션 3: RIGHT → LEFT → UP
};

//-----------------------------TCB & STACK ---------------------------------
// AppTaskStart
static OS_TCB AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_CFG_ApptaskCreate_STK_SIZE];

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

// USART 초기화 함수
static void USART_InitCOM(COM_TypeDef com, const USART_InitTypeDef *cfg)
{
  if (com != COM1)
    return; // COM1 외에는 무시

  // GPIO 및 USART 클럭 활성화
  RCC_AHB1PeriphClockCmd(NUCLEO_COM1_TX_PORT_CLK | NUCLEO_COM1_RX_PORT_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(NUCLEO_COM1_CLK, ENABLE);

  // TX/RX 핀을 USART 기능으로 설정
  GPIO_PinAFConfig(NUCLEO_COM1_TX_PORT, NUCLEO_COM1_TX_SRC, NUCLEO_COM1_TX_AF);
  GPIO_PinAFConfig(NUCLEO_COM1_RX_PORT, NUCLEO_COM1_RX_SRC, NUCLEO_COM1_RX_AF);

  // GPIO 설정: Push-Pull, Pull-up, 50MHz
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  // TX 핀 초기화
  GPIO_InitStruct.GPIO_Pin = NUCLEO_COM1_TX_PIN;
  GPIO_Init(NUCLEO_COM1_TX_PORT, &GPIO_InitStruct);

  // RX 핀 초기화
  GPIO_InitStruct.GPIO_Pin = NUCLEO_COM1_RX_PIN;
  GPIO_Init(NUCLEO_COM1_RX_PORT, &GPIO_InitStruct);

  // USART 설정 적용 및 활성화
  USART_Init(NUCLEO_COM1, (USART_InitTypeDef *)cfg);
  USART_Cmd(NUCLEO_COM1, ENABLE);
}

// USART 설정 함수
static void USART_Config(void)
{
  USART_InitTypeDef cfg;

  // 통신 속도 설정 (115200bps)
  cfg.USART_BaudRate = 115200;

  // 데이터 비트 길이: 8비트
  cfg.USART_WordLength = USART_WordLength_8b;

  // 정지 비트: 1비트
  cfg.USART_StopBits = USART_StopBits_1;

  // 패리티 비트 없음
  cfg.USART_Parity = USART_Parity_No;

  // 하드웨어 흐름 제어 사용 안 함
  cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  // 송신 + 수신 모드 활성화
  cfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  // 설정값으로 COM1 초기화
  USART_InitCOM(COM1, &cfg);
}

// RTC 초기화 함수 (LSI 사용)
static void RTC_CustomInit(void)
{
  // 전원 인터페이스 클럭 활성화
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  // 백업 레지스터 접근 허용
  PWR_BackupAccessCmd(ENABLE);

  // LSI(내부 저속 클럭) 활성화
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    ; // LSI 안정화 대기

  // RTC 클럭 소스로 LSI 설정 후 활성화
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  RCC_RTCCLKCmd(ENABLE);

  // RTC 레지스터 동기화 대기
  RTC_WaitForSynchro();

  // RTC 기본 설정 (24시간제, 프리스케일러 설정)
  RTC_InitTypeDef RTC_InitStruct;
  RTC_StructInit(&RTC_InitStruct);
  RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStruct.RTC_AsynchPrediv = 0x7F;  // 비동기 프리스케일러
  RTC_InitStruct.RTC_SynchPrediv = 0x0130; // 동기 프리스케일러
  RTC_Init(&RTC_InitStruct);
}

// 터치 센서 초기화 함수
static void TouchSensor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIOA 클럭 활성화
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // PA3 (A0 핀)을 입력 모드 + Pull-down으로 설정
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;      // A0 핀 (터치 센서 연결)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;   // 입력 모드
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN; // 기본값 LOW 유지 (풀다운)
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// 부저 센서 초기화 함수
static void Buzzer_Init(void)
{
  // GPIOA 클럭 활성화
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;         // D13 핀 (PA5, 부저 연결)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;     // 출력 모드
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;    // Push-Pull 출력
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  // 풀업/풀다운 없음
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 속도 설정
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// 조이스틱 초기화 함수
static void JoyStick_Init(void)
{
  // GPIOA, ADC1 클럭 활성화
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  // 조이스틱 X, Y 핀을 아날로그 입력 모드로 설정 (PA3, PA7)
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = JOYSTICK_X_PIN | JOYSTICK_Y_PIN; // X=PA3, Y=PA7
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;                   // 아날로그 입력
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;               // 풀업/풀다운 없음
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ADC 설정 (12비트 해상도)
  ADC_InitTypeDef ADC_InitStruct;
  ADC_StructInit(&ADC_InitStruct); // 기본 설정 초기화
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_Init(ADC1, &ADC_InitStruct);

  // ADC1 활성화 및 변환 시작
  ADC_Cmd(ADC1, ENABLE);
  ADC_SoftwareStartConv(ADC1);
}

// 버튼 초기화 함수
static void Button_Init(void)
{
  // GPIOC 클럭 활성화
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;    // PC6 (버튼 연결)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // 입력 모드
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // 풀업 설정 (기본 HIGH)
  GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/*
*********************************************************************************************************
*                                       USART FUNCTIONS
*********************************************************************************************************
*/

// USART로 문자 1개 전송
static void USART_SendChar(uint8_t c)
{
  // 송신 버퍼가 빌 때까지 대기
  while (USART_GetFlagStatus(NUCLEO_COM1, USART_FLAG_TXE) == RESET)
    ;
  // 문자 전송
  USART_SendData(NUCLEO_COM1, c);
}

// USART로 문자열 전송
static void USART_SendString(const char *s)
{
  // 문자열 끝(null 문자)까지 반복 전송
  while (*s)
    USART_SendChar((uint8_t)*s++);
}

// USART로 문자열 수신
void USART_ReceiveString(char *buffer, uint16_t len)
{
  uint16_t i = 0;

  while (i < len - 1) // 마지막은 널 문자('\0')용
  {
    // 수신 데이터가 올 때까지 대기
    while (USART_GetFlagStatus(NUCLEO_COM1, USART_FLAG_RXNE) == RESET)
      ;

    char c = USART_ReceiveData(NUCLEO_COM1);

    // 엔터 입력 시 수신 종료
    if (c == '\r' || c == '\n')
    {
      USART_SendString("\r\n"); // 줄바꿈 출력
      break;
    }

    // 백스페이스 또는 Delete 처리
    if (c == '\b' || c == 127)
    {
      if (i > 0)
      {
        i--;                       // 인덱스 감소
        buffer[i] = '\0';          // 문자 제거
        USART_SendString("\b \b"); // 화면에서도 지움
      }
      continue;
    }

    buffer[i++] = c;   // 문자 저장
    USART_SendChar(c); // 에코 출력
  }

  buffer[i] = '\0'; // 문자열 종료 (null 문자)
}

/*
*********************************************************************************************************
*                                       RTC FUNCTIONS
*********************************************************************************************************
*/

// "hh:mm:ss" 형식의 문자열이 올바른 시간인지 확인하는 함수
int is_valid_time_format(const char *str)
{
  // 1. 문자열 길이가 8이어야 함 ("hh:mm:ss")
  if (strlen(str) != 8)
    return 0;

  // 2. 구분자 ':'가 올바른 위치에 있어야 함 (index 2, 5)
  if (str[2] != ':' || str[5] != ':')
    return 0;

  // 3. 나머지 문자들이 숫자인지 확인
  for (int i = 0; i < 8; i++)
  {
    if (i == 2 || i == 5)
      continue;
    if (str[i] < '0' || str[i] > '9')
      return 0;
  }

  // 4. 숫자로 변환 후 시간 범위 검사
  int hours = (str[0] - '0') * 10 + (str[1] - '0');
  int minutes = (str[3] - '0') * 10 + (str[4] - '0');
  int seconds = (str[6] - '0') * 10 + (str[7] - '0');

  if (hours > 23 || minutes > 59 || seconds > 59)
    return 0;

  return 1;
}

// RTC 시간 설정 함수
void RTC_CustomSetTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  RTC_TimeTypeDef RTC_Time;
  RTC_Time.RTC_Hours = hours;
  RTC_Time.RTC_Minutes = minutes;
  RTC_Time.RTC_Seconds = seconds;
  RTC_SetTime(RTC_Format_BIN, &RTC_Time); // 바이너리 형식으로 설정
}

// RTC 시간 문자열로 가져오기 (형식: "hh:mm:ss")
void RTC_GetTimeStr(char *buf, size_t len)
{
  RTC_TimeTypeDef RTC_Time;
  RTC_GetTime(RTC_Format_BIN, &RTC_Time);
  snprintf(buf, len, "%02d:%02d:%02d",
           RTC_Time.RTC_Hours, RTC_Time.RTC_Minutes, RTC_Time.RTC_Seconds);
}

// 매일 특정 시간에 울리는 RTC 알람 설정 함수
void RTC_SetAlarmDaily(void)
{
  // 1. 기존 알람 및 인터럽트 비활성화, 플래그 초기화
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
  RTC_ClearITPendingBit(RTC_IT_ALRA);
  EXTI_ClearITPendingBit(EXTI_Line17);

  // 2. EXTI17(RTC 알람용) 인터럽트 라인 설정
  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.EXTI_Line = EXTI_Line17;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  // 3. 사용자로부터 알람 시간 입력 받기 (HH:MM:SS 형식)
  USART_Config(); // 시리얼 통신 설정
  uint8_t hours, minutes, seconds;
  char time_buffer[9]; // 입력받을 문자열 버퍼

  while (1)
  {
    USART_SendString("알람 시각을 설정하세요 (HH:MM:SS)\r\n");
    USART_ReceiveString(time_buffer, 9);

    if (is_valid_time_format(time_buffer)) // 입력 유효성 확인
    {
      hours = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
      minutes = (time_buffer[3] - '0') * 10 + (time_buffer[4] - '0');
      seconds = (time_buffer[6] - '0') * 10 + (time_buffer[7] - '0');
      break;
    }

    USART_SendString("\r\n잘못된 시간 형식 또는 범위입니다. 다시 입력해주세요.\r\n");
  }

  // 4. 알람 시간 설정 (요일/날짜는 무시)
  RTC_AlarmTypeDef alarm;
  alarm.RTC_AlarmTime.RTC_Hours = hours;
  alarm.RTC_AlarmTime.RTC_Minutes = minutes;
  alarm.RTC_AlarmTime.RTC_Seconds = seconds;
  alarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
  alarm.RTC_AlarmDateWeekDay = 1;
  alarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &alarm);

  // 5. 현재 시간 및 알람 설정 안내 출력
  char buf[32];
  RTC_GetTimeStr(buf, sizeof(buf));
  USART_SendString("\r\n[현재 시각] ");
  USART_SendString(buf);
  USART_SendString("\r\n");

  char alarm_time_msg[64];
  snprintf(alarm_time_msg, sizeof(alarm_time_msg), "%02d:%02d:%02d 에 알람이 울립니다...\r\n\r\n",
           hours, minutes, seconds);
  USART_SendString(alarm_time_msg);

  // 6. 알람 인터럽트 활성화 및 알람 시작
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
  NVIC_EnableIRQ(RTC_Alarm_IRQn);

  // 7. 이후 사용을 위한 전역 알람 시간 저장
  g_alarmTime.RTC_Hours = hours;
  g_alarmTime.RTC_Minutes = minutes;
  g_alarmTime.RTC_Seconds = seconds;
}

/*
*********************************************************************************************************
*                                           BUZZER FUNCTIONS
*********************************************************************************************************
*/

// 부저 ON (GPIO LOW일 때 작동하는 능동 부저)
void Buzzer_On(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_5); // LOW 출력 → 부저 울림
}

// 부저 OFF (GPIO HIGH로 설정)
void Buzzer_Off(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_5); // HIGH 출력 → 부저 정지
}

/*
*********************************************************************************************************
*                                           JOYSTICK FUNCTIONS
*********************************************************************************************************
*/

// 조이스틱 X축 아날로그 값 읽기 (PA0)
uint16_t JoyStick_ReadX(void)
{
  // ADC 채널 0 (X축) 설정 및 변환 시작
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
  ADC_SoftwareStartConv(ADC1);

  // 변환 완료까지 대기
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    ;

  // 변환 결과 반환
  return ADC_GetConversionValue(ADC1);
}

// 조이스틱 Y축 아날로그 값 읽기 (PA7)
uint16_t JoyStick_ReadY(void)
{
  // ADC 채널 7 (Y축) 설정 및 변환 시작
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
  ADC_SoftwareStartConv(ADC1);

  // 변환 완료까지 대기
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    ;

  // 변환 결과 반환
  return ADC_GetConversionValue(ADC1);
}

/*
*********************************************************************************************************
*                                           BUTTON FUNCTIONS
*********************************************************************************************************
*/

// 버튼 상태 읽기 (PC6)
// 눌림: 0 (LOW), 안 눌림: 1 (HIGH) - Pull-up 기준
uint8_t Button_Read(void)
{
  return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
}

/*
*********************************************************************************************************
*                                                main
*********************************************************************************************************
*/

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

  // 현재 시간 설정 (8시 59분 50초로 세팅)
  RTC_CustomInit();
  RTC_CustomSetTime(8, 59, 50);

  // 사용자 알람 설정
  RTC_SetAlarmDaily();

  /* OS Init */
  OSInit(&err);

  // AppTaskStart task로 진입
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

  OSStart(&err);

  (void)&err;

  return (0u);
}

/*
*********************************************************************************************************
*                                           TASK FUNCTION
*********************************************************************************************************
*/

// 시스템 시작 태스크 (초기화 및 RTOS 자원 생성)
static void AppTaskStart(void *p_arg)
{
  OS_ERR err;
  (void)p_arg;

  // 1. 하드웨어 초기화
  BSP_Init();         // 기본 보드 초기화
  BSP_Tick_Init();    // SysTick 타이머 설정
  Buzzer_Init();      // 부저 설정
  TouchSensor_Init(); // 터치 센서 설정
  JoyStick_Init();    // 조이스틱 설정
  Button_Init();      // 버튼 설정
  USART_Config();     // 시리얼 통신 설정

  Buzzer_Off(); // 부저 기본 OFF 상태

  // 2. RTOS 통신 자원 생성
  OSFlagCreate(&AlarmFlagGroup, "Alarm Flag Group", (OS_FLAGS)0, &err); // 플래그 그룹 생성
  OSQCreate(&USARTMsgQ, "USART Msg Queue", 10, &err);                   // USART 메시지 큐
  OSQCreate(&SensorInputQ, "Sensor Input Queue", 10, &err);             // 센서 입력 큐

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err); // CPU 사용률 측정 태스크 시작
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset(); // 인터럽트 길이 측정용 초기화
#endif

  // 3. 사용자 태스크 생성
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

  // joystick task create(PRIO 6)
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

// Buzzer Task - 알람 발생 시 부저를 깜빡이며 울리는 태스크
static void BuzzerTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 알람 ON 신호가 올 때까지 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_BUZZER,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 시작 직후 알람 OFF 신호가 이미 있는지 체크
    OSFlagPend(&AlarmFlagGroup,
               ALARM_FLAG_OFF,
               0,
               OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_NON_BLOCKING,
               0,
               &err);

    // 2. 알람 울림 안내 메시지 전송
    static const char msg_on[] = "\r\n알람이 울리는 중...\r\n터치 센서를 눌러 미션을 수행하세요!\r\n";
    OSQPost(&USARTMsgQ, (void *)msg_on, sizeof(msg_on), OS_OPT_POST_FIFO, &err);

    // 3. 부저 ON/OFF 깜빡임 반복 (알람 해제될 때까지)
    while (DEF_TRUE)
    {
      Buzzer_On(); // 부저 ON
      OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);

      Buzzer_Off(); // 부저 OFF
      OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);

      // 알람 OFF 신호가 들어왔는지 확인
      if (OSFlagPend(&AlarmFlagGroup, ALARM_FLAG_OFF, 0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0, &err))
      {
        break;
      }
    }

    // 4. 알람 종료 처리
    Buzzer_Off(); // 완전히 OFF
    static const char msg_off[] = "알람이 해제되었습니다!\r\n\r\n";
    OSQPost(&USARTMsgQ, (void *)msg_off, sizeof(msg_off), OS_OPT_POST_FIFO, &err);
  }
}

// Touch Sensor Task - 터치 입력 감지 및 미션 시작 신호 전송
static void TouchSensorTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  uint8_t prev, curr;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 알람 울림 신호 대기 (터치 활성화 시점)
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_TOUCH,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 터치센서가 손에서 떨어질 때까지 대기 (초기화 목적)
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET)
    {
      OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    prev = Bit_RESET;

    // 3. 터치 한 번만 감지
    int touched = 0;
    while (!touched)
    {
      curr = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
      if (curr == Bit_SET && prev == Bit_RESET) // 상승 엣지 감지
      {
        // 터치 성공 메시지 전송
        static char mission_start[] = "\r\n터치 성공! 미션 시작!\r\n";
        OSQPost(&USARTMsgQ, (void *)mission_start, sizeof(mission_start), OS_OPT_POST_FIFO, &err);
        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);

        // RTC 시간의 초(second)를 이용해 미션 랜덤 인덱스 생성 (0~2)
        RTC_TimeTypeDef rtc_time;
        RTC_GetTime(RTC_Format_BIN, &rtc_time);
        random_index = rtc_time.RTC_Seconds % 3;

        // MissionTask 시작 신호 전송
        OSFlagPost(&AlarmFlagGroup,
                   ALARM_FLAG_MISSION,
                   OS_OPT_POST_FLAG_SET,
                   &err);

        touched = 1;
      }
      prev = curr;
      OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    }

    // 4. 알람이 완전히 해제될 때까지 Block
    OSFlagPend(&AlarmFlagGroup,
               ALARM_FLAG_OFF,
               0,
               OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
               0,
               &err);
  }
}

// MissionTask - 미션 수행을 담당하는 태스크 (버튼 + 조이스틱)
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
    // 1. 터치 태스크로부터 미션 시작 신호 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_MISSION,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 랜덤 인덱스를 기반으로 버튼/조이스틱 미션 설정
    int required_button = button_missions[random_index];
    int *required_joy = Joystick_missions[random_index];

    // 각 조이스틱 방향 카운터 초기화
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

    // 3. 미션 1 안내 (버튼 누르기)
    char buf[128];
    snprintf(buf, sizeof(buf),
             "[미션 1]: 버튼 %d회를 누르세요!\r\n", required_button);
    OSQPost(&USARTMsgQ, (void *)buf, strlen(buf) + 1, OS_OPT_POST_FIFO, &err);

    // 버튼 태스크 시작 신호 전송
    OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_BUTTON, OS_OPT_POST_FLAG_SET, &err);

    int button_cnt = 0;
    int mission1_done = 0;

    // 4. 버튼 미션 수행 루프
    while (!mission1_done)
    {
      // 알람 강제 종료 체크
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_OFF,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0, &err))
        return;

      // 센서 입력 큐에서 데이터 수신
      msg = OSQPend(&SensorInputQ, 0, OS_OPT_PEND_BLOCKING, &msg_size, 0, &err);
      if (err != OS_ERR_NONE)
        continue;

      input = *(SensorInput *)msg;

      // 버튼 입력 감지
      if (input.type == SENSOR_BUTTON)
      {
        button_cnt++;
        if (button_cnt >= required_button)
        {
          // 미션1 완료
          OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
          static const char msg[] = "미션 1 성공!\r\n\r\n[미션 2]: ";
          OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);

          mission1_done = 1;

          // 버튼 태스크 종료 신호 전송
          OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_BUTTON_SUCCESS, OS_OPT_POST_FLAG_SET, &err);

          // 조이스틱 미션 안내 출력
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

    // 5. 조이스틱 미션 수행 루프
    while (1)
    {
      // 알람 강제 종료 체크
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_OFF,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0, &err))
        return;

      // 센서 입력 대기
      msg = OSQPend(&SensorInputQ, 0, OS_OPT_PEND_BLOCKING, &msg_size, 0, &err);
      if (err != OS_ERR_NONE)
        continue;

      input = *(SensorInput *)msg;

      // 방향별로 카운터 감소
      if (input.type == SENSOR_JOYSTICK_LEFT && left > 0)
        left--;
      else if (input.type == SENSOR_JOYSTICK_RIGHT && right > 0)
        right--;
      else if (input.type == SENSOR_JOYSTICK_UP && up > 0)
        up--;
      else if (input.type == SENSOR_JOYSTICK_DOWN && down > 0)
        down--;

      // 모든 방향 조건 충족 시 미션 성공
      if (left <= 0 && right <= 0 && up <= 0 && down <= 0)
      {
        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
        static const char msg[] = "\r\n미션 2 성공! 알람이 해제됩니다.\r\n\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);

        // 알람 종료
        OSFlagPost(&AlarmFlagGroup, ALARM_FLAG_OFF, OS_OPT_POST_FLAG_SET, &err);
        break;
      }
    }
  }
}

// Button Task - 버튼 입력을 감지하고 미션 시스템에 전달하는 태스크
static void ButtonTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  uint8_t prev, curr;
  SensorInput input;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 미션 시작 신호(ALARM_FLAG_BUTTON) 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_BUTTON,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 버튼 상태 초기화 (직전 상태 저장)
    prev = Button_Read();

    while (1)
    {
      // 3. 미션 성공 신호 받으면 조이스틱 태스크 시작 플래그 전송 후 종료
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

      // 4. 버튼 눌림 감지 (하강 엣지: HIGH → LOW)
      curr = Button_Read();
      if (curr == Bit_RESET && prev == Bit_SET)
      {
        // 입력 정보 큐로 전송
        input.type = SENSOR_BUTTON;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);

        // 로그 메시지 전송
        static const char msg[] = "[버튼] 눌림 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);

        // 디바운싱 (연속입력 방지)
        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
      }

      // 5. 상태 갱신 및 짧은 딜레이
      prev = curr;
      OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    }
  }
}

// Joystick Task - 조이스틱 방향 감지 → SensorInput 구조체로 전송
static void JoystickTask(void *p_arg)
{
  OS_ERR err;
  OS_FLAGS flags;
  SensorInput input;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 조이스틱 미션 시작 신호 대기
    flags = OSFlagPend(&AlarmFlagGroup,
                       ALARM_FLAG_JOYSTICK_START,
                       0,
                       OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING,
                       0,
                       &err);

    // 2. 조이스틱을 중앙에 둘 때까지 대기 (초기화 상태 확보)
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
      // 미션 성공 시 종료
      if (OSFlagPend(&AlarmFlagGroup,
                     ALARM_FLAG_OFF,
                     0,
                     OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_NON_BLOCKING,
                     0, &err))
      {
        break;
      }

      joystickX = JoyStick_ReadX();
      joystickY = JoyStick_ReadY();

      // → 오른쪽 감지 (X 증가)
      if (joystickX * 7 > 1000 && prevX * 7 <= 1000)
      {
        input.type = SENSOR_JOYSTICK_RIGHT;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);
        static const char msg[] = "[조이스틱] → 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }
      // ← 왼쪽 감지 (X 감소)
      else if (joystickX * 7 < 100 && prevX * 7 >= 100)
      {
        input.type = SENSOR_JOYSTICK_LEFT;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);
        static const char msg[] = "[조이스틱] ← 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }
      // ↑ 위쪽 감지 (Y 증가)
      else if (joystickY > 3000 && prevY <= 3000)
      {
        input.type = SENSOR_JOYSTICK_UP;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);
        static const char msg[] = "[조이스틱] ↑ 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }
      // ↓ 아래 감지 (Y 감소)
      else if (joystickY < 1000 && prevY >= 1000)
      {
        input.type = SENSOR_JOYSTICK_DOWN;
        OSQPost(&SensorInputQ, &input, sizeof(input), OS_OPT_POST_FIFO, &err);
        static const char msg[] = "[조이스틱] ↓ 감지\r\n";
        OSQPost(&USARTMsgQ, (void *)msg, sizeof(msg), OS_OPT_POST_FIFO, &err);
      }

      // 이전 값 업데이트
      prevX = joystickX;
      prevY = joystickY;

      // 0.1초 대기 후 다음 루프
      OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
  }
}

// USART Task - 메시지 큐에서 문자열을 받아 시리얼 터미널에 출력하는 태스크
static void USARTTask(void *p_arg)
{
  OS_ERR err;
  void *msg;
  OS_MSG_SIZE size;

  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 메시지 큐(USARTMsgQ)에서 문자열 수신 (블로킹 대기)
    msg = OSQPend(&USARTMsgQ,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  &size,
                  0,
                  &err);

    // 2. 수신 성공 시 문자열 출력
    if (err == OS_ERR_NONE && msg != NULL)
    {
      USART_SendString((char *)msg);
    }
  }
}

// Timer Monitor Task - 현재 시간을 확인하고 알람 직전 3초부터 시각 출력
static void TimeMonitorTask(void *p_arg)
{
  OS_ERR err;
  (void)p_arg;

  while (DEF_TRUE)
  {
    // 1. 현재 시간 읽기
    RTC_TimeTypeDef now;
    RTC_GetTime(RTC_Format_BIN, &now);

    // 2. 초 단위로 현재 시각과 알람 시각 계산
    int now_sec = now.RTC_Hours * 3600 + now.RTC_Minutes * 60 + now.RTC_Seconds;
    int alarm_sec = g_alarmTime.RTC_Hours * 3600 + g_alarmTime.RTC_Minutes * 60 + g_alarmTime.RTC_Seconds;
    int remaining = alarm_sec - now_sec;

    // 3. 알람 울리기 3초 전부터 시리얼 출력
    if (remaining <= 3 && remaining > 0)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d\r\n", now.RTC_Hours, now.RTC_Minutes, now.RTC_Seconds);
      OSQPost(&USARTMsgQ, (void *)buf, sizeof(buf), OS_OPT_POST_FIFO, &err);
    }

    // 4. 1초 간격 반복
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
