/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright
*laws.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h" // USART 설정용
#include <includes.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

// USART, 핀, 포트, 클럭 등 하드웨어 세팅 상수들 정의
#define COMn 1

#define Nucleo_COM1 USART3
#define Nucleo_COM1_CLK RCC_APB1Periph_USART3
#define Nucleo_COM1_TX_PIN GPIO_Pin_9
#define Nucleo_COM1_TX_GPIO_PORT GPIOD
#define Nucleo_COM1_TX_GPIO_CLK RCC_AHB1Periph_GPIOD
#define Nucleo_COM1_TX_SOURCE GPIO_PinSource9
#define Nucleo_COM1_TX_AF GPIO_AF_USART3
#define Nucleo_COM1_RX_PIN GPIO_Pin_8
#define Nucleo_COM1_RX_GPIO_PORT GPIOD
#define Nucleo_COM1_RX_GPIO_CLK RCC_AHB1Periph_GPIOD
#define Nucleo_COM1_RX_SOURCE GPIO_PinSource8
#define Nucleo_COM1_RX_AF GPIO_AF_USART3
#define Nucleo_COM1_IRQn USART3_IRQn

/*
*********************************************************************************************************
*                                         TYPES DEFINITIONS
*********************************************************************************************************
*/

typedef enum { COM1 = 0 } COM_TypeDef;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void AppTaskCreate(void);
static void Setup_Gpio(void);

static void AppTask_1000ms(void *p_arg);
static void UsartTask(void *p_arg);

static void USART_Config(void);
void STM_Nucleo_COMInit(COM_TypeDef COM, USART_InitTypeDef *USART_InitStruct);
void send_string(const char *str);

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

// RTOS에서 사용하는 태스크 관련 변수, 스택(메모리), 태스크 제어 블록(TCB)
static OS_TCB AppTaskStartTCB;
static CPU_STK AppTaskStartStk[256];
static void AppTaskStart(void *p_arg);

static OS_TCB Task_1000ms_TCB;
static CPU_STK Task_1000ms_Stack[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB UsartTaskTCB;
static CPU_STK UsartTaskStk[APP_CFG_TASK_START_STK_SIZE];

// USART 포트/핀 배열화
USART_TypeDef *COM_USART[COMn] = {Nucleo_COM1};
GPIO_TypeDef *COM_TX_PORT[COMn] = {Nucleo_COM1_TX_GPIO_PORT};
GPIO_TypeDef *COM_RX_PORT[COMn] = {Nucleo_COM1_RX_GPIO_PORT};
const uint32_t COM_USART_CLK[COMn] = {Nucleo_COM1_CLK};
const uint32_t COM_TX_PORT_CLK[COMn] = {Nucleo_COM1_TX_GPIO_CLK};
const uint32_t COM_RX_PORT_CLK[COMn] = {Nucleo_COM1_RX_GPIO_CLK};
const uint16_t COM_TX_PIN[COMn] = {Nucleo_COM1_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {Nucleo_COM1_RX_PIN};
const uint16_t COM_TX_PIN_SOURCE[COMn] = {Nucleo_COM1_TX_SOURCE};
const uint16_t COM_RX_PIN_SOURCE[COMn] = {Nucleo_COM1_RX_SOURCE};
const uint16_t COM_TX_AF[COMn] = {Nucleo_COM1_TX_AF};
const uint16_t COM_RX_AF[COMn] = {Nucleo_COM1_RX_AF};

/*
*********************************************************************************************************
*                                                main()
*********************************************************************************************************
*/

int main(void) {
  OS_ERR err;

  /* 기본 하드웨어(클럭/포트) 초기화 */
  RCC_DeInit();
  // SystemCoreClockUpdate(); // 필요시 사용
  Setup_Gpio(); // LED용 GPIO 초기화

  /* BSP 및 uC/OS-III, 라이브러리 초기화 */
  BSP_IntDisAll(); // 모든 인터럽트 OFF

  CPU_Init();
  Mem_Init();
  Math_Init();

  /* uC/OS-III RTOS 초기화 */
  OSInit(&err);

  // RTOS 태스크(시작 Task) 생성
  OSTaskCreate(
      (OS_TCB *)&AppTaskStartTCB, (CPU_CHAR *)"App Task Start",
      (OS_TASK_PTR)AppTaskStart, (void *)0u, (OS_PRIO)APP_CFG_TASK_START_PRIO,
      (CPU_STK *)&AppTaskStartStk[0u],
      (CPU_STK_SIZE)AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
      (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE, (OS_MSG_QTY)0u, (OS_TICK)0u,
      (void *)0u, (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&err);

  OSStart(&err); // RTOS 스케줄러(여기서부터는 RTOS가 모든 Task를 관리)

  (void)&err;
  return (0u);
}

/*
*********************************************************************************************************
*                                            STARTUP TASK
*********************************************************************************************************
*/

static void AppTaskStart(void *p_arg) {
  OS_ERR err;

  (void)p_arg;

  BSP_Init();      // 보드 관련 초기화
  BSP_Tick_Init(); // SysTick(타이머) 초기화
  USART_Config();  // USART(시리얼) 설정

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  // BSP_LED_Off(0u);

  // Task 생성 함수 호출 (LED/USART Task 등)
  AppTaskCreate();
}

/*
*********************************************************************************************************
*                                         AppTaskCreate()
*********************************************************************************************************
*/

// 실제 동작하는 태스크(LED, USART) 등록
static void AppTaskCreate(void) {
  OS_ERR err;

  // 1초 주기 LED 토글 Task 생성
  OSTaskCreate(
      (OS_TCB *)&Task_1000ms_TCB, /* LED task */
      (CPU_CHAR *)"AppTask_1000ms", (OS_TASK_PTR)AppTask_1000ms, (void *)0u,
      (OS_PRIO)0u, (CPU_STK *)&Task_1000ms_Stack[0u],
      (CPU_STK_SIZE)Task_1000ms_Stack[APP_CFG_TASK_START_STK_SIZE / 10u],
      (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE, (OS_MSG_QTY)0u, (OS_TICK)0u,
      (void *)0u, (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&err);

  // USART Task 생성
  OSTaskCreate((OS_TCB *)&UsartTaskTCB, (CPU_CHAR *)"UsartTask",
               (OS_TASK_PTR)UsartTask, (void *)0u,
               (OS_PRIO)1u, // LED와 우선순위 다르게
               (CPU_STK *)&UsartTaskStk[0u],
               (CPU_STK_SIZE)UsartTaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
               (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE, (OS_MSG_QTY)0u,
               (OS_TICK)0u, (void *)0u,
               (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR *)&err);
}

/*
*********************************************************************************************************
*                                            USART Task
*********************************************************************************************************
*/

// 시리얼(USART)로 입력받은 문자를 그대로 터미널에 출력
// 백틱(`) 입력하면 종료 메시지까지 출력
static void UsartTask(void *p_arg) {
  (void)p_arg;
  char c;
  while (1) {
    // 1라운드(한 줄) 수신
    do {
      // 문자가 수신될 때까지 대기
      while (USART_GetFlagStatus(Nucleo_COM1, USART_FLAG_RXNE) == RESET)
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT,
                      NULL); // task 독점 방지

      // 문자 수신
      c = USART_ReceiveData(Nucleo_COM1);

      // 모든 문자(백틱 포함) 에코
      while (USART_GetFlagStatus(Nucleo_COM1, USART_FLAG_TXE) == RESET)
        ;
      USART_SendData(Nucleo_COM1, c);

    } while (c != '`'); // 백틱 나오면 반복 종료

    // 종료 메시지
    send_string("\n\rReceive end\n\r");
  }
}

/*
*********************************************************************************************************
*                                          AppTask_1000ms
*********************************************************************************************************
*/

// 1초마다 LED 토글
static void AppTask_1000ms(void *p_arg) {
  OS_ERR err;
  BSP_LED_On(2);
  while (DEF_TRUE) {
    BSP_LED_Toggle(2);
    OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT,
                  &err); // task 독점 방지
  }
}

/*
*********************************************************************************************************
*                                      USART Config/Functions
*********************************************************************************************************
*/

// USART 통신 속도/데이터 비트 등 기본 설정 함수
static void USART_Config(void) {
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  STM_Nucleo_COMInit(COM1, &USART_InitStructure);
}

// 실제 하드웨어 USART/핀 매핑 및 활성화 함수
void STM_Nucleo_COMInit(COM_TypeDef COM, USART_InitTypeDef *USART_InitStruct) {
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);
  RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);

  GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);
  GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

  GPIO_InitStruct.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStruct);

  USART_Init(COM_USART[COM], USART_InitStruct);
  USART_Cmd(COM_USART[COM], ENABLE);
}

// 문자열 송신 함수
void send_string(const char *str) {
  while (*str) {
    while (USART_GetFlagStatus(Nucleo_COM1, USART_FLAG_TXE) == RESET)
      ;
    USART_SendData(Nucleo_COM1, *str++);
  }
}

/*
*********************************************************************************************************
*                                          Setup_Gpio()
*
* Description : Configure LED GPIOs directly
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     :
*              LED1 PB0
*              LED2 PB7
*              LED3 PB14
*
*********************************************************************************************************
*/

// LED 3개용 포트/핀 초기화 함수(PB0, PB7, PB14)
static void Setup_Gpio(void) {
  GPIO_InitTypeDef led_init = {0};

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  led_init.GPIO_Mode = GPIO_Mode_OUT;
  led_init.GPIO_OType = GPIO_OType_PP;
  led_init.GPIO_Speed = GPIO_Speed_2MHz;
  led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  led_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14;

  GPIO_Init(GPIOB, &led_init);
}
