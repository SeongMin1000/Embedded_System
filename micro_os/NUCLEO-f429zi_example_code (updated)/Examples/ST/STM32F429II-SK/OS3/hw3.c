// /*
// *********************************************************************************************************
// *                                              EXAMPLE CODE
// *
// *                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
// *
// *                   All rights reserved.  Protected by international copyright
// *laws.
// *********************************************************************************************************
// */

// /*
// *********************************************************************************************************
// *                                             INCLUDE FILES
// *********************************************************************************************************
// */

// #include "stm32f4xx.h"
// #include "stm32f4xx_gpio.h"
// #include "stm32f4xx_rcc.h"
// #include "stm32f4xx_usart.h" // USART 설정용
// #include <includes.h>

// /*
// *********************************************************************************************************
// *                                            LOCAL DEFINES
// *********************************************************************************************************
// */

// // LED 제어 모드
// #define LED_OFF 0
// #define LED_ON 1
// #define LED_BLINK 2

// // USART, 핀, 포트, 클럭 등 하드웨어 세팅 상수들 정의
// #define COMn 1

// #define Nucleo_COM1 USART3
// #define Nucleo_COM1_CLK RCC_APB1Periph_USART3
// #define Nucleo_COM1_TX_PIN GPIO_Pin_9
// #define Nucleo_COM1_TX_GPIO_PORT GPIOD
// #define Nucleo_COM1_TX_GPIO_CLK RCC_AHB1Periph_GPIOD
// #define Nucleo_COM1_TX_SOURCE GPIO_PinSource9
// #define Nucleo_COM1_TX_AF GPIO_AF_USART3
// #define Nucleo_COM1_RX_PIN GPIO_Pin_8
// #define Nucleo_COM1_RX_GPIO_PORT GPIOD
// #define Nucleo_COM1_RX_GPIO_CLK RCC_AHB1Periph_GPIOD
// #define Nucleo_COM1_RX_SOURCE GPIO_PinSource8
// #define Nucleo_COM1_RX_AF GPIO_AF_USART3
// #define Nucleo_COM1_IRQn USART3_IRQn

// /*
// *********************************************************************************************************
// *                                         TYPES DEFINITIONS
// *********************************************************************************************************
// */

// typedef enum { COM1 = 0 } COM_TypeDef;

// /*
// *********************************************************************************************************
// *                                         FUNCTION PROTOTYPES
// *********************************************************************************************************
// */

// static void AppTaskCreate(void);
// static void Setup_Gpio(void);

// static void LedTask(void *p_arg);
// static void UsartTask(void *p_arg);

// static void USART_Config(void);
// void STM_Nucleo_COMInit(COM_TypeDef COM, USART_InitTypeDef *USART_InitStruct);
// void send_string(const char *str);

// /*
// *********************************************************************************************************
// *                                       LOCAL GLOBAL VARIABLES
// *********************************************************************************************************
// */

// // RTOS에서 사용하는 태스크 관련 변수, 스택(메모리), 태스크 제어 블록(TCB)
// static OS_TCB AppTaskStartTCB;
// static CPU_STK AppTaskStartStk[256];
// static void AppTaskStart(void *p_arg);

// static OS_TCB LedTaskTCB;
// static CPU_STK LedTaskStk[APP_CFG_TASK_START_STK_SIZE];

// static OS_TCB UsartTaskTCB;
// static CPU_STK UsartTaskStk[APP_CFG_TASK_START_STK_SIZE];

// // USART 포트/핀 배열화
// USART_TypeDef *COM_USART[COMn] = {Nucleo_COM1};
// GPIO_TypeDef *COM_TX_PORT[COMn] = {Nucleo_COM1_TX_GPIO_PORT};
// GPIO_TypeDef *COM_RX_PORT[COMn] = {Nucleo_COM1_RX_GPIO_PORT};
// const uint32_t COM_USART_CLK[COMn] = {Nucleo_COM1_CLK};
// const uint32_t COM_TX_PORT_CLK[COMn] = {Nucleo_COM1_TX_GPIO_CLK};
// const uint32_t COM_RX_PORT_CLK[COMn] = {Nucleo_COM1_RX_GPIO_CLK};
// const uint16_t COM_TX_PIN[COMn] = {Nucleo_COM1_TX_PIN};
// const uint16_t COM_RX_PIN[COMn] = {Nucleo_COM1_RX_PIN};
// const uint16_t COM_TX_PIN_SOURCE[COMn] = {Nucleo_COM1_TX_SOURCE};
// const uint16_t COM_RX_PIN_SOURCE[COMn] = {Nucleo_COM1_RX_SOURCE};
// const uint16_t COM_TX_AF[COMn] = {Nucleo_COM1_TX_AF};
// const uint16_t COM_RX_AF[COMn] = {Nucleo_COM1_RX_AF};

// // LED 제어 상태 변수
// volatile int LED1CMD = LED_OFF, LED2CMD = LED_OFF,
//              LED3CMD = LED_OFF; // 기본적으로 다 꺼진 상태
// volatile int LED1_BLINK_T = 1, LED2_BLINK_T = 1,
//              LED3_BLINK_T = 1; // 주기 기본 1초

// /*
// *********************************************************************************************************
// *                                                main()
// *********************************************************************************************************
// */

// int main(void) {
//   OS_ERR err;

//   /* 기본 하드웨어(클럭/포트) 초기화 */
//   RCC_DeInit();
//   // SystemCoreClockUpdate(); // 필요시 사용
//   Setup_Gpio(); // LED용 GPIO 초기화

//   /* BSP 및 uC/OS-III, 라이브러리 초기화 */
//   BSP_IntDisAll(); // 모든 인터럽트 OFF

//   CPU_Init();
//   Mem_Init();
//   Math_Init();

//   /* uC/OS-III RTOS 초기화 */
//   OSInit(&err);

//   // RTOS 태스크(시작 Task) 생성
//   OSTaskCreate(
//       (OS_TCB *)&AppTaskStartTCB, (CPU_CHAR *)"App Task Start",
//       (OS_TASK_PTR)AppTaskStart, (void *)0u, (OS_PRIO)APP_CFG_TASK_START_PRIO,
//       (CPU_STK *)&AppTaskStartStk[0u],
//       (CPU_STK_SIZE)AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
//       (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE, (OS_MSG_QTY)0u, (OS_TICK)0u,
//       (void *)0u, (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//       (OS_ERR *)&err);

//   OSStart(&err); // RTOS 스케줄러(여기서부터는 RTOS가 모든 Task를 관리)

//   (void)&err;
//   return (0u);
// }

// /*
// *********************************************************************************************************
// *                                            STARTUP TASK
// *********************************************************************************************************
// */

// static void AppTaskStart(void *p_arg) {
//   OS_ERR err;

//   (void)p_arg;

//   BSP_Init();      // 보드 관련 초기화
//   BSP_Tick_Init(); // SysTick(타이머) 초기화
//   USART_Config();  // USART(시리얼) 설정

// #if OS_CFG_STAT_TASK_EN > 0u
//   OSStatTaskCPUUsageInit(&err);
// #endif

// #ifdef CPU_CFG_INT_DIS_MEAS_EN
//   CPU_IntDisMeasMaxCurReset();
// #endif

//   // BSP_LED_Off(0u);

//   // Task 생성 함수 호출 (LED/USART Task 등)
//   AppTaskCreate();
// }

// /*
// *********************************************************************************************************
// *                                         AppTaskCreate()
// *********************************************************************************************************
// */

// // 실제 동작하는 태스크(LED, USART) 등록
// static void AppTaskCreate(void) {
//   OS_ERR err;

//   // LedTask 생성
//   OSTaskCreate(
//       (OS_TCB *)&LedTaskTCB, (CPU_CHAR *)"LedTask", (OS_TASK_PTR)LedTask,
//       (void *)0u, (OS_PRIO)0u, (CPU_STK *)&LedTaskStk[0u],
//       (CPU_STK_SIZE)LedTaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
//       (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE, (OS_MSG_QTY)0u, (OS_TICK)0u,
//       (void *)0u, (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//       (OS_ERR *)&err);

//   // USART Task 생성
//   OSTaskCreate((OS_TCB *)&UsartTaskTCB, (CPU_CHAR *)"UsartTask",
//                (OS_TASK_PTR)UsartTask, (void *)0u,
//                (OS_PRIO)1u, // LED와 우선순위 다르게
//                (CPU_STK *)&UsartTaskStk[0u],
//                (CPU_STK_SIZE)UsartTaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
//                (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE, (OS_MSG_QTY)0u,
//                (OS_TICK)0u, (void *)0u,
//                (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//                (OS_ERR *)&err);
// }

// /*
// *********************************************************************************************************
// *                                            USART Task
// *********************************************************************************************************
// */

// static void UsartTask(void *p_arg) {
//   (void)p_arg;
//   char buf[32];
//   int idx = 0;
//   char c;

//   while (1) {
//     idx = 0;
//     // 문자열 한 줄 입력 받기
//     while (1) {
//       // 문자가 수신될 때까지 대기
//       while (USART_GetFlagStatus(Nucleo_COM1, USART_FLAG_RXNE) == RESET)
//         OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT,
//                       NULL); // task 독점 방지

//       // 문자 수신
//       c = USART_ReceiveData(Nucleo_COM1);
//       if (c == '\r' || c == '\n')
//         break; // 엔터로 끝
//       if (idx < sizeof(buf) - 1)
//         buf[idx++] = c;
//       USART_SendData(Nucleo_COM1, c); // 출력
//     }
//     buf[idx] = 0; // // 문자열 끝에 null

//     // 명령어 파싱
//     int is_valid = 0; // 명령어 성공여부 플래그

//     // critical section 시작
//     CPU_SR_ALLOC();
//     CPU_CRITICAL_ENTER();

//     if (strncmp(buf, "led", 3) == 0) {
//       int led_num = buf[3] - '0'; // 몇 번 LED인지 추출 (1~3)
//       char *cmd = &buf[4];        // on/off/blinkN
//       int blink_val = 0;

//       if (led_num >= 1 && led_num <= 3) {
//         if (strncmp(cmd, "on", 2) == 0) {
//           if (led_num == 1)
//             LED1CMD = LED_ON;
//           if (led_num == 2)
//             LED2CMD = LED_ON;
//           if (led_num == 3)
//             LED3CMD = LED_ON;
//           is_valid = 1;
//         } else if (strncmp(cmd, "off", 3) == 0) {
//           if (led_num == 1)
//             LED1CMD = LED_OFF;
//           if (led_num == 2)
//             LED2CMD = LED_OFF;
//           if (led_num == 3)
//             LED3CMD = LED_OFF;
//           is_valid = 1;
//         } else if (strncmp(cmd, "blink", 5) == 0) {
//           blink_val = atoi(&cmd[5]);
//           if (blink_val <= 0)
//             blink_val = 1;
//           if (led_num == 1) {
//             LED1CMD = LED_BLINK;
//             LED1_BLINK_T = blink_val;
//           }
//           if (led_num == 2) {
//             LED2CMD = LED_BLINK;
//             LED2_BLINK_T = blink_val;
//           }
//           if (led_num == 3) {
//             LED3CMD = LED_BLINK;
//             LED3_BLINK_T = blink_val;
//           }
//           is_valid = 1;
//         }
//       }
//     } else if (strncmp(buf, "reset", 5) == 0) {
//       LED1CMD = LED2CMD = LED3CMD = LED_OFF;
//       LED1_BLINK_T = LED2_BLINK_T = LED3_BLINK_T = 1;
//       is_valid = 1;
//     }

//     CPU_CRITICAL_EXIT();
//     // critical section 종료

//     // 유효한 명령인지 체크
//     if (is_valid)
//       send_string("\n\r[OK]\n\r");
//     else
//       send_string("\n\r[INVALID INPUT ]\n\r");
//   }
// }

// /*
// *********************************************************************************************************
// *                                          LEDTask
// *********************************************************************************************************
// */

// static void LedTask(void *p_arg) {
//   OS_ERR err;
//   int prev1 = -1, prev2 = -1, prev3 = -1; // 이전 상태 저장(중복제어 방지)

//   while (1) {

//     // critical section 시작
//     CPU_SR_ALLOC();
//     CPU_CRITICAL_ENTER();
//     int cmd1 = LED1CMD, blink1 = LED1_BLINK_T;
//     int cmd2 = LED2CMD, blink2 = LED2_BLINK_T;
//     int cmd3 = LED3CMD, blink3 = LED3_BLINK_T;
//     CPU_CRITICAL_EXIT();
//     // critical section 종료

//     // LED1 제어
//     if (cmd1 == LED_ON && prev1 != LED_ON) {
//       BSP_LED_On(1);
//       prev1 = LED_ON;
//     } else if (cmd1 == LED_OFF && prev1 != LED_OFF) {
//       BSP_LED_Off(1);
//       prev1 = LED_OFF;
//     } else if (cmd1 == LED_BLINK) {
//       BSP_LED_Toggle(1);
//       OSTimeDlyHMSM(0, 0, blink1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//     }

//     // LED2 제어
//     if (cmd2 == LED_ON && prev2 != LED_ON) {
//       BSP_LED_On(2);
//       prev2 = LED_ON;
//     } else if (cmd2 == LED_OFF && prev2 != LED_OFF) {
//       BSP_LED_Off(2);
//       prev2 = LED_OFF;
//     } else if (cmd2 == LED_BLINK) {
//       BSP_LED_Toggle(2);
//       OSTimeDlyHMSM(0, 0, blink2, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//     }

//     // LED3 제어
//     if (cmd3 == LED_ON && prev3 != LED_ON) {
//       BSP_LED_On(3);
//       prev3 = LED_ON;
//     } else if (cmd3 == LED_OFF && prev3 != LED_OFF) {
//       BSP_LED_Off(3);
//       prev3 = LED_OFF;
//     } else if (cmd3 == LED_BLINK) {
//       BSP_LED_Toggle(3);
//       OSTimeDlyHMSM(0, 0, blink3, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//     }

//     // ON/OFF 상태일 때 너무 빨리 돌지 않게 delay 추가
//     if (cmd1 != LED_BLINK && cmd2 != LED_BLINK && cmd3 != LED_BLINK)
//       OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT,
//                     &err); // task 독점 방지
//   }
// }

// /*
// *********************************************************************************************************
// *                                      USART Config/Functions
// *********************************************************************************************************
// */

// // USART 통신 속도/데이터 비트 등 기본 설정 함수
// static void USART_Config(void) {
//   USART_InitTypeDef USART_InitStructure;

//   USART_InitStructure.USART_BaudRate = 115200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl =
//       USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//   STM_Nucleo_COMInit(COM1, &USART_InitStructure);
// }

// // 실제 하드웨어 USART/핀 매핑 및 활성화 함수
// void STM_Nucleo_COMInit(COM_TypeDef COM, USART_InitTypeDef *USART_InitStruct) {
//   GPIO_InitTypeDef GPIO_InitStruct;

//   RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);
//   RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);

//   GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);
//   GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

//   GPIO_InitStruct.GPIO_Pin = COM_TX_PIN[COM];
//   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStruct);

//   GPIO_InitStruct.GPIO_Pin = COM_RX_PIN[COM];
//   GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStruct);

//   USART_Init(COM_USART[COM], USART_InitStruct);
//   USART_Cmd(COM_USART[COM], ENABLE);
// }

// // 문자열 송신 함수
// void send_string(const char *str) {
//   while (*str) {
//     while (USART_GetFlagStatus(Nucleo_COM1, USART_FLAG_TXE) == RESET)
//       ;
//     USART_SendData(Nucleo_COM1, *str++);
//   }
// }

// /*
// *********************************************************************************************************
// *                                          Setup_Gpio()
// *
// * Description : Configure LED GPIOs directly
// *
// * Argument(s) : none
// *
// * Return(s)   : none
// *
// * Caller(s)   : AppTaskStart()
// *
// * Note(s)     :
// *              LED1 PB0
// *              LED2 PB7
// *              LED3 PB14
// *
// *********************************************************************************************************
// */

// // LED 3개용 포트/핀 초기화 함수(PB0, PB7, PB14)
// static void Setup_Gpio(void) {
//   GPIO_InitTypeDef led_init = {0};

//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//   RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

//   led_init.GPIO_Mode = GPIO_Mode_OUT;
//   led_init.GPIO_OType = GPIO_OType_PP;
//   led_init.GPIO_Speed = GPIO_Speed_2MHz;
//   led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
//   led_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14;

//   GPIO_Init(GPIOB, &led_init);
// }
