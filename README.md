# ⏰ 스마트 알람 시스템

>임베디드 시스템 텀 프로젝트 

본 프로젝트는 STM32F429 MCU와 uC/OS-III 실시간 운영체제 상에서 구현된 **행동 기반 알람 해제 시스템**입니다.  
알람 발생 시, 사용자는 터치, 버튼, 조이스틱을 순차적으로 조작하는 미션을 클리어해야만 알람을 비활성화할 수 있도록 설계하여, 수동적인 알람 확인을 넘어 능동적인 각성을 유도합니다.


## 📌 프로젝트 개요

### ✅ 문제의식
- 시험 기간 수면 부족으로 인해 학습 집중력이 크게 떨어짐
- 단순 알람은 무의식 중 끄고 다시 잠드는 문제 발생

### ✅ 해결책
- 사용자의 **행동을 요구하는 미션 기반 알람 시스템** 설계
- 센서 입력을 통해 확실한 각성과 집중력 회복 유도

<br/>

## 🛠️ 사용한 하드웨어 모듈

| 모듈명            | 설명 |
|------------------|------|
| RTC 모듈         | 현재 시간 및 알람 시간 설정/비교 |
| 터치 센서        | 미션 시작 트리거 |
| 푸시 버튼        | 버튼 입력 미션 수행용 |
| 조이스틱 모듈     | 방향 입력 미션 수행용 |
| 부저(Buzzer)     | 알람 발생 시 소리 출력 |
| USART            | 터미널 출력용 시리얼 통신 |

<br/>

## ⚙ 시스템 구조 및 RTOS 설계

### 🔧 Task 구조

| Task 이름            | 우선순위 | 역할 |
|----------------------|----------|------|
| `AppTaskStart`       | 2        | 시스템 초기화, 자원 생성 |
| `BuzzerTask`         | 3        | 알람 시 부저 깜빡임 작동 |
| `TouchSensorTask`    | 4        | 터치 입력 감지 및 미션 시작 |
| `MissionTask`        | 5        | 버튼 + 조이스틱 미션 흐름 제어 |
| `ButtonTask`         | 6        | 버튼 입력 감지 |
| `JoystickTask`       | 6        | 조이스틱 방향 감지 |
| `USARTTask`          | 7        | 메시지 큐 → 시리얼 터미널 출력 |
| `TimeMonitorTask`    | 8        | 알람 직전 3초 동안 시간 출력 |

<br/>

### 📡 RTOS 통신 메커니즘

  - `ISR`:  
    - `BSP_IntHandlerRTCAlarm()`:  지정된 시각에 `ALARM_FLAG_BUZZER`와 `ALARM_FLAG_TOUCH` 전달  

- `OSFlagGroup`:  
  알람 상태 및 미션 플래그 전달 (예: `ALARM_FLAG_BUZZER`, `ALARM_FLAG_OFF`)

- `OSQ`:
  - `SensorInputQ`: 센서(Task) 입력 전달
  - `USARTMsgQ`: 터미널 출력 문자열 전달

<br/>

## 📈 전체 동작 시퀀스

```text
[0] 사용자가 시리얼 모니터로 알람 시각을 입력
[1] RTC 시간 도달 → 알람 발생
[2] BuzzerTask + TouchSensorTask 시작
[3] 터치 감지 → 무작위 미션 선택
[4] ButtonTask → JoystickTask 순차 수행
[5] 성공 시 → Buzzer OFF + 알람 종료 메시지 출력
```

<br/>

## 🎬 시연 영상 링크

- https://www.youtube.com/shorts/GV5GMh5ydpw

<br/>

## 🛠 개발 환경

- **MCU 보드**: STM32F429II  
- **개발 IDE**: STM32 TrueSTUDIO  
- **RTOS**: uC/OS-III  
- **프로그래밍 언어**: C  
- **디버깅 방식**: JTAG/SWD + 시리얼 터미널 출력  
- **통신 인터페이스**: USART3 (115200 baudrate)

<br/>

## 📚 참고 자료

- PM0214-Programming manual-STM32 Cortex®-M4 MCUs and MPUs programming manual
