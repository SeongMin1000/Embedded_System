#include "mbed.h"

// 3개의 LED에 PWM 출력
PwmOut leds[] = { LED1, LED2, LED3 };

int main() {
    // 각 LED에 대해 PWM 주기 설정
    for (int i = 0; i < 3; i++) {
        leds[i].period(0.001f); // 주기를 최대한 작게 해서 부드럽게 함
    }

    while (true) {
        // 밝기 증가: duty cycle 0.0 → 1.0
        for (float duty = 0.0f; duty <= 1.0f; duty += 0.01f) {
            for (int i = 0; i < 3; i++) {
                leds[i].write(duty);
            }
            ThisThread::sleep_for(20);
        }

        // 밝기 감소: duty cycle 1.0 → 0.0
        for (float duty = 1.0f; duty >= 0.0f; duty -= 0.01f) {
            for (int i = 0; i < 3; i++) {
                leds[i].write(duty);
            }
            ThisThread::sleep_for(20);
        }
    }
}
