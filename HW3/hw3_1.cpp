#include "mbed.h"

DigitalOut leds[] = {LED1, LED2, LED3};
InterruptIn button(USER_BUTTON);
Ticker led_ticker;
Timer timer;
Timer debounce_timer; // 버튼 여러 번 눌림(Debounce) 방지

Serial pc(SERIAL_TX, SERIAL_RX); 

// 상태가 실행 중 계속 바뀌기 때문에 volatile로 선언
volatile bool blinking = false;
volatile int led_index = 0;
volatile bool print_flag = false;
volatile int last_elapsed = 0;

void blink_leds() {
    for (int i = 0; i < 3; i++) {
        leds[i] = 0;
    }

    leds[led_index] = 1;
    led_index = (led_index + 1) % 3;
}


void button_pressed() {
    // 0.2초 안에 버튼 누를 때 무시
    if (debounce_timer.read_ms() < 200) return;
    debounce_timer.reset();

    if (!blinking) {
        timer.reset();
        timer.start();
        led_index = 0;
        led_ticker.attach(&blink_leds, 0.5);
    } else {
        timer.stop();
        led_ticker.detach();

        for (int i = 0; i < 3; i++) leds[i] = 0;

        last_elapsed = timer.read_ms();
        print_flag = true;  // 여기선 그냥 플래그만 세움
    }
    blinking = !blinking;
}

int main() {
    debounce_timer.start();

    // 버튼은 기본적으로 pull-up 상태
    button.fall(&button_pressed);

    while (true) {
        // printf()는 기본적으로 lock을 획득함
        // ISR 내부에서 mutex 문제로 printf() 사용 불가
        if (print_flag) {
            print_flag = false;
            printf("Elapsed time: %d ms\r\n", last_elapsed);
        }
        ThisThread::sleep_for(100);
    }
}

