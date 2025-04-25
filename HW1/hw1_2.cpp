#include "mbed.h"

Serial pc(SERIAL_TX, SERIAL_RX);                                                  

int main()
{
    while (true) {
        pc.printf("Hello, World!\n");
        ThisThread::sleep_for(1000);
    }
}
