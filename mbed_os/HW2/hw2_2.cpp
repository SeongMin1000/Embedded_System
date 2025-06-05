#include "mbed.h"

// Serial port setup
Serial pc(SERIAL_TX, SERIAL_RX, 9600);

// External ASM function declaration
extern "C" int factorial(int n);

int main() {
    int result = factorial(5);   // Call the ASM factorial function with 5 as input
    pc.printf("Factorial: %d\n", result);  // Output the result via serial
}