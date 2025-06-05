#include "mbed.h"

//lab 1
extern "C" void my_strcpy(const char *src, char *dst);
extern "C" void my_capitalize(char *src);

Serial pc(SERIAL_TX, SERIAL_RX); 

int main(void)
{
    const char a[] = "Hello world!";
    char b[20];

    pc.printf("a: %s\t b: %s\n", a, b);
    my_strcpy(a, b);
    pc.printf("a: %s\t b: %s\n", a, b);
    my_capitalize(b);
    pc.printf("a: %s\t b: %s\n", a, b);
    
    while (1);
}