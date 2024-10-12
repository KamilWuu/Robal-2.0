#include "defines.h"

int main(void)
{
    StartGPIO();
    printf("Hello! I am Robal!\n");

    int running = 1;

    while (running)
    {
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
    }

    return 0;
}