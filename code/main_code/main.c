#include "defines.h"
#include "servo.h"

int main(void)
{
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1)
        return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1)
        return -1;

    printf("Hello! I am Robal!\n");

    int running = 1;
    
    // Set servo angles on PCA1
    for (int i = 0; i < 9; i++)
    {
        SetServoAngle(pca_left, i, 135); // Set each servo to 135° (halfway) on PCA1
        delay(100);                      // Brief delay between setting each servo
    }

    // Set servo angles on PCA2
    for (int i = 0; i < 9; i++)
    {
        SetServoAngle(pca_right, i, 135); // Set each servo to 135° (halfway) on PCA2
        delay(100);                       // Brief delay between setting each servo
    }

    delay(1000);

    while (running)
    {
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
    }

    return 0;
}