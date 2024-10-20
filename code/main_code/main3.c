#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"


int main(void) {
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;

    // Inicjalizacja robota
    initRobot(&Hexapod);

    return 0;
}
