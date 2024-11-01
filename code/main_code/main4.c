#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"
#include "moves.h"

int main(void)
{
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1)
        return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1)
        return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;

    int Z = 0;

    int delay_time = 000;

    int z_move = 80;

    // variables
    int y = 0;
    int z = 0;

    // Inicjalizacja robota
    initRobot(&Hexapod);

    delay(delay_time);

    setWalkingPosition(&Hexapod, x_const, y_const, z_const_zero);

    delay(delay_time * 10);

    standUp(&Hexapod, x_const, y_const, y, &z, z_move);

    delay(1000);

    int pose_delay = 10;
    float y_speed = 4;
    float y_move_delta = -50;

    int z_var = 1;
    int z_speed = 4;

    int it = 10;

    for (int q = 0; q < it; q++)
    {
        printf("IT = %d\n", q);

        setPoseOne(&Hexapod, z_var, z_speed);
        delay(pose_delay);

        setPoseTwo(&Hexapod, y_move_delta, y_speed);

        delay(pose_delay);

        setPoseThree(&Hexapod, z_var, z_speed);

        delay(pose_delay);

        setPoseFour(&Hexapod, z_var, z_speed);

        delay(pose_delay);

        setPoseTwo(&Hexapod, -y_move_delta, y_speed);

        delay(pose_delay);

        setPoseTwoPrim(&Hexapod, y_move_delta, y_speed);

        delay(pose_delay);

        setPoseThreePrim(&Hexapod, z_var, z_speed);

        delay(pose_delay);

        setPoseFourPrim(&Hexapod, z_var, z_speed);

        delay(pose_delay);

        setPoseTwoPrim(&Hexapod, -y_move_delta, y_speed);

        delay(pose_delay);

        // setPoseThree(&Hexapod, z_var, z_speed);

        // delay(pose_delay);
    }

    setPoseThree(&Hexapod, z_var, z_speed);
    delay(1000);
    SitDown(&Hexapod, x_const, y_const, y, &z, z_move);
    delay(1000);
    return 0;
}
