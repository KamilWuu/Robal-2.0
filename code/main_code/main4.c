#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"
#include "moves.h"


int main(void) {
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;

    int Z = 0;

    int x_const = 260; 
    int y_const = d3+80; 
    int z_const = 0;
    int delay_time = 200;


    int z_move = 80;

    //variables
    int y = 0;
    int z = 0;

    // Inicjalizacja robota
    initRobot(&Hexapod);

    delay(delay_time);
    
    setWalkingPosition(&Hexapod, x_const, y_const, z_const);

    delay(delay_time *10);



    standUp(&Hexapod, x_const, y_const, y, &z, z_move);

    delay(delay_time * 10);
    
    for(int j = 0; j < 1; j++){
        
        RobotForward(&Hexapod, x_const, y_const, &y, z, 70);
        delay(delay_time *5);
        RobotBack(&Hexapod, x_const, y_const, &y, z, 70);
    }

    delay(delay_time*5);
    

    SitDown(&Hexapod, x_const, y_const, y, &z, z_move);



    delay(1000);
    




    

    return 0;
}
