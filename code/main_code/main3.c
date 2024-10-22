#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"


void evaluatePosition(Robot * rob, LegPosition pos, double xp, double yp, double zp){

    if(((rob->_legs[pos]._side == RIGHT) && (xp > L1)) || ((rob->_legs[pos]._side == LEFT) && (xp < L1)))
    {
        //printf("pozycja %d dla: %.2f, %.2f, %.2f: \n" , pos, xp, yp, zp);
        setTargetPos(&rob->_legs[pos], xp, yp, zp);
        calculateInvertedKinematics(&rob->_legs[pos]);
        moveToTargetPosition(&rob->_legs[pos]);
        //printLeg(rob->_legs[pos]);
    }else{
        printf("Nieprawidlowa dana Xp\n");
    }



}

int main(void) {
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;

    int x_pos = 110; 
    int y_pos = 100; 
    int delay_time = 200;
    // Inicjalizacja robota
    initRobot(&Hexapod);

    delay(delay_time);
    
    evaluatePosition(&Hexapod, LEFT_FRONT, -L1 - x_pos, y_pos, 0 );
    delay(delay_time);
    evaluatePosition(&Hexapod, LEFT_MIDDLE, -L1 - x_pos, 0, 0 );
    delay(delay_time);
    evaluatePosition(&Hexapod, LEFT_BACK, -L1 - x_pos, -y_pos, 0 );



    delay(delay_time);

    evaluatePosition(&Hexapod, RIGHT_FRONT, L1 + x_pos, y_pos, 0 );
    delay(delay_time);
    evaluatePosition(&Hexapod, RIGHT_MIDDLE, L1 + x_pos, 0, 0 );
    delay(delay_time);
    evaluatePosition(&Hexapod, RIGHT_BACK, L1 + x_pos, -y_pos, 0 );
    delay(delay_time);




    int z = 0;

    for(int j = 0; j < 80; j++){
        
        evaluatePosition(&Hexapod, LEFT_FRONT, -L1 - x_pos, y_pos, z );
        evaluatePosition(&Hexapod, LEFT_MIDDLE, -L1 - x_pos, 0, z );
        evaluatePosition(&Hexapod, LEFT_BACK, -L1 - x_pos, -y_pos, z );

        evaluatePosition(&Hexapod, RIGHT_FRONT, L1 + x_pos, y_pos, z );
        evaluatePosition(&Hexapod, RIGHT_MIDDLE, L1 + x_pos, 0, z );
        evaluatePosition(&Hexapod, RIGHT_BACK, L1 + x_pos, -y_pos, z );
        z = z - 1; 
        delay(5);
        

    }

    delay(5000);

    int y = 0;

    for(int j = 0; j < 80; j++){
        
        evaluatePosition(&Hexapod, LEFT_FRONT, -L1 - x_pos, y_pos + y, z );
        evaluatePosition(&Hexapod, LEFT_MIDDLE, -L1 - x_pos, y, z );
        evaluatePosition(&Hexapod, LEFT_BACK, -L1 - x_pos, -y_pos + y, z );

        evaluatePosition(&Hexapod, RIGHT_FRONT, L1 + x_pos, y_pos + y, z );
        evaluatePosition(&Hexapod, RIGHT_MIDDLE, L1 + x_pos, y, z );
        evaluatePosition(&Hexapod, RIGHT_BACK, L1 + x_pos, -y_pos + y, z );

        z = z + 1; 
        delay(5);
        

    }



    




    delay(1000);
    




    

    return 0;
}
