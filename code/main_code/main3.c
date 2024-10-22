#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"


void evaluatePosition(Robot * rob, LegPosition pos, double xp, double yp, double zp){
    printf("pozycja %d dla: %.2f, %.2f, %.2f: \n" , pos, xp, yp, zp);
    setTargetPos(&rob->_legs[pos], xp, yp, zp);
    
    calculateInvertedKinematics(&rob->_legs[pos]);
    
    moveToTargetPosition(&rob->_legs[pos]);
    printLeg(rob->_legs[pos]);
}

int main(void) {
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;

    // Inicjalizacja robota
    initRobot(&Hexapod);

    delay(2000);

    /*evaluatePosition(&Hexapod, LEFT_FRONT, -L1 - 130, 30, 0 );
    delay(2000);
    evaluatePosition(&Hexapod, LEFT_MIDDLE, -L1 - 130, 0, 0 );
    delay(2000);
    evaluatePosition(&Hexapod, LEFT_BACK, -L1 - 130, -30, 0 );



    delay(2000);

    evaluatePosition(&Hexapod, RIGHT_FRONT, L1 + 130, 30, 0 );
    delay(2000);
    evaluatePosition(&Hexapod, RIGHT_MIDDLE, L1 + 130, 0, 0 );
    delay(2000);
    evaluatePosition(&Hexapod, RIGHT_BACK, L1 + 130, -30, 0 );
    delay(2000);*/


    int z = 0;

    /*for(int j = 0; j < 100; j++){
        
        evaluatePosition(&Hexapod, RIGHT_FRONT, L1 + 130, 0, z );
        z = z - 1; 
        delay(5);
        printf("wszedl\n");

    }*/
    

    evaluatePosition(&Hexapod, LEFT_FRONT, -L1 - 130, 0, 0 );
    delay(2000);

    evaluatePosition(&Hexapod, LEFT_FRONT, -L1 - 130, 60, 0 );
    delay(2000);


    

    return 0;
}
