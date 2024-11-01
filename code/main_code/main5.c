#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"
#include "trajectory.h"

int main(void)
{
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1)
        return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1)
        return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;
    initRobot(&Hexapod);



    int Z = 0;

    int delay_time = 200;
    int delay_between_fases = 100;
    double Z_speed = 10.0; 
    double Z_step_time = 0.01;

    double Y_move = 80;
    double Y_speed = 10;
    double Y_step_time = 0.01;

    StepFase start_fase = LM_RF_RB_PROT;

    setWalkingPosition(&Hexapod, delay_time);

    delay(delay_time *10);

    moveLegsZ(&Hexapod, z_const_stand_up, Z_speed,  Z_step_time);

    delay(delay_time *10);

    prepareForStepFase(&Hexapod, start_fase, Y_move, Y_speed, Y_step_time);

    if(start_fase == LM_RF_RB_PROT){
        for(int i = 0; i < 4; i++){
            doStepFase(&Hexapod, LF_LB_RM_PROT__LM_RF_RB_RETR);
            delay(delay_between_fases);
            doStepFase(&Hexapod, LF_LB_RM_RETR__LM_RF_RB_PROT);
            delay(delay_between_fases);
        }
    }else if(start_fase == LF_LB_RM_PROT){
        for(int i = 0; i < 4; i++){
            doStepFase(&Hexapod, LF_LB_RM_RETR__LM_RF_RB_PROT);
            delay(delay_between_fases);
            doStepFase(&Hexapod, LF_LB_RM_PROT__LM_RF_RB_RETR);
            delay(delay_between_fases);
        }
    }

    delay(delay_time *10);

    moveLegsZ(&Hexapod, -z_const_stand_up, Z_speed,  Z_step_time);

    delay(delay_time *10);

    setWalkingPosition(&Hexapod, delay_time);

    



    return 0;
}
