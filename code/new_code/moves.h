#ifndef MOVES_H
#define MOVES_H

#include "robot.h"
#include "kinematics.h"

double y_front_fix = 20.0;

void setWalkingPosition(Robot *Hexapod, int delay_time){

    evaluateLegPositionRobotCenter(Hexapod, LEFT_FRONT, -x_const, +y_const-y_front_fix, z_const_zero);
    delay(delay_time);
    evaluateLegPositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_const, 0, z_const_zero);
    delay(delay_time);
    evaluateLegPositionRobotCenter(Hexapod, LEFT_BACK, -x_const, -y_const, z_const_zero);
    delay(delay_time);
    evaluateLegPositionRobotCenter(Hexapod, RIGHT_FRONT, x_const, +y_const-y_front_fix, z_const_zero);
    delay(delay_time);
    evaluateLegPositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_const, 0, z_const_zero);
    delay(delay_time);
    evaluateLegPositionRobotCenter(Hexapod, RIGHT_BACK, x_const, -y_const, z_const_zero);
    delay(delay_time);
    
    Hexapod->_robotStepFase = WALKING_POSITION;
}


bool isRobotStanding(Robot *Hexapod){
    double z_tab[6];

    for(int i = 0; i < 6; i++){
        z_tab[i] = Hexapod->_LegsPositionRobotCenter[i].data[Z];
    }

    for(int i = 0; i < 6; i++){
        if(z_tab[i] < (z_const_zero) + (z_const_stand_up/2)){
            return true;
        }else{
            return false;
        }
    }


}



void moveLegsZ(Robot *Hexapod, double Z_distance, double Z_speed, double step_time){ // [mm],  [mm/s], [s]


    if(((Z_distance < 0)&&(Hexapod->_robotStepFase == STAND_UP) && isRobotStanding(Hexapod)) || ((Z_distance > 0)&&(Hexapod->_robotStepFase == SIT_DOWN) && (isRobotStanding(Hexapod) == false))){
            
        if(Hexapod->_robotStepFase == STAND_UP){
            printf("Robot juz stoi -> blad w robotMoveZ\n");
            global_error++;
        }else if(Hexapod->_robotStepFase == SIT_DOWN){
            printf("Robot juz siedzi -> blad w robotMoveZ\n");   
            global_error++;
        } 

    }else{

        if (Z_speed == 0) { 
            printf("Wpisano zerową prędkość w robotMoveZ!\n"); 
            global_error++;
            return; 
        }

        if (step_time == 0) { 
            printf("Wpisano zerowy step_time w robotMoveZ!\n"); 
            global_error++;
            return; 
        }

        // Oblicz całkowity czas ruchu oraz kierunek
        double moving_time = Z_distance / Z_speed;

        if(Z_distance < 0){
            moving_time = moving_time * (-1);
        }

        // Oblicz liczbę kroków
        int num_steps = (int)(moving_time / step_time);
        if (num_steps == 0) { 
            printf("Liczba kroków w robotMoveZ wynosi 0!\n"); 
            global_error++;
            return; 
        }

        // Oblicz wartość przesunięcia y dla każdego kroku
        double delta_z = Z_distance / num_steps ;

        

        


        for(int j = 0; j < num_steps; j++){
            for(int i = 0; i < 6; i++){
                evaluateLegPositionRobotCenter(Hexapod, i, Hexapod->_LegsPositionRobotCenter[i].data[0], Hexapod->_LegsPositionRobotCenter[i].data[1], Hexapod->_LegsPositionRobotCenter[i].data[2] + delta_z);
            }
            //printLegsPositions(Hexapod);
            delay(step_time*1000);
        }

        if(delta_z < 0){
            Hexapod->_robotStepFase = STAND_UP;
        }else if(delta_z > 0){
            Hexapod->_robotStepFase = SIT_DOWN;
        }else{
        Hexapod->_robotStepFase = UNKNOWN; 
        }
    }

}



#endif // MOVES_H