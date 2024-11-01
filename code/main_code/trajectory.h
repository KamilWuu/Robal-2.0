#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <time.h>
#include <stdio.h>


#include "robot.h"



 //   LF_LB_RM_PROT, //przygotowanie do ruchu, protrakcja LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE  bez retrakcji pozostalych
  //  LM_RF_RB_PROT, //przygotowanie do ruchu, protrakcja LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK bez retrakcji pozostalych


double calculateElipseZ(StepFase fase, LegPosition leg_pos, double y, double Y_0, double Z_s, double Y_move_distance, double h) {
    double z;
    double r = Y_move_distance / 2;
    double Y_00;
    if(leg_pos == RIGHT_BACK  || leg_pos == RIGHT_BACK){
        Y_00 = (-1)*Y_0;
    }else{
        Y_00 = Y_0;
    }


    double Y_s = Y_0 + r;
    
    // Funkcja pomocnicza do obliczeń dla fazy protrakcji
    bool isProtraction = ((fase == LF_LB_RM_PROT__LM_RF_RB_RETR || fase == LF_LB_RM_PROT ) && (leg_pos == LEFT_FRONT || leg_pos == LEFT_BACK || leg_pos == RIGHT_MIDDLE)) 
                         || 
                         ((fase == LF_LB_RM_RETR__LM_RF_RB_PROT || fase ==LM_RF_RB_PROT ) && (leg_pos == LEFT_MIDDLE || leg_pos == RIGHT_FRONT || leg_pos == RIGHT_BACK));

    if (isProtraction) {
        z = Z_s + h * sqrt(1 - pow((y - Y_s) / r, 2));
    } else {
        z = Z_s;  // W fazie retrakcji noga pozostaje na poziomie Z_s
    }

    return z;
}

int y_front_fix = 20;

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
        z_tab[i] = Hexapod->_LegsPositionRobotCenter[i].Pr_z;
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
        }else if(Hexapod->_robotStepFase == SIT_DOWN){
            printf("Robot juz siedzi -> blad w robotMoveZ\n");   
        } 

    }else{

        if (Z_speed == 0) { 
            printf("Wpisano zerową prędkość w robotMoveZ!\n"); 
            return; 
        }

        if (step_time == 0) { 
            printf("Wpisano zerowy step_time w robotMoveZ!\n"); 
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
            return; 
        }

        // Oblicz wartość przesunięcia y dla każdego kroku
        double delta_z = Z_distance / num_steps ;

        printf("num steps= %d, delta_z = %f, moving_time = %f\n", num_steps, delta_z, moving_time );

        


        for(int j = 0; j < num_steps; j++){
            for(int i = 0; i < 6; i++){
                evaluateLegPositionRobotCenter(Hexapod, i, Hexapod->_LegsPositionRobotCenter[i].Pr_x, Hexapod->_LegsPositionRobotCenter[i].Pr_y, Hexapod->_LegsPositionRobotCenter[i].Pr_z + delta_z);
            }
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


void prepareForStepFase(Robot *Hexapod, StepFase fase, double Y_move_distance, double Y_speed, double step_time){ // [mm],  [mm/s], [s]
    
    

    double z_elipse[6];
    double Y_0_tab[6];
    double Z_s_tab[6];
    double x_tab[6];
    double y_tab_with_delta[6];

    if (Y_speed == 0) { 
        printf("Wpisano zerową prędkość w prepareForStepFase!\n"); 
        return; 
    }

    if (step_time == 0) { 
        printf("Wpisano zerowy step_time w prepareForStepFase!\n"); 
        return; 
    }

    // Oblicz całkowity czas ruchu oraz kierunek
    double moving_time = Y_move_distance / Y_speed;

        if(Y_move_distance < 0){
            moving_time = moving_time * (-1);
        }


    // Oblicz liczbę kroków
    int num_steps = (int)(moving_time / step_time);
    if (num_steps == 0) { 
        printf("Liczba kroków w prepareForStepFase wynosi 0!\n"); 
        return; 
    }

    // Oblicz wartość przesunięcia y dla każdego kroku
    double delta_y = Y_move_distance / num_steps;

    for (int i = 0; i < 6; i++) {
        x_tab[i] =      Hexapod->_LegsPositionRobotCenter[i].Pr_x;
        Y_0_tab[i] =    Hexapod->_LegsPositionRobotCenter[i].Pr_y;
        Z_s_tab[i] =    z_const_stand_up;
    }

    printf("num steps= %d, delta_y = %f, moving_time = %f\n", num_steps, delta_y, moving_time );



    if (Hexapod->_robotStepFase == STAND_UP) {

        if (fase == LF_LB_RM_PROT) {

            for (int i = 0; i < num_steps; i++) {

                /*================ PROTRAKCJA =================*/
                y_tab_with_delta[LEFT_FRONT] =      Hexapod->_LegsPositionRobotCenter[LEFT_FRONT].Pr_y      + delta_y;
                y_tab_with_delta[LEFT_BACK] =       Hexapod->_LegsPositionRobotCenter[LEFT_BACK].Pr_y       + delta_y;
                y_tab_with_delta[RIGHT_MIDDLE] =    Hexapod->_LegsPositionRobotCenter[RIGHT_MIDDLE].Pr_y    + delta_y;

                z_elipse[LEFT_FRONT] =      calculateElipseZ(fase, LEFT_FRONT,      y_tab_with_delta[LEFT_FRONT],   Y_0_tab[LEFT_FRONT],    Z_s_tab[LEFT_FRONT],    Y_move_distance, h_const);
                z_elipse[LEFT_BACK] =       calculateElipseZ(fase, LEFT_BACK,       y_tab_with_delta[LEFT_BACK],    Y_0_tab[LEFT_BACK],     Z_s_tab[LEFT_BACK],     Y_move_distance, h_const);
                z_elipse[RIGHT_MIDDLE] =    calculateElipseZ(fase, RIGHT_MIDDLE,    y_tab_with_delta[RIGHT_MIDDLE], Y_0_tab[RIGHT_MIDDLE],  Z_s_tab[RIGHT_MIDDLE],  Y_move_distance, h_const);

                evaluateLegPositionRobotCenter(Hexapod, LEFT_FRONT,      x_tab[LEFT_FRONT],      y_tab_with_delta[LEFT_FRONT],   z_elipse[LEFT_FRONT]);
                evaluateLegPositionRobotCenter(Hexapod, LEFT_BACK,       x_tab[LEFT_BACK],       y_tab_with_delta[LEFT_BACK],    z_elipse[LEFT_BACK]);
                evaluateLegPositionRobotCenter(Hexapod, RIGHT_MIDDLE,    x_tab[RIGHT_MIDDLE],    y_tab_with_delta[RIGHT_MIDDLE], z_elipse[RIGHT_MIDDLE]);
                /*=============================================*/

                delay(step_time*1000); // `delay` przyjmuje wartość w milisekundach, więc konwersja na ms
            }

            Hexapod->_robotStepFase = LF_LB_RM_PROT;

        } else if (fase == LM_RF_RB_PROT) {

            for (int i = 0; i < num_steps; i++) {

                /*================ PROTRAKCJA =================*/
                y_tab_with_delta[LEFT_MIDDLE] = Hexapod->_LegsPositionRobotCenter[LEFT_MIDDLE].Pr_y + delta_y;
                y_tab_with_delta[RIGHT_FRONT] = Hexapod->_LegsPositionRobotCenter[RIGHT_FRONT].Pr_y + delta_y;
                y_tab_with_delta[RIGHT_BACK] =  Hexapod->_LegsPositionRobotCenter[RIGHT_BACK].Pr_y  + delta_y;

                z_elipse[LEFT_MIDDLE] =     calculateElipseZ(fase, LEFT_MIDDLE, y_tab_with_delta[LEFT_MIDDLE],  Y_0_tab[LEFT_MIDDLE],   Z_s_tab[LEFT_MIDDLE],   Y_move_distance, h_const);
                z_elipse[RIGHT_FRONT] =     calculateElipseZ(fase, RIGHT_FRONT, y_tab_with_delta[RIGHT_FRONT],  Y_0_tab[RIGHT_FRONT],   Z_s_tab[RIGHT_FRONT],   Y_move_distance, h_const);
                z_elipse[RIGHT_BACK] =      calculateElipseZ(fase, RIGHT_BACK,  y_tab_with_delta[RIGHT_BACK],   Y_0_tab[RIGHT_BACK],    Z_s_tab[RIGHT_BACK],    Y_move_distance, h_const);

                evaluateLegPositionRobotCenter(Hexapod, LEFT_MIDDLE,    x_tab[LEFT_MIDDLE], y_tab_with_delta[LEFT_MIDDLE],  z_elipse[LEFT_MIDDLE]);
                evaluateLegPositionRobotCenter(Hexapod, RIGHT_FRONT,    x_tab[RIGHT_FRONT], y_tab_with_delta[RIGHT_FRONT],  z_elipse[RIGHT_FRONT]);
                evaluateLegPositionRobotCenter(Hexapod, RIGHT_BACK,     x_tab[RIGHT_BACK],  y_tab_with_delta[RIGHT_BACK],   z_elipse[RIGHT_BACK]);
                /*=============================================*/
                printf("Dla RIGHT BACK: y = %f,  z = %f\n", y_tab_with_delta[RIGHT_BACK], z_elipse[RIGHT_BACK] );
                delay(step_time*1000); // `delay` przyjmuje wartość w milisekundach, więc konwersja na ms
            }

            Hexapod->_robotStepFase = LM_RF_RB_PROT;

        } else {
            printf("Wprowadzono złą pozycję przygotowania do ruchu w prepareForStepFase\n");
        }

    } else {
        printf("Aby ustawić pozycję przygotowania do ruchu, robot musi znajdować się w fazie STAND_UP -> błąd w prepareForStepFase\n");
    }



}


//LEFT_PROTRACTION, RIGHT_RETRACTION = LF_LB_RM_PROT__LM_RF_RB_RETR

//LEFT_RETRACTION, RIGHT_PROTRACTION = LF_LB_RM_RETR__LM_RF_RB_PROT


void doStepFase(Robot *Hexapod, StepFase fase, double Y_move_distance, double Y_speed, double step_time){ // [mm],  [mm/s], [s]



    double z_elipse[6];
    double Y_0_tab[6];
    double Z_s_tab[6];
    double x_tab[6];
    double y_tab_with_delta[6];

    if (Y_speed == 0) { 
        printf("Wpisano zerową prędkość w doStepFase!\n"); 
        return; 
    }

    if (step_time == 0) { 
        printf("Wpisano zerowy step_time w doStepFase!\n"); 
        return; 
    }

    // Oblicz całkowity czas ruchu
    double moving_time = Y_move_distance / Y_speed;

        if(Y_move_distance < 0){
            moving_time = moving_time * (-1);
        }


    // Oblicz liczbę kroków
    int num_steps = (int)(moving_time / step_time);
    if (num_steps == 0) { 
        printf("Liczba kroków w doStepFase wynosi 0!\n"); 
        return; 
    }

    // Oblicz wartość przesunięcia y dla każdego kroku
    double delta_y = Y_move_distance / num_steps;

    for(int i = 0; i < 6; i++){
        x_tab[i] =      Hexapod->_LegsPositionRobotCenter[i].Pr_x;
        Y_0_tab[i] =    Hexapod->_LegsPositionRobotCenter[i].Pr_y;
        Z_s_tab[i] =    z_const_stand_up;
        
    }

    printf("num steps= %d, delta_y = %f, moving_time = %f\n", num_steps, delta_y, moving_time );



    if (fase == LF_LB_RM_PROT__LM_RF_RB_RETR  && ((Hexapod->_robotStepFase == LM_RF_RB_PROT) || (Hexapod->_robotStepFase == LF_LB_RM_RETR__LM_RF_RB_PROT))) {

        for (int i = 0; i < num_steps; i++) {
            
            
            


            /*================ PROTRAKCJA =================*/
            y_tab_with_delta[LEFT_FRONT] =      Hexapod->_LegsPositionRobotCenter[LEFT_FRONT].Pr_y      + delta_y;
            y_tab_with_delta[LEFT_BACK] =       Hexapod->_LegsPositionRobotCenter[LEFT_BACK].Pr_y       + delta_y;
            y_tab_with_delta[RIGHT_MIDDLE] =    Hexapod->_LegsPositionRobotCenter[RIGHT_MIDDLE].Pr_y    + delta_y;

            z_elipse[LEFT_FRONT] =      calculateElipseZ(fase, LEFT_FRONT,      y_tab_with_delta[LEFT_FRONT],   Y_0_tab[LEFT_FRONT],    Z_s_tab[LEFT_FRONT],    Y_move_distance, h_const);
            z_elipse[LEFT_BACK] =       calculateElipseZ(fase, LEFT_BACK,       y_tab_with_delta[LEFT_BACK],    Y_0_tab[LEFT_BACK],     Z_s_tab[LEFT_BACK],     Y_move_distance, h_const);
            z_elipse[RIGHT_MIDDLE] =    calculateElipseZ(fase, RIGHT_MIDDLE,    y_tab_with_delta[RIGHT_MIDDLE], Y_0_tab[RIGHT_MIDDLE],  Z_s_tab[RIGHT_MIDDLE],  Y_move_distance, h_const);

            evaluateLegPositionRobotCenter(Hexapod, LEFT_FRONT,     x_tab[LEFT_FRONT],      y_tab_with_delta[LEFT_FRONT],   z_elipse[LEFT_FRONT]);
            evaluateLegPositionRobotCenter(Hexapod, LEFT_BACK,      x_tab[LEFT_BACK],       y_tab_with_delta[LEFT_BACK],    z_elipse[LEFT_BACK]);
            evaluateLegPositionRobotCenter(Hexapod, RIGHT_MIDDLE,   x_tab[RIGHT_MIDDLE],    y_tab_with_delta[RIGHT_MIDDLE], z_elipse[RIGHT_MIDDLE]);
            /*=============================================*/
            
            /*================ RETRAKCJA ==================*/
            y_tab_with_delta[LEFT_MIDDLE] = Hexapod->_LegsPositionRobotCenter[LEFT_MIDDLE].Pr_y - delta_y;
            y_tab_with_delta[RIGHT_FRONT] = Hexapod->_LegsPositionRobotCenter[RIGHT_FRONT].Pr_y - delta_y;
            y_tab_with_delta[RIGHT_BACK] =  Hexapod->_LegsPositionRobotCenter[RIGHT_BACK].Pr_y  - delta_y;

            z_elipse[LEFT_MIDDLE] =     calculateElipseZ(fase, LEFT_MIDDLE, y_tab_with_delta[LEFT_MIDDLE],  Y_0_tab[LEFT_MIDDLE],   Z_s_tab[LEFT_MIDDLE],   Y_move_distance, h_const);
            z_elipse[RIGHT_FRONT] =     calculateElipseZ(fase, RIGHT_FRONT, y_tab_with_delta[RIGHT_FRONT],  Y_0_tab[RIGHT_FRONT],   Z_s_tab[RIGHT_FRONT],   Y_move_distance, h_const);
            z_elipse[RIGHT_BACK] =      calculateElipseZ(fase, RIGHT_BACK,  y_tab_with_delta[RIGHT_BACK],   Y_0_tab[RIGHT_BACK],    Z_s_tab[RIGHT_BACK],    Y_move_distance, h_const);
            
            evaluateLegPositionRobotCenter(Hexapod, LEFT_MIDDLE,    x_tab[LEFT_MIDDLE], y_tab_with_delta[LEFT_MIDDLE],  z_elipse[LEFT_MIDDLE]);
            evaluateLegPositionRobotCenter(Hexapod, RIGHT_FRONT,    x_tab[RIGHT_FRONT], y_tab_with_delta[RIGHT_FRONT],  z_elipse[RIGHT_FRONT]);
            evaluateLegPositionRobotCenter(Hexapod, RIGHT_BACK,     x_tab[RIGHT_BACK],  y_tab_with_delta[RIGHT_BACK],   z_elipse[RIGHT_BACK]);
            /*=============================================*/
            printf("Dla RIGHT FRONT: y = %f, y_0 = %f, z = %f\n", y_tab_with_delta[RIGHT_FRONT], Y_0_tab[RIGHT_FRONT], z_elipse[RIGHT_FRONT] );
            delay(step_time*1000); // `delay` przyjmuje wartość w milisekundach, więc konwersja na ms
        }

        Hexapod->_robotStepFase = LF_LB_RM_PROT__LM_RF_RB_RETR;

    } else if (fase == LF_LB_RM_RETR__LM_RF_RB_PROT  && ((Hexapod->_robotStepFase == LF_LB_RM_PROT) || (Hexapod->_robotStepFase == LF_LB_RM_PROT__LM_RF_RB_RETR))) {

        for (int i = 0; i < num_steps; i++) {
            
            /*================ PROTRAKCJA =================*/
            y_tab_with_delta[LEFT_MIDDLE] = Hexapod->_LegsPositionRobotCenter[LEFT_MIDDLE].Pr_y + delta_y;
            y_tab_with_delta[RIGHT_FRONT] = Hexapod->_LegsPositionRobotCenter[RIGHT_FRONT].Pr_y + delta_y;
            y_tab_with_delta[RIGHT_BACK] =  Hexapod->_LegsPositionRobotCenter[RIGHT_BACK].Pr_y  + delta_y;

            z_elipse[LEFT_MIDDLE] = calculateElipseZ(fase, LEFT_MIDDLE, y_tab_with_delta[LEFT_MIDDLE],  Y_0_tab[LEFT_MIDDLE],   Z_s_tab[LEFT_MIDDLE],   Y_move_distance, h_const);
            z_elipse[RIGHT_FRONT] = calculateElipseZ(fase, RIGHT_FRONT, y_tab_with_delta[RIGHT_FRONT],  Y_0_tab[RIGHT_FRONT],   Z_s_tab[RIGHT_FRONT],   Y_move_distance, h_const);
            z_elipse[RIGHT_BACK] =  calculateElipseZ(fase, RIGHT_BACK,  y_tab_with_delta[RIGHT_BACK],   Y_0_tab[RIGHT_BACK],    Z_s_tab[RIGHT_BACK],    Y_move_distance, h_const);

            evaluateLegPositionRobotCenter(Hexapod, LEFT_MIDDLE,    x_tab[LEFT_MIDDLE], y_tab_with_delta[LEFT_MIDDLE],  z_elipse[LEFT_MIDDLE]);
            evaluateLegPositionRobotCenter(Hexapod, RIGHT_FRONT,    x_tab[RIGHT_FRONT], y_tab_with_delta[RIGHT_FRONT],  z_elipse[RIGHT_FRONT]);
            evaluateLegPositionRobotCenter(Hexapod, RIGHT_BACK,     x_tab[RIGHT_BACK],  y_tab_with_delta[RIGHT_BACK],   z_elipse[RIGHT_BACK]);
            /*=============================================*/

            /*================ RETRAKCJA ==================*/
            y_tab_with_delta[LEFT_FRONT] =      Hexapod->_LegsPositionRobotCenter[LEFT_FRONT].Pr_y      - delta_y;
            y_tab_with_delta[LEFT_BACK] =       Hexapod->_LegsPositionRobotCenter[LEFT_BACK].Pr_y       - delta_y;
            y_tab_with_delta[RIGHT_MIDDLE] =    Hexapod->_LegsPositionRobotCenter[RIGHT_MIDDLE].Pr_y    - delta_y;

            z_elipse[LEFT_FRONT] =      calculateElipseZ(fase, LEFT_FRONT,      y_tab_with_delta[LEFT_FRONT],   Y_0_tab[LEFT_FRONT],    Z_s_tab[LEFT_FRONT],    Y_move_distance, h_const);
            z_elipse[LEFT_BACK] =       calculateElipseZ(fase, LEFT_BACK,       y_tab_with_delta[LEFT_BACK],    Y_0_tab[LEFT_BACK],     Z_s_tab[LEFT_BACK],     Y_move_distance, h_const);
            z_elipse[RIGHT_MIDDLE] =    calculateElipseZ(fase, RIGHT_MIDDLE,    y_tab_with_delta[RIGHT_MIDDLE], Y_0_tab[RIGHT_MIDDLE],  Z_s_tab[RIGHT_MIDDLE],  Y_move_distance, h_const);

            evaluateLegPositionRobotCenter(Hexapod, LEFT_FRONT,     x_tab[LEFT_FRONT],      y_tab_with_delta[LEFT_FRONT],   z_elipse[LEFT_FRONT]);
            evaluateLegPositionRobotCenter(Hexapod, LEFT_BACK,      x_tab[LEFT_BACK],       y_tab_with_delta[LEFT_BACK],    z_elipse[LEFT_BACK]);
            evaluateLegPositionRobotCenter(Hexapod, RIGHT_MIDDLE,   x_tab[RIGHT_MIDDLE],    y_tab_with_delta[RIGHT_MIDDLE], z_elipse[RIGHT_MIDDLE]);
            /*=============================================*/
            
            printf("Dla RIGHT FRONT: y = %f, y_0 = %f, z = %f\n", y_tab_with_delta[RIGHT_FRONT], Y_0_tab[RIGHT_FRONT], z_elipse[RIGHT_FRONT] );
            delay(step_time*1000); // `delay` przyjmuje wartość w milisekundach, więc konwersja na ms
        }

        Hexapod->_robotStepFase = LF_LB_RM_RETR__LM_RF_RB_PROT;

    } else {
        if (fase == LF_LB_RM_PROT__LM_RF_RB_RETR) {
            printf("Ostatnio przypisana faza ruchu nie pozwala na wykonanie fazy: LF_LB_RM_PROT__LM_RF_RB_RETR \n");
        } else if (fase == LF_LB_RM_RETR__LM_RF_RB_PROT) {
            printf("Ostatnio przypisana faza ruchu nie pozwala na wykonanie fazy: LF_LB_RM_RETR__LM_RF_RB_PROT \n");
        } else {
            printf("Błędnie zadeklarowana faza ruchu w doStepFase\n");
        }
    }



}


#endif