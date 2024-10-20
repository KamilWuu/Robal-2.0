#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdio.h>


#include "servo.h"
#include "defines.h"
#include "kinematics.h"

// Definicja typu dla stron robota (lewa, prawa)
typedef enum {
    LEFT,  
    RIGHT  
} RobotSide;



// Definicja typu dla nóg robota
typedef enum {
    LEFT_FRONT,   // Lewa przednia
    LEFT_MIDDLE,  // Lewa środkowa
    LEFT_BACK,    // Lewa tylna
    RIGHT_FRONT,  // Prawa przednia
    RIGHT_MIDDLE, // Prawa środkowa
    RIGHT_BACK    // Prawa tylna
} LegPosition;


// Struktura dla jednego serwa
typedef struct {
     
    uint8_t _pca_channel; // Kanał na PCA
    float _min_angle;     // Minimalny kąt serwa
    float _max_angle;     // Maksymalny kąt serwa
    float _current_angle; // Aktualny kąt serwa
    float _target_angle;  // Docelowy kąt serwa
    float _last_angle;    // Ostatni zapisany kąt serwa
} Servo;

// Struktura dla jednej nogi
typedef struct {

    LegPosition _leg_position; // Pozycja: przód, środek, tył
    RobotSide _side;           // Lewa (LEFT) lub prawa (RIGHT) strona
    uint8_t _pca_address; // Adres PCA dla danej nogi

    Servo _q1_servo;      // Serwo dla kąta Q1
    Servo _q2_servo;      // Serwo dla kąta Q2
    Servo _q3_servo;      // Serwo dla kąta Q3

    float _last_pos[3];
    float _current_pos[3]; // Aktualna pozycja końcówki nogi [x, y, z]
    float _target_pos[3];  // Docelowa pozycja końcówki nogi [x, y, z]
} Leg;

// Struktura dla robota, który ma 6 nóg
typedef struct {
    Leg _legs[6]; // Tablica sześciu nóg (po trzy na każdą stronę)
} Robot;



// Funkcja do pobierania kanału PCA na podstawie pozycji i kąta
int getPCAChannel(LegPosition pos, int Q) {
    switch (pos) {
        case LEFT_FRONT:
            if (Q == 1) { return LEFT_FRONT_Q1; }
            if (Q == 2) { return LEFT_FRONT_Q2; }
            if (Q == 3) { return LEFT_FRONT_Q3; }
            break;
        
        case LEFT_MIDDLE:
            if (Q == 1) { return LEFT_MIDDLE_Q1; }
            if (Q == 2) { return LEFT_MIDDLE_Q2; }
            if (Q == 3) { return LEFT_MIDDLE_Q3; }
            break;
        
        case LEFT_BACK:
            if (Q == 1) { return LEFT_BACK_Q1; }
            if (Q == 2) { return LEFT_BACK_Q2; }
            if (Q == 3) { return LEFT_BACK_Q3; }
            break;
        
        case RIGHT_FRONT:
            if (Q == 1) { return RIGHT_FRONT_Q1; }
            if (Q == 2) { return RIGHT_FRONT_Q2; }
            if (Q == 3) { return RIGHT_FRONT_Q3; }
            break;
        
        case RIGHT_MIDDLE:
            if (Q == 1) { return RIGHT_MIDDLE_Q1; }
            if (Q == 2) { return RIGHT_MIDDLE_Q2; }
            if (Q == 3) { return RIGHT_MIDDLE_Q3; }
            break;
        
        case RIGHT_BACK:
            if (Q == 1) { return RIGHT_BACK_Q1; }
            if (Q == 2) { return RIGHT_BACK_Q2; }
            if (Q == 3) { return RIGHT_BACK_Q3; }
            break;

        default:
            // Obsługa błędnych danych wejściowych
            printf("Invalid LegPosition or Q value.\n");
            return -1; // Zwracamy -1 dla błędnych danych
    }
    return -1; // Zwracamy -1 dla błędnych danych
}

int getInitialAngle(int Q, enum RobotSide side){
    int Q_variable; 
    int Q_side_multiplier; 

    if(side == LEFT){
        if(Q == 1){
            Q_side_multiplier = 1;
        }else{
            Q_side_multiplier = -1;
        }
        
    }else if(side == RIGHT ){
        if(Q == 1){
            Q_side_multiplier = -1;
        }else{
            Q_side_multiplier = 1;
        }
    }

    switch (Q)
    {
    case 1:
        Q_variable = 135 - Q1_initial_angle;
        break;
    
    case 2:
        Q_variable = 135 - Q2_initial_angle;
        break;
    
    case 3:
        Q_variable = 135 - Q3_initial_angle;
        break;

    default:
        break;
    }

    return 135 - (Q_variable * Q_side_multiplier);

}







void initLeg(Leg *leg, LegPosition leg_position){
     // Przypisz stronę i pozycję nogi
    leg->_leg_position = leg_position;
    if(leg_position == LEFT_FRONT || leg_position == LEFT_MIDDLE || leg_position ==  LEFT_BACK){
        leg->_side = LEFT;
        leg->_pca_address = pca_left;
    }else if(leg_position == RIGHT_FRONT || leg_position == RIGHT_MIDDLE || leg_position ==  RIGHT_BACK){
        leg->_side = RIGHT;
        leg->_pca_address = pca_right;
    }

   


    // Inicjalizacja serw Q1
    int q1_min, q1_max;

    if(leg_position == LEFT_FRONT || leg_position == RIGHT_FRONT){
        q1_min = FRONT_Q1_MIN_ANGLE;
        q1_max = FRONT_Q1_MAX_ANGLE;
    }else if(leg_position == LEFT_MIDDLE || leg_position == RIGHT_MIDDLE){
        q1_min = MIDDLE_Q1_MIN_ANGLE;
        q1_max = MIDDLE_Q1_MAX_ANGLE;
    }else if(leg_position == LEFT_BACK || leg_position == RIGHT_BACK){
        q1_min = BACK_Q1_MIN_ANGLE;
        q1_max = BACK_Q1_MAX_ANGLE;
    }

    leg->_q1_servo.pca_channel = getPCAChannel(position, 1); 

    leg->_q1_servo.min_angle = q1_min;
    leg->_q1_servo.max_angle = q1_max; // Maksymalny kąt dla serwa

    leg->_q1_servo.current_angle = getInitialAngle(1, leg->_side); // Początkowy kąt
    leg->_q1_servo.target_angle = getInitialAngle(1, leg->_side); // Początkowy kąt
    leg->_q1_servo.last_angle = getInitialAngle(1, leg->_side); // Początkowy kąt


    // Inicjalizacja serw Q2
    leg->_q2_servo.pca_channel = getPCAChannel(position, 2); 

    leg->_q2_servo.min_angle = Q2_MIN_ANGLE;
    leg->_q2_servo.max_angle = Q2_MAX_ANGLE; // Maksymalny kąt dla serwa

    leg->_q2_servo.current_angle = getInitialAngle(2, leg->_side); // Początkowy kąt
    leg->_q2_servo.target_angle = getInitialAngle(2, leg->_side); // Początkowy kąt
    leg->_q2_servo.last_angle = getInitialAngle(2, leg->_side); // Początkowy kąt


    // Inicjalizacja serw Q3
    leg->_q3_servo.pca_channel = getPCAChannel(position, 3); 

    leg->_q3_servo.min_angle = Q3_MIN_ANGLE;
    leg->_q3_servo.max_angle = Q3_MAX_ANGLE; // Maksymalny kąt dla serwa

    leg->_q3_servo.current_angle = getInitialAngle(3, leg->_side); // Początkowy kąt
    leg->_q3_servo.target_angle = getInitialAngle(3, leg->_side); // Początkowy kąt
    leg->_q3_servo.last_angle = getInitialAngle(3, leg->_side); // Początkowy kąt
    

    if(leg->_side == LEFT ){

        int temp1, temp2, temp3;

        temp1 = leg->_q1_servo.min_angle;
        leg->_q1_servo.min_angle = leg->_q1_servo.max_angle;
        leg->_q1_servo.max_angle = temp1;

        temp2 = leg->_q2_servo.min_angle;
        leg->_q2_servo.min_angle = leg->_q2_servo.max_angle;
        leg->_q2_servo.max_angle = temp2;

        temp3 = leg->_q3_servo.min_angle;
        leg->_q3_servo.min_angle = leg->_q3_servo.max_angle;
        leg->_q3_servo.max_angle = temp3;

    }


}


void initRobot(Robot *robot) {
    // Inicjalizuj nogi robota
    initLeg(&robot->_legs[0], LEFT_FRONT);
    initLeg(&robot->_legs[1], LEFT_MIDDLE);
    initLeg(&robot->_legs[2], LEFT_BACK);
    initLeg(&robot->_legs[3], RIGHT_FRONT);
    initLeg(&robot->_legs[4], RIGHT_MIDDLE);
    initLeg(&robot->_legs[5], RIGHT_BACK);
}





#endif // ROBOT_H