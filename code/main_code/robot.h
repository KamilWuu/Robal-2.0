#ifndef ROBOT_H
#define ROBOT_H


#include <stdio.h>
#include <math.h>
#include <stdlib.h>

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
    int _pca; // które pca

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

int getInitialAngle(int Q, RobotSide side){
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

    //return 135 - (Q_variable * Q_side_multiplier);


    return 135;
}



// Funkcja zwracająca minimalny kąt dla danego przegubu (Q) i pozycji nogi
int getMinAngle(LegPosition pos, int Q) {
    switch (Q) {
        case 1: // Dla Q1, różne wartości w zależności od pozycji nogi
            switch (pos) {
                case LEFT_FRONT:
                    return FRONT_LEFT_Q1_MIN_ANGLE; // Kąty dla lewej przedniej nogi
                case RIGHT_FRONT:
                    return FRONT_RIGHT_Q1_MIN_ANGLE; // Kąty dla prawej przedniej nogi
                case LEFT_MIDDLE:
                    return MIDDLE_LEFT_Q1_MIN_ANGLE; // Kąty dla lewej środkowej nogi
                case RIGHT_MIDDLE:
                    return MIDDLE_RIGHT_Q1_MIN_ANGLE; // Kąty dla prawej środkowej nogi
                case LEFT_BACK:
                    return BACK_LEFT_Q1_MIN_ANGLE; // Kąty dla lewej tylnej nogi
                case RIGHT_BACK:
                    return BACK_RIGHT_Q1_MIN_ANGLE; // Kąty dla prawej tylnej nogi
                default:
                    return -1; // Błąd, nieprawidłowa pozycja
            }
        case 2: // Dla Q2 tylko podział na lewą i prawą stronę
            switch (pos) {
                case LEFT_FRONT:
                case LEFT_MIDDLE:
                case LEFT_BACK:
                    return LEFT_Q2_MIN_ANGLE;
                case RIGHT_FRONT:
                case RIGHT_MIDDLE:
                case RIGHT_BACK:
                    return RIGHT_Q2_MIN_ANGLE;
                default:
                    return -1;
            }
        case 3: // Dla Q3 tylko podział na lewą i prawą stronę
            switch (pos) {
                case LEFT_FRONT:
                case LEFT_MIDDLE:
                case LEFT_BACK:
                    return LEFT_Q3_MIN_ANGLE;
                case RIGHT_FRONT:
                case RIGHT_MIDDLE:
                case RIGHT_BACK:
                    return RIGHT_Q3_MIN_ANGLE;
                default:
                    return -1;
            }
        default:
            return -1; // Błąd, nieprawidłowy przegub
    }
}

// Funkcja zwracająca maksymalny kąt dla danego przegubu (Q) i pozycji nogi
int getMaxAngle(LegPosition pos, int Q) {
    switch (Q) {
        case 1: // Dla Q1, różne wartości w zależności od pozycji nogi
            switch (pos) {
                case LEFT_FRONT:
                    return FRONT_LEFT_Q1_MAX_ANGLE; // Kąty dla lewej przedniej nogi
                case RIGHT_FRONT:
                    return FRONT_RIGHT_Q1_MAX_ANGLE; // Kąty dla prawej przedniej nogi
                case LEFT_MIDDLE:
                    return MIDDLE_LEFT_Q1_MAX_ANGLE; // Kąty dla lewej środkowej nogi
                case RIGHT_MIDDLE:
                    return MIDDLE_RIGHT_Q1_MAX_ANGLE; // Kąty dla prawej środkowej nogi
                case LEFT_BACK:
                    return BACK_LEFT_Q1_MAX_ANGLE; // Kąty dla lewej tylnej nogi
                case RIGHT_BACK:
                    return BACK_RIGHT_Q1_MAX_ANGLE; // Kąty dla prawej tylnej nogi
                default:
                    return -1; // Błąd, nieprawidłowa pozycja
            }
        case 2: // Dla Q2 tylko podział na lewą i prawą stronę
            switch (pos) {
                case LEFT_FRONT:
                case LEFT_MIDDLE:
                case LEFT_BACK:
                    return LEFT_Q2_MAX_ANGLE;
                case RIGHT_FRONT:
                case RIGHT_MIDDLE:
                case RIGHT_BACK:
                    return RIGHT_Q2_MAX_ANGLE;
                default:
                    return -1;
            }
        case 3: // Dla Q3 tylko podział na lewą i prawą stronę
            switch (pos) {
                case LEFT_FRONT:
                case LEFT_MIDDLE:
                case LEFT_BACK:
                    return LEFT_Q3_MAX_ANGLE;
                case RIGHT_FRONT:
                case RIGHT_MIDDLE:
                case RIGHT_BACK:
                    return RIGHT_Q3_MAX_ANGLE;
                default:
                    return -1;
            }
        default:
            return -1; // Błąd, nieprawidłowy przegub
    }
}





void initLeg(Leg *leg, LegPosition leg_position){
     // Przypisz stronę i pozycję nogi
    leg->_leg_position = leg_position;
    if(leg_position == LEFT_FRONT || leg_position == LEFT_MIDDLE || leg_position ==  LEFT_BACK){
        leg->_side = LEFT;
        leg->_pca = pca_left;
    }else if(leg_position == RIGHT_FRONT || leg_position == RIGHT_MIDDLE || leg_position ==  RIGHT_BACK){
        leg->_side = RIGHT;
        leg->_pca = pca_right;
    }


    // Inicjalizacja serw Q1
    leg->_q1_servo._pca_channel = getPCAChannel(leg->_leg_position, 1); 
    if(leg->_q1_servo._pca_channel > 15){printf("Cos poszlo nie tak dla pca channel q1\n");}

    leg->_q1_servo._min_angle = getMinAngle(leg->_leg_position, 1);
    leg->_q1_servo._max_angle = getMaxAngle(leg->_leg_position, 1);

    leg->_q1_servo._current_angle = getInitialAngle(1, leg->_side); 
    leg->_q1_servo._target_angle = getInitialAngle(1, leg->_side); 
    leg->_q1_servo._last_angle = getInitialAngle(1, leg->_side); 


    // Inicjalizacja serw Q2
    leg->_q2_servo._pca_channel = getPCAChannel(leg->_leg_position, 2); 
    if(leg->_q2_servo._pca_channel > 15){printf("Cos poszlo nie tak dla pca channel q2\n");}

    leg->_q2_servo._min_angle = getMinAngle(leg->_leg_position, 2);
    leg->_q2_servo._max_angle = getMaxAngle(leg->_leg_position, 2);

    leg->_q2_servo._current_angle = getInitialAngle(2, leg->_side); 
    leg->_q2_servo._target_angle = getInitialAngle(2, leg->_side); 
    leg->_q2_servo._last_angle = getInitialAngle(2, leg->_side); 


    // Inicjalizacja serw Q3
    leg->_q3_servo._pca_channel = getPCAChannel(leg->_leg_position, 3); 
    if(leg->_q3_servo._pca_channel > 15){printf("Cos poszlo nie tak dla pca channel q3\n");}

    leg->_q3_servo._min_angle = getMinAngle(leg->_leg_position, 3);
    leg->_q3_servo._max_angle = getMaxAngle(leg->_leg_position, 3);

    leg->_q3_servo._current_angle = getInitialAngle(3, leg->_side); 
    leg->_q3_servo._target_angle = getInitialAngle(3, leg->_side); 
    leg->_q3_servo._last_angle = getInitialAngle(3, leg->_side); 
    

    for(int i = 0; i < 3; i++){
        leg->_current_pos[i] = 0;
        leg->_target_pos[i] = 0;
        leg->_last_pos[i] = 0;
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


void calculateInvertedKinematics(Leg * leg){

    double Q1, Q2, Q3;
    double Q3_1, Q3_2;

    double Xp = leg ->_target_pos[0];
    double Yp = leg ->_target_pos[1];
    double Zp = leg ->_target_pos[2];

    double Xpz;



    if(leg->_side == LEFT) {Xp = -Xp ;}

    Xpz = sqrt((Xp*Xp) + (Yp*Yp));
    Q1 = atan2(Yp, Xp);
    Q3_1 = MY_PI - acos(((L2*L2) + (L3*L3) - (Zp*Zp) - ((Xpz - L1)*(Xpz - L1))) / (2 * L2 * L3));
    Q3_2 = acos(((Zp*Zp) + ((Xpz - L1)*(Xpz - L1)) - (L2*L2) - (L3*L3)) / (2 * L2 * L3));
    Q3 = -Q3_1; //lub -Q3_2
    Q2 = atan2(Zp, (Xpz - L1)) + atan2(L3 * sin(-Q3), (L2 + L3 * cos(-Q3)));

    if(leg->_side == RIGHT){

        leg->_q1_servo._target_angle = 135 - (Q1*RAD2DEG);
        leg->_q2_servo._target_angle = 135 + (Q2*RAD2DEG);
        leg->_q3_servo._target_angle = 135 + (Q3*RAD2DEG);
    }


    if(leg->_side == LEFT){
        


        leg->_q1_servo._target_angle = 135 + (Q1*RAD2DEG);
        leg->_q2_servo._target_angle = 135 - (Q2*RAD2DEG);
        leg->_q3_servo._target_angle = 135 - (Q3*RAD2DEG);

    }



    /*printf("Q1 = %.2f\n", leg->_q1_servo._target_angle );
    printf("Q2 = %.2f\n", leg->_q2_servo._target_angle );
    printf("Q3 = %.2f\n", leg->_q3_servo._target_angle );*/

    


}


void setTargetPos(Leg * leg, double xp, double yp, double zp){
    leg->_target_pos[0] = xp;
    leg->_target_pos[1] = yp;
    leg->_target_pos[2] = zp;

}

void moveToTargetPosition(Leg * leg){
    /*

    Na poczatku sprawdza czy dane Q_docelowe mieści sie w przedziale ograniczonym jako min max, jesli sie miesci ustawia ten kąt docelowy na danym serwie, i potem przypisuje docelowy kąt jako aktualny kąt

    jesli nie miesci sie w przedziale granicznym sprawdza na poczatku czy dany kat docelowy jest mniejszy niz min_angle jesli jest ustawia min angle na serwie i nastepnie przypisuje min angle jako aktualny kąt, 
    podobnie to dziala dla max angle.

    */


    if( (leg->_q1_servo._target_angle > leg->_q1_servo._min_angle)    && (leg->_q1_servo._target_angle < leg->_q1_servo._max_angle)){
        SetServoAngle(leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._target_angle);
        leg->_q1_servo._current_angle = leg->_q1_servo._target_angle;
    }else{

        if(leg->_q1_servo._target_angle < leg->_q1_servo._min_angle){
            SetServoAngle(leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._min_angle);
            leg->_q1_servo._current_angle = leg->_q1_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q1!\n");
        }else 
        if(leg->_q1_servo._target_angle > leg->_q1_servo._max_angle){
            SetServoAngle(leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._max_angle);
            leg->_q1_servo._current_angle = leg->_q1_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q1!\n");
        }

    }

    if( (leg->_q2_servo._target_angle > leg->_q2_servo._min_angle)    && (leg->_q2_servo._target_angle < leg->_q2_servo._max_angle)){
        SetServoAngle(leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._target_angle);
        leg->_q2_servo._current_angle = leg->_q2_servo._target_angle;
    }else{

        if(leg->_q2_servo._target_angle < leg->_q2_servo._min_angle){
            SetServoAngle(leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._min_angle);
            leg->_q2_servo._current_angle = leg->_q2_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q2!\n");
        }else 
        if(leg->_q2_servo._target_angle > leg->_q2_servo._max_angle){
            SetServoAngle(leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._max_angle);
            leg->_q2_servo._current_angle = leg->_q2_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q2!\n");
        }

    }

    if( (leg->_q3_servo._target_angle > leg->_q3_servo._min_angle)    && (leg->_q3_servo._target_angle < leg->_q3_servo._max_angle)){
        SetServoAngle(leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._target_angle);
        leg->_q3_servo._current_angle = leg->_q3_servo._target_angle;
    }else{

        if(leg->_q3_servo._target_angle < leg->_q3_servo._min_angle){
            SetServoAngle(leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._min_angle);
            leg->_q3_servo._current_angle = leg->_q3_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q3!\n");
        }else 
        if(leg->_q3_servo._target_angle > leg->_q3_servo._max_angle){
            SetServoAngle(leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._max_angle);
            leg->_q3_servo._current_angle = leg->_q3_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q3!\n");
        }

    }




}



void printServo(Servo servo) {
    printf("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
    printf("PCA Channel: %d\n", servo._pca_channel);
    printf("Min Angle: %.2f\n", servo._min_angle);
    printf("Max Angle: %.2f\n", servo._max_angle);
    printf("Current Angle: %.2f\n", servo._current_angle);
    printf("Target Angle: %.2f\n", servo._target_angle);
    printf("Last Angle: %.2f\n", servo._last_angle);
    printf("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
}   


void printLeg(Leg leg) {
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
    // Wyświetlanie informacji o pozycji nogi i stronie robota
    printf("Leg Position: %d\n", leg._leg_position);  // Zakładam, że LegPosition jest typu int lub enum
    printf("Side: %d\n", leg._side);  // Zakładam, że RobotSide jest typu int lub enum
    printf("PCA: %d\n", leg._pca);

    // Wyświetlanie informacji o serwach
    printf("\nQ1 Servo:\n");
    printServo(leg._q1_servo);
    
    printf("\nQ2 Servo:\n");
    printServo(leg._q2_servo);
    
    printf("\nQ3 Servo:\n");
    printServo(leg._q3_servo);

    // Wyświetlanie pozycji końcówki nogi
    printf("\nLast Position: [%.2f, %.2f, %.2f]\n", leg._last_pos[0], leg._last_pos[1], leg._last_pos[2]);
    printf("Current Position: [%.2f, %.2f, %.2f]\n", leg._current_pos[0], leg._current_pos[1], leg._current_pos[2]);
    printf("Target Position: [%.2f, %.2f, %.2f]\n", leg._target_pos[0], leg._target_pos[1], leg._target_pos[2]);
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
}


#endif // ROBOT_H