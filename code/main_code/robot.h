#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "servo.h"
#include "defines.h"
#include "kinematics.h"
#include "transformMatrix.h"


// Definicja typu dla stron robota (lewa, prawa)
typedef enum
{
    LEFT,
    RIGHT
} RobotSide;

// Definicja typu dla nóg robota
typedef enum
{
    LEFT_FRONT,   // Lewa przednia
    LEFT_MIDDLE,  // Lewa środkowa
    LEFT_BACK,    // Lewa tylna
    RIGHT_FRONT,  // Prawa przednia
    RIGHT_MIDDLE, // Prawa środkowa
    RIGHT_BACK    // Prawa tylna
} LegPosition;

// Struktura dla jednego serwa
typedef struct
{

    uint8_t _pca_channel; // Kanał na PCA
    float _min_angle;     // Minimalny kąt serwa
    float _max_angle;     // Maksymalny kąt serwa
    float _current_angle; // Aktualny kąt serwa
    float _target_angle;  // Docelowy kąt serwa
    float _last_angle;    // Ostatni zapisany kąt serwa
} Servo;

// Struktura dla jednej nogi
typedef struct
{

    LegPosition _leg_position; // Pozycja: przód, środek, tył
    RobotSide _side;           // Lewa (LEFT) lub prawa (RIGHT) strona
    int _pca;                  // które pca

    Servo _q1_servo; // Serwo dla kąta Q1
    Servo _q2_servo; // Serwo dla kąta Q2
    Servo _q3_servo; // Serwo dla kąta Q3

    float _last_pos[3];
    float _current_pos[3]; // Aktualna pozycja końcówki nogi [x, y, z]
    float _target_pos[3];  // Docelowa pozycja końcówki nogi [x, y, z]
} Leg;



/*
Protraction – This is the phase where the robot’s leg is lifted and moved forward in preparation 
for contact with the ground in a new position. During protraction, the leg swings forward to set 
up the next step. This phase is essential for redistributing weight and preparing for the next stride.

Retraction – This phase occurs when the robot’s leg presses down against the ground and moves backward 
relative to the robot's body, pushing the robot forward. Retraction is when the leg is in contact with 
the ground and generates the propulsive force needed for movement.
*/


typedef enum {
    UNKNOWN,
    TRANSPORT_POSITION,
    WALKING_POSITION,
    STAND_UP,
    SIT_DOWN,
    LF_LB_RM_PROT, //przygotowanie do ruchu, protrakcja LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE  bez retrakcji pozostalych
    LM_RF_RB_PROT, //przygotowanie do ruchu, protrakcja LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK bez retrakcji pozostalych
    LF_LB_RM_PROT__LM_RF_RB_RETR,   // LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE_PROTRACTION__LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK_RETRACTION
    LF_LB_RM_RETR__LM_RF_RB_PROT    // LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE_RETRACTION__LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK_PROTRACTION
} StepFase;


// Struktura dla robota, który ma 6 nóg
typedef struct
{
    Leg _legs[6]; // Tablica sześciu nóg (po trzy na każdą stronę)
    PositionVector _LegsPositionRobotCenter[6];
    StepFase _robotStepFase;
} Robot;




void printRobotStepFase(Robot *robot) {
    switch (robot->_robotStepFase) {
        case UNKNOWN:
            printf("Stan robota: UNKNOWN\n");
            break;

        case TRANSPORT_POSITION:
            printf("Stan robota: TRANSPORT POSITION\n");
            break;

        case WALKING_POSITION:
            printf("Stan robota: WALKING POSITION\n");
            break;

        case STAND_UP:
            printf("Stan robota: STAND UP\n");
            break;

        case SIT_DOWN:
            printf("Stan robota: SIT DOWN\n");
            break;

        case LF_LB_RM_PROT:
            printf("Stan robota: LEFT FRONT LEFT BACK RIGHT MIDDLE PROTRACTION\n");
            break;

        case LM_RF_RB_PROT:
            printf("Stan robota: LEFT MIDDLE RIGHT FRONT RIGHT BACK PROTRACTION\n");
            break;

        case LF_LB_RM_PROT__LM_RF_RB_RETR:
            printf("Stan robota: LEFT FRONT LEFT BACK RIGHT MIDDLE PROTRACTION with LEFT MIDDLE RIGHT FRONT RIGHT BACK RETRACTION\n");
            break;

        case LF_LB_RM_RETR__LM_RF_RB_PROT:
            printf("Stan robota: LEFT FRONT LEFT BACK RIGHT MIDDLE RETRACTION with LEFT MIDDLE RIGHT FRONT RIGHT BACK PROTRACTION\n");
            break;

        default:
            printf("Stan robota: NIEZNANY STAN\n");
            break;
    }
}



void printLegsPositions(Robot *robot) {
    
    
    // Wyczyść terminal
    #ifdef _WIN32
        system("cls"); // Windows
    #else
        system("clear"); // Unix/Linux/MacOS
    #endif
    

    // Definicje kolorów
    const char* RED = "\033[1;31m";
    const char* BLUE = "\033[1;34m";
    const char* RESET = "\033[0m";

    // Wyświetlenie pozycji nóg
    printf("====================");
    printRobotStepFase(robot);
    printf("===========================================================================\n");
    printf("%sLEFT_FRONT\t->\t[%lf; %lf; %lf]%s\t\t\t%s[%lf; %lf; %lf] <- RIGHT_FRONT%s\n",
           RED,
           robot->_LegsPositionRobotCenter[LEFT_FRONT].P_x,
           robot->_LegsPositionRobotCenter[LEFT_FRONT].P_y,
           robot->_LegsPositionRobotCenter[LEFT_FRONT].P_z,
           RESET,
           BLUE,
           robot->_LegsPositionRobotCenter[RIGHT_FRONT].P_x,
           robot->_LegsPositionRobotCenter[RIGHT_FRONT].P_y,
           robot->_LegsPositionRobotCenter[RIGHT_FRONT].P_z,
           RESET);

    printf("%sLEFT_MIDDLE\t->\t[%lf; %lf; %lf]%s\t\t\t%s[%lf; %lf; %lf] <- RIGHT_MIDDLE%s\n",
           BLUE,
           robot->_LegsPositionRobotCenter[LEFT_MIDDLE].P_x,
           robot->_LegsPositionRobotCenter[LEFT_MIDDLE].P_y,
           robot->_LegsPositionRobotCenter[LEFT_MIDDLE].P_z,
           RESET,
           RED,
           robot->_LegsPositionRobotCenter[RIGHT_MIDDLE].P_x,
           robot->_LegsPositionRobotCenter[RIGHT_MIDDLE].P_y,
           robot->_LegsPositionRobotCenter[RIGHT_MIDDLE].P_z,
           RESET);

    printf("%sLEFT_BACK\t->\t[%lf; %lf; %lf]%s\t\t\t%s[%lf; %lf; %lf] <- RIGHT_BACK%s\n",
           RED,
           robot->_LegsPositionRobotCenter[LEFT_BACK].P_x,
           robot->_LegsPositionRobotCenter[LEFT_BACK].P_y,
           robot->_LegsPositionRobotCenter[LEFT_BACK].P_z,
           RESET,
           BLUE,
           robot->_LegsPositionRobotCenter[RIGHT_BACK].P_x,
           robot->_LegsPositionRobotCenter[RIGHT_BACK].P_y,
           robot->_LegsPositionRobotCenter[RIGHT_BACK].P_z,
           RESET);
    printf("=================================================================================================================\n\n\n\n");
}





// Funkcja do pobierania kanału PCA na podstawie pozycji i kąta
int getPCAChannel(LegPosition pos, int Q)
{
    switch (pos)
    {
    case LEFT_FRONT:
        if (Q == 1)
        {
            return LEFT_FRONT_Q1;
        }
        if (Q == 2)
        {
            return LEFT_FRONT_Q2;
        }
        if (Q == 3)
        {
            return LEFT_FRONT_Q3;
        }
        break;

    case LEFT_MIDDLE:
        if (Q == 1)
        {
            return LEFT_MIDDLE_Q1;
        }
        if (Q == 2)
        {
            return LEFT_MIDDLE_Q2;
        }
        if (Q == 3)
        {
            return LEFT_MIDDLE_Q3;
        }
        break;

    case LEFT_BACK:
        if (Q == 1)
        {
            return LEFT_BACK_Q1;
        }
        if (Q == 2)
        {
            return LEFT_BACK_Q2;
        }
        if (Q == 3)
        {
            return LEFT_BACK_Q3;
        }
        break;

    case RIGHT_FRONT:
        if (Q == 1)
        {
            return RIGHT_FRONT_Q1;
        }
        if (Q == 2)
        {
            return RIGHT_FRONT_Q2;
        }
        if (Q == 3)
        {
            return RIGHT_FRONT_Q3;
        }
        break;

    case RIGHT_MIDDLE:
        if (Q == 1)
        {
            return RIGHT_MIDDLE_Q1;
        }
        if (Q == 2)
        {
            return RIGHT_MIDDLE_Q2;
        }
        if (Q == 3)
        {
            return RIGHT_MIDDLE_Q3;
        }
        break;

    case RIGHT_BACK:
        if (Q == 1)
        {
            return RIGHT_BACK_Q1;
        }
        if (Q == 2)
        {
            return RIGHT_BACK_Q2;
        }
        if (Q == 3)
        {
            return RIGHT_BACK_Q3;
        }
        break;

    default:
        // Obsługa błędnych danych wejściowych
        printf("Invalid LegPosition or Q value.\n");
        global_error++;
        return -1; // Zwracamy -1 dla błędnych danych
    }
    return -1; // Zwracamy -1 dla błędnych danych
}

int getInitialAngle(int Q, RobotSide side)
{
    int Q_variable;
    int Q_side_multiplier;

    if (side == LEFT)
    {
        if (Q == 1)
        {
            Q_side_multiplier = 1;
        }
        else
        {
            Q_side_multiplier = -1;
        }
    }
    else if (side == RIGHT)
    {
        if (Q == 1)
        {
            Q_side_multiplier = -1;
        }
        else
        {
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

    // return 135 - (Q_variable * Q_side_multiplier);

    return 135;
}

// Funkcja zwracająca minimalny kąt dla danego przegubu (Q) i pozycji nogi
int getMinAngle(LegPosition pos, int Q)
{
    switch (Q)
    {
    case 1: // Dla Q1, różne wartości w zależności od pozycji nogi
        switch (pos)
        {
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
        switch (pos)
        {
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
        switch (pos)
        {
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
int getMaxAngle(LegPosition pos, int Q)
{
    switch (Q)
    {
    case 1: // Dla Q1, różne wartości w zależności od pozycji nogi
        switch (pos)
        {
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
        switch (pos)
        {
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
        switch (pos)
        {
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

void initLeg(Leg *leg, LegPosition leg_position)
{
    // Przypisz stronę i pozycję nogi
    leg->_leg_position = leg_position;
    if (leg_position == LEFT_FRONT || leg_position == LEFT_MIDDLE || leg_position == LEFT_BACK)
    {
        leg->_side = LEFT;
        leg->_pca = pca_left;
    }
    else if (leg_position == RIGHT_FRONT || leg_position == RIGHT_MIDDLE || leg_position == RIGHT_BACK)
    {
        leg->_side = RIGHT;
        leg->_pca = pca_right;
    }

    // Inicjalizacja serw Q1
    leg->_q1_servo._pca_channel = getPCAChannel(leg->_leg_position, 1);
    if (leg->_q1_servo._pca_channel > 15)
    {
        printf("Cos poszlo nie tak dla pca channel q1\n");
        global_error++;
    }

    leg->_q1_servo._min_angle = getMinAngle(leg->_leg_position, 1);
    leg->_q1_servo._max_angle = getMaxAngle(leg->_leg_position, 1);

    leg->_q1_servo._current_angle = getInitialAngle(1, leg->_side);
    leg->_q1_servo._target_angle = getInitialAngle(1, leg->_side);
    leg->_q1_servo._last_angle = getInitialAngle(1, leg->_side);

    // Inicjalizacja serw Q2
    leg->_q2_servo._pca_channel = getPCAChannel(leg->_leg_position, 2);
    if (leg->_q2_servo._pca_channel > 15)
    {
        printf("Cos poszlo nie tak dla pca channel q2\n");
        global_error++;
    }

    leg->_q2_servo._min_angle = getMinAngle(leg->_leg_position, 2);
    leg->_q2_servo._max_angle = getMaxAngle(leg->_leg_position, 2);

    leg->_q2_servo._current_angle = getInitialAngle(2, leg->_side);
    leg->_q2_servo._target_angle = getInitialAngle(2, leg->_side);
    leg->_q2_servo._last_angle = getInitialAngle(2, leg->_side);

    // Inicjalizacja serw Q3
    leg->_q3_servo._pca_channel = getPCAChannel(leg->_leg_position, 3);
    if (leg->_q3_servo._pca_channel > 15)
    {
        printf("Cos poszlo nie tak dla pca channel q3\n");
        global_error++;
    }

    leg->_q3_servo._min_angle = getMinAngle(leg->_leg_position, 3);
    leg->_q3_servo._max_angle = getMaxAngle(leg->_leg_position, 3);

    leg->_q3_servo._current_angle = getInitialAngle(3, leg->_side);
    leg->_q3_servo._target_angle = getInitialAngle(3, leg->_side);
    leg->_q3_servo._last_angle = getInitialAngle(3, leg->_side);

    for (int i = 0; i < 3; i++)
    {
        leg->_current_pos[i] = 0;
        leg->_target_pos[i] = 0;
        leg->_last_pos[i] = 0;
    }
}


int checkPosition(LegPosition pos, double x, double y, double z)
{

    switch (pos)
    {
    case LEFT_FRONT:
        if (x < -L1 - d1 - L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT FRONT, x = %2.f, warunek: X < %2.f\n", x, -L1 - d1 - L23);
            global_error++;
            return 0;
        }
        break;

    case LEFT_MIDDLE:
        if (x < -L1 - d2 - L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT MIDDLE, x = %2.f, warunek: X < %2.f\n", x, -L1 - d2 - L23);
            global_error++;
            return 0;
        }
        break;

    case LEFT_BACK:
        if (x < -L1 - d1 - L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT BACK, x = %2.f, warunek: X < %2.f\n", x, -L1 - d1 - L23);
            global_error++;
            return 0;
        }
        break;

    case RIGHT_FRONT:
        if (x > L1 + d1 + L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT FRONT, x = %2.f, warunek: X > %2.f\n", x, L1 + d1 + L23);
            global_error++;
            return 0;
        }
        break;

    case RIGHT_MIDDLE:
        if (x > L1 + d2 + L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT FRONT, x = %2.f, warunek: X > %2.f\n", x, L1 + d2 + L23);
            global_error++;
            return 0;
        }
        break;

    case RIGHT_BACK:
        if (x > L1 + d1 + L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT BACK, x = %2.f, warunek: X > %2.f\n", x, L1 + d1 + L23);
            global_error++;
            return 0;
        }
        break;

    default:
        printf("Nieprawidlowa pozycja nogi\n");
        global_error++;
        break;
    }
}



void calculateInvertedKinematics(Leg *leg)
{

    double Q1, Q2, Q3;
    double Q3_1, Q3_2;

    double Xp = leg->_target_pos[0];
    double Yp = leg->_target_pos[1];
    double Zp = leg->_target_pos[2];

    double Xpz;

    if (leg->_side == LEFT)
    {
        Xp = -Xp;
    }

    Xpz = sqrt((Xp * Xp) + (Yp * Yp));
    Q1 = atan2(Yp, Xp);
    Q3_1 = MY_PI - acos(((L2 * L2) + (L3 * L3) - (Zp * Zp) - ((Xpz - L1) * (Xpz - L1))) / (2 * L2 * L3));
    Q3_2 = acos(((Zp * Zp) + ((Xpz - L1) * (Xpz - L1)) - (L2 * L2) - (L3 * L3)) / (2 * L2 * L3));
    Q3 = -Q3_1; // lub -Q3_2
    Q2 = atan2(Zp, (Xpz - L1)) + atan2(L3 * sin(-Q3), (L2 + L3 * cos(-Q3)));

    if (leg->_side == RIGHT)
    {

        leg->_q1_servo._target_angle = 135 - (Q1 * RAD2DEG);
        leg->_q2_servo._target_angle = 135 + (Q2 * RAD2DEG);
        leg->_q3_servo._target_angle = 135 + (Q3 * RAD2DEG);
    }

    if (leg->_side == LEFT)
    {

        leg->_q1_servo._target_angle = 135 + (Q1 * RAD2DEG);
        leg->_q2_servo._target_angle = 135 - (Q2 * RAD2DEG);
        leg->_q3_servo._target_angle = 135 - (Q3 * RAD2DEG);
    }

    /*printf("Q1 = %.2f\n", leg->_q1_servo._target_angle );
    printf("Q2 = %.2f\n", leg->_q2_servo._target_angle );
    printf("Q3 = %.2f\n", leg->_q3_servo._target_angle );*/
}

void setTargetPos(Leg *leg, double xp, double yp, double zp)
{
    leg->_target_pos[0] = xp;
    leg->_target_pos[1] = yp;
    leg->_target_pos[2] = zp;
}


void initLegPositionRobotCenter(Robot *robot, LegPosition pos, double x, double y, double z)
{
    double xp, yp, zp;
    zp = z + z_0;

    

    if (checkPosition(pos, x, y, z))
    {

        robot->_LegsPositionRobotCenter[pos].P_x = x;
        robot->_LegsPositionRobotCenter[pos].P_y = y;
        robot->_LegsPositionRobotCenter[pos].P_z = z;

        switch (pos)
        {
        case LEFT_FRONT:
            xp = x + d1;
            yp = y - d3;
            break;

        case LEFT_MIDDLE:
            xp = x + d2;
            yp = y;
            break;

        case LEFT_BACK:
            xp = x + d1;
            yp = y + d3;
            break;

        case RIGHT_FRONT:
            xp = x - d1;
            yp = y - d3;
            break;

        case RIGHT_MIDDLE:
            xp = x - d2;
            yp = y;
            break;

        case RIGHT_BACK:
            xp = x - d1;
            yp = y + d3;
            break;

        default:
            printf("bledna pozycja nogi w liczeniu kinematyki nogi wzgledem srodka\n");
            global_error++;
            return;
            break;
        }

        setTargetPos(&robot->_legs[pos], xp, yp, zp);
        calculateInvertedKinematics(&robot->_legs[pos]);
    }
}

void initRobot(Robot *robot)
{
    // Inicjalizuj nogi robota
    initLeg(&robot->_legs[0], LEFT_FRONT);
    initLeg(&robot->_legs[1], LEFT_MIDDLE);
    initLeg(&robot->_legs[2], LEFT_BACK);
    initLeg(&robot->_legs[3], RIGHT_FRONT);
    initLeg(&robot->_legs[4], RIGHT_MIDDLE);
    initLeg(&robot->_legs[5], RIGHT_BACK);


    initLegPositionRobotCenter(robot, LEFT_FRONT, -x_const, y_const, z_const_zero);
    initLegPositionRobotCenter(robot, LEFT_MIDDLE, -x_const, 0, z_const_zero);
    initLegPositionRobotCenter(robot, LEFT_FRONT, -x_const, -y_const, z_const_zero);

    initLegPositionRobotCenter(robot, RIGHT_FRONT, x_const, y_const, z_const_zero);
    initLegPositionRobotCenter(robot, RIGHT_MIDDLE, x_const, 0, z_const_zero);
    initLegPositionRobotCenter(robot, RIGHT_BACK, x_const, -y_const, z_const_zero);


    robot->_robotStepFase == UNKNOWN;
}



void moveToTargetPosition(Leg *leg)
{
    /*

    Na poczatku sprawdza czy dane Q_docelowe mieści sie w przedziale ograniczonym jako min max, jesli sie miesci ustawia ten kąt docelowy na danym serwie, i potem przypisuje docelowy kąt jako aktualny kąt

    jesli nie miesci sie w przedziale granicznym sprawdza na poczatku czy dany kat docelowy jest mniejszy niz min_angle jesli jest ustawia min angle na serwie i nastepnie przypisuje min angle jako aktualny kąt,
    podobnie to dziala dla max angle.

    */

    int pos_true = 1;

    if ((leg->_q1_servo._target_angle > leg->_q1_servo._min_angle) && (leg->_q1_servo._target_angle < leg->_q1_servo._max_angle))
    {
        SetServoAngle(leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._target_angle);
        leg->_q1_servo._current_angle = leg->_q1_servo._target_angle;
    }
    else
    {

        if (leg->_q1_servo._target_angle < leg->_q1_servo._min_angle)
        {
            SetServoAngle(leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._min_angle);
            leg->_q1_servo._current_angle = leg->_q1_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q1!\n");
        }
        else if (leg->_q1_servo._target_angle > leg->_q1_servo._max_angle)
        {
            SetServoAngle(leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._max_angle);
            leg->_q1_servo._current_angle = leg->_q1_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q1!\n");
        }
        pos_true = 0;
    }

    if ((leg->_q2_servo._target_angle > leg->_q2_servo._min_angle) && (leg->_q2_servo._target_angle < leg->_q2_servo._max_angle))
    {
        SetServoAngle(leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._target_angle);
        leg->_q2_servo._current_angle = leg->_q2_servo._target_angle;
    }
    else
    {

        if (leg->_q2_servo._target_angle < leg->_q2_servo._min_angle)
        {
            SetServoAngle(leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._min_angle);
            leg->_q2_servo._current_angle = leg->_q2_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q2!\n");
        }
        else if (leg->_q2_servo._target_angle > leg->_q2_servo._max_angle)
        {
            SetServoAngle(leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._max_angle);
            leg->_q2_servo._current_angle = leg->_q2_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q2!\n");
        }
        pos_true = 0;
    }

    if ((leg->_q3_servo._target_angle > leg->_q3_servo._min_angle) && (leg->_q3_servo._target_angle < leg->_q3_servo._max_angle))
    {
        SetServoAngle(leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._target_angle);
        leg->_q3_servo._current_angle = leg->_q3_servo._target_angle;
    }
    else
    {

        if (leg->_q3_servo._target_angle < leg->_q3_servo._min_angle)
        {
            SetServoAngle(leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._min_angle);
            leg->_q3_servo._current_angle = leg->_q3_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q3!\n");
        }
        else if (leg->_q3_servo._target_angle > leg->_q3_servo._max_angle)
        {
            SetServoAngle(leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._max_angle);
            leg->_q3_servo._current_angle = leg->_q3_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q3!\n");
        }
        pos_true = 0;
    }

    if (pos_true == 1)
    {
        leg->_current_pos[0] = leg->_target_pos[0];
        leg->_current_pos[1] = leg->_target_pos[1];
        leg->_current_pos[2] = leg->_target_pos[2];
    }
}

void printServo(Servo servo)
{
    printf("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
    printf("PCA Channel: %d\n", servo._pca_channel);
    printf("Min Angle: %.2f\n", servo._min_angle);
    printf("Max Angle: %.2f\n", servo._max_angle);
    printf("Current Angle: %.2f\n", servo._current_angle);
    printf("Target Angle: %.2f\n", servo._target_angle);
    printf("Last Angle: %.2f\n", servo._last_angle);
    printf("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
}

void printLeg(Leg leg)
{
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
    // Wyświetlanie informacji o pozycji nogi i stronie robota
    printf("Leg Position: %d\n", leg._leg_position); // Zakładam, że LegPosition jest typu int lub enum
    printf("Side: %d\n", leg._side);                 // Zakładam, że RobotSide jest typu int lub enum
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



void evaluateLegPositionArcCenter()





void evaluateLegPositionRobotCenter(Robot *robot, LegPosition pos, double x, double y, double z)
{
    double xp, yp, zp;
    zp = z + z_0;

    

    if (checkPosition(pos, x, y, z))
    {

        robot->_LegsPositionRobotCenter[pos].P_x = x;
        robot->_LegsPositionRobotCenter[pos].P_y = y;
        robot->_LegsPositionRobotCenter[pos].P_z = z;

        switch (pos)
        {
        case LEFT_FRONT:
            xp = x + d1;
            yp = y - d3;
            break;

        case LEFT_MIDDLE:
            xp = x + d2;
            yp = y;
            break;

        case LEFT_BACK:
            xp = x + d1;
            yp = y + d3;
            break;

        case RIGHT_FRONT:
            xp = x - d1;
            yp = y - d3;
            break;

        case RIGHT_MIDDLE:
            xp = x - d2;
            yp = y;
            break;

        case RIGHT_BACK:
            xp = x - d1;
            yp = y + d3;
            break;

        default:
            printf("bledna pozycja nogi w liczeniu kinematyki nogi wzgledem srodka\n");
            global_error++;
            return;
            break;
        }

        // printf("pozycja %d dla: %.2f, %.2f, %.2f: \n" , pos, xp, yp, zp);
        setTargetPos(&robot->_legs[pos], xp, yp, zp);
        calculateInvertedKinematics(&robot->_legs[pos]);
        moveToTargetPosition(&robot->_legs[pos]);
        //printLeg(robot->_legs[pos]);
    }
}




#endif // ROBOT_H