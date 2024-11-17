#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "servo_hardware.h"
#include "defines.h"
#include "kinematics.h"
#include "enums.h"
#include "data_structures.h"



// Struktura dla jednego serwa
typedef struct Servo
{
    uint8_t _pca_channel; // Kanał na PCA
    double _min_angle;     // Minimalny kąt serwa
    double _max_angle;     // Maksymalny kąt serwa
    double _current_angle; // Aktualny kąt serwa
    double _last_angle;    // Ostatni zapisany kąt serwa
    double _target_angle;   
    
} Servo;

// Struktura dla jednej nogi
typedef struct Leg
{

    LegType _leg_type;        // Pozycja: przód, środek, tył
    RobotSide _side;           // Lewa (LEFT) lub prawa (RIGHT) strona
    int _pca;                  // które pca

    Servo _q1_servo; // Serwo dla kąta Q1
    Servo _q2_servo; // Serwo dla kąta Q2
    Servo _q3_servo; // Serwo dla kąta Q3

    Vector3 _current_pos; // Aktualna pozycja końcówki nogi [x, y, z]
    Vector3 _last_pos;
    Vector3 _target_pos;

    Vector3 _calculated_leg_pos;

    Vector3 _pos_error; 


    Vector3 _leg_speed;
    

} Leg;






// Struktura dla robota, który ma 6 nóg
typedef struct Robot
{
    Leg _legs[6]; // Tablica sześciu nóg (po trzy na każdą stronę)
    Vector3 _LegsPositionRobotCenter[6];
    StepFase _robotStepFase;
    Vector3 _robot_speed; //mm/s

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
           robot->_LegsPositionRobotCenter[LEFT_FRONT].data[X],
           robot->_LegsPositionRobotCenter[LEFT_FRONT].data[Y],
           robot->_LegsPositionRobotCenter[LEFT_FRONT].data[Z],
           RESET,
           BLUE,
           robot->_LegsPositionRobotCenter[RIGHT_FRONT].data[X],
           robot->_LegsPositionRobotCenter[RIGHT_FRONT].data[Y],
           robot->_LegsPositionRobotCenter[RIGHT_FRONT].data[Z],
           RESET);

    printf("%sLEFT_MIDDLE\t->\t[%lf; %lf; %lf]%s\t\t\t%s[%lf; %lf; %lf] <- RIGHT_MIDDLE%s\n",
           BLUE,
           robot->_LegsPositionRobotCenter[LEFT_MIDDLE].data[X],
           robot->_LegsPositionRobotCenter[LEFT_MIDDLE].data[Y],
           robot->_LegsPositionRobotCenter[LEFT_MIDDLE].data[Z],
           RESET,
           RED,
           robot->_LegsPositionRobotCenter[RIGHT_MIDDLE].data[X],
           robot->_LegsPositionRobotCenter[RIGHT_MIDDLE].data[Y],
           robot->_LegsPositionRobotCenter[RIGHT_MIDDLE].data[Z],
           RESET);

    printf("%sLEFT_BACK\t->\t[%lf; %lf; %lf]%s\t\t\t%s[%lf; %lf; %lf] <- RIGHT_BACK%s\n",
           RED,
           robot->_LegsPositionRobotCenter[LEFT_BACK].data[X],
           robot->_LegsPositionRobotCenter[LEFT_BACK].data[Y],
           robot->_LegsPositionRobotCenter[LEFT_BACK].data[Z],
           RESET,
           BLUE,
           robot->_LegsPositionRobotCenter[RIGHT_BACK].data[X],
           robot->_LegsPositionRobotCenter[RIGHT_BACK].data[Y],
           robot->_LegsPositionRobotCenter[RIGHT_BACK].data[Z],
           RESET);
    printf("=================================================================================================================\n\n\n\n");
}


// Funkcja do pobierania kanału PCA na podstawie pozycji i kąta
int getPCAChannel(LegType leg_type, int Q)
{
    switch (leg_type)
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
        printf("Invalid LegType or Q value.\n");
        global_error++;
        return -1; // Zwracamy -1 dla błędnych danych
    }
    return -1; // Zwracamy -1 dla błędnych danych
}


int getInitialAngle(int Q, RobotSide side)
{
    /*int Q_variable;
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

    // return 135 - (Q_variable * Q_side_multiplier);*/

    return 0;
}


// Funkcja zwracająca minimalny kąt dla danego przegubu (Q) i pozycji nogi
int getMinAngle(LegType leg_type, int Q)
{
    switch (Q)
    {
    case 1: // Dla Q1, różne wartości w zależności od pozycji nogi
        switch (leg_type)
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
        switch (leg_type)
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
        switch (leg_type)
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
int getMaxAngle(LegType leg_type, int Q)
{
    switch (Q)
    {
    case 1: // Dla Q1, różne wartości w zależności od pozycji nogi
        switch (leg_type)
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
        switch (leg_type)
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
        switch (leg_type)
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


void initLeg(Leg *leg, LegType leg_type)
{
    // Przypisz stronę i pozycję nogi
    leg->_leg_type = leg_type;
    if (leg_type == LEFT_FRONT || leg_type == LEFT_MIDDLE || leg_type == LEFT_BACK)
    {
        leg->_side = LEFT;
        leg->_pca = pca_left;
    }
    else if (leg_type == RIGHT_FRONT || leg_type == RIGHT_MIDDLE || leg_type == RIGHT_BACK)
    {
        leg->_side = RIGHT;
        leg->_pca = pca_right;
    }

    // Inicjalizacja serw Q1
    leg->_q1_servo._pca_channel = getPCAChannel(leg->_leg_type, 1);
    if (leg->_q1_servo._pca_channel > 15)
    {
        printf("Cos poszlo nie tak dla pca channel q1\n");
        global_error++;
    }

    leg->_q1_servo._min_angle = getMinAngle(leg->_leg_type, 1);
    leg->_q1_servo._max_angle = getMaxAngle(leg->_leg_type, 1);

    leg->_q1_servo._current_angle = getInitialAngle(1, leg->_side);
    leg->_q1_servo._target_angle = getInitialAngle(1, leg->_side);
    leg->_q1_servo._last_angle = getInitialAngle(1, leg->_side);

    // Inicjalizacja serw Q2
    leg->_q2_servo._pca_channel = getPCAChannel(leg->_leg_type, 2);
    if (leg->_q2_servo._pca_channel > 15)
    {
        printf("Cos poszlo nie tak dla pca channel q2\n");
        global_error++;
    }

    leg->_q2_servo._min_angle = getMinAngle(leg->_leg_type, 2);
    leg->_q2_servo._max_angle = getMaxAngle(leg->_leg_type, 2);

    leg->_q2_servo._current_angle = getInitialAngle(2, leg->_side);
    leg->_q2_servo._target_angle = getInitialAngle(2, leg->_side);
    leg->_q2_servo._last_angle = getInitialAngle(2, leg->_side);

    // Inicjalizacja serw Q3
    leg->_q3_servo._pca_channel = getPCAChannel(leg->_leg_type, 3);
    if (leg->_q3_servo._pca_channel > 15)
    {
        printf("Cos poszlo nie tak dla pca channel q3\n");
        global_error++;
    }

    leg->_q3_servo._min_angle = getMinAngle(leg->_leg_type, 3);
    leg->_q3_servo._max_angle = getMaxAngle(leg->_leg_type, 3);

    leg->_q3_servo._current_angle = getInitialAngle(3, leg->_side);
    leg->_q3_servo._target_angle = getInitialAngle(3, leg->_side);
    leg->_q3_servo._last_angle = getInitialAngle(3, leg->_side);

    for (int i = 0; i < 3; i++)
    {
        leg->_current_pos.data[i] = 0;
        leg->_target_pos.data[i] = 0;
        leg->_last_pos.data[i] = 0;
    }
}


int checkPosition(LegType leg_type, Vector3 pos)
{
    double x = pos.data[X];
    double y = pos.data[Y];
    double z = pos.data[Z];

    switch (leg_type)
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

    double Xp = leg->_target_pos.data[X];
    double Yp = leg->_target_pos.data[Y];
    double Zp = leg->_target_pos.data[Z];

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


    leg->_q1_servo._target_angle =  Q1 ;
    leg->_q2_servo._target_angle =  Q2 ;
    leg->_q3_servo._target_angle =  Q3 ;
    

    

    /*printf("Q1 = %.2f\n", leg->_q1_servo._target_angle );
    printf("Q2 = %.2f\n", leg->_q2_servo._target_angle );
    printf("Q3 = %.2f\n", leg->_q3_servo._target_angle );*/
}


void calculateForwardKinematics(Leg *leg) {
    // Pobranie kątów przegubów
    double Q1 = leg->_q1_servo._target_angle;
    double Q2 = leg->_q2_servo._target_angle;
    double Q3 = leg->_q3_servo._target_angle;

    double Xp, Yp, Zp, Xpz;

    // Obliczenie pośrednich wartości dla pozycji końcówki nogi
    Xpz = L1 + L2 * cos(Q2) + L3 * cos(Q2 + Q3);  // Współrzędna w płaszczyźnie XY
    Xp = Xpz * cos(Q1);  // Współrzędna X
    Yp = Xpz * sin(Q1);  // Współrzędna Y
    Zp = L2 * sin(Q2) + L3 * sin(Q2 + Q3);  // Współrzędna Z

    // Dopasowanie osi X dla strony robota
    if (leg->_side == LEFT) {
        Xp = -Xp;
    }

    // Przekształcenie lokalnych współrzędnych na globalne
    switch (leg->_leg_type) {
        case LEFT_FRONT:
            leg->_calculated_leg_pos.data[X] = Xp - d1;
            leg->_calculated_leg_pos.data[Y] = Yp + d3;
            break;
        case LEFT_MIDDLE:
            leg->_calculated_leg_pos.data[X] = Xp - d2;
            leg->_calculated_leg_pos.data[Y] = Yp;
            break;
        case LEFT_BACK:
            leg->_calculated_leg_pos.data[X] = Xp - d1;
            leg->_calculated_leg_pos.data[Y] = Yp - d3;
            break;
        case RIGHT_FRONT:
            leg->_calculated_leg_pos.data[X] = Xp + d1;
            leg->_calculated_leg_pos.data[Y] = Yp + d3;
            break;
        case RIGHT_MIDDLE:
            leg->_calculated_leg_pos.data[X] = Xp + d2;
            leg->_calculated_leg_pos.data[Y] = Yp;
            break;
        case RIGHT_BACK:
            leg->_calculated_leg_pos.data[X] = Xp + d1;
            leg->_calculated_leg_pos.data[Y] = Yp - d3;
            break;
        default:
            printf("Nieprawidłowa pozycja nogi\n");
            global_error++;
            return;
    }
    
    // Zapis współrzędnej Z, która jest taka sama dla wszystkich nóg
    leg->_calculated_leg_pos.data[Z] = Zp;
    
    /* Debug: Wypisz pozycję końcówki nogi
    printf("Pozycja końcówki nogi: X=%.2f, Y=%.2f, Z=%.2f\n", 
           leg->_target_pos.data[X], leg->_target_pos.data[Y], leg->_target_pos.data[Z]);
    */
}


void setTargetPos(Leg *leg, Vector3 pos)
{
    leg->_target_pos.data[X] = pos.data[X];
    leg->_target_pos.data[Y] = pos.data[Y];
    leg->_target_pos.data[Z] = pos.data[Z];
}


void initLegPositionRobotCenter(Robot *robot, LegType leg_type, double x, double y, double z)
{
    double xp, yp, zp;

    Vector3 calc_pos;
    Vector3 pos; 

    pos.data[X] = x;
    pos.data[Y] = y;
    pos.data[Z] = z;

    zp = z + z_0;

    if (checkPosition(leg_type, pos))
    {

        robot->_LegsPositionRobotCenter[leg_type].data[X] = x;
        robot->_LegsPositionRobotCenter[leg_type].data[Y] = y;
        robot->_LegsPositionRobotCenter[leg_type].data[Z] = z;

        switch (leg_type)
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

        calc_pos.data[X] = xp;
        calc_pos.data[Y] = yp;
        calc_pos.data[Z] = zp;

        setTargetPos(&robot->_legs[leg_type], calc_pos);
        calculateInvertedKinematics(&robot->_legs[leg_type]);
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
        SetServoAngle(leg->_side, 1, leg->_pca,  leg->_q1_servo._pca_channel, leg->_q1_servo._target_angle);
        leg->_q1_servo._current_angle = leg->_q1_servo._target_angle;
    }
    else
    {

        if (leg->_q1_servo._target_angle < leg->_q1_servo._min_angle)
        {
            SetServoAngle(leg->_side, 1, leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._min_angle);
            leg->_q1_servo._current_angle = leg->_q1_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q1!\n");
        }
        else if (leg->_q1_servo._target_angle > leg->_q1_servo._max_angle)
        {
            SetServoAngle(leg->_side, 1, leg->_pca, leg->_q1_servo._pca_channel, leg->_q1_servo._max_angle);
            leg->_q1_servo._current_angle = leg->_q1_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q1!\n");
        }
        pos_true = 0;
    }

    if ((leg->_q2_servo._target_angle > leg->_q2_servo._min_angle) && (leg->_q2_servo._target_angle < leg->_q2_servo._max_angle))
    {
        SetServoAngle(leg->_side, 2, leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._target_angle);
        leg->_q2_servo._current_angle = leg->_q2_servo._target_angle;
    }
    else
    {

        if (leg->_q2_servo._target_angle < leg->_q2_servo._min_angle)
        {
            SetServoAngle(leg->_side, 2, leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._min_angle);
            leg->_q2_servo._current_angle = leg->_q2_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q2!\n");
        }
        else if (leg->_q2_servo._target_angle > leg->_q2_servo._max_angle)
        {
            SetServoAngle(leg->_side, 2, leg->_pca, leg->_q2_servo._pca_channel, leg->_q2_servo._max_angle);
            leg->_q2_servo._current_angle = leg->_q2_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q2!\n");
        }
        pos_true = 0;
    }

    if ((leg->_q3_servo._target_angle > leg->_q3_servo._min_angle) && (leg->_q3_servo._target_angle < leg->_q3_servo._max_angle))
    {
        SetServoAngle(leg->_side, 3, leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._target_angle);
        leg->_q3_servo._current_angle = leg->_q3_servo._target_angle;
    }
    else
    {

        if (leg->_q3_servo._target_angle < leg->_q3_servo._min_angle)
        {
            SetServoAngle(leg->_side, 3, leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._min_angle);
            leg->_q3_servo._current_angle = leg->_q3_servo._min_angle;
            printf("Osiagnieto minimalna wartosc dla q3!\n");
        }
        else if (leg->_q3_servo._target_angle > leg->_q3_servo._max_angle)
        {
            SetServoAngle(leg->_side, 3, leg->_pca, leg->_q3_servo._pca_channel, leg->_q3_servo._max_angle);
            leg->_q3_servo._current_angle = leg->_q3_servo._max_angle;
            printf("Osiagnieto maksymalna wartosc dla q3!\n");
        }
        pos_true = 0;
    }

    if (pos_true == 1)
    {
        leg->_current_pos.data[X] = leg->_target_pos.data[X];
        leg->_current_pos.data[Y] = leg->_target_pos.data[Y];
        leg->_current_pos.data[Z] = leg->_target_pos.data[Z];
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
    printf("Leg Position: %d\n", leg._leg_type); // Zakładam, że LegType jest typu int lub enum
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
    printf("\nLast Position: [%.2f, %.2f, %.2f]\n", leg._last_pos.data[0], leg._last_pos.data[1], leg._last_pos.data[2]);
    printf("Current Position: [%.2f, %.2f, %.2f]\n", leg._current_pos.data[0], leg._current_pos.data[1], leg._current_pos.data[2]);
    printf("Target Position: [%.2f, %.2f, %.2f]\n", leg._target_pos.data[0], leg._target_pos.data[1], leg._target_pos.data[2]);
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
}


void evaluateLegPositionRobotCenter(Robot *robot, LegType leg_type, Vector3 pos)
{
    Vector3 calc_pos;
    double xp, yp, zp;
    double x, y, z;
    zp = z + z_0;

    x = pos.data[X];
    y = pos.data[Y];
    z = pos.data[Z];

    if (checkPosition(leg_type, pos))
    {

        robot->_LegsPositionRobotCenter[leg_type] = pos;


        switch (leg_type)
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

        calc_pos.data[X] = xp;
        calc_pos.data[Y] = yp;
        calc_pos.data[Z] = zp;

        // printf("pozycja %d dla: %.2f, %.2f, %.2f: \n" , pos, xp, yp, zp);
        setTargetPos(&robot->_legs[leg_type], calc_pos);
        calculateInvertedKinematics(&robot->_legs[leg_type]);
        moveToTargetPosition(&robot->_legs[leg_type]);
        //printLeg(robot->_legs[pos]);
    }
}


void calculatePosFromAngles(Leg * leg){

}


void calculatePosError(Leg * leg){

    for(int i = 0; i < 3; i++){
        leg->_pos_error.data[i] = leg->_target_pos.data[i] - leg->_calculated_leg_pos.data[i]; 
    }

}




void positionCorrection(Leg * leg){

    if(fabs(leg->_pos_error.data[X]) > X_POS_MAX_ERROR){

    }
    if(fabs(leg->_pos_error.data[Y]) > Y_POS_MAX_ERROR){
        
    }
    if(fabs(leg->_pos_error.data[Z]) > Z_POS_MAX_ERROR){
        
    }


}

#endif // ROBOT_H