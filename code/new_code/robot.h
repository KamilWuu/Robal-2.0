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
#include "jakobian.h"
#include "velocity.h"

// Struktura dla jednej nogi
typedef struct Leg
{

    LegType _leg_type; // Pozycja: przód, środek, tył
    RobotSide _side;   // Lewa (LEFT) lub prawa (RIGHT) strona
    uint8_t _pca;      // które pca

    Servo _leg_servos[3]; // Serwa dla nogi

    LegFase _leg_fase;

    Vector3 _leg_position;        // Aktualna pozycja końcówki nogi [x, y, z]
    Vector3 _leg_linear_velocity; // predkosc liniowa nogi [x_d, y_d, z_d]

    Vector3 _leg_curve_t;         // trajektoria nogi od t
    Vector3 _leg_curve_t_delta_t; // trajektoria nogi od t + delta_t

    Vector3 _leg_q_delta;  // zmiana kątów w danej chwili , prędkosc rad/delta_t
    Vector3 _leg_actual_q; // aktualne kąty w przegubach w danej chwili w radianach

    Vector3 _leg_start_q; // kąty początkowe do korekcji błędów

    // Vector3 _calculated_leg_pos; //wyliczona pozycja z kątów do korekcji bledow
    // Vector3 _pos_error;          //blad pozycji

} Leg;

// Struktura dla robota, który ma 6 nóg
typedef struct Robot
{
    Leg _legs[6];                        // Tablica sześciu nóg (po trzy na każdą stronę)
    Vector3 _LegsPositionRobotCenter[6]; // pozycje koncowek nóg względem środka robota
    Vector3 _LegsStartPositions[6];      // pozycje początkowe wzgledem srodka robota do korekcji bledow
    StepFase _robotStepFase;             // faza w jakiej znajduje się robot
    int _robot_velocity;                 // predkosc srodka robota wzdluz osi y w ukladzie jego srodka mm/s

} Robot;

void setServoAngle(Leg *leg, Servo *servo, int q, double angle_rad) // zwraca false gdy osiągnieto maksymalny lub minimalny kąt
{

    double angle_deg;
    double servo_angle_deg_temp;

    angle_deg = angle_rad * RAD2DEG;

    RobotSide side = leg->_side;

    if (side == RIGHT)
    {
        switch (q)
        {
        case 1:
            servo_angle_deg_temp = 135 - angle_deg;

            break;
        case 2:
            servo_angle_deg_temp = 135 + angle_deg;
            break;
        case 3:
            servo_angle_deg_temp = 135 + angle_deg;
            break;

        default:
            break;
        }
    }

    if (side == LEFT)
    {
        switch (q)
        {
        case 1:

            servo_angle_deg_temp = 135 + angle_deg - 180;

            break;
        case 2:
            servo_angle_deg_temp = 135 - angle_deg;
            break;
        case 3:
            servo_angle_deg_temp = 135 - angle_deg;
            break;

        default:
            break;
        }
    }

    if (servo_angle_deg_temp < servo->_min_angle)
    {
        printf("error: na serwie %d, na nodze: %d, osiągnieto minimalny kąt rowny: %.2f ustawiam kąt: %.2f\n", q, leg->_leg_type, servo_angle_deg_temp, servo->_min_angle);
        servo->_servo_angle = servo->_min_angle;
        global_error++;
    }
    else if (servo_angle_deg_temp > servo->_max_angle)
    {
        servo->_servo_angle = servo->_max_angle;
        printf("error: na serwie %d, na nodze: %d, osiagnieto mkasymalny kąt, ustawiono kąt: %.2f\n", q, leg->_leg_type, servo_angle_deg_temp);
        global_error++;
    }
    else
    {
        servo->_servo_angle = servo_angle_deg_temp;
        // printf("na serwie %d, na nodze: %d, ustawiono kąt: %.2f\n", q, leg->_leg_type, servo_angle_deg_temp);
    }

    writeServo(servo, leg->_pca);
}

void printRobotStepFase(Robot *robot)
{
    switch (robot->_robotStepFase)
    {
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
    case PREPARE_FOR_WALK:
        printf("Stan robota: PREPARE_FOR_WALK\n");
        break;
    case WALK:
        printf("Stan robota: WALK\n");
        break;
    case STOP_WALK:
        printf("Stan robota: STOP_WALK\n");
        break;
    default:
        printf("Stan robota: NIEZNANY STAN\n");
        break;
    }
}

void printLegsPositions(Robot robot)
{

// Wyczyść terminal
#ifdef _WIN32
    system("cls"); // Windows
#else
    system("clear"); // Unix/Linux/MacOS
#endif

    // Definicje kolorów
    const char *RED = "\033[1;31m";
    const char *BLUE = "\033[1;34m";
    const char *RESET = "\033[0m";

    printf("=================================================================================================================\n");
    printf("%sLEFT_FRONT\t->\t[%.2f; %.2f; %.2f]%s\t\t\t%s[%.2f; %.2f; %.2f] <- RIGHT_FRONT%s\n",
           RED,
           robot._LegsPositionRobotCenter[LEFT_FRONT].data[X],
           robot._LegsPositionRobotCenter[LEFT_FRONT].data[Y],
           robot._LegsPositionRobotCenter[LEFT_FRONT].data[Z],
           RESET,
           BLUE,
           robot._LegsPositionRobotCenter[RIGHT_FRONT].data[X],
           robot._LegsPositionRobotCenter[RIGHT_FRONT].data[Y],
           robot._LegsPositionRobotCenter[RIGHT_FRONT].data[Z],
           RESET);

    printf("%sLEFT_MIDDLE\t->\t[%.2f; %.2f; %.2f]%s\t\t\t%s[%.2f; %.2f; %.2f] <- RIGHT_MIDDLE%s\n",
           BLUE,
           robot._LegsPositionRobotCenter[LEFT_MIDDLE].data[X],
           robot._LegsPositionRobotCenter[LEFT_MIDDLE].data[Y],
           robot._LegsPositionRobotCenter[LEFT_MIDDLE].data[Z],
           RESET,
           RED,
           robot._LegsPositionRobotCenter[RIGHT_MIDDLE].data[X],
           robot._LegsPositionRobotCenter[RIGHT_MIDDLE].data[Y],
           robot._LegsPositionRobotCenter[RIGHT_MIDDLE].data[Z],
           RESET);

    printf("%sLEFT_BACK\t->\t[%.2f; %.2f; %.2f]%s\t\t\t%s[%.2f; %.2f; %.2f] <- RIGHT_BACK%s\n",
           RED,
           robot._LegsPositionRobotCenter[LEFT_BACK].data[X],
           robot._LegsPositionRobotCenter[LEFT_BACK].data[Y],
           robot._LegsPositionRobotCenter[LEFT_BACK].data[Z],
           RESET,
           BLUE,
           robot._LegsPositionRobotCenter[RIGHT_BACK].data[X],
           robot._LegsPositionRobotCenter[RIGHT_BACK].data[Y],
           robot._LegsPositionRobotCenter[RIGHT_BACK].data[Z],
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

// Funkcja zwracająca minimalny kąt dla danego przegubu (Q) i pozycji nogi
int getServoMinAngle(LegType leg_type, int Q)
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
int getServoMaxAngle(LegType leg_type, int Q)
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

int getServoInitialAngle(Leg *leg, int Q)
{

    return 0;
}

void initLegServos(Leg *leg)
{

    for (int i = 0; i < 3; i++)
    {

        leg->_leg_servos[i]._pca_channel = getPCAChannel(leg->_leg_type, i + 1);

        if (leg->_leg_servos[i]._pca_channel > 15)
        {
            printf("Cos poszlo nie tak dla pca channel q%d\n", i + 1);
            global_error++;
        }

        leg->_leg_servos[i]._max_angle = getServoMaxAngle(leg->_leg_type, i + 1);
        leg->_leg_servos[i]._min_angle = getServoMinAngle(leg->_leg_type, i + 1);
        leg->_leg_servos[i]._servo_angle = getServoInitialAngle(leg, i + 1);
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

    initLegServos(leg);

    leg->_leg_fase = UNKNOWN_LEG_FASE;

    for (int i = 0; i < 3; i++)
    {
        leg->_leg_position.data[i] = 0;
        leg->_leg_linear_velocity.data[i] = 0;
        leg->_leg_curve_t.data[i] = 0;
        leg->_leg_curve_t_delta_t.data[i] = 0;
        leg->_leg_q_delta.data[i] = 0;
        leg->_leg_actual_q.data[i] = 0;
        leg->_leg_start_q.data[i] = 0;
    }
}

bool checkPosition(LegType leg_type, Vector3 pos)
{

    double x = pos.data[X];
    double y = pos.data[Y];
    double z = pos.data[Z];

    switch (leg_type)
    {
    case LEFT_FRONT:
        if (x < -L1 - d1 - L23)
        {
            return true;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT FRONT, x = %2.f, warunek: X < %2.f\n", x, -L1 - d1 - L23);
            global_error++;
            return false;
        }
        break;

    case LEFT_MIDDLE:
        if (x < -L1 - d2 - L23)
        {
            return true;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT MIDDLE, x = %2.f, warunek: X < %2.f\n", x, -L1 - d2 - L23);
            global_error++;
            return false;
        }
        break;

    case LEFT_BACK:
        if (x < -L1 - d1 - L23)
        {
            return true;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT BACK, x = %2.f, warunek: X < %2.f\n", x, -L1 - d1 - L23);
            global_error++;
            return false;
        }
        break;

    case RIGHT_FRONT:
        if (x > L1 + d1 + L23)
        {
            return true;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT FRONT, x = %2.f, warunek: X > %2.f\n", x, L1 + d1 + L23);
            global_error++;
            return false;
        }
        break;

    case RIGHT_MIDDLE:
        if (x > L1 + d2 + L23)
        {
            return true;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT FRONT, x = %2.f, warunek: X > %2.f\n", x, L1 + d2 + L23);
            global_error++;
            return false;
        }
        break;

    case RIGHT_BACK:
        if (x > L1 + d1 + L23)
        {
            return true;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT BACK, x = %2.f, warunek: X > %2.f\n", x, L1 + d1 + L23);
            global_error++;
            return false;
        }
        break;

    default:
        printf("Nieprawidlowa typ nogi\n");
        global_error++;
        break;
    }
    return true;
}

Vector3 calculateLegInvertedKinematics(RobotSide leg_side, Vector3 leg_position)
{

    double Q1, Q2, Q3;
    double Q3_1, Q3_2;

    double Xp = leg_position.data[0];
    double Yp = leg_position.data[1];
    double Zp = leg_position.data[2];

    double Xpz;

    Vector3 leg_angles;

    if (leg_side == LEFT)
    {
        Xp = -Xp;
    }

    Xpz = sqrt((Xp * Xp) + (Yp * Yp));
    Q1 = atan2(Yp, Xp);
    Q3_1 = MY_PI - acos(((L2 * L2) + (L3 * L3) - (Zp * Zp) - ((Xpz - L1) * (Xpz - L1))) / (2 * L2 * L3));
    Q3_2 = acos(((Zp * Zp) + ((Xpz - L1) * (Xpz - L1)) - (L2 * L2) - (L3 * L3)) / (2 * L2 * L3));
    Q3 = -Q3_1; // lub -Q3_2
    Q2 = atan2(Zp, (Xpz - L1)) + atan2(L3 * sin(-Q3), (L2 + L3 * cos(-Q3)));

    if (leg_side == LEFT)
    {
        Q1 += 180 * DEG2RAD;
    }

    leg_angles.data[0] = Q1;
    leg_angles.data[1] = Q2;
    leg_angles.data[2] = Q3;

    return leg_angles;
}

Vector3 calculateLegForwardKinematics(RobotSide leg_side, Vector3 leg_angles)
{
    // Pobranie kątów przegubów
    double Q1 = leg_angles.data[0];
    double Q2 = leg_angles.data[1];
    double Q3 = leg_angles.data[2];

    double Xp, Yp, Zp, Xpz;
    Vector3 leg_pos;
    // Obliczenie pośrednich wartości dla pozycji końcówki nogi
    Xpz = L1 + L2 * cos(Q2) + L3 * cos(Q2 + Q3); // Współrzędna w płaszczyźnie XY
    Xp = Xpz * cos(Q1);                          // Współrzędna X
    Yp = Xpz * sin(Q1);                          // Współrzędna Y
    Zp = L2 * sin(Q2) + L3 * sin(Q2 + Q3);       // Współrzędna Z

    // Dopasowanie osi X dla strony robota
    if (leg_side == LEFT)
    {
        Xp = -Xp;
    }

    leg_pos.data[0] = Xp;
    leg_pos.data[1] = Yp;
    leg_pos.data[2] = Zp;

    return leg_pos;

    // // Przekształcenie lokalnych współrzędnych na globalne
    // switch (leg->_leg_type)
    // {
    // case LEFT_FRONT:
    //     leg->_calculated_leg_pos.data[X] = Xp - d1;
    //     leg->_calculated_leg_pos.data[Y] = Yp + d3;
    //     break;
    // case LEFT_MIDDLE:
    //     leg->_calculated_leg_pos.data[X] = Xp - d2;
    //     leg->_calculated_leg_pos.data[Y] = Yp;
    //     break;
    // case LEFT_BACK:
    //     leg->_calculated_leg_pos.data[X] = Xp - d1;
    //     leg->_calculated_leg_pos.data[Y] = Yp - d3;
    //     break;
    // case RIGHT_FRONT:
    //     leg->_calculated_leg_pos.data[X] = Xp + d1;
    //     leg->_calculated_leg_pos.data[Y] = Yp + d3;
    //     break;
    // case RIGHT_MIDDLE:
    //     leg->_calculated_leg_pos.data[X] = Xp + d2;
    //     leg->_calculated_leg_pos.data[Y] = Yp;
    //     break;
    // case RIGHT_BACK:
    //     leg->_calculated_leg_pos.data[X] = Xp + d1;
    //     leg->_calculated_leg_pos.data[Y] = Yp - d3;
    //     break;
    // default:
    //     printf("Nieprawidłowa pozycja nogi\n");
    //     global_error++;
    //     return;
    // }

    // // Zapis współrzędnej Z, która jest taka sama dla wszystkich nóg
    // leg->_calculated_leg_pos.data[Z] = Zp;

    /* Debug: Wypisz pozycję końcówki nogi
    printf("Pozycja końcówki nogi: X=%.2f, Y=%.2f, Z=%.2f\n",
           leg->_target_pos.data[X], leg->_target_pos.data[Y], leg->_target_pos.data[Z]);
    */
}

void setLegPosition(Leg *leg, Vector3 pos)
{
    leg->_leg_position = pos;
    leg->_leg_actual_q = calculateLegInvertedKinematics(leg->_side, pos);
    for (int i = 0; i < 3; i++)
    {
        setServoAngle(leg, &leg->_leg_servos[i], i + 1, leg->_leg_actual_q.data[i]);
    }
}

void setLegActualAngles(Leg *leg, Vector3 angles)
{
    leg->_leg_actual_q = angles;
    leg->_leg_position = calculateLegForwardKinematics(leg->_side, angles);
    for (int i = 0; i < 3; i++)
    {
        setServoAngle(leg, &leg->_leg_servos[i], i + 1, leg->_leg_actual_q.data[i]);
    }
}

void initRobot(Robot *robot)
{
    printf("#ROBAL: Cześć, to ja Robal!\n\n");
    // Inicjalizuj nogi robota
    initLeg(&robot->_legs[0], LEFT_FRONT);
    initLeg(&robot->_legs[1], LEFT_MIDDLE);
    initLeg(&robot->_legs[2], LEFT_BACK);
    initLeg(&robot->_legs[3], RIGHT_FRONT);
    initLeg(&robot->_legs[4], RIGHT_MIDDLE);
    initLeg(&robot->_legs[5], RIGHT_BACK);

    for (int i = 0; i < 6; i++)
    {
        robot->_LegsStartPositions[i] = robot->_LegsPositionRobotCenter[i];
    }

    robot->_robotStepFase = UNKNOWN;
    robot->_robot_velocity = 0;
}

void printServo(Servo servo)
{
    printf("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
    printf("PCA Channel: %d\n", servo._pca_channel);
    printf("Min Angle: %.2f\n", servo._min_angle);
    printf("Max Angle: %.2f\n", servo._max_angle);
    printf("Current Angle: %.2f\n", servo._servo_angle);
    printf("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
}

void printLeg(Leg leg, double t, double loop_time)
{
    // printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

    // Wyświetlanie informacji o pozycji nogi i stronie robota
    //   printf("Leg Type (Position): %d\n", leg._leg_type); // Zakładam, że LegType jest typu int lub enum
    // printf("Leg  Fase: %d\n", leg._leg_fase);
    // //  printf("Side: %d\n", leg._side); // Zakładam, że RobotSide jest typu int lub enum
    // //  printf("PCA: %d\n", leg._pca);

    // // Wyświetlanie informacji o serwach
    // // for (int i = 0; i < 3; i++)
    // // {
    // //     printf("\nServo %d:\n", i + 1);
    // //     printServo(leg._leg_servos[i]); // Wyświetlanie informacji o każdym serwie
    // // }

    // // Wyświetlanie pozycji końcówki nogi
    // printf("\nLeg Position (Tip): [%.2f, %.2f, %.2f]\n",
    //        leg._leg_position.data[0],
    //        leg._leg_position.data[1],
    //        leg._leg_position.data[2]);

    // // Wyświetlanie prędkości liniowej nogi
    // // printf("Linear Velocity: [%.2f, %.2f, %.2f]\n",
    // //        leg._leg_linear_velocity.data[0],
    // //        leg._leg_linear_velocity.data[1],
    // //        leg._leg_linear_velocity.data[2]);

    // // Wyświetlanie trajektorii
    // printf("Leg Curve at t: [%.2f, %.2f, %.2f]\n",
    //        leg._leg_curve_t.data[0],
    //        leg._leg_curve_t.data[1],
    //        leg._leg_curve_t.data[2]);

    // printf("Leg Curve at t + delta_t: [%.2f, %.2f, %.2f]\n",
    //        leg._leg_curve_t_delta_t.data[0],
    //        leg._leg_curve_t_delta_t.data[1],
    //        leg._leg_curve_t_delta_t.data[2]);

    // // // Wyświetlanie kątów w przegubach
    // printf("\nJoint Angles (Delta): [%.2f, %.2f, %.2f]\n",
    //        leg._leg_q_delta.data[0],
    //        leg._leg_q_delta.data[1],
    //        leg._leg_q_delta.data[2]);

    // // printf("Joint Angles (Actual): [%.2f, %.2f, %.2f]\n",
    // //        leg._leg_actual_q.data[0],
    // //        leg._leg_actual_q.data[1],
    // //        leg._leg_actual_q.data[2]);

    // printf("Joint Angles (Actual): [%.2f, %.2f, %.2f]\n",
    //        leg._leg_actual_q.data[0] * RAD2DEG,
    //        leg._leg_actual_q.data[1] * RAD2DEG,
    //        leg._leg_actual_q.data[2] * RAD2DEG);

    // Wyświetlanie kątów początkowych
    // printf("Initial Joint Angles: [%.2f, %.2f, %.2f]\n",
    //        leg._leg_start_q.data[0],
    //        leg._leg_start_q.data[1],
    //        leg._leg_start_q.data[2]);
    printf("time: %.2f, step_t: %.2f faza: %d, pos Y = %.2f, pos Z = %.2f\n", loop_time, t, leg._leg_fase, leg._leg_position.data[1], leg._leg_position.data[2]);
    // printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
}

void evaluateLegPositionRobotCenter(Robot *robot, LegType leg_type, double x, double y, double z)
{
    Vector3 calc_pos, pos;
    double xp, yp, zp;

    zp = z + z_0;

    pos.data[0] = x;
    pos.data[1] = y;
    pos.data[2] = z;

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

        setLegPosition(&robot->_legs[leg_type], calc_pos);
    }
}

void calculateLegsVelocities(Robot *robot, int arc_radius, int robot_velocity)
{

    for (int i = 0; i < 6; i++)
    {
        robot->_legs[i]._leg_linear_velocity = calculateLegVelocity(robot->_legs[i]._leg_position, robot_velocity, arc_radius);
        // robot->_legs[i]._leg_linear_velocity.data[Y] = 40;
    }
}

void calculateLegsCurvesT(Robot *robot, double t, double delta_time, double period)
{

    for (int i = 0; i < 6; i++)
    {

        if ((robot->_legs[i]._leg_fase == BACK_POS) || (robot->_legs[i]._leg_fase == IN_PROTRACTION))
        {
            // calculate protraction curve

            robot->_legs[i]._leg_curve_t = makeProtractionCurve(robot->_legs[i]._leg_linear_velocity, t, period, robot->_robot_velocity);
            robot->_legs[i]._leg_curve_t_delta_t = makeProtractionCurve(robot->_legs[i]._leg_linear_velocity, t + delta_time, period, robot->_robot_velocity);
            robot->_legs[i]._leg_fase = IN_PROTRACTION;
        }
        else if ((robot->_legs[i]._leg_fase == FRONT_POS) || (robot->_legs[i]._leg_fase == IN_RETRACTION))
        {
            // calculate retraction curve

            robot->_legs[i]._leg_curve_t = vectorMultiplyByConst(robot->_legs[i]._leg_linear_velocity, t);
            robot->_legs[i]._leg_curve_t_delta_t = vectorMultiplyByConst(robot->_legs[i]._leg_linear_velocity, t + delta_time);
            robot->_legs[i]._leg_fase = IN_RETRACTION;
        }
    }
}

void actualizeLegs(Robot *robot, double t, double delta_time, double period, double arc)
{
    calculateLegsVelocities(robot, arc, robot->_robot_velocity);
    calculateLegsCurvesT(robot, t, delta_time, period);

    for (int i = 0; i < 6; i++)
    {
        robot->_legs[i]._leg_q_delta = calculateDeltaQ(robot->_legs[i]._leg_type, robot->_legs[i]._side, robot->_legs[i]._leg_actual_q, t, delta_time, robot->_legs[i]._leg_curve_t, robot->_legs[i]._leg_curve_t_delta_t);

        setLegActualAngles(&robot->_legs[i], vectorAdd(robot->_legs[i]._leg_actual_q, robot->_legs[i]._leg_q_delta));

        robot->_LegsPositionRobotCenter[i] = getRobotCenterPositionFromAngles(robot->_legs[i]._leg_type, robot->_legs[i]._side, robot->_legs[i]._leg_actual_q);
    }
}

// void calculatePosError(Leg *leg)
// {

//     for (int i = 0; i < 3; i++)
//     {
//         leg->_pos_error.data[i] = leg->_target_pos.data[i] - leg->_calculated_leg_pos.data[i];
//     }
// }

// void positionCorrection(Leg *leg)
// {

//     if (fabs(leg->_pos_error.data[X]) > X_POS_MAX_ERROR)
//     {
//     }
//     if (fabs(leg->_pos_error.data[Y]) > Y_POS_MAX_ERROR)
//     {
//     }
//     if (fabs(leg->_pos_error.data[Z]) > Z_POS_MAX_ERROR)
//     {
//     }
// }

#endif // ROBOT_H