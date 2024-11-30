#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>

#include "kinematics.h"
#include "defines.h"
#include "robot.h"
#include "jakobian.h"
#include "data_structures.h"
#include "velocity.h"
#include "controller.h"
#include "moves.h"
#include <stdio.h>

// Funkcja mapująca
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh)
{
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

int calculateArc(int x_axis_v)
{

    double arc_;

    if (x_axis_v == AXIS_ZERO)
    {
        return ARC_STRAIGHT;
    }
    else
    {
        arc_ = map(fabs(x_axis_v), MAX_ARC, MAX_AXIS_VALUE, MAX_ARC, MIN_ARC);

        if (x_axis_v < 0)
        {
            arc_ = arc_ * (-1);
        }
    }

    return arc_;
}

// Funkcja do obsługi stanu przycisków
void handle_buttons(const ControllerStates *input_state, Robot *robot)
{
    for (int i = 0; i < MAX_BUTTONS; i++)
    {

        if ((input_state->buttons[CONTROLLER_SETUP]) & ((robot->_robotStepFase == SIT_DOWN) || (robot->_robotStepFase == WALKING_POSITION)))
        {
            double Z_speed = 100;
            double Z_step_time = 0.01;
            moveLegsZ(robot, z_const_stand_up, Z_speed, Z_step_time);
            robot->_robotStepFase = STAND_UP;
            printf("Stan robota: STAND_UP\n");
        }
        if ((input_state->buttons[CONTROLLER_BACK]) & ((robot->_robotStepFase == STAND_UP) || (robot->_robotStepFase == STOP_WALK) || (robot->_robotStepFase == PREPARE_FOR_WALK)))
        {
            double Z_speed = 100;
            double Z_step_time = 0.01;
            moveLegsZ(robot, -z_const_stand_up, Z_speed, Z_step_time);
            robot->_robotStepFase = SIT_DOWN;
            printf("Stan robota: SIT_DOWN\n");
        }
        if ((input_state->buttons[CONTROLLER_START]) & (robot->_robotStepFase == STAND_UP))
        {
            // przygotowanie do ruchu

            robot->_robotStepFase = PREPARE_FOR_WALK;
            printf("Stan robota: PREPARE_FOR_WALK\n");
        }
        if ((input_state->buttons[CONTROLLER_Y]) & ((robot->_robotStepFase == PREPARE_FOR_WALK) || (robot->_robotStepFase == STOP_WALK)))
        {
            // start ruchu
            robot->_robotStepFase = WALK;
            printf("Stan robota: WALK\n");
        }
        if ((input_state->buttons[CONTROLLER_A]) & (robot->_robotStepFase == WALK))
        {
            // stop ruchu
            robot->_robotStepFase = STOP_WALK;
            printf("Stan robota: STOP_WALK\n");
        }
    }
}

// Funkcja do obsługi osi
void handle_axes(const ControllerStates *input_state, Robot *robot, int *arc)
{
    for (int i = 0; i < MAX_AXES; i++)
    {
        robot->_robot_velocity = map(input_state->axes[CONTROLLER_RIGHT_AXIS_Y], -MAX_AXIS_VALUE, MAX_AXIS_VALUE, MAX_SPEED, -MAX_SPEED);
        *arc = calculateArc(input_state->axes[CONTROLLER_LEFT_AXIS_X]);

        // printf("Oś %d: %d\n", i, input_state->axes[i]);
        // printf("Predkość: %d\t, promień łuku: %d\n", robot->_robot_velocity, *arc);
    }
}

// Funkcja, która obsługuje sygnał SIGINT (Ctrl+C)
void handle_sigint(int sig)
{
    printf("\nOtrzymano sygnał SIGINT (Ctrl+C). Zamykanie programu...\n");
    printf("bylo %d errorow\n", global_error);
    SDL_Quit();
    exit(0); // Zakończ program
}

int main()
{

    /*======CONSTANTS_TIME_S======*/
    double delta_time = 0.01;
    double step_time = 1;
    double period = step_time * 2;
    /*======CONSTANTS======*/

    /*=====CONTROL=======*/
    // int robot_velocity = 0;    // mm/s
    int arc_radius = 10000000; // mm

    double move_t;
    double time;
    /*=====CONTROL=======*/

    // Rejestracja handlera dla sygnału SIGINT (Ctrl+C)
    signal(SIGINT, handle_sigint);

    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1)
        return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1)
        return -1;

    Robot hexapod;
    initRobot(&hexapod);
    delay(500);

    setWalkingPosition(&hexapod, 500);
    delay(100);
    //setReadyToWalk(&hexapod, 500);
    printLegsPositions(hexapod);
    printLegsAngles(hexapod);
    printServosAngles(hexapod);

    // for(int i = 0; i< 6; i++){
    // hexapod._LegsPositionRobotCenter[i] = getRobotCenterPositionFromAngles(hexapod._legs[i]._leg_type, hexapod._legs[i]._side, hexapod._legs[i]._leg_start_q);
    // }
    printf("po\n");
    printLegsPositions(hexapod);
    printLegsAngles(hexapod);
    printServosAngles(hexapod);


    printTwoVectors("lewa srodkowa kąty", vectorMultiplyByConst(hexapod._legs[LEFT_MIDDLE]._leg_actual_q, RAD2DEG), "pozycja", hexapod._LegsPositionRobotCenter[LEFT_MIDDLE]);
    SDL_Joystick *joystick = initialize_joystick();
    if (!joystick)
    {
        return 1; // Jeśli joystick się nie otworzył, kończymy program
    }

    // Struktura przechowująca stan przycisków i osi
    ControllerStates input_state = {0}; // Początkowy stan bez przycisków i osi



    // Główna pętla
    int running = 1;
    while (running)
    {

        printf("=========== ROZPOCZĘCIE SYMULACJI RUCHU ===========\n");

        int iteration_count = 0;
        for (;;)
        {

            // Pętla czasowa - symulacja ruchu
            unsigned long interval_ms = (unsigned long)(delta_time * 1000);
            unsigned long start_time = millis();
            unsigned long previous_time = start_time;
            unsigned long target_duration_ms = (unsigned long)(period * 1000);

            move_t = 0;
            time = 0;
            bool was_period_middle = false;
            int x = 1;

            for (int i = 0; i < 6; i++)
            {
                hexapod._legs[i]._leg_start_q = hexapod._legs[i]._leg_actual_q;
                hexapod._LegsStartPositions[i] = getRobotCenterPositionFromAngles(hexapod._legs[i]._leg_type, hexapod._legs[i]._side, hexapod._legs[i]._leg_start_q);
                // printTwoVectors("katy", vectorMultiplyByConst(hexapod._legs[i]._leg_start_q, RAD2DEG), "pozycja", hexapod._LegsStartPositions[i]);
            }
            printf("\n\n");

            SDL_Event event;
            while (SDL_PollEvent(&event))
            {
                if (event.type == SDL_QUIT)
                {
                    running = 0;
                }
            }

            // Odczyt wszystkich danych z kontrolera
            read_controller_input(joystick, &input_state);

            // Obsługuje przyciski
            handle_buttons(&input_state, &hexapod);

            // Obsługuje osie
            handle_axes(&input_state, &hexapod, &arc_radius);

            do
            {

                unsigned long current_time = millis();

                if (current_time - previous_time >= interval_ms)
                {

                    previous_time = current_time;
                    iteration_count++;

                    /**
                     * Petla wykonująca się co delta_time
                     */

                    if ((move_t == 0) || (move_t >= step_time))
                    {

                        // printf("faza nogi prawej srodkowej:%d\t\t\tfaza nogi prawej przedniej:%d \n", hexapod._legs[RIGHT_MIDDLE]._leg_fase, hexapod._legs[RIGHT_FRONT]._leg_fase);
                        for (int it = 0; it < 6; it++)
                        {

                            if (hexapod._legs[it]._leg_fase == IN_PROTRACTION)
                            {
                                hexapod._legs[it]._leg_fase = FRONT_POS;
                            }
                            else if (hexapod._legs[it]._leg_fase == IN_RETRACTION)
                            {
                                hexapod._legs[it]._leg_fase = BACK_POS;
                            }
                        }

                        move_t = 0;
                    }

                    // else if (t < step_time)
                    // {
                    //     if (x)
                    //     {
                    //         //printf("FAZA PROTRAKCJI: \n");
                    //         x = 0;
                    //     }
                    // }
                    // system("vcgencmd measure_temp");

                    // if (hexapod._robot_velocity != 0)
                    // {
                    actualizeLegs(&hexapod, move_t, delta_time, period, arc_radius);

                    // printTwoVectors("prawa srodkowa kąty", vectorMultiplyByConst(hexapod._legs[RIGHT_MIDDLE]._leg_actual_q, RAD2DEG), "pozycja", hexapod._LegsPositionRobotCenter[RIGHT_MIDDLE]);
                    // }
                    printLegsPositions(hexapod);
                    printServosAngles(hexapod);
                    printLegsAngles(hexapod);
                    //  Wyczyść terminal

                    // system("clear"); // Unix/Linux/MacOS

                    // printLeg(hexapod._legs[LEFT_MIDDLE], move_t, time);
                    //   printLeg(hexapod._legs[RIGHT_MIDDLE]);
                    //     printTwoVectors("actual_angles_deg", vectorMultiplyByConst(actual_q, RAD2DEG), "actual_pos", actual_pos);
                    move_t += delta_time;
                    time += delta_time;
                }
            } while (millis() - start_time < target_duration_ms);
            // tutaj korekcja bledow, ustawienie katow i pozycji początkowych
            if (hexapod._robot_velocity != 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    hexapod._legs[i]._leg_actual_q = hexapod._legs[i]._leg_start_q;
                    hexapod._LegsPositionRobotCenter[i] = getRobotCenterPositionFromAngles(hexapod._legs[i]._leg_type, hexapod._legs[i]._side, hexapod._legs[i]._leg_start_q);
                }
            }

            // printTwoVectors("ustawiam pozycjcje pocz angles", vectorMultiplyByConst(actual_q, RAD2DEG), "ustawiam pozycje pocz", actual_pos);
        }

        // SDL_Delay(16); // Opóźnienie dla odciążenia CPU
    }

    // Zamknięcie joysticka i wyczyszczenie SDL
    SDL_JoystickClose(joystick);
    SDL_Quit();

    return 0;
}