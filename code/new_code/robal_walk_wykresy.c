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
// Zmienna do zapisu wyników
FILE *log_file;

void log_data(FILE *file, double total_time, double time, const Robot *robot, int arc_radius, int leg) {
    int i = leg;
        fprintf(file,
                "%d,%.3f,%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f;\n",
                i, total_time, time,
                robot->_LegsPositionRobotCenter[i].data[0], robot->_LegsPositionRobotCenter[i].data[1], robot->_LegsPositionRobotCenter[i].data[2],
                robot->_legs[i]._leg_linear_velocity.data[0], robot->_legs[i]._leg_linear_velocity.data[1], robot->_legs[i]._leg_linear_velocity.data[2],
                robot->_legs[i]._leg_actual_q.data[0]*RAD2DEG, robot->_legs[i]._leg_actual_q.data[1]*RAD2DEG, robot->_legs[i]._leg_actual_q.data[2]*RAD2DEG, robot->_legs[i]._leg_q_delta.data[0]*RAD2DEG, robot->_legs[i]._leg_q_delta.data[1]*RAD2DEG, robot->_legs[i]._leg_q_delta.data[2]*RAD2DEG);
    
    
}


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
        arc_ = -map(fabs(x_axis_v), MAX_ARC, MAX_AXIS_VALUE, MAX_ARC, MIN_ARC);

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
    double arc_temp;
    for (int i = 0; i < MAX_AXES; i++)
    {
        robot->_robot_velocity = map(input_state->axes[CONTROLLER_RIGHT_AXIS_Y], -MAX_AXIS_VALUE, MAX_AXIS_VALUE, MAX_SPEED, -MAX_SPEED);
        arc_temp = calculateArc(input_state->axes[CONTROLLER_LEFT_AXIS_X]);
        *arc = arc_temp;
        if (((arc_temp < 250) && (arc_temp >= MIN_ARC)) || ((arc_temp > -250) && (arc_temp <= -MIN_ARC)))
        {
            if (robot->_robot_velocity > 0)
            {
                robot->_robot_velocity = 20;
            }
            else if (robot->_robot_velocity < 0)
            {
                robot->_robot_velocity = -20;
            }
        }
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

int main(int argc, char *argv[]) {



    // Rejestracja handlera dla sygnału SIGINT
    signal(SIGINT, handle_sigint);

    if (argc != 5) {
        fprintf(stderr, "Błąd: Należy podać dokładnie cztery wartości: test_v [mm/s], test_arc [mm], indeks nogi i czas symulacji [s].\n");
        return 1;
    }


    // Przypisanie wartości z argumentów do zmiennych
    double test_v = atof(argv[1]);  // Pierwszy argument - test_v
    double test_arc = atof(argv[2]); // Drugi argument - test_arc
    int export_leg = atof(argv[3]); // trzeci argument - indeks nogi
    double simulation_time = atof(argv[4]); // trzeci argument - indeks nogi
    if(export_leg > 5){
        fprintf(stderr, "Błąd: Zly indeks nogi, nalezy podac z przedzielu <0,5>\n");
        return 1;
    }
    if(export_leg < 0){
        fprintf(stderr, "Błąd: Zly indeks nogi, nalezy podac z przedzielu <0,5>\n");
        return 1;
    }

    // Tworzymy nazwę folderu, w którym zapisujemy pliki
    const char *folder_name = "dane_csv";  // Nazwa folderu
    char filename[200];  // Bufor na pełną ścieżkę i nazwę pliku

    // Formatujemy nazwę pliku z folderem i zmiennymi
    snprintf(filename, sizeof(filename), "%s/R_%.0f_V_%.0f_leg_%d.csv", folder_name, test_arc, test_v, export_leg);

    // Tworzenie folderu, jeśli nie istnieje (opcjonalnie)
    if (system("mkdir -p dane_csv") == -1) {
        perror("Błąd przy tworzeniu folderu.");
        return 1;
    }

    // Otwieramy plik do zapisu
    FILE *log_file = fopen(filename, "w");
    if (!log_file) {
        perror("Nie można otworzyć pliku logowania.");
        return 1;
    }

    // Nagłówki w pliku logowania
    fprintf(log_file, "%% LegID, total_time, time, PosX, PosY, PosZ, VelX, VelY, VelZ, Q1, Q2, Q3, Delta_Q1, Delta_Q2, Delta_Q3\n");

    // Inicjalizacja robota
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    Robot hexapod;
    initRobot(&hexapod);
    delay(500);

    setWalkingPosition(&hexapod, 500);
    delay(100);

    double Z_speed = 100;
            double Z_step_time = 0.01;
            moveLegsZ(&hexapod, z_const_stand_up, Z_speed, Z_step_time);
            hexapod._robotStepFase = STAND_UP;
            printf("Stan robota: STAND_UP\n");

    SDL_Joystick *joystick = initialize_joystick();
    if (!joystick) {
        printf("Nie można otworzyć joysticka.\n");
        return 1;
    }

    // Główne zmienne

    double delta_time = 0.01;
    double step_time = 0.6;
    double period = step_time * 2;
    int arc_radius = 10000000;
    double total_time = 0;

    ControllerStates input_state = {0};
    int running = 1;
    bool minus_velocity = false;
    bool plus_velocity = false;


    // Główna pętla
    while (running) {
        printf("=========== ROZPOCZĘCIE SYMULACJI RUCHU ===========\n");

        while (1) {
            unsigned long interval_ms = (unsigned long)(delta_time * 1000);
            unsigned long start_time = millis();
            unsigned long previous_time = start_time;

            double move_t = 0;
            double time = 0;
            bool was_period_middle = false;

            for (int i = 0; i < 6; i++) {
                hexapod._legs[i]._leg_start_q = hexapod._legs[i]._leg_actual_q;
                hexapod._LegsPositionRobotCenter[i] = getRobotCenterPositionFromAngles(
                    hexapod._legs[i]._leg_type, hexapod._legs[i]._side, hexapod._legs[i]._leg_start_q);
            }

            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    running = 0;
                }
            }

            //read_controller_input(joystick, &input_state);
            //handle_buttons(&input_state, &hexapod);
            //handle_axes(&input_state, &hexapod, &arc_radius);
            hexapod._robot_velocity = test_v;
            arc_radius = test_arc;

            fprintf(log_file, "%% Robot velocity: %.3f, Arc radius: %d, Leg fase: %d\n", hexapod._robot_velocity, arc_radius, hexapod._legs[export_leg]._leg_fase);
            do {
                unsigned long current_time = millis();
                if (current_time - previous_time >= interval_ms) {
                    previous_time = current_time;

                    if (move_t == 0 || move_t >= step_time) {
                        for (int i = 0; i < 6; i++) {
                            if (hexapod._legs[i]._leg_fase == IN_PROTRACTION) {
                                hexapod._legs[i]._leg_fase = FRONT_POS;
                            } else if (hexapod._legs[i]._leg_fase == IN_RETRACTION) {
                                hexapod._legs[i]._leg_fase = BACK_POS;
                            }
                        }
                        move_t = 0;
                    }

                     if ((hexapod._robot_velocity > 0) && !plus_velocity)
                    {
                        for (int i = 0; i < 6; i++)
                        {

                            if (hexapod._legs[i]._leg_fase == BACK_POS)
                            {
                                hexapod._legs[i]._leg_fase = FRONT_POS;
                            }
                            else if (hexapod._legs[i]._leg_fase == FRONT_POS)
                            {
                                hexapod._legs[i]._leg_fase = BACK_POS;
                            }
                        }
                        minus_velocity = false;
                        plus_velocity = true;
                    }
                    else if ((hexapod._robot_velocity < 0) && !minus_velocity)
                    {
                        for (int i = 0; i < 6; i++)
                        {

                            if (hexapod._legs[i]._leg_fase == BACK_POS)
                            {
                                hexapod._legs[i]._leg_fase = FRONT_POS;
                            }
                            else if (hexapod._legs[i]._leg_fase == FRONT_POS)
                            {
                                hexapod._legs[i]._leg_fase = BACK_POS;
                            }
                        }
                        minus_velocity = true;
                        plus_velocity = false;
                    }


                    actualizeLegs(&hexapod, move_t, delta_time, period, arc_radius);
                    system("clear");
                    printLegsPositions(hexapod);
                    printLegsVelocities(hexapod);
                    printf("robot center velocity: %.2f\n", hexapod._robot_velocity);                   

                    
                    log_data(log_file, total_time,time, &hexapod, arc_radius, export_leg);
                    
                    move_t += delta_time;
                    time += delta_time;
                    total_time += delta_time;
                    if(total_time > simulation_time){
                        fprintf(stderr, "Koniec symulacji\n");
                        printf("bylo %d errorow\n", global_error);
                        return 1;
                    }
                }
            } while (millis() - start_time < (unsigned long)(period * 1000));
            // tutaj korekcja bledow, ustawienie katow i pozycji początkowych
            if (hexapod._robot_velocity != 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    hexapod._legs[i]._leg_actual_q = hexapod._legs[i]._leg_start_q;
                    hexapod._LegsPositionRobotCenter[i] = getRobotCenterPositionFromAngles(hexapod._legs[i]._leg_type, hexapod._legs[i]._side, hexapod._legs[i]._leg_start_q);
                }
            }
        }
    }

    // Zamknięcie joysticka i pliku
    SDL_JoystickClose(joystick);
    fclose(log_file);
    SDL_Quit();

    return 0;
}
