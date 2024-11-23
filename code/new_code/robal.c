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

// Funkcja, która obsługuje sygnał SIGINT (Ctrl+C)
void handle_sigint(int sig)
{
    printf("\nOtrzymano sygnał SIGINT (Ctrl+C). Zamykanie programu...\n");
    SDL_Quit();
    exit(0); // Zakończ program
}

int main()
{

    /*======CONSTANTS_TIME_S======*/
    double delta_t = 0.001;
    double step_time = 1;
    double period = step_time * 2;
    /*======CONSTANTS======*/

    /*=====CONTROL=======*/
    double robot_velocity = 0;    // mm/s
    double arc_radius = 10000000; // mm
    /*=====CONTROL=======*/

    // Rejestracja handlera dla sygnału SIGINT (Ctrl+C)
    signal(SIGINT, handle_sigint);

    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1)
        return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1)
        return -1;

    Robot Robal;
    initRobot(&Robal);

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
        handle_buttons(&input_state);

        // Obsługuje osie
        handle_axes(&input_state);

        SDL_Delay(16); // Opóźnienie dla odciążenia CPU
    }

    // Zamknięcie joysticka i wyczyszczenie SDL
    SDL_JoystickClose(joystick);
    SDL_Quit();

    return 0;
}
