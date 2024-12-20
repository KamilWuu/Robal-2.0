#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <SDL2/SDL.h>
#include <stdio.h>

#define MAX_AXIS_VALUE 32767
#define AXIS_ZERO 128
#define MAX_SPEED 120
#define MIN_ARC 50
#define MAX_ARC 1500
#define ARC_STRAIGHT 1000000000

#define CONTROLLER_A 0
#define CONTROLLER_B 1
#define CONTROLLER_Y 4
#define CONTROLLER_X 3
// #define CONTROLLER_UP 11
// #define CONTROLLER_DOWN 12
// #define CONTROLLER_LEFT 13
// #define CONTROLLER_RIGHT 14
#define CONTROLLER_START 11
#define CONTROLLER_BACK 10
#define CONTROLLER_SETUP 13 // kulka
#define CONTROLLER_R_AXIS_BUTTON 14
#define CONTROLLER_LB 6
#define CONTROLLER_RB 7
#define CONTROLLER_LT 8
#define CONTROLLER_RT 9

#define CONTROLLER_LEFT_AXIS_X 0
#define CONTROLLER_LEFT_AXIS_Y 1
#define CONTROLLER_RIGHT_AXIS_X 2
#define CONTROLLER_RIGHT_AXIS_Y 3
#define CONTROLLER_LEFT_TRIGGER_AXIS 5
#define CONTROLLER_RIGHT_TRIGGER_AXIS 6

#define MAX_BUTTONS 15 // Maksymalna liczba przycisków
#define MAX_AXES 6     // Maksymalna liczba osi

// Struktura do przechowywania stanu przycisków i osi
typedef struct
{
    bool buttons[MAX_BUTTONS]; // Stan przycisków
    int axes[MAX_AXES];        // Wartości osi
} ControllerStates;

// Funkcja do inicjalizacji joysticka
SDL_Joystick *initialize_joystick()
{
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
    {
        printf("Nie można zainicjalizować SDL: %s\n", SDL_GetError());
        return NULL;
    }

    // Sprawdzenie liczby podłączonych kontrolerów
    if (SDL_NumJoysticks() < 1)
    {
        printf("Nie znaleziono żadnego kontrolera.\n");
        SDL_Quit();
        return NULL;
    }

    // Otwórz pierwszy joystick
    SDL_Joystick *joystick = SDL_JoystickOpen(0);
    if (!joystick)
    {
        printf("Nie można otworzyć joysticka: %s\n", SDL_GetError());
        SDL_Quit();
        return NULL;
    }

    printf("Joystick podłączony: %s\n", SDL_JoystickName(joystick));
    return joystick;
}

// Funkcja do odczytu stanu przycisków i osi
void read_controller_input(SDL_Joystick *joystick, ControllerStates *input_state)
{
    // Sprawdzenie stanu przycisków
    int num_buttons = SDL_JoystickNumButtons(joystick);
    for (int i = 0; i < num_buttons; i++)
    {
        input_state->buttons[i] = SDL_JoystickGetButton(joystick, i);
    }

    // Sprawdzanie osi
    int num_axes = SDL_JoystickNumAxes(joystick);
    for (int i = 0; i < num_axes; i++)
    {
        input_state->axes[i] = SDL_JoystickGetAxis(joystick, i);
    }
}

#endif // CONTROLLER.H
