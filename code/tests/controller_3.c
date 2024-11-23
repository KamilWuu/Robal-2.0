#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h> // Do obsługi sygnałów
#include <wiringPi.h>
#include <stdbool.h>

#define MAX_BUTTONS 15 // Maksymalna liczba przycisków
#define MAX_AXES 6     // Maksymalna liczba osi

// Struktura do przechowywania stanu przycisków i osi
typedef struct
{
    bool buttons[MAX_BUTTONS]; // Stan przycisków
    int axes[MAX_AXES];        // Wartości osi
} InputState;

// Funkcja do odczytu stanu przycisków i osi
void read_controller_input(SDL_Joystick *joystick, InputState *input_state)
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

// Funkcja do obsługi stanu przycisków
void handle_buttons(const InputState *input_state)
{
    for (int i = 0; i < MAX_BUTTONS; i++)
    {
        if (input_state->buttons[i])
        {
            printf("Przycisk %d wciśnięty\n", i);
        }
        else
        {
            printf("Przycisk %d puszczony\n", i);
        }
    }
}

// Funkcja do obsługi osi
void handle_axes(const InputState *input_state)
{
    for (int i = 0; i < MAX_AXES; i++)
    {
        printf("Oś %d: %d\n", i, input_state->axes[i]);
    }
}

// Funkcja, która obsługuje sygnał SIGINT (Ctrl+C)
void handle_sigint(int sig)
{
    printf("\nOtrzymano sygnał SIGINT (Ctrl+C). Zamykanie programu...\n");
    SDL_Quit();
    exit(0); // Zakończ program
}

int main(int argc, char *argv[])
{
    // Rejestracja handlera dla sygnału SIGINT (Ctrl+C)
    signal(SIGINT, handle_sigint);

    SDL_Init(SDL_INIT_JOYSTICK); // Inicjalizacja SDL z obsługą joysticków

    // Sprawdzenie liczby podłączonych kontrolerów
    if (SDL_NumJoysticks() < 1)
    {
        printf("Nie znaleziono żadnego kontrolera.\n");
        SDL_Quit();
        return 1;
    }

    // Otwórz pierwszy joystick
    SDL_Joystick *joystick = SDL_JoystickOpen(0);
    if (!joystick)
    {
        printf("Nie można otworzyć joysticka: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    printf("Joystick podłączony: %s\n", SDL_JoystickName(joystick));

    // Struktura przechowująca stan przycisków i osi
    InputState input_state = {0}; // Początkowy stan bez przycisków i osi

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
