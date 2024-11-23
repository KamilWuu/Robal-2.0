#include <SDL2/SDL.h>
#include <stdio.h>

#define MAX_AXIS_VALUE 32767
#define AXIS_ZERO 128
#define MAX_SPEED 160
#define MIN_ARC 500
#define MAX_ARC 10000
#define ARC_STRAIGHT 10000000



int main(int argc, char *argv[])
{
    // Deklaracja zmiennych do przechowywania wartości osi
    int axis_x = 0; // Wartość osi X (Axis 0)
    int axis_y = 0; // Wartość osi Y (Axis 1)
    int y_max = 0, y_min = 0, x_max = 0, x_min = 0;

    int arc = 0, v = 0;

    // Inicjalizacja SDL
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) < 0)
    {
        printf("Nie można zainicjalizować SDL: %s\n", SDL_GetError());
        return 1;
    }

    // Sprawdzenie liczby podłączonych kontrolerów
    if (SDL_NumJoysticks() < 1)
    {
        printf("Nie znaleziono żadnego kontrolera.\n");
        SDL_Quit();
        return 1;
    }

    // Otwórz pierwszy joystick
    SDL_GameController *controller = SDL_GameControllerOpen(0);
    if (!controller)
    {
        printf("Nie można otworzyć kontrolera: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    printf("Kontroler podłączony: %s\n", SDL_GameControllerName(controller));

    // Główna pętla
    int running = 1;
    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                running = 0;
                break;

            case SDL_CONTROLLERBUTTONDOWN:
                printf("Przycisk wciśnięty: %d\n", event.cbutton.button);
                break;

            case SDL_CONTROLLERBUTTONUP:
                printf("Przycisk puszczony: %d\n", event.cbutton.button);
                break;

            case SDL_CONTROLLERAXISMOTION:
                // Obsługa ruchu osi
                if (event.caxis.axis == 0)
                { // Oś X
                    axis_x = event.caxis.value;
                    printf("Oś X: %d\n", axis_x);
                }
                else if (event.caxis.axis == 1)
                { // Oś Y
                    axis_y = event.caxis.value;
                    printf("Oś Y: %d\n", axis_y);
                }
                else if (event.caxis.axis == 2)
                { // Oś Y

                    printf("Oś 2:%d\n", event.caxis.value);
                }
                else if (event.caxis.axis == 3)
                { // Oś Y

                    printf("Oś 3:%d\n", event.caxis.value);
                }
                else if (event.caxis.axis == 4)
                { // Oś Y

                    printf("Oś 4:%d\n", event.caxis.value);
                }
                else if (event.caxis.axis == 5)
                { // Oś Y

                    printf("Oś 5:%d\n", event.caxis.value);
                }
                else if (event.caxis.axis == 6)
                { // Oś Y

                    printf("Oś 6:%d\n", event.caxis.value);
                }

                break;

            default:
                break;
            }
        }

        // Możesz tu użyć wartości axis_x i axis_y w innych obliczeniach
        // Na przykład:

        v = map(axis_y, -MAX_AXIS_VALUE, MAX_AXIS_VALUE, MAX_SPEED, -MAX_SPEED);
        arc = calculateArc(axis_x);

        // printf(" V = %d\t\t\t, arc = %d, \t\t\n", v, arc);

        // Dodaj opóźnienie, aby odciążyć CPU
        SDL_Delay(16);
    }

    // Zamknij kontroler i wyczyść SDL
    SDL_GameControllerClose(controller);
    SDL_Quit();

    return 0;
}
