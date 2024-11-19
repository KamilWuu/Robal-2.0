#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <SDL2/SDL.h>
#include <stdio.h>

SDL_GameController* initXboxController() {
    // Inicjalizacja SDL
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        printf("Nie udało się zainicjować SDL: %s\n", SDL_GetError());
        return NULL;
    }

    // Sprawdzenie, czy jest dostępny kontroler
    if (SDL_NumJoysticks() < 1) {
        printf("Nie podłączono żadnego kontrolera.\n");
        SDL_Quit();
        return NULL;
    } else {
        printf("Kontroler jest gotowy!\n");
    }

    // Otwarcie kontrolera Xbox
    SDL_GameController *controller = SDL_GameControllerOpen(0);
    if (!controller) {
        printf("Nie udało się otworzyć kontrolera: %s\n", SDL_GetError());
        SDL_Quit();
        return NULL;
    }

    return controller;  // Zwracamy wskaźnik do kontrolera
}


#endif //CONTROLLER.H