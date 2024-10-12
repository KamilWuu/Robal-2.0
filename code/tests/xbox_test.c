/*
Przyklad pokazujace wyorzystanie przycisków do wlaczania i wylaczania diod

kopilacja:
gcc xbox_test.c -o x_box -lSDL2 -lwiringPi

*/

#include <SDL2/SDL.h>
#include <wiringPi.h>
#include <stdio.h>

#define LED_PIN_A 4  // Pin GPIO dla diody sterowanej przyciskiem A
#define LED_PIN_B 6  // Pin GPIO dla diody sterowanej przyciskiem B

int main() {
    // Inicjalizacja SDL oraz wiringPi
    wiringPiSetupGpio();
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        fprintf(stderr, "Nie udało się zainicjalizować SDL: %s\n", SDL_GetError());
        return 1;
    }
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Nie udało się zainicjalizować wiringPi\n");
        SDL_Quit();
        return 1;
    }
    
    // Konfiguracja pinów LED jako wyjścia
    pinMode(LED_PIN_A, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);

    // Sprawdzenie, czy dostępny jest kontroler
    if (SDL_NumJoysticks() < 1) {
        printf("Nie wykryto żadnych kontrolerów!\n");
        SDL_Quit();
        return 1;
    }

    // Otwieranie pierwszego kontrolera
    SDL_GameController *controller = SDL_GameControllerOpen(0);
    if (!controller) {
        fprintf(stderr, "Nie udało się otworzyć kontrolera: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Pętla testowa
    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else if (event.type == SDL_CONTROLLERBUTTONDOWN) {
                // Obsługa przycisku A
                if (event.cbutton.button == SDL_CONTROLLER_BUTTON_A) {
                    printf("Przycisk A wciśnięty - zapalanie LED na pinie %d\n", LED_PIN_A);
                    digitalWrite(LED_PIN_A, HIGH);
                }
                // Obsługa przycisku B
                if (event.cbutton.button == SDL_CONTROLLER_BUTTON_B) {
                    printf("Przycisk B wciśnięty - zapalanie LED na pinie %d\n", LED_PIN_B);
                    digitalWrite(LED_PIN_B, HIGH);
                }
            } else if (event.type == SDL_CONTROLLERBUTTONUP) {
                // Obsługa przycisku A
                if (event.cbutton.button == SDL_CONTROLLER_BUTTON_A) {
                    printf("Przycisk A zwolniony - gaszenie LED na pinie %d\n", LED_PIN_A);
                    digitalWrite(LED_PIN_A, LOW);
                }
                // Obsługa przycisku B
                if (event.cbutton.button == SDL_CONTROLLER_BUTTON_B) {
                    printf("Przycisk B zwolniony - gaszenie LED na pinie %d\n", LED_PIN_B);
                    digitalWrite(LED_PIN_B, LOW);
                }
            }
        }
    }

    // Zamknięcie kontrolera i SDL
    SDL_GameControllerClose(controller);
    SDL_Quit();
    return 0;
}
