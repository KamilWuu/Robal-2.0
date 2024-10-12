/*
Test pada od xboxa wykorzystujący 
trigery na padzie do sterowania 
jasnoscią diód za pomocą softPWM 

kompilacja: 

gcc test_xbox_2.c -o x_box2 -lSDL2 -lwiringPi
 */



#include <SDL2/SDL.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define LED_PIN_RT 6  // Pin GPIO dla diody kontrolowanej triggerem RT
#define LED_PIN_LT 4  // Pin GPIO dla diody kontrolowanej triggerem LT
#define LED_PIN_A 5   // Pin GPIO dla diody kontrolowanej przyciskiem A

int main() {
    // Inicjalizacja SDL oraz wiringPi
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        fprintf(stderr, "Nie udało się zainicjalizować SDL: %s\n", SDL_GetError());
        return 1;
    }
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Nie udało się zainicjalizować wiringPi\n");
        SDL_Quit();
        return 1;
    }
    
    // Konfiguracja pinów LED
    pinMode(LED_PIN_A, OUTPUT);  // Ustawienie pinu LED_A jako wyjście
    if (softPwmCreate(LED_PIN_RT, 0, 100) != 0 || softPwmCreate(LED_PIN_LT, 0, 100) != 0) {  // Inicjalizacja PWM
        fprintf(stderr, "Nie udało się utworzyć programowego PWM\n");
        SDL_Quit();
        return 1;
    }

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
            }

            // Sprawdzenie wciśnięcia przycisku A
            if (event.type == SDL_CONTROLLERBUTTONDOWN && event.cbutton.button == SDL_CONTROLLER_BUTTON_A) {
                printf("Przycisk A wciśnięty - zapalanie LED na pinie %d\n", LED_PIN_A);
                digitalWrite(LED_PIN_A, HIGH);  // Zapal LED na pinie 4
            } else if (event.type == SDL_CONTROLLERBUTTONUP && event.cbutton.button == SDL_CONTROLLER_BUTTON_A) {
                printf("Przycisk A zwolniony - gaszenie LED na pinie %d\n", LED_PIN_A);
                digitalWrite(LED_PIN_A, LOW);   // Zgaś LED na pinie 4
            }

            // Odczyt wartości triggera RT i normalizacja na zakres 0-100
            int rt_value = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
            int pwm_value_rt = (rt_value * 100) / 32767;

            // Odczyt wartości triggera LT i normalizacja na zakres 0-100
            int lt_value = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
            int pwm_value_lt = (lt_value * 100) / 32767;

            // Wyświetlenie wartości z triggerów RT i LT oraz odpowiadających wartości PWM
            printf("RT: %d, PWM RT: %d | LT: %d, PWM LT: %d\n", rt_value, pwm_value_rt, lt_value, pwm_value_lt);

            // Ustawienie jasności diod za pomocą programowego PWM
            softPwmWrite(LED_PIN_RT, pwm_value_rt);
            softPwmWrite(LED_PIN_LT, pwm_value_lt);
        }
        delay(10); // Małe opóźnienie dla stabilności odczytów
    }

    // Zamknięcie kontrolera i SDL
    SDL_GameControllerClose(controller);
    SDL_Quit();
    return 0;
}
