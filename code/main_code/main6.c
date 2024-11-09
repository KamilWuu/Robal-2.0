#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <SDL2/SDL.h>
#include "defines.h"
#include "servo.h"
#include "kinematics.h"
#include "robot.h"
#include "trajectory.h"





int main() {

    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1)
        return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1)
        return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    Robot Hexapod;
    initRobot(&Hexapod);

    // Inicjalizacja SDL
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        printf("Nie udało się zainicjować SDL: %s\n", SDL_GetError());
        return -1;
    }

    // Otwarcie kontrolera Xbox
    SDL_GameController *controller = NULL;
    if (SDL_NumJoysticks() < 1) {
        printf("Nie podłączono żadnego kontrolera.\n");
        SDL_Quit();
        return -1;
    }else {
        printf("controller ready!\n");
    }

    controller = SDL_GameControllerOpen(0);
    if (!controller) {
        printf("Nie udało się otworzyć kontrolera: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    int Z = 0;

    int delay_time = 100;
    int delay_between_fases = 0;
    double Z_speed = 100; 
    double Z_step_time = 0.01;

    double Y_move = 100;
    double Y_speed = 1000;
    double Y_step_time = 0.01;

    double max_Y_speed = 3000;
    double min_Y_speed = 100;
    double Y_speed_variable = 100;

    int it = 100;

    bool czyMaChodzic = false;
    bool exit_program = false;



    printZMoveParams(z_const_stand_up, Z_speed, Z_step_time);
    printYMoveParams(Y_move, Y_speed, Y_step_time);
    

    StepFase start_fase = LM_RF_RB_PROT;

    setWalkingPosition(&Hexapod, delay_time);




    // Pętla główna
    SDL_Event event;
    int quit = 0;
    while (!quit) {
        if(exit_program){
            break;
        }

        if(czyMaChodzic){
                /*** Pętla służąca do naprzemiennego stawiania nóg ***************************** */
                if(start_fase == LM_RF_RB_PROT){
                    
                        doStepFase(&Hexapod, LF_LB_RM_PROT__LM_RF_RB_RETR, Y_move, Y_speed, Y_step_time);
                        delay(delay_between_fases);
                        doStepFase(&Hexapod, LF_LB_RM_RETR__LM_RF_RB_PROT, Y_move, Y_speed, Y_step_time);
                        delay(delay_between_fases);
                        
                    
                } else if (start_fase == LF_LB_RM_PROT){
                    
                        
                        
                        
                        doStepFase(&Hexapod, LF_LB_RM_RETR__LM_RF_RB_PROT, Y_move, Y_speed, Y_step_time);
                        delay(delay_between_fases);
                        doStepFase(&Hexapod, LF_LB_RM_PROT__LM_RF_RB_RETR, Y_move, Y_speed, Y_step_time);
                        delay(delay_between_fases);
                        
                    
                }
                /*********************************************************** */
                
            }

        // Sprawdzanie zdarzeń
        while (SDL_PollEvent(&event)) {

            



            if (event.type == SDL_QUIT) {
                quit = 1;
            }

            // Sprawdzanie przycisków
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_Y)) {
                //tutaj robot wstaje 
                moveLegsZ(&Hexapod, z_const_stand_up, Z_speed,  Z_step_time);
                printZMoveParams(z_const_stand_up, Z_speed, Z_step_time);
                delay(delay_time *10);
                
                delay(delay_time *10);
                printf("Robal wstal \n");
                delay(500);
            } 
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_START)) {
                

                
               
                //tu jest przygotowanie nog przed chodzeniem
                prepareForStepFase(&Hexapod, start_fase, Y_move, Y_speed, Y_step_time);
                delay(delay_time *10);
                printf("Robal jest gotowy do ruchu\n");
                


                delay(500);
            } 
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B)) {
                /*************tutaj robot siada */
                moveLegsZ(&Hexapod, -z_const_stand_up, Z_speed,  Z_step_time);
                /*********************************/
                delay(delay_time *10);
                printf("Robal usiadl \n");
                delay(500);
            } 
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_X)) {
                czyMaChodzic = true;

                printf("kstart petli \n");
                delay(500);
            }
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A)) {
                czyMaChodzic = false;

                printf("koniec petli \n");
                printYMoveParams(Y_move, Y_speed, Y_step_time);
                delay(500);
            }
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_UP)) {
                if((Y_speed + Y_speed_variable) < max_Y_speed){
                    Y_speed = Y_speed + Y_speed_variable;
                }else{
                    Y_speed = max_Y_speed;
                }
                printf("Zmieniono predkosc Y na: %lf \n", Y_speed);
                
            } 
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
                if((Y_speed - Y_speed_variable) > min_Y_speed){
                    Y_speed = Y_speed - Y_speed_variable;
                }else{
                    Y_speed = min_Y_speed;
                }
                printf("Zmieniono predkosc Y na: %lf \n", Y_speed);
                
            }
            if(SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_BACK)) {
                printf("nacisnieto back \n");
                if(Hexapod._robotStepFase == SIT_DOWN){
                    printf("zakonczenie programu\n");
                    delay(3000);

                    //to nie jest konieczne
                    setWalkingPosition(&Hexapod, 500);
                    /********* */

                    exit_program = true;
                    break;                   
                }
            }
        }

        // Opóźnienie, aby uniknąć zbyt dużego zużycia procesora
        SDL_Delay(10);
    }

    // Zamknięcie kontrolera i SDL
    SDL_GameControllerClose(controller);
    SDL_Quit();

    if(global_error == 0){
        printf("Nie wykryto żadnego bledu w trakcie wykonywania kodu : ) \n");
        }else{
        printf("Wykryto %d bledow w trakcie wykonywania programu : ( \n", global_error);
    }

    return 0;
}