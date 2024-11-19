#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   
#include "robot.h"
#include "controller.h"




int main(){

    SDL_GameController * controller;
    controller = initXboxController();
    
    

    Robot hexapod;

    initRobot(&hexapod);
    

    return 0;
}

