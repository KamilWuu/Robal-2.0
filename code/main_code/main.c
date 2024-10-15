#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "defines.h"
#include "servo.h"

#define NUM_CHANNELS 9  // Liczba kanałów serw na jednym PCA

int current_angles_left[NUM_CHANNELS] = {0};  // Tablica przechowująca kąty serw dla PCA_LEFT
int current_angles_right[NUM_CHANNELS] = {0}; // Tablica przechowująca kąty serw dla PCA_RIGHT

// Funkcja do jednoczesnego ustawiania kąta kilku serw
void MoveServosGradually(int pca, int channels[], int target_angles[], int num_servos, int steps, int delay_ms) {
    int *current_angles = (pca == pca_left) ? current_angles_left : current_angles_right;

    // Obliczenie kroku kąta dla każdego serwa
    int angle_steps[NUM_CHANNELS] = {0};  // Przechowuje kroki kąta dla każdego serwa
    for (int i = 0; i < num_servos; i++) {
        int channel = channels[i];
        if (channel < 0 || channel >= NUM_CHANNELS) {
            fprintf(stderr, "Invalid channel: %d\n", channel);
            continue;
        }
        angle_steps[channel] = (target_angles[i] - current_angles[channel]) / steps;
    }

    // Stopniowa aktualizacja kąta dla wszystkich serw
    for (int step = 0; step <= steps; step++) {
        for (int i = 0; i < num_servos; i++) {
            int channel = channels[i];
            int current_angle = current_angles[channel] + angle_steps[channel] * step;
            
            SetServoAngle(pca, channel, current_angle);
        }
        delay(delay_ms);  // Opóźnienie między krokami dla płynności ruchu
    }

    // Zaktualizowanie bieżących kątów serw w tablicy
    for (int i = 0; i < num_servos; i++) {
        int channel = channels[i];
        current_angles[channel] = target_angles[i];
    }
}

int main(void) {
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    printf("Hello! I am Robal!\n");

    delay(1000);
    SetServoAngle(pca_left, LEFT_FRONT_Q1, 165);
    delay(1000);
    SetServoAngle(pca_left, LEFT_MIDDLE_Q1, 135);
    delay(1000);
    SetServoAngle(pca_left, LEFT_BACK_Q1, 120);
    delay(1000);
    SetServoAngle(pca_right, RIGHT_FRONT_Q1, 135);
    delay(1000);
    SetServoAngle(pca_right, RIGHT_MIDDLE_Q1, 150);
    delay(1000);
    SetServoAngle(pca_right, RIGHT_BACK_Q1, 165);

    delay(2000); 

    

    int channels_left[] = {LEFT_BACK_Q1, LEFT_BACK_Q2, LEFT_BACK_Q3};     
    int channels_right[] = {RIGHT_BACK_Q1, RIGHT_BACK_Q2, RIGHT_BACK_Q3};          // Kanały, które chcemy kontrolować
    //int target_angles[] = {135, 135, 135};   // Docelowe kąty dla tych kanałów

    //MoveServosGradually(pca_right, channels, target_angles, 3, 10, 100);  // Przykład ruchu kilku serw na raz

    SetServoAngle(pca_left, LEFT_BACK_Q1, 135);
    current_angles_left[LEFT_BACK_Q1] = 135;
    SetServoAngle(pca_left, LEFT_BACK_Q2, 135);
    current_angles_left[LEFT_BACK_Q2] = 135;
    SetServoAngle(pca_left, LEFT_BACK_Q3, 135);
    current_angles_left[LEFT_BACK_Q3] = 135;

    SetServoAngle(pca_right, RIGHT_BACK_Q1, 135);
    current_angles_right[RIGHT_BACK_Q1] = 135;
    SetServoAngle(pca_right, RIGHT_BACK_Q2, 135);
    current_angles_right[RIGHT_BACK_Q2] = 135;
    SetServoAngle(pca_right, RIGHT_BACK_Q3, 135);
    current_angles_right[RIGHT_BACK_Q3] = 135;





    delay(2000);

    while (1) {
        
        /*int dupa = digitalRead(TOUCH_SENSOR_LEFT_BACK);

        if(dupa == 1){
            printf("wcisniety\n");
        }else if(dupa == 0){
            printf("nie wcisniety\n");
        }



        delay(100);*/

        int new_angles_1_right[] = {195, 195, 75};   // Nowe docelowe kąty
        MoveServosGradually(pca_right, channels_right, new_angles_1_right, 3, 30, 15);
        //delay(100);


        int new_angles_2_right[] = {135, 135, 135};   // Nowe docelowe kąty
        MoveServosGradually(pca_right, channels_right, new_angles_2_right, 3, 30, 15);
        //delay(10);


        int new_angles_1_left[] = {75 , 105, 225};   // Nowe docelowe kąty
        MoveServosGradually(pca_left, channels_left, new_angles_1_left, 3, 30, 15);
        //delay(100);




        int new_angles_2_left[] = {135, 165, 165};   // Nowe docelowe kąty
        MoveServosGradually(pca_left, channels_left, new_angles_2_left, 3, 30, 15);
        //delay(100);

        
        


    }


    return 0;
}
