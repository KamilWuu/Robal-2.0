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

void setZeroPosition(int delay_time){
    printf("Ustawianie pozycji zerowej serw\n");
    for(int i = 0; i < 9; i++ )
    {
        SetServoAngle(pca_left, i, 135);
        printf("ruszyl %d lewy\n", i);
        delay(delay_time);
    }
    

    for(int i = 0; i < 9; i++ ){

        
        SetServoAngle(pca_right, i, 135);
        printf("ruszyl %d lewy\n", i);
        delay(delay_time);
    }
}


void SetTransportPosition(int delay_time){

    int zero_angle = 135;
    int Q2_angle_variable = 75;
    int Q3_angle_variable = 120;

    SetServoAngle(pca_right, RIGHT_BACK_Q3,  zero_angle + Q3_angle_variable);
    SetServoAngle(pca_left, LEFT_BACK_Q3,  zero_angle - Q3_angle_variable);
    printf("Q3_BACK\n");
    delay(delay_time);
    



    SetServoAngle(pca_right, RIGHT_MIDDLE_Q3,  zero_angle + Q3_angle_variable);
    SetServoAngle(pca_left, LEFT_MIDDLE_Q3,  zero_angle - Q3_angle_variable);
    printf("Q3_MIDDLE\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_FRONT_Q3,  zero_angle + Q3_angle_variable);
    SetServoAngle(pca_left, LEFT_FRONT_Q3,  zero_angle - Q3_angle_variable);
    printf("Q3_FRONT\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_BACK_Q2,  zero_angle + Q2_angle_variable);
    SetServoAngle(pca_left, LEFT_BACK_Q2,  zero_angle - Q2_angle_variable);
    printf("Q2_BACK\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_MIDDLE_Q2,  zero_angle + Q2_angle_variable);
    SetServoAngle(pca_left, LEFT_MIDDLE_Q2,  zero_angle - Q2_angle_variable);
    printf("Q2_MIDDLE\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_FRONT_Q2,  zero_angle + Q2_angle_variable);
    SetServoAngle(pca_left, LEFT_FRONT_Q2,  zero_angle - Q2_angle_variable);
    printf("Q2_FRONT\n");
    delay(delay_time);





    SetServoAngle(pca_left, LEFT_BACK_Q1, zero_angle);
    SetServoAngle(pca_right, RIGHT_BACK_Q1,  zero_angle);
    printf("Q1_BACK\n");
    delay(delay_time);


    SetServoAngle(pca_left, LEFT_MIDDLE_Q1, zero_angle);
    SetServoAngle(pca_right, RIGHT_MIDDLE_Q1,  zero_angle);
    printf("Q1_MIDDLE\n");
    delay(delay_time);



    SetServoAngle(pca_left, LEFT_FRONT_Q1,  zero_angle);
    SetServoAngle(pca_right, RIGHT_FRONT_Q1,  zero_angle);
    printf("Q1_FRONT\n\n");
    delay(delay_time);




}

void SetWalkingPosition(int delay_time, int Q1_plus, int Q2_plus, int Q3_plus){

    int zero_angle = 135;
    int Q2_angle_variable = 55 - Q2_plus;
    int Q3_angle_variable = -100 - Q3_plus;


    SetServoAngle(pca_right, RIGHT_BACK_Q2,  zero_angle + Q2_angle_variable);
    SetServoAngle(pca_left, LEFT_BACK_Q2,  zero_angle - Q2_angle_variable);
    printf("Q2_BACK\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_MIDDLE_Q2,  zero_angle + Q2_angle_variable);
    SetServoAngle(pca_left, LEFT_MIDDLE_Q2,  zero_angle - Q2_angle_variable);
    printf("Q2_MIDDLE\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_FRONT_Q2,  zero_angle + Q2_angle_variable);
    SetServoAngle(pca_left, LEFT_FRONT_Q2,  zero_angle - Q2_angle_variable);
    printf("Q2_FRONT\n");
    delay(delay_time);

    SetServoAngle(pca_right, RIGHT_BACK_Q3,  zero_angle + Q3_angle_variable);
    SetServoAngle(pca_left, LEFT_BACK_Q3,  zero_angle - Q3_angle_variable);
    printf("Q3_BACK\n");
    delay(delay_time);
    



    SetServoAngle(pca_right, RIGHT_MIDDLE_Q3,  zero_angle + Q3_angle_variable);
    SetServoAngle(pca_left, LEFT_MIDDLE_Q3,  zero_angle - Q3_angle_variable);
    printf("Q3_MIDDLE\n");
    delay(delay_time);



    SetServoAngle(pca_right, RIGHT_FRONT_Q3,  zero_angle + Q3_angle_variable);
    SetServoAngle(pca_left, LEFT_FRONT_Q3,  zero_angle - Q3_angle_variable);
    printf("Q3_FRONT\n");
    delay(delay_time);



    


    


    SetServoAngle(pca_left, LEFT_BACK_Q1, zero_angle - Q1_plus);
    SetServoAngle(pca_right, RIGHT_BACK_Q1,  zero_angle + Q1_plus);
    printf("Q1_BACK\n");
    delay(delay_time);


    SetServoAngle(pca_left, LEFT_MIDDLE_Q1, zero_angle);
    SetServoAngle(pca_right, RIGHT_MIDDLE_Q1,  zero_angle);
    printf("Q1_MIDDLE\n");
    delay(delay_time);



    SetServoAngle(pca_left, LEFT_FRONT_Q1,  zero_angle + Q1_plus);
    SetServoAngle(pca_right, RIGHT_FRONT_Q1,  zero_angle - Q1_plus);
    printf("Q1_FRONT\n\n");
    delay(delay_time);




}

void AlgorytmWstawaniaZPozycjaTransportową(){
    for(int i = 0; i < 4; i++){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
        printf("czeka\n");
    }
    SetTransportPosition(500);
    for(int i = 0; i < 2; i++){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
        printf("czeka\n");
    }


    SetWalkingPosition(500, 45, 0, 0);
    
   
    for(int i = 0; i < 1; i++){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
        printf("czeka\n");
    }


    

    int q2 = 5;
    int q3 = 5;
    for(int x = 0; x < 5; x++){

        SetWalkingPosition(500, 45, q2, q3);
        printf("wstawanie krok: %d z 5 \n", x+1);
        delay(400);
        q2 = q2+3;
        q3 = q3+6;
    }
    q2 = q2-3;
    q3 = q3-6;




    for(int i = 0; i < 7; i++){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
        printf("czeka\n");
    }

    for(int x = 0; x < 5; x++){

        SetWalkingPosition(500, 45, q2, q3);
        printf("kladzenie krok: %d z 5 \n", x+1);
        delay(400);
        q2 = q2-3;
        q3 = q3-6;
    }

    SetWalkingPosition(500, 0, 0, 0);



    for(int i = 0; i < 3; i++){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
        printf("czeka\n");
    }

    SetTransportPosition(800);
}


int main(void) {
    StartGPIO();
    if (InitPCA9685(&pca_left, PCA_ADDRESS_LEFT) == -1) return -1;
    if (InitPCA9685(&pca_right, PCA_ADDRESS_RIGHT) == -1) return -1;

    printf("#ROBAL: Cześć, to ja Robal!\n\n");

    

    

    printf("#ROBAL: Ustawiam pozycje chodu!\n\n") ;
   //SetTransportPosition(500);
   SetWalkingPosition(500, 45, 0, 0);
   printf("#ROBAL: Jestem gotowy do ruchu!\n") ;
    /*while(1){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
        printf("czeka\n");
    }*/

    

    return 0;
}
