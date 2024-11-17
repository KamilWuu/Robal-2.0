#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   // Do usleep (milisekundy)
#include "square_function.h"

int main() {
    SquareFunction square_function;

    double y1 = 160;        // Początkowa wartość y w mm
    double d = 80;          // Zasięg zmiany y w mm
    double h = 60;          // Parametr h
    double z_shift = -60;   // Przemieszczenie w osi Z

    double z = 0;
    double z_d = 0;
    double y = y1;          // Początkowa wartość y
    double y_d = 100;        // Prędkość Y w mm/s (dy/dt)

    double x = 200;           // Początkowa wartość x w mm
    double x_d = 5;         // Prędkość X w mm/s (dx/dt)

    double time_step = 0.01; // Czas trwania pojedynczego kroku w sekundach (np. 0.1 sekundy)

    calculateSquareFunctionParams(&square_function, y1, d, h, z_shift);

    // Obliczamy liczbę iteracji
    double total_time_y = d / y_d;              // Czas potrzebny do przebycia całej drogi y
    int num_iterations = (int)(total_time_y / time_step);  // Liczba iteracji w zależności od czasu trwania pętli

    // Zakładamy, że y <= y1 + d to obszar, w którym chcemy pracować
    for (int i = 0; i < num_iterations; ++i) {
        z = getZ_FromY(square_function, y);
        z_d = getZVelocity_FromYVelocity(square_function, y, y_d);

        // Wyświetlanie wyników z prędkościami oraz pozycjami w mm
        //printf("Iteracja %d:\n", i + 1);
        printf("dla x = %.2f mm,\t x_d = %.2f mm/s,\t y = %.2f mm,\t y_d = %.2f mm/s,\t z = %.2f mm,\t z_d = %.2f mm/s\n", 
               x, x_d, y, y_d, z, z_d);
        
        // Czekaj czas_step (0.1 sekundy)
        usleep(time_step * 1000000); // usleep przyjmuje czas w mikrosekundach

        // Aktualizujemy y i x w oparciu o ich prędkości, z uwzględnieniem czasu kroku
        y += y_d * time_step; // Aktualizacja pozycji y (w mm)
        x += x_d * time_step; // Aktualizacja pozycji x (w mm)
    }

    return 0;
}
