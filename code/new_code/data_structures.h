#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <stdio.h>
#include <stdbool.h>

typedef struct Vector3
{
    double data[3];
}Vector3;

void printVector(Vector3 vector) {
    printf(" = [");

    printf(" %8.3f, ", vector.data[0]);
    printf(" %8.3f, ", vector.data[1]);
    printf(" %8.3f ", vector.data[2]);

    printf("]\n");
    
}

typedef struct Matrix3
{
    double data[3][3];
}Matrix3;

void printMatrix(Matrix3 matrix) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%8.3f ", matrix.data[i][j]);
        }
        printf("\n");
    }
}

// Funkcja mnożąca macierz 3x3 przez wektor 3
Vector3 multiplyMatrixByVector(Matrix3 matrix, Vector3 vector) {
    Vector3 result;

    // Mnożenie macierzy przez wektor
    for (int i = 0; i < 3; i++) {
        result.data[i] = 0;  // Inicjalizujemy elementy wektora wynikowego
        for (int j = 0; j < 3; j++) {
            result.data[i] += matrix.data[i][j] * vector.data[j];
        }
    }

    return result;
}



bool inverseMatrix(Matrix3 input_matrix, Matrix3 *inversed_matrix) {
    // Tworzymy rozszerzoną macierz [input_matrix | I] dla metody Gaussa-Jordana
    double augmented[3][6];
    
    // Inicjalizacja rozszerzonej macierzy
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            augmented[i][j] = input_matrix.data[i][j];         // Kopiujemy oryginalną macierz
            augmented[i][j + 3] = (i == j) ? 1.0 : 0.0;       // Dodajemy macierz jednostkową
        }
    }
    
    // Eliminacja Gaussa-Jordana
    for (int i = 0; i < 3; i++) {
        // Znajdujemy maksymalny element w kolumnie do pivotowania
        double pivot = augmented[i][i];
        if (pivot == 0) {
            // Jeśli pivot jest 0, macierz jest nieodwracalna
            return false;
        }
        
        // Normalizujemy wiersz pivotowy
        for (int j = 0; j < 6; j++) {
            augmented[i][j] /= pivot;
        }
        
        // Zerujemy elementy w innych wierszach tej kolumny
        for (int k = 0; k < 3; k++) {
            if (k != i) {
                double factor = augmented[k][i];
                for (int j = 0; j < 6; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }
    
    // Kopiujemy prawą stronę rozszerzonej macierzy do macierzy wyjściowej
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inversed_matrix->data[i][j] = augmented[i][j + 3];
        }
    }
    
    return true;
}


Vector3 crossProduct(Vector3 v1, Vector3 v2)
{
    Vector3 result;

    result.data[0] = v1.data[1] * v2.data[2] - v1.data[2] * v2.data[1];
    result.data[1] = v1.data[2] * v2.data[0] - v1.data[0] * v2.data[2];
    result.data[2] = v1.data[0] * v2.data[1] - v1.data[1] * v2.data[0];

    return result;
}


#endif //DATA_STRUCTURES.H