/*
kod sluzacy do znalezienia adresów podlaczonych 
urządzeń na magistrali I2C

wynik:
Znaleziono urządzenie na adresie 0x41
Znaleziono urządzenie na adresie 0x42
Znaleziono urządzenie na adresie 0x68
Znaleziono urządzenie na adresie 0x70
Znaleziono urządzenie na adresie 0x77
Sprawdzanie zakończone.

kompilacja:
 gcc find_i2c_devices.c -o find_i2c_devices  -lwiringPi


*/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>

#define MAX_I2C_ADDRESS 0x77  // Maksymalny adres I2C
#define MIN_I2C_ADDRESS 0x03   // Minimalny adres I2C

// Funkcja sprawdzająca połączenie z danym adresem I2C
void check_i2c_address(int address) {
    int fd = wiringPiI2CSetup(address);  // Inicjalizacja I2C na danym adresie
    if (fd == -1) {
        return;  // Brak połączenia
    }

    // Próba odczytu z rejestru WHO_AM_I (lub innego rejestru)
    int result = wiringPiI2CReadReg8(fd, 0x00); // Używamy rejestru 0x00 dla testu
    if (result != -1) {
        printf("Znaleziono urządzenie na adresie 0x%02X\n", address);
    }
}

int main() {
    // Inicjalizacja wiringPi
    if (wiringPiSetup() == -1) {
        printf("Nie udało się zainicjalizować wiringPi\n");
        return 1;
    }

    // Sprawdzanie adresów I2C w zadanym zakresie
    for (int address = MIN_I2C_ADDRESS; address <= MAX_I2C_ADDRESS; address++) {
        check_i2c_address(address);
    }

    printf("Sprawdzanie zakończone.\n");
    return 0;
}
