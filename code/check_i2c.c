/*
    kod sluzacy do sprawdzenia komunikacji z urządzeniami I2C

wynik:
kamil@raspberrypi:~/robal/ROBAL_REPO/code $ ./check_pca 
Połączenie z urządzeniem PCA9685_1 na adresie 0x41: OK
Połączenie z urządzeniem PCA9685_2 na adresie 0x42: OK
Połączenie z urządzeniem MPU6050 na adresie 0x68: OK
Połączenie z urządzeniem QMC5883L na adresie 0x70: OK
Połączenie z urządzeniem BMP180 na adresie 0x77: OK
Sprawdzanie zakończone: wykryto przynajmniej jeden moduł.

kompilacja:
gcc check_i2c.c -o check_i2c  -lwiringPi
./check_i2c

*/


#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>

#define PCA9685_MODE1 0x00          // Rejestr MODE1 w PCA9685
#define PCA9685_ADDRESS_1 0x41      // Adres pierwszego PCA9685
#define PCA9685_ADDRESS_2 0x42      // Adres drugiego PCA9685
#define MPU6050_ADDRESS 0x68        // Adres MPU6050
#define QMC5883L_ADDRESS 0x70       // Adres QMC5883L
#define BMP180_ADDRESS 0x77         // Adres BMP180

// Funkcja sprawdzająca połączenie z urządzeniem
int check_connection(int address, const char *device_name) {
    int fd = wiringPiI2CSetup(address);  // Inicjalizacja I2C na danym adresie
    if (fd == -1) {
        printf("Nie można połączyć się z urządzeniem na adresie 0x%X\n", address);
        return 0;  // Brak połączenia
    }

    // Próba odczytu z rejestru MODE1 dla PCA9685 lub WHO_AM_I dla MPU6050
    int result;
    if (address == PCA9685_ADDRESS_1 || address == PCA9685_ADDRESS_2) {
        result = wiringPiI2CReadReg8(fd, PCA9685_MODE1);
    } else if (address == MPU6050_ADDRESS) {
        result = wiringPiI2CReadReg8(fd, 0x75); // Rejestr WHO_AM_I dla MPU6050
    } else if (address == QMC5883L_ADDRESS) {
        result = wiringPiI2CReadReg8(fd, 0x0A); // Rejestr Configuration Register A dla QMC5883L
    } else if (address == BMP180_ADDRESS) {
        result = wiringPiI2CReadReg8(fd, 0xD0); // Rejestr WHO_AM_I dla BMP180
    } else {
        return 0; // Nieznane urządzenie
    }

    if (result == -1) {
        printf("Brak odpowiedzi z urządzenia %s na adresie 0x%X\n", device_name, address);
        return 0;  // Brak połączenia
    }

    printf("Połączenie z urządzeniem %s na adresie 0x%X: OK\n", device_name, address);
    return 1;  // Połączenie OK
}

int main() {
    // Inicjalizacja biblioteki wiringPi
    if (wiringPiSetup() == -1) {
        printf("Nie udało się zainicjalizować wiringPi\n");
        return 1;
    }

    // Sprawdzenie połączenia dla obu PCA9685, MPU6050, QMC5883L i BMP180
    int connected_pca9685_1 = check_connection(PCA9685_ADDRESS_1, "PCA9685_1");
    int connected_pca9685_2 = check_connection(PCA9685_ADDRESS_2, "PCA9685_2");
    int connected_mpu6050 = check_connection(MPU6050_ADDRESS, "MPU6050");
    int connected_qmc5883l = check_connection(QMC5883L_ADDRESS, "QMC5883L");
    int connected_bmp180 = check_connection(BMP180_ADDRESS, "BMP180");

    // Wyświetlenie wyniku
    if (connected_pca9685_1 || connected_pca9685_2 || connected_mpu6050 || connected_qmc5883l || connected_bmp180) {
        printf("Sprawdzanie zakończone: wykryto przynajmniej jeden moduł.\n");
    } else {
        printf("Nie wykryto żadnego modułu.\n");
    }

    return 0;
}
