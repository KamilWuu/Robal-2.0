#ifndef DEFINES_H
#define DEFINES_H

/* ============================= OUTPUTS ============================= */
#define LED_RED 6
#define LED_GREEN 4
#define BUZZER 5

#define D_0 22
#define D_1 17

#define PCA_1_OE 27
#define PCA_2_OE 18
/* =========================== END OF OUTPUTS ========================= */

/* ============================= INPUTS ============================== */
#define TOUCH_SENSOR_1 12
#define TOUCH_SENSOR_2 25
#define TOUCH_SENSOR_3 16
#define TOUCH_SENSOR_4 24
#define TOUCH_SENSOR_5 20
#define TOUCH_SENSOR_6 23

#define BUTTON_1 21
#define BUTTON_2 26

#define INTA_MPU 19
#define DRDY 13
/* =========================== END OF INPUTS ========================== */

/* ========================== COMMUNICATION ========================== */
#define SPI_CEO0 8
#define SPI_CEO1 7
#define SPI_SCLK 11
#define SPI_MOSI 10
#define SPI_MISO 9

#define I2C_SDA 2
#define I2C_SCL 3

#define TX 14
#define RX 15
/* ======================= END OF COMMUNICATION ======================= */

/* =========================== DEVICE ADDRESSES ======================= */
#define PCA9685_MODE1 0x00
#define PCA_ADDRESS_1 0x41
#define PCA_ADDRESS_2 0x42
#define MPU6050_ADDRESS 0x68
#define QMC5883L_ADDRESS 0x70
#define BMP180_ADDRESS 0x77
/* ===================== END OF DEVICE ADDRESSES ====================== */

#include <stdio.h>
#include <wiringPi.h>

void StartGPIO()
{
    // Ustawienie GPIO na schemat numeracji Broadcom (BCM)
    wiringPiSetupGpio();

    /* =========================== OUTPUTS =========================== */
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    pinMode(D_0, OUTPUT);
    pinMode(D_1, OUTPUT);

    pinMode(PCA_1_OE, OUTPUT);
    pinMode(PCA_2_OE, OUTPUT);

    /* ============================ INPUTS =========================== */
    pinMode(TOUCH_SENSOR_1, INPUT);
    pinMode(TOUCH_SENSOR_2, INPUT);
    pinMode(TOUCH_SENSOR_3, INPUT);
    pinMode(TOUCH_SENSOR_4, INPUT);
    pinMode(TOUCH_SENSOR_5, INPUT);
    pinMode(TOUCH_SENSOR_6, INPUT);

    pinMode(BUTTON_1, INPUT);
    pinMode(BUTTON_2, INPUT);

    pinMode(INTA_MPU, INPUT);
    pinMode(DRDY, INPUT);

    /* ========================= COMMUNICATION ======================== */
    // SPI and I2C pins are often managed by the system driver and may not need manual configuration.
    // However, set their mode if manual configuration is required:

    /*

    pinMode(SPI_CEO0, OUTPUT);  // SPI Chip Enable 0
    pinMode(SPI_CEO1, OUTPUT);  // SPI Chip Enable 1
    pinMode(SPI_SCLK, OUTPUT);  // SPI Clock
    pinMode(SPI_MOSI, OUTPUT);  // SPI Master Out Slave In
    pinMode(SPI_MISO, INPUT);   // SPI Master In Slave Out

    pinMode(I2C_SDA, INPUT);    // I2C Data
    pinMode(I2C_SCL, INPUT);    // I2C Clock

    pinMode(TX, OUTPUT);        // UART TX
    pinMode(RX, INPUT);         // UART RX

    */

    /* ==================== Initial States for Outputs =================== */
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(BUZZER, LOW);
    digitalWrite(D_0, LOW);
    digitalWrite(D_1, LOW);
    digitalWrite(PCA_1_OE, LOW);
    digitalWrite(PCA_2_OE, LOW);

    printf("GPIO succesfully started!\n");
}

#endif // DEFINES_H