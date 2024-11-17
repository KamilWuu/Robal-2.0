#ifndef SERVO_HARDWARE_H
#define SERVO_HARDWARE_H

#include "defines.h"
#include <wiringPiI2C.h>
#include "enums.h"

#define BLOCK_SERVOS 1 //<- 1 - zablokowane, 0 - odblokowane

#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define LED0_ON_L           0x06
#define PWM_FREQ            50   // Standard PWM frequency for servos

#define SERVO_MIN  102     // PWM value for 0° (0.5ms)
#define SERVO_MAX  512     // PWM value for 270° (2.7ms)

/**500 - 2500, 1500 neutral [usec] */

int pca_left, pca_right;

// Function to initialize a specific PCA9685 address
int InitPCA9685(int *pca, int address) {
    *pca = wiringPiI2CSetup(address);
    if (*pca == -1) {
        fprintf(stderr, "Failed to initialize PCA9685 at address 0x%X.\n", address);
        return -1;
    }

    wiringPiI2CWriteReg8(*pca, PCA9685_MODE1, 0x10);  // Enter sleep mode to set frequency
    int prescale_val = (int)(25000000.0 / (4096 * PWM_FREQ) - 1);
    wiringPiI2CWriteReg8(*pca, PCA9685_PRESCALE, prescale_val);

    wiringPiI2CWriteReg8(*pca, PCA9685_MODE1, 0x20);  // Wake up and enable auto-increment
    delay(1);
    wiringPiI2CWriteReg8(*pca, PCA9685_MODE1, 0xA1);

    //printf("PCA9685 at address 0x%X initialized successfully.\n", address);
    return 0;
}


// Function to set the angle of a servo on a given PCA and channel
void SetServoAngle(RobotSide side, int q, int pca, int channel, double calculated_angle_rad) {

    double angle;

    if (side == RIGHT)
    {
        switch (q)
        {
        case 1:
            angle = 135 - (calculated_angle_rad * RAD2DEG)
            break;
        case 2:
            angle = 135 + (calculated_angle_rad * RAD2DEG)
            break;
        case 3:
            angle = 135 + (calculated_angle_rad * RAD2DEG)
            break;
        
        
        default:
            break;
        }
    }

    if (side == LEFT)
    {
        switch (q)
        {
        case 1:
            angle = 135 + (calculated_angle_rad * RAD2DEG)
            break;
        case 2:
            angle = 135 - (calculated_angle_rad * RAD2DEG)
            break;
        case 3:
            angle = 135 - (calculated_angle_rad * RAD2DEG)
            break;
        
        
        default:
            break;
        }
    }


    if (angle < 0) angle = 0;
    if (angle > 270) angle = 270;

    int pulse_length = SERVO_MIN + (angle * (SERVO_MAX - SERVO_MIN) / 270);
    int on_time = 0;
    int off_time = pulse_length;

    int led_on_l = LED0_ON_L + 4 * channel;
    int led_off_l = led_on_l + 2;

    wiringPiI2CWriteReg16(pca, led_on_l, on_time);
    wiringPiI2CWriteReg16(pca, led_off_l, off_time);
}

#endif // SERVO_H




#endif //SERVO_HARDWARE.H

