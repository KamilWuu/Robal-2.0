#ifndef KINEMATICS_H
#define KINEMATICS_H

#define L1 73.5
#define L2 84.5
#define L3 118.5
#define L23 70

#define d1 55.0
#define d2 75.0
#define d3 113.25

#define z_0 0.0

#define Q1_initial_angle 135
#define Q2_initial_angle (135 + 55)
#define Q3_initial_angle (135 - 100)

#define MY_PI 3.14159265358979323846

#define x_const 245.0
#define y_const (d3 + 50.0) // d3
#define z_const_zero 0.0
#define z_const_stand_up (-60.0)

#define h_const 35 // elipse h

#define DEG2RAD (MY_PI / 180)
#define RAD2DEG (180 / MY_PI)

/*maksymalne wartosci bledow pozycji do korekcji bledow*/
#define X_POS_MAX_ERROR 10
#define Y_POS_MAX_ERROR 10
#define Z_POS_MAX_ERROR 10

/*wartosci graniczne ustalone dla prawej strony, dla lewej nalezy zamienic max z min*/
// Q1: Front
#define FRONT_LEFT_Q1_MIN_ANGLE (-135 * DEG2RAD)
#define FRONT_LEFT_Q1_MAX_ANGLE (135 * DEG2RAD)

#define FRONT_RIGHT_Q1_MIN_ANGLE (-135 * DEG2RAD)
#define FRONT_RIGHT_Q1_MAX_ANGLE (135 * DEG2RAD)

// Q1: Middle
#define MIDDLE_LEFT_Q1_MIN_ANGLE (-135 * DEG2RAD)
#define MIDDLE_LEFT_Q1_MAX_ANGLE (135 * DEG2RAD)

#define MIDDLE_RIGHT_Q1_MIN_ANGLE (-135 * DEG2RAD)
#define MIDDLE_RIGHT_Q1_MAX_ANGLE (135 * DEG2RAD)

// Q1: Back
#define BACK_LEFT_Q1_MIN_ANGLE (-135 * DEG2RAD)
#define BACK_LEFT_Q1_MAX_ANGLE (135 * DEG2RAD)

#define BACK_RIGHT_Q1_MIN_ANGLE (-135 * DEG2RAD)
#define BACK_RIGHT_Q1_MAX_ANGLE (135 * DEG2RAD)

// Q2: Left
#define LEFT_Q2_MIN_ANGLE (-135 * DEG2RAD)
#define LEFT_Q2_MAX_ANGLE (135 * DEG2RAD)

// Q2: Right
#define RIGHT_Q2_MIN_ANGLE (-135 * DEG2RAD)
#define RIGHT_Q2_MAX_ANGLE (135 * DEG2RAD)

// Q3: Left
#define LEFT_Q3_MIN_ANGLE (-135 * DEG2RAD)
#define LEFT_Q3_MAX_ANGLE (135 * DEG2RAD)

// Q3: Right
#define RIGHT_Q3_MIN_ANGLE (-135 * DEG2RAD)
#define RIGHT_Q3_MAX_ANGLE (135 * DEG2RAD)

#endif // KINEMATICS_H
