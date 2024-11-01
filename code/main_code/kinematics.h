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
#define y_const (d3 + 70.0) // d3
#define z_const_zero 0.0
#define z_const_stand_up (-80.0)

#define h_const 30 //elipse h

/*wartosci graniczne ustalone dla prawej strony, dla lewej nalezy zamienic max z min*/
// Q1: Front
#define FRONT_LEFT_Q1_MIN_ANGLE 0
#define FRONT_LEFT_Q1_MAX_ANGLE 270

#define FRONT_RIGHT_Q1_MIN_ANGLE 0
#define FRONT_RIGHT_Q1_MAX_ANGLE 270

// Q1: Middle
#define MIDDLE_LEFT_Q1_MIN_ANGLE 0
#define MIDDLE_LEFT_Q1_MAX_ANGLE 270

#define MIDDLE_RIGHT_Q1_MIN_ANGLE 0
#define MIDDLE_RIGHT_Q1_MAX_ANGLE 270

// Q1: Back
#define BACK_LEFT_Q1_MIN_ANGLE 0
#define BACK_LEFT_Q1_MAX_ANGLE 270

#define BACK_RIGHT_Q1_MIN_ANGLE 0
#define BACK_RIGHT_Q1_MAX_ANGLE 270

// Q2: Left
#define LEFT_Q2_MIN_ANGLE 0
#define LEFT_Q2_MAX_ANGLE 270

// Q2: Right
#define RIGHT_Q2_MIN_ANGLE 0
#define RIGHT_Q2_MAX_ANGLE 270

// Q3: Left
#define LEFT_Q3_MIN_ANGLE 0
#define LEFT_Q3_MAX_ANGLE 270

// Q3: Right
#define RIGHT_Q3_MIN_ANGLE 0
#define RIGHT_Q3_MAX_ANGLE 270

#define DEG2RAD MY_PI / 180
#define RAD2DEG 180 / MY_PI

#endif // KINEMATICS_H