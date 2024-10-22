#ifndef KINEMATICS_H
#define KINEMATICS_H



#define L1  73.5 
#define L2  84.5
#define L3  118.5

#define d1      226.5
#define d1_2    113.25
#define d2      150
#define d3      110
#define d4      110


#define Q1_initial_angle 135
#define Q2_initial_angle 135 + 55
#define Q3_initial_angle 135 - 100

#define MY_PI 3.14159265358979323846

/*wartosci graniczne ustalone dla prawej strony, dla lewej nalezy zamienic max z min*/
#define FRONT_Q1_MIN_ANGLE 0
#define FRONT_Q1_MAX_ANGLE 270

#define MIDDLE_Q1_MIN_ANGLE 0
#define MIDDLE_Q1_MAX_ANGLE 270

#define BACK_Q1_MIN_ANGLE 0
#define BACK_Q1_MAX_ANGLE 270



#define Q2_MIN_ANGLE 0
#define Q2_MAX_ANGLE 270

#define Q3_MIN_ANGLE 0
#define Q3_MAX_ANGLE 270

#define DEG2RAD MY_PI/180
#define RAD2DEG 180/MY_PI 




#endif // KINEMATICS_H