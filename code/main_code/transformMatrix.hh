#ifndef TRANSFORMMATRIX_H
#define TRANSFORMMATRIX_H


#include <math.h>
#include "kinematics.h"
typedef struct 
{
    double P_x;
    double P_y;
    double P_z;

}PositionVector;


typedef struct Matrix3{
    int rows;
    int columns;
    double data[3][3];
};

typedef struct TransformMatrix{
    Matrix3 _rotationMatrix;
    PositionVector _translationVector;
};

Matrix3 createRotMatrix(double rot_X, double rot_Y, double rot_Z){

    Matrix3 rot_matrix;

    double alfa = rot_X * DEG2RAD;
    double beta = rot_Y * DEG2RAD;
    double gamma = rot_Z * DEG2RAD;

    double sin_a = sin(alfa);   double cos_a = cos(alfa);
    double sin_b = sin(beta);   double cos_b = cos(beta);
    double sin_g = sin(gamma);  double cos_g = cos(gamma);


    rot_matrix.data[0][0] = cos_g * cos_b;  
    rot_matrix.data[0][1] = (cos_g * sin_b * sin_a) - (sin_g * cos_a);
    rot_matrix.data[0][2] = (cos_g * sin_b * cos_a) + (sin_g * sin_a);

    rot_matrix.data[1][0] = sin_g * cos_b;  
    rot_matrix.data[1][1] = (sin_g * sin_b * sin_a) + (cos_g * cos_a);
    rot_matrix.data[1][2] = (sin_g * sin_b * cos_a) - (cos_g * sin_a);

    rot_matrix.data[2][0] = -sin_b;  
    rot_matrix.data[2][1] = cos_b*sin_a;
    rot_matrix.data[2][2] = cos_b*cos_a;


    return rot_matrix;
}

TransformMatrix createTransformMatrix(double rot_X, double rot_Y, double rot_Z, double t_x, double t_y, double t_z){
    TransformMatrix transform_matrix;
    
    transform_matrix._rotationMatrix = createRotMatrix(rot_X, rot_Y, rot_Z);
    transform_matrix._translationVector.P_x = t_x;
    transform_matrix._translationVector.P_y = t_y; 
    transform_matrix._translationVector.P_z = t_z;

    return transform_matrix;
}

void initTransformMatrix(TransformMatrix * transform_matrix, double rot_X, double rot_Y, double rot_Z, double t_x, double t_y, double t_z){

}


#endif //TRANSFORMMATRIX.H