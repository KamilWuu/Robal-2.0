#ifndef TRANSFORMMATRIX_H
#define TRANSFORMMATRIX_H

#include <math.h>
#include "kinematics.h"


typedef struct PositionVector
{
    double P_x;
    double P_y;
    double P_z;

};

PositionVector setPositionVector(double x, double y, double z){
    PositionVector temp_pos_vector;

    temp_pos_vector.P_x = x;
    temp_pos_vector.P_x = y;
    temp_pos_vector.P_x = z;

    return temp_pos_vector;
}

typedef struct RobotRotationsAngles
{
    double _rot_X;
    double _rot_Y;
    double _rot_Z;
};  

RobotRotationsAngles setRotationsAngles(double rot_x, double rot_y, double rot_z){
    RobotRotationsAngles temp_angles;

    temp_angles._rot_X = rot_x;
    temp_angles._rot_Y = rot_y;
    temp_angles._rot_Z = rot_z;

    return temp_angles;
}

void printPositionVector(PositionVector  vector){
    printf("| %f | %f | %f |\n", vector.P_x, vector.P_y, vector.P_z);
}

typedef struct Matrix3{
    double data[3][3];
};

typedef struct TransformMatrix{
    Matrix3 _rotationMatrix;
    PositionVector _translationVector;
};

Matrix3 createRotMatrix(RobotRotationsAngles angles){

    Matrix3 rot_matrix;

    double alfa = angles._rot_X * DEG2RAD;
    double beta = angles._rot_Y  * DEG2RAD;
    double gamma = angles._rot_Z  * DEG2RAD;

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

TransformMatrix createTransformMatrix(RobotRotationsAngles angles, PositionVector pos_vector){
    TransformMatrix out_transform_matrix;
    
    out_transform_matrix._rotationMatrix = createRotMatrix(angles._rot_X, angles._rot_Y, angles._rot_Z);
    out_transform_matrix._translationVector = pos_vector;


    return out_transform_matrix;
}

//wspolrzednie wzgledem srodka robota na wspolrzedne w ukladzie globalnym. Na wejsciu wspolrzedne wzgledem robota na wyjsciu wspolrzedne w ukladzie globalnym
PositionVector TransformVector(PositionVector input_vector, TransformMatrix transform_matrix) { 
    PositionVector output_vector, temp_vector;
    Matrix3 rot_matrix = transform_matrix._rotationMatrix;
    PositionVector trans_vector = transform_matrix._translationVector;

    // Zastosowanie rotacji
    temp_vector.P_x = rot_matrix.data[0][0] * input_vector.P_x + rot_matrix.data[0][1] * input_vector.P_y + rot_matrix.data[0][2] * input_vector.P_z;
    temp_vector.P_y = rot_matrix.data[1][0] * input_vector.P_x + rot_matrix.data[1][1] * input_vector.P_y + rot_matrix.data[1][2] * input_vector.P_z;
    temp_vector.P_z = rot_matrix.data[2][0] * input_vector.P_x + rot_matrix.data[2][1] * input_vector.P_y + rot_matrix.data[2][2] * input_vector.P_z;

    // Dodanie wektora translacji
    output_vector.P_x = temp_vector.P_x + trans_vector.P_x;
    output_vector.P_y = temp_vector.P_y + trans_vector.P_y;
    output_vector.P_z = temp_vector.P_z + trans_vector.P_z;

    return output_vector;
}

//wspolrzedne globalne na wspolrzedne wzgledem srodka robota. Na wejsciu wspolrzedne globalne na wyjsciu wspolrzedne w ukladzie robota
PositionVector inverseTransformVector(PositionVector input_vector, TransformMatrix transform_matrix) {
    PositionVector output_vector, temp_vector;
    Matrix3 rot_matrix = transform_matrix._rotationMatrix;
    PositionVector trans_vector = transform_matrix._translationVector;

    // Transpozycja macierzy rotacji
    Matrix3 rot_matrix_T;
    rot_matrix_T.data[0][0] = rot_matrix.data[0][0];
    rot_matrix_T.data[0][1] = rot_matrix.data[1][0];
    rot_matrix_T.data[0][2] = rot_matrix.data[2][0];

    rot_matrix_T.data[1][0] = rot_matrix.data[0][1];
    rot_matrix_T.data[1][1] = rot_matrix.data[1][1];
    rot_matrix_T.data[1][2] = rot_matrix.data[2][1];

    rot_matrix_T.data[2][0] = rot_matrix.data[0][2];
    rot_matrix_T.data[2][1] = rot_matrix.data[1][2];
    rot_matrix_T.data[2][2] = rot_matrix.data[2][2];

    // Przesunięcie translacji w odniesieniu do odwróconej rotacji
    PositionVector trans_inverse;
    trans_inverse.P_x = -(rot_matrix_T.data[0][0] * trans_vector.P_x + rot_matrix_T.data[0][1] * trans_vector.P_y + rot_matrix_T.data[0][2] * trans_vector.P_z);
    trans_inverse.P_y = -(rot_matrix_T.data[1][0] * trans_vector.P_x + rot_matrix_T.data[1][1] * trans_vector.P_y + rot_matrix_T.data[1][2] * trans_vector.P_z);
    trans_inverse.P_z = -(rot_matrix_T.data[2][0] * trans_vector.P_x + rot_matrix_T.data[2][1] * trans_vector.P_y + rot_matrix_T.data[2][2] * trans_vector.P_z);

    // Zastosowanie odwróconej rotacji do punktu wejściowego
    temp_vector.P_x = rot_matrix_T.data[0][0] * input_vector.P_x + rot_matrix_T.data[0][1] * input_vector.P_y + rot_matrix_T.data[0][2] * input_vector.P_z;
    temp_vector.P_y = rot_matrix_T.data[1][0] * input_vector.P_x + rot_matrix_T.data[1][1] * input_vector.P_y + rot_matrix_T.data[1][2] * input_vector.P_z;
    temp_vector.P_z = rot_matrix_T.data[2][0] * input_vector.P_x + rot_matrix_T.data[2][1] * input_vector.P_y + rot_matrix_T.data[2][2] * input_vector.P_z;

    // Dodanie odwróconego wektora translacji
    output_vector.P_x = temp_vector.P_x + trans_inverse.P_x;
    output_vector.P_y = temp_vector.P_y + trans_inverse.P_y;
    output_vector.P_z = temp_vector.P_z + trans_inverse.P_z;

    return output_vector;
}

#endif //TRANSFORMMATRIX.H