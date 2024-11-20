#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "data_structures.h"
#include "enums.h"
#include "math.h"
#include "kinematics.h"

Matrix3 createJacobian(double q1, double q2, double q3, double k)
{ // kąty w radianach

    Matrix3 jacobian;

    double l1 = L1;
    double l2 = L2;
    double l3 = L3;

    double s1 = sin(q1);
    double c1 = cos(q1);

    double s2 = sin(q2);
    double c2 = cos(q2);

    double s3 = sin(q3);
    double c3 = cos(q3);

    double s23 = sin(q2 + q3);
    double c23 = cos(q2 + q3);

    double a = c23 * l3 + c2 * l2 + l1;
    double b = s23 * l3 + s2 * l2;

    jacobian.data[0][0] = -k * s1 * a;
    jacobian.data[0][1] = -k * c1 * b;
    jacobian.data[0][2] = -k * c1 * s23 * l3;
    jacobian.data[1][0] = k * c1 * a;
    jacobian.data[1][1] = -k * s1 * b;
    jacobian.data[1][2] = -k * s1 * s23 * l3;
    jacobian.data[2][0] = 0;
    jacobian.data[2][1] = -c23 * l3 - c2 * l2;
    jacobian.data[2][2] = -c23 * l3;

    return jacobian;
}

Matrix3 createInversedJacobian(Vector3 initial_angles)
{

     Matrix3 inversed_jacobian;

    double l1 = L1;
    double l2 = L2;
    double l3 = L3;

    double q1 = initial_angles.data[0];
    double q2 = initial_angles.data[1];
    double q3 = initial_angles.data[2];

    double s1 = sin(q1);
    double c1 = cos(q1);

    double s2 = sin(q2);
    double c2 = cos(q2);

    double s3 = sin(q3);
    double c3 = cos(q3);

    double s23 = sin(q2 + q3);
    double c23 = cos(q2 + q3);

    double csc3;

    if (s3 != 0)
    {
        csc3 = 1 / s3;
    }
    else
    {
        printf("nie wiem co zrobic, pojawilo sie dzielenie przez 0\n");
        exit(0);
    }

    double a = (l2 * c2) + (l3 * c23);

    inversed_jacobian.data[0][0] = (-s1) / (l1 + a);
    inversed_jacobian.data[0][1] = (c1) / (l1 + a);
    inversed_jacobian.data[0][2] = 0;
    inversed_jacobian.data[1][0] = (c1 * c23 * csc3) / (l2);
    inversed_jacobian.data[1][1] = (s1 * c23 * csc3) / (l2);
    inversed_jacobian.data[1][2] = (-s23 * csc3) / (l2);
    inversed_jacobian.data[2][0] = (-c1 * csc3 * a) / (l2 * l3);
    inversed_jacobian.data[2][1] = (-s1 * csc3 * a) / (l2 * l3);
    inversed_jacobian.data[2][2] = csc3 * ((s2 / l3) + (s23 / l2));

    return inversed_jacobian;
}

Vector3 calculateLegVelocity(Vector3 leg_start_pos, double robot_center_velocity, double robot_arc_radius)
{
    double omega = robot_center_velocity / robot_arc_radius;
    Vector3 leg_radius;                // r_s1 ^R
    Vector3 leg_velocity_arc_center;   // v_s1^R
    Vector3 leg_velocity_robot_center; // v_sq1R^R
    Vector3 omega_vector;

    leg_radius.data[X] = leg_start_pos.data[X] - robot_arc_radius;
    leg_radius.data[Y] = leg_start_pos.data[Y];
    leg_radius.data[Z] = 0;

    omega_vector.data[X] = 0;
    omega_vector.data[Y] = 0;
    omega_vector.data[Z] = omega;

    leg_velocity_arc_center = crossProduct(omega_vector, leg_radius);

    /*leg_velocity_arc_center.data[X] = -omega * leg_radius.data[Y];
    leg_velocity_arc_center.data[Y] = omega * leg_radius.data[X];
    leg_velocity_arc_center.data[Z] = 0;*/

    leg_velocity_robot_center.data[X] = -leg_velocity_arc_center.data[X] - 0;
    leg_velocity_robot_center.data[Y] = -leg_velocity_arc_center.data[Y] - robot_center_velocity;
    leg_velocity_robot_center.data[Z] = 0;

    return leg_velocity_robot_center;
}

#endif // JACOBIAN.H