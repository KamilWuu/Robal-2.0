#ifndef VELOCITY_H
#define VELOCITY_H

#include "data_structures.h"
#include "robot.h"

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

    // printVector("leg_radius", leg_radius);

    // omega_vector.data[X] = 0;
    // omega_vector.data[Y] = 0;
    // omega_vector.data[Z] = omega;

    // leg_velocity_arc_center = crossProduct(omega_vector, leg_radius);

    // printVector("leg_velocity_arv", leg_velocity_arc_center);

    leg_velocity_arc_center.data[X] = -omega * leg_radius.data[Y];
    leg_velocity_arc_center.data[Y] = omega * leg_radius.data[X];
    leg_velocity_arc_center.data[Z] = 0;

    leg_velocity_arc_center = vectorMultiplyByConst(leg_velocity_arc_center, 1);

    return leg_velocity_arc_center;

    // printVector("leg_velocity_arc_2", leg_velocity_arc_center); // spytac sie po co to jest

    // leg_velocity_robot_center.data[X] = -leg_velocity_arc_center.data[X] - 0;
    // leg_velocity_robot_center.data[Y] = -leg_velocity_arc_center.data[Y] - robot_center_velocity;
    // leg_velocity_robot_center.data[Z] = 0;

    // return leg_velocity_robot_center;
}

#endif // VELOCITY.H