#include <stdio.h>
#include <unistd.h>

#include "kinematics.h"
#include "defines.h"
#include "robot.h"
#include "jakobian.h"
#include "data_structures.h"

int main()
{
    Robot hexapod;
    initRobot(&hexapod);
    /*
        Vector3 start_pos;
        start_pos.data[X] = x_const;
        start_pos.data[Y] = 0;
        start_pos.data[Z] = z_const_stand_up;
        evaluateLegPositionRobotCenter(&hexapod, RIGHT_MIDDLE, start_pos);*/

    Vector3 start_leg_pos;
    Vector3 start_leg_angles;
    Matrix3 inversed_jacobian;

    start_leg_pos = hexapod._LegsPositionRobotCenter[LEFT_MIDDLE];
    start_leg_angles = hexapod._legs[LEFT_MIDDLE]._leg_joint_angles;

    inversed_jacobian = createInversedJacobian(start_leg_angles);

    printf("pozycja startowa nogi:\n");
    printVector(start_leg_pos);

    printf("katy poczatkowe:\n");
    printVector(start_leg_angles);

    printf("jakobian odwrotny:\n");
    printMatrix(inversed_jacobian);

    return 0;
}