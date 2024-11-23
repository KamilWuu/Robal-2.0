#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>

#include "kinematics.h"
#include "defines.h"
#include "robot.h"
#include "jakobian.h"
#include "data_structures.h"
#include "velocity.h"
#include <stdio.h>

int main()
{
    Vector3 start_pos_left_front = {-250, y_const + 40, -50};
    Vector3 start_pos_right_front = {250, y_const + 40, -50};

    Vector3 start_pos_left = {-250, 40, -50};
    Vector3 start_pos_right = {250, 40, -50};

    Vector3 start_pos_left_back = {-250, -y_const + 40, -50};
    Vector3 start_pos_right_back = {250, -y_const + 40, -50};

    double arc_radius = -1000;

    double robot_speed = 80;

    Vector3 leg_speed_left, leg_speed_right, leg_speed_left_front, leg_speed_right_front, leg_speed_left_back, leg_speed_right_back;

    leg_speed_left_front = calculateLegVelocity(start_pos_left_front, robot_speed, arc_radius);
    leg_speed_right_front = calculateLegVelocity(start_pos_right_front, robot_speed, arc_radius);

    leg_speed_left = calculateLegVelocity(start_pos_left, robot_speed, arc_radius);
    leg_speed_right = calculateLegVelocity(start_pos_right, robot_speed, arc_radius);

    leg_speed_left_back = calculateLegVelocity(start_pos_left_back, robot_speed, arc_radius);
    leg_speed_right_back = calculateLegVelocity(start_pos_right_back, robot_speed, arc_radius);

    printVector("\nleg_speed_left_front: ", leg_speed_left_front);
    printVector("leg_speed_right_front: ", leg_speed_right_front);
    printVector("leg_speed_left: ", leg_speed_left);
    printVector("leg_speed_right: ", leg_speed_right);
    printVector("leg_speed_left_back: ", leg_speed_left_back);
    printVector("leg_speed_right_back: ", leg_speed_right_back);
    Vector3 pos_left;
    Vector3 pos_right;

    pos_left = start_pos_left;
    pos_right = start_pos_right;

    // for (int i = 0; i < 10; i++)
    // {

    //     pos_left = vectorAdd(pos_left, leg_speed_left);
    //     pos_right = vectorAdd(pos_right, leg_speed_right);

    //     // leg_speed_left = calculateLegVelocity(pos_left, robot_speed, arc_radius);
    //     // leg_speed_right = calculateLegVelocity(pos_right, robot_speed, arc_radius);

    //     printTwoVectors("pos_left: ", pos_left, "pos_right: ", pos_right);
    //     // printTwoVectors("speed_left", leg_speed_left, "speed_roght", leg_speed_right);
    // }

    return 0;
}
