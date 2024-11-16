#ifndef GLOBALCOORDSYS_H
#define GLOBALCOORDSYS_H

#include "robot.h"

typedef struct RobotRotationsAngles
{
    double _rot_X;
    double _rot_Y;
    double _rot_Z;
};

typedef struct ArcCenterCoordSys
{
    Robot _robot;
    PositionVector _legs_positions_arc[6];
    PositionVector _robot_center_position_arc;
    RobotRotationsAngles _robot_angles;
    double _arc_radius;
    double _arc_angle;
};

typedef struct GlobalCoordSys
{
    ArcCenterCoordSys _arc_center_coord_sys;
    PositionVector _robot_center_position_global;
    PositionVector _legs_positions_global[6];
    PositionVector _arc_center_position_global;
};




#endif // GLOBALCOORDSYS_H