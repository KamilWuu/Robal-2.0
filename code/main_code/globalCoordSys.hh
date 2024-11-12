#ifndef GLOBALCOORDSYS_H
#define GLOBALCOORDSYS_H

#include "robot.h"

typedef struct TransformMatrix
{
    double RotationMatrix_X[3][3];
};



typedef struct RobotRotationsAngles
{
    double _rot_X;
    double _rot_Y;
    double _rot_Z;
};

typedef struct ArcCenterCoordSys
{
    PositionVector LegPositionsArcCenter[6];

};


typedef struct GlobalCoordSys
{
    RobotRotationsAngles _robot_angles;
    Robot _robot;

};




#endif // GLOBALCOORDSYS_H