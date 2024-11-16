    #ifndef GLOBALCOORDSYS_H
#define GLOBALCOORDSYS_H

#include "robot.h"
#include "transformMatrix.h"

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
    Robot _robot;
    PositionVector _robot_center_global;
    RobotRotationsAngles _robot_angles; //in degrees
    PositionVector _robot_legs_positions_global[6];
    PositionVector _robot_legs_positions_arc_center[6];

    double _arc_radius;
    double _robot_speed;

    PositionVector _robot_center_arc_center;
    PositionVector _arc_center_global; 
    
};

void setLegPositionsGlobal(GlobalCoordSys *global_cord_sys, int leg, PositionVector position ){
    global_cord_sys->_robot_legs_positions_global[leg] = position;
}


void initLegsPositionsGlobal(GlobalCoordSys *global_cord_sys){
    PositionVector positions[6];

    positions[LEFT_FRONT] = setPositionVector(-x_const, y_const, z_const_zero);
    positions[LEFT_MIDDLE] = setPositionVector(-x_const, 0, z_const_zero);
    positions[LEFT_FRONT] = setPositionVector(-x_const, -y_const, z_const_zero);
    positions[RIGHT_FRONT] = setPositionVector(x_const, y_const, z_const_zero);
    positions[RIGHT_MIDDLE] = setPositionVector(x_const, 0, z_const_zero);
    positions[RIGHT_BACK] = setPositionVector(x_const, -y_const, z_const_zero);

    for(int i = 0; i < 6; i++){
        setLegPositionsGlobal(global_cord_sys, i, positions[i]);
    }


}




void calculateLegsPositionsFromGlobalToRobotCenter(GlobalCoordSys *global_cord_sys){
    PositionVector legs_positions_global[6];   

    for (int i = 0; i < 6; i++) {
        legs_positions_global[i] = global_cord_sys->_robot_legs_positions_global[i]; //<- pozycje globalne
    }        

    TransformMatrix transform_matrix; 
    transform_matrix = createTransformMatrix(global_cord_sys._robot_angles, _robot_center_global);
    for(int i = 0; i < 6, i++ ){
       global_cord_sys->_robot._LegsPositionRobotCenter[i] = inverseTransformVector(legs_positions_global[i], transform_matrix); 
       //ustawienie pozycji globalnych na pozycje wzgledem srodka robota 
    }

}

void calculateLegsPositionsFromGlobalToArcCenter(GlobalCoordSys *global_cord_sys){

    for(int i = 0; i < 6; i++){
        global_cord_sys->_robot_legs_positions_arc_center[i].P_x = global_cord_sys->_robot_legs_positions_global[i].P_x - global_cord_sys->_arc_center_global.P_x;
        global_cord_sys->_robot_legs_positions_arc_center[i].P_y = global_cord_sys->_robot_legs_positions_global[i].P_y - global_cord_sys->_arc_center_global.P_y;
        global_cord_sys->_robot_legs_positions_arc_center[i].P_z = global_cord_sys->_robot_legs_positions_global[i].P_z;
    }
}

void calculateLegsPositionsFromArcCenterToRobotCenter(GlobalCoordSys *global_cord_sys){
    PositionVector legs_positions_arc_center[6];   

    for (int i = 0; i < 6; i++) {
        legs_positions_arc_center[i] = global_cord_sys->_robot_legs_positions_arc_center[i]; //<- pozycje wzgledem srodka luku
    }        

    TransformMatrix transform_matrix; 
    transform_natrix = createTransformMatrix(global_cord_sys->_robot_angles, _robot_center_arc_center);
    for(int i = 0; i < 6, i++ ){
       global_cord_sys->_robot._LegsPositionRobotCenter[i] = inverseTransformVector(legs_positions_arc_center[i], transform_matrix); 
       //ustawienie pozycji globalnych na pozycje wzgledem srodka robota 
    }

}

void setArcRadius(GlobalCoordSys *global_cord_sys, double arc_radius){
    global_cord_sys->_arc_radius = arc_radius;
}

void setRobotSpeed(GlobalCoordSys *global_cord_sys, double robot_speed){
    global_cord_sys->_robot_speed = robot_speed;
}


void calculateArcCenterGlobal(GlobalCoordSys *global_cord_sys){
    global_cord_sys->_arc_center_global.P_x = _robot_center_global.P_x - (_arc_radius * cos(_robot_angles._rot_Z*DEG2RAD));
    global_cord_sys->_arc_center_global.P_y = _robot_center_global.P_y - (_arc_radius * sin(_robot_angles._rot_Z*DEG2RAD));
    global_cord_sys->_arc_center_global.P_z = _robot_center_global.P_z;
}

void calculateRobotCenterArcCenter(GlobalCoordSys *global_cord_sys){
    global_cord_sys->_robot_center_arc_center.P_x = _robot_center_global.P_x - global_cord_sys->_arc_center_global.P_x; 
    global_cord_sys->_robot_center_arc_center.P_y = _robot_center_global.P_y - global_cord_sys->_arc_center_global.P_y; 
    global_cord_sys->_robot_center_arc_center.P_z = _robot_center_global.P_z
}


void initGlobalSys(GlobalCoordSys *global_cord_sys){

    initRobot(global_cord_sys->_robot);
    global_cord_sys->_robot_center_global = setPositionVector(START_X,START_Y,START_Z);
    global_cord_sys->_robot_angles = setRotationsAngles(START_ROT_X,START_ROT_Y,START_ROT_Z);

    initLegsPositionsGlobal(global_cord_sys);
    calculateLegsPositionsFromGlobalToRobotCenter(global_cord_sys);
    calculateLegsPositionsFromGlobalToArcCenter(global_cord_sys)

    setArcRadius(global_cord_sys, START_RADIUS);
    setRobotSpeed(global_cord_sys, START_SPEED);

    calculateArcCenterGlobal(global_cord_sys);
    calculateRobotCenterArcCenter(global_cord_sys);

}


#endif // GLOBALCOORDSYS_H