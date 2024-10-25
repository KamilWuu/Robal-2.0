#ifndef MOVES_H
#define MOVES_H

#include "robot.h"
#include "kinematics.h"

void setWalkingPosition(Robot * Hexapod, double x_pos, double y_pos, double z_pos){
    int delay_time = 200;
    evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos, 0 );
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0, 0 );
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos, 0 );
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos, 0 );
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0, 0 );
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos, 0 );
}

void standUp(Robot * Hexapod, double x_pos, double y_pos, double y, int * z, int z_move){
    for(int i = 0; i < z_move; i++){
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos - y, *z );
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0 - y, *z );
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos - y, *z );
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - y, *z );
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - y, *z );
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - y, *z );

        *z = *z - 1;
        delay(1);
    }
}

void SitDown(Robot * Hexapod, double x_pos, double y_pos, double y, int * z, int z_move){
    for(int i = 0; i < z_move; i++){
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos - y, *z );
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0 - y, *z );
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos - y, *z );
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - y, *z );
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - y, *z );
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - y, *z );

        *z = *z + 1;
        delay(1);
    }
}


void RobotForward(Robot * Hexapod, double x_pos, double y_pos, int * y, double z, int y_move){
    for(int i = 0; i < y_move; i++){
            evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0 - *y, z );
            evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - *y, z );

            *y = *y + 1;
            delay(1);
        }
}

void RobotBack(Robot * Hexapod, double x_pos, double y_pos, int * y, double z, int y_move){
    for(int i = 0; i < y_move; i++){
            evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0 - *y, z );
            evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - *y, z );

            *y = *y - 1;
            delay(1);
        }
}



//MOVE POSES

// POSE: 0
void setPoseZero(Robot * Hexapod, double x_pos, double y_pos, double z){


    evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos, z );
    evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0, z );
    evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos, z );
    evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos, z );
    evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0, z );
    evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos, z );

}

/*void setPoseOne(Robot * Hexapod){
    int z_move = Hexapod->_legs[0]._current_pos[2];
    for(int i = 0; i < z_move; i++){
            evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, - x_pos, y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, - x_pos, 0 - *y, z );
            evaluatePositionRobotCenter(Hexapod, LEFT_BACK, - x_pos, -y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - *y, z );
            evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - *y, z );

            *y = *y - 1;
            delay(1);
        }
}*/


/* przykladowo actual pos y = 60, target pos = 90, wiec move = 30 */
#endif // MOVES_H