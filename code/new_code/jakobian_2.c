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

    Vector3 q;

    Vector3 d_p, d_q;

    d_p.data[0]= 0; //mm/s
    d_p.data[1]= 0;
    d_p.data[2]= 0;

    double delta_time = 1; //s
    double time = 10;      //s

    int num_steps = (int)time/delta_time;





    start_leg_pos = hexapod._LegsPositionRobotCenter[LEFT_MIDDLE];
    start_leg_angles = hexapod._legs[LEFT_MIDDLE]._leg_joint_angles;

    for(int i = 0; i < 3; i ++){
        q.data[i] = start_leg_angles.data[i];
    }



    inversed_jacobian = createInversedJacobian(start_leg_angles);

    printf("pozycja startowa nogi:\n");
    printVector(start_leg_pos);

    printf("katy poczatkowe:\n");
    printVector(start_leg_angles);

    printf("jakobian odwrotny:\n");
    printMatrix(inversed_jacobian);



    printf("linear velocities:\n");
    printVector(d_p);


    d_q = multiplyMatrixByVector(inversed_jacobian, d_p);
    
    printf("joint velocities:\n");
    printVector(d_q);


    
    for(int i = 0; i < num_steps; i++){

        if(/*sprawdzaj czy minal odpowiedni czas*/){ 
            for(int j = 0; j < 3; j++){
                q.data[j] += d_q.data[j];  
                //aktualizowanie kątów na serwach[j]
            } 
            
        }
    }


    printf("katy po wykonaniu ruchu:\n");
    printVector(q);


    return 0;
}