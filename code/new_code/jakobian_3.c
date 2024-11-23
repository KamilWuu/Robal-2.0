#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>

#include "kinematics.h"
#include "defines.h"
#include "robot.h"
#include "jakobian.h"
#include "data_structures.h"

#include <stdio.h>

// Zakładam, że struktury i funkcje typu Vector3, Matrix3, printVector, itp. są już zadeklarowane

int main()
{
    Vector3 q_delta = {0, 0, 0};
    Vector3 start_pos, start_angles_rad, start_angles_deg;
    Vector3 actual_q;
    double delta_time = 0.001; // s
    double move_time = 1;      // s
    double t = 0;
    double period = 2 * move_time;

    LegType leg_type = RIGHT_FRONT;
    RobotSide robot_side;

    if (leg_type < 3)
    {
        robot_side = LEFT;
    }
    else
    {
        robot_side = RIGHT;
    }

    // Konfiguracja początkowych prędkości liniowych dla nóg
    Vector3 d_p;

    d_p.data[0] = 0; // mm/s
    d_p.data[1] = -80;
    d_p.data[2] = 0;

    Robot hexapod;

    initRobot(&hexapod);
    initLegPositionRobotCenter(&hexapod, leg_type, 245, y_const + 40, -50);

    start_pos = hexapod._LegsPositionRobotCenter[leg_type];

    double arc = 500;
    double robot_speed = 0;

    d_p = calculateLegVelocity(start_pos, robot_speed, arc);

    start_angles_rad = hexapod._legs[leg_type]._leg_joint_angles;
    start_angles_deg = vectorMultiplyByConst(start_angles_rad, RAD2DEG);

    printVector("pozycja startowa: ", start_pos);

    printVector("katy startowe rad: ", start_angles_rad);

    printVector("katy startowe deg: ", start_angles_deg);

    /*q_delta = calculateDeltaQ(leg_type, robot_side, start_angles_rad, t, delta_time, start_pos);

    printVector("q_delta", q_delta);*/

    printf("=========== ROZPOCZĘCIE SYMULACJI RUCHU ===========\n");
    Vector3 actual_pos;
    actual_q = start_angles_rad;
    actual_pos = getPositionFromAngles(leg_type, robot_side, actual_q);

    Vector3 curve_t, curve_t_delta_t;
    int iteration_count = 0;
    for (int j = 0; j < 5; j++)
    {
        // Pętla czasowa - symulacja ruchu
        unsigned long interval_ms = (unsigned long)(delta_time * 1000);
        unsigned long start_time = millis();
        unsigned long previous_time = start_time;
        unsigned long target_duration_ms = (unsigned long)(period * 1000);

        t = 0;
        int x = 1;
        printf("FAZA RETRAKCJI: \n");
        do
        {
            unsigned long current_time = millis();

            if (current_time - previous_time >= interval_ms)
            {
                previous_time = current_time;
                iteration_count++;

                /**
                 * Petla wykonująca się co delta_time
                 */
                if (t < move_time)
                {
                    curve_t = vectorMultiplyByConst(d_p, t);
                    curve_t_delta_t = vectorMultiplyByConst(d_p, t + delta_time);
                }
                else
                {
                    if (x)
                    {
                        printf("FAZA PROTRAKCJI: \n");
                        x = 0;
                    }
                    curve_t = makeProtractionCurve(d_p, t, period);
                    curve_t_delta_t = makeProtractionCurve(d_p, t + delta_time, period);
                }
                q_delta = calculateDeltaQ(leg_type, robot_side, actual_q, t, delta_time, curve_t, curve_t_delta_t);
                actual_q = vectorAdd(actual_q, q_delta);
                actual_pos = getPositionFromAngles(leg_type, robot_side, actual_q);
                printTwoVectors("actual_angles_deg", vectorMultiplyByConst(actual_q, RAD2DEG), "actual_pos", actual_pos);
                t += delta_time;
            }

        } while (millis() - start_time < target_duration_ms);
        actual_q = start_angles_rad;
        actual_pos = getPositionFromAngles(leg_type, robot_side, actual_q);

        printTwoVectors("ustawiam pozycjcje pocz angles", vectorMultiplyByConst(actual_q, RAD2DEG), "ustawiam pozycje pocz", actual_pos);
    }
    printf("liczba iteracji:%d\n", iteration_count);

    actual_q = start_angles_rad;
    actual_pos = getPositionFromAngles(leg_type, robot_side, actual_q);

    printTwoVectors("pocz", vectorMultiplyByConst(actual_q, RAD2DEG), "ustawiam pozycje pocz", actual_pos);

    return 0;
}
