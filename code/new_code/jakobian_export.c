#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include "kinematics.h"
#include "defines.h"
#include "robot.h"
#include "jakobian.h"
#include "data_structures.h"

// Zakładam, że struktury i funkcje typu Vector3, Matrix3, printVector, itp. są już zadeklarowane

// Funkcja zapisująca wektor do pliku CSV
void writeVectorToCSV(FILE *file, const char *phase, double t, Vector3 q_delta, Vector3 actual_q, Vector3 actual_pos)
{
    fprintf(file, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f \n",
             t,
            q_delta.data[0], q_delta.data[1], q_delta.data[2],
            actual_q.data[0], actual_q.data[1], actual_q.data[2],
            actual_pos.data[0], actual_pos.data[1], actual_pos.data[2]);
}

int main()
{
    // Plik CSV do zapisu
    FILE *file = fopen("robot_motion.csv", "w");
    if (!file)
    {
        perror("Nie udało się otworzyć pliku do zapisu");
        return 1;
    }

    // Nagłówki w pliku CSV
    //fprintf(file, "Phase,t,q_delta_x,q_delta_y,q_delta_z,actual_q_x,actual_q_y,actual_q_z,actual_pos_x,actual_pos_y,actual_pos_z\n");

    Vector3 q_delta = {0, 0, 0};
    Vector3 start_pos, start_angles_rad, start_angles_deg;
    Vector3 actual_q;
    double delta_time = 0.001; // s
    double move_time = 1;     // s
    double t = 0;
    double period = 2 * move_time;

    LegType leg_type = RIGHT_MIDDLE;
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
    initLegPositionRobotCenter(&hexapod, leg_type, 245, 40, -50);

    start_pos = hexapod._LegsPositionRobotCenter[leg_type];
    start_angles_rad = hexapod._legs[leg_type]._leg_joint_angles;
    start_angles_deg = vectorMultiplyByConst(start_angles_rad, RAD2DEG);

    // // Zapis warunków początkowych
    // fprintf(file, "# delta_time: %.4f, move_time: %.4f, period: %.4f\n", delta_time, move_time, period);
    // fprintf(file, "# start_angles_rad: %.4f, %.4f, %.4f\n", start_angles_rad.data[0], start_angles_rad.data[1], start_angles_rad.data[2]);
    // fprintf(file, "# start_pos: %.4f, %.4f, %.4f\n", start_pos.data[0], start_pos.data[1], start_pos.data[2]);
    // fprintf(file, "# d_p: %.4f, %.4f, %.4f\n", d_p.data[0], d_p.data[1], d_p.data[2]);

    /* Pętla czasowa - symulacja ruchu */
    unsigned long interval_ms = (unsigned long)(delta_time * 1000);
    unsigned long start_time = millis();
    unsigned long previous_time = start_time;
    unsigned long target_duration_ms = (unsigned long)(period * 1000);
    int iteration_count = 0;

    printf("=========== ROZPOCZĘCIE SYMULACJI RUCHU ===========\n");
    Vector3 actual_pos;
    actual_q = start_angles_rad;
    actual_pos = getPositionFromAngles(leg_type, robot_side, actual_q);

    Vector3 curve_t, curve_t_delta_t;

    int x = 1;
    printf("FAZA RETRAKCJI: \n");
    do
    {
        unsigned long current_time = millis();

        if (current_time - previous_time >= interval_ms)
        {
            previous_time = current_time;
            iteration_count++;

            /* Pętla wykonująca się co delta_time */
            const char *phase = (t < move_time) ? "RETRAKCJA" : "PROTRAKCJA";

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

            // Zapis danych do pliku CSV
            writeVectorToCSV(file, phase, t, vectorMultiplyByConst(q_delta, RAD2DEG), vectorMultiplyByConst(actual_q, RAD2DEG), actual_pos);

            printTwoVectors("actual_angles_deg", vectorMultiplyByConst(actual_q, RAD2DEG), "actual_pos", actual_pos);
            t += delta_time;
        }

    } while (millis() - start_time < target_duration_ms);

    printf("liczba iteracji:%d\n", iteration_count);

    fclose(file);
    return 0;
}
