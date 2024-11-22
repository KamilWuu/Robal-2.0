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
    LegType leg_type = RIGHT_MIDDLE;
    RobotSide leg_side = RIGHT;
    Robot hexapod;
    initRobot(&hexapod);

    Vector3 start_leg_pos, start_leg_angles, q, d_p, d_q, delta_dp, delta_dq, delta_dq_deg, start_leg_angles_deg, d_q_deg;
    Matrix3 inversed_jacobian;
    Vector3 measured_distance = {0, 0, 0};

    double delta_time = 0.001; // s
    double total_time = 0.5;   // s

    // Konfiguracja początkowych prędkości liniowych dla nóg
    d_p.data[0] = 0; // mm/s
    d_p.data[1] = -160;
    d_p.data[2] = 0;

    printf("=========== PARAMETRY POCZĄTKOWE ===========\n");
    printf("delta_time = %.4f s, total_time = %.2f s\n", delta_time, total_time);
    printf("Prędkości liniowe (d_p): ");
    printVector(d_p);
    printf("\n\n");

    initLegPositionRobotCenter(&hexapod, leg_type, 220, 0, 0);

    start_leg_pos = hexapod._LegsPositionRobotCenter[leg_type];
    start_leg_angles = hexapod._legs[leg_type]._leg_joint_angles;
    Vector3 calculated_position_1 = getPositionFromAngles(leg_type, leg_side, start_leg_angles);
    // Ustawienie wartości startowych dla pozycji i kątów
    for (int i = 0; i < 3; i++)
    {
        q.data[i] = start_leg_angles.data[i];
        delta_dp.data[i] = d_p.data[i] * delta_time;
        start_leg_angles_deg.data[i] = start_leg_angles.data[i] * RAD2DEG;
    }

    inversed_jacobian = createInversedJacobian(start_leg_angles);

    printf("=========== POZYCJE STARTOWE ===========\n");
    printf("Pozycja startowa nogi: ");
    printVector(start_leg_pos);
    printf("Pozycja startowa nogi wyliczona: ");
    printVector(calculated_position_1);
    printf("\nKąty początkowe (w stopniach): ");
    printVector(start_leg_angles_deg);
    printf("\n\n");

    printf("=========== OBLICZENIA PRĘDKOŚCI ===========\n");
    printf("Delta prędkości liniowe (delta_dp): ");
    printVector(delta_dp);
    printf("\n");

    d_q = multiplyMatrixByVector(inversed_jacobian, d_p);
    // delta_dq = multiplyMatrixByVector(inversed_jacobian, delta_dp);

    for (int i = 0; i < 3; i++)
    {

        d_q_deg.data[i] = d_q.data[i] * RAD2DEG;
        delta_dq.data[i] = d_q.data[i] * delta_time; //
        delta_dq_deg.data[i] = delta_dq.data[i] * RAD2DEG;
    }

    printf("Prędkości stawów (d_q) w stopniach: ");
    printVector(d_q_deg);
    printf("\nDelta prędkości stawów (delta_dq) w stopniach: ");
    printVector(delta_dq_deg);
    printf("\n\n");

    // Pętla czasowa - symulacja ruchu
    unsigned long interval_ms = (unsigned long)(delta_time * 1000);
    unsigned long start_time = millis();
    unsigned long previous_time = start_time;
    unsigned long target_duration_ms = (unsigned long)(total_time * 1000);
    int iteration_count = 0;

    printf("=========== ROZPOCZĘCIE SYMULACJI RUCHU ===========\n");

    do
    {
        unsigned long current_time = millis();

        if (current_time - previous_time >= interval_ms)
        {
            previous_time = current_time;

            for (int j = 0; j < 3; j++)
            {
                q.data[j] += delta_dq.data[j];

                measured_distance.data[j] += delta_dp.data[j];
            }
            iteration_count++;
        }

    } while (millis() - start_time < target_duration_ms);

    // Obliczanie kąty końcowe
    Vector3 q_deg;
    for (int i = 0; i < 3; i++)
    {
        q_deg.data[i] = q.data[i] * RAD2DEG;
    }

    printf("=========== WYNIKI KOŃCOWE ===========\n");
    printf("Kąty po wykonaniu ruchu (w stopniach): ");
    printVector(q_deg);
    printf("\nPrzebyty dystans: ");
    printVector(measured_distance);
    printf("\n");

    // Obliczenia końcowe i błędy
    Vector3 calculated_position = getPositionFromAngles(leg_type, leg_side, q);
    Vector3 end_position, pos_error;

    for (int i = 0; i < 3; i++)
    {
        end_position.data[i] = measured_distance.data[i] + start_leg_pos.data[i];

        pos_error.data[i] = end_position.data[i] - calculated_position.data[i];
    }

    printf("Pozycja obliczona z kątów końcowych: ");
    printVector(calculated_position);
    printf("\nPozycja żądana (end_position): ");
    printVector(end_position);
    printf("\nBłąd pozycji: ");
    printVector(pos_error);
    printf("\n\n");

    initLegPositionRobotCenter(&hexapod, leg_type, end_position.data[0], end_position.data[1], end_position.data[2]);

    Vector3 end_leg_angles = hexapod._legs[leg_type]._leg_joint_angles;
    Vector3 angles_error;

    for (int i = 0; i < 3; i++)
    {
        end_leg_angles.data[i] *= RAD2DEG;
        angles_error.data[i] = end_leg_angles.data[i] - q_deg.data[i];
    }

    printf("Obliczone kąty z żądanej pozycji: ");
    printVector(end_leg_angles);
    printf("\nKąty końcowe (po ruchu): ");
    printVector(q_deg);
    printf("\nBłąd kątów: ");
    printVector(angles_error);
    printf("\n");

    printf("=========== PODSUMOWANIE ===========\n");
    printf("Liczba iteracji: %d\n", iteration_count);

    return 0;
}
