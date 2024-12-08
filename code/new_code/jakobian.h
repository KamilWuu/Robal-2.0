#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "data_structures.h"
#include "enums.h"
#include "math.h"
#include "kinematics.h"

Vector3 getRobotCenterPositionFromAngles(LegType leg_type, RobotSide side, Vector3 angles)
{
    double Q1 = angles.data[0];

    double Q2 = angles.data[1];
    double Q3 = angles.data[2];

    Vector3 position;

    // Oblicz współrzędne końcówki nogi w układzie odniesienia nogi
    // double Xpz = L1 + L2 * cos(Q2) + L3 * cos(Q2 + Q3);
    // double Xp = Xpz * cos(Q1);
    // double Yp = Xpz * sin(Q1);
    // double Zp = L2 * sin(Q2) + L3 * sin(Q2 + Q3);

    double c1 = cos(Q1);
    double s1 = sin(Q1);

    double s2 = sin(Q2);
    double c2 = cos(Q2);

    double s23 = sin(Q2 + Q3);
    double c23 = cos(Q2 + Q3);

    // kinematyka z denavita hartenberga
    double Xp = c1 * (c23 * L3 + c2 * L2 + L1);
    double Yp = s1 * (c23 * L3 + c2 * L2 + L1);
    double Zp = (s23 * L3 + s2 * L2);

    // // Dopasuj znak osi X zależnie od strony robota
    // if (side == LEFT)
    // {
    //     Xp = -Xp;
    // }

    // Przekształcenie współrzędnych nogi na globalne współrzędne robota
    switch (leg_type)
    {
    case LEFT_FRONT:
        position.data[0] = Xp - d1;
        position.data[1] = Yp + d3;
        position.data[2] = Zp - z_0;
        break;
    case LEFT_MIDDLE:
        position.data[0] = Xp - d2;
        position.data[1] = Yp;
        position.data[2] = Zp - z_0;
        break;
    case LEFT_BACK:
        position.data[0] = Xp - d1;
        position.data[1] = Yp - d3;
        position.data[2] = Zp - z_0;
        break;
    case RIGHT_FRONT:
        position.data[0] = Xp + d1;
        position.data[1] = Yp + d3;
        position.data[2] = Zp - z_0;
        break;
    case RIGHT_MIDDLE:
        position.data[0] = Xp + d2;
        position.data[1] = Yp;
        position.data[2] = Zp - z_0;
        break;
    case RIGHT_BACK:
        position.data[0] = Xp + d1;
        position.data[1] = Yp - d3;
        position.data[2] = Zp - z_0;
        break;
    default:
        printf("Błędna pozycja nogi\n");
        global_error++;
        break;
    }

    return position;
}

Matrix3 createJacobian(double q1, double q2, double q3, double k)
{ // kąty w radianach

    Matrix3 jacobian;

    double l1 = L1;
    double l2 = L2;
    double l3 = L3;

    double s1 = sin(q1);
    double c1 = cos(q1);

    double s2 = sin(q2);
    double c2 = cos(q2);

    double s3 = sin(q3);
    double c3 = cos(q3);

    double s23 = sin(q2 + q3);
    double c23 = cos(q2 + q3);

    double a = c23 * l3 + c2 * l2 + l1;
    double b = s23 * l3 + s2 * l2;

    jacobian.data[0][0] = -k * s1 * a;
    jacobian.data[0][1] = -k * c1 * b;
    jacobian.data[0][2] = -k * c1 * s23 * l3;
    jacobian.data[1][0] = k * c1 * a;
    jacobian.data[1][1] = -k * s1 * b;
    jacobian.data[1][2] = -k * s1 * s23 * l3;
    jacobian.data[2][0] = 0;
    jacobian.data[2][1] = -c23 * l3 - c2 * l2;
    jacobian.data[2][2] = -c23 * l3;

    return jacobian;
}

Matrix3 createInversedJacobian(Vector3 initial_angles)
{

    Matrix3 inversed_jacobian;

    double l1 = L1;
    double l2 = L2;
    double l3 = L3;

    double q1 = initial_angles.data[0];
    double q2 = initial_angles.data[1];
    double q3 = initial_angles.data[2];

    double s1 = sin(q1);
    double c1 = cos(q1);

    double s2 = sin(q2);
    double c2 = cos(q2);

    double s3 = sin(q3);
    double c3 = cos(q3);

    double s23 = sin(q2 + q3);
    double c23 = cos(q2 + q3);

    double csc3;

    if (s3 != 0)
    {
        csc3 = 1 / s3;
    }
    else
    {
        printf("nie wiem co zrobic, pojawilo sie dzielenie przez 0\n");
        exit(0);
    }

    double a = (l2 * c2) + (l3 * c23);

    inversed_jacobian.data[0][0] = (-s1) / (l1 + a);
    inversed_jacobian.data[0][1] = (c1) / (l1 + a);
    inversed_jacobian.data[0][2] = 0;
    inversed_jacobian.data[1][0] = (c1 * c23 * csc3) / (l2);
    inversed_jacobian.data[1][1] = (s1 * c23 * csc3) / (l2);
    inversed_jacobian.data[1][2] = (-s23 * csc3) / (l2);
    inversed_jacobian.data[2][0] = (-c1 * csc3 * a) / (l2 * l3);
    inversed_jacobian.data[2][1] = (-s1 * csc3 * a) / (l2 * l3);
    inversed_jacobian.data[2][2] = csc3 * ((s2 / l3) + (s23 / l2));

    return inversed_jacobian;
}

Vector3 makeProtractionCurve(Vector3 dp, double t, double period, double v, double *vz_out)
{

    Vector3 result;

    result = vectorMultiplyByConst(dp, t);
    // result.data[X] = result.data[X] * -1;
    // result.data[Y] = result.data[Y] * -1;

    if (v != 0)
    {
        result.data[Z] = sin((MY_PI * (t - (period / 2))) / (period / 2)) * h_const;
        // Obliczanie prędkości w Z
        *vz_out = -cos((MY_PI * (t - (period / 2))) / (period / 2)) * (2 * MY_PI / period) * h_const;
    }
    else
    {
        result.data[Z] = 0;
        *vz_out = 0.0;
    }
    // printf("Z curve: %.2f", result.data[Z]);

    return result;
}

Vector3 calculateDeltaQ(LegType leg_type, RobotSide robot_side, Vector3 q_actual, double t, double delta_t, Vector3 curve_t, Vector3 curve_t_delta_t)
{
    // Oblicz aktualną pozycję końcówki nogi na podstawie kątów przegubów
    Vector3 p_actual = getRobotCenterPositionFromAngles(leg_type, robot_side, q_actual);

    // Oblicz przewidywane pozycje trajektorii w chwili t i t + delta_t
    Vector3 y_tray_t = vectorAdd(p_actual, curve_t);
    Vector3 y_tray_t_delta_t = vectorAdd(p_actual, curve_t_delta_t);

    // Oblicz różnicę pozycji trajektorii
    Vector3 delta_p = vectorSub(y_tray_t_delta_t, y_tray_t);

    // Utwórz odwróconą macierz Jacobiego na podstawie aktualnych kątów przegubów
    Matrix3 inversed_jacobian = createInversedJacobian(q_actual);

    // Oblicz różnicę kątów przegubów
    Vector3 delta_q = multiplyMatrixByVector(inversed_jacobian, delta_p);

    return delta_q;
}

#endif // JACOBIAN.H