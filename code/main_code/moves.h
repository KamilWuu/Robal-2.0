#ifndef MOVES_H
#define MOVES_H

#include "robot.h"
#include "kinematics.h"

void setWalkingPosition(Robot *Hexapod, double x_pos, double y_pos, double z_pos)
{
    int delay_time = 200;
    evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_pos, y_pos, 0);
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_pos, 0, 0);
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_pos, -y_pos, 0);
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos, 0);
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0, 0);
    delay(delay_time);
    evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos, 0);
}

void standUp(Robot *Hexapod, double x_pos, double y_pos, double y, int *z, int z_move)
{
    for (int i = 0; i < z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_pos, y_pos - y, *z);
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_pos, 0 - y, *z);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_pos, -y_pos - y, *z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - y, *z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - y, *z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - y, *z);

        *z = *z - 1;
        delay(1);
    }
}

void SitDown(Robot *Hexapod, double x_pos, double y_pos, double y, int *z, int z_move)
{
    for (int i = 0; i < z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_pos, y_pos - y, *z);
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_pos, 0 - y, *z);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_pos, -y_pos - y, *z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - y, *z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - y, *z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - y, *z);

        *z = *z + 1;
        delay(1);
    }
}

void RobotForward(Robot *Hexapod, double x_pos, double y_pos, int *y, double z, int y_move)
{
    for (int i = 0; i < y_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_pos, y_pos - *y, z);
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_pos, 0 - *y, z);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_pos, -y_pos - *y, z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - *y, z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - *y, z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - *y, z);

        *y = *y + 1;
        delay(1);
    }
}

void RobotBack(Robot *Hexapod, double x_pos, double y_pos, int *y, double z, int y_move)
{
    for (int i = 0; i < y_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_pos, y_pos - *y, z);
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_pos, 0 - *y, z);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_pos, -y_pos - *y, z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos - *y, z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0 - *y, z);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos - *y, z);

        *y = *y - 1;
        delay(1);
    }
}

// MOVE POSES

// POSE: 0
void setPoseZero(Robot *Hexapod, double x_pos, double y_pos, double z)
{

    evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_pos, y_pos, z);
    evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_pos, 0, z);
    evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_pos, -y_pos, z);
    evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_pos, y_pos, z);
    evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_pos, 0, z);
    evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_pos, -y_pos, z);
}

// POSE: 1
void setPoseOne(Robot *Hexapod, int z_variable, float speed_factor)
{
    int z_move = Hexapod->_legs[0]._current_pos[2] / z_variable;
    //printf("zmienna= %2.f\n", Hexapod->_legs[0]._current_pos[2]);
    //printf("z move = %d\n", z_move);
    printf("setPoseOne\n");
    for (int i = 0; i < (-1) * z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_const, y_const, Hexapod->_legs[LEFT_FRONT]._current_pos[2] + z_variable);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_const, -y_const, Hexapod->_legs[LEFT_BACK]._current_pos[2] + z_variable);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_const, 0, Hexapod->_legs[RIGHT_MIDDLE]._current_pos[2] + z_variable);

        delay(1 / speed_factor); // Skróć czas opóźnienia zależnie od prędkości
    }
}

int max(int a, int b)
{
    return (a > b) ? a : b;
}

void setPoseTwo(Robot *Hexapod, int delta_y, float velocity_mm_per_ms)
{
    printf("setPoseTwo\n");

    // Pobierz aktualne pozycje nóg w osi Y
    int start_y_left_front = Hexapod->_legs[LEFT_FRONT]._current_pos[1] + d3;
    int start_y_left_back = Hexapod->_legs[LEFT_BACK]._current_pos[1] - d3;
    int start_y_right_middle = Hexapod->_legs[RIGHT_MIDDLE]._current_pos[1];

    // Oblicz docelowe pozycje jako przesunięcie relatywne
    int target_y_left_front = start_y_left_front + delta_y;
    int target_y_left_back = start_y_left_back + delta_y;
    int target_y_right_middle = start_y_right_middle + delta_y;

    // Oblicz dystans do przemieszczenia dla każdej nogi
    int distance_left_front = abs(target_y_left_front - start_y_left_front);
    int distance_left_back = abs(target_y_left_back - start_y_left_back);
    int distance_right_middle = abs(target_y_right_middle - start_y_right_middle);

    // Określ liczbę kroków na podstawie maksymalnego dystansu i prędkości
    int max_distance = max(distance_left_front, max(distance_left_back, distance_right_middle));
    int steps = max_distance / velocity_mm_per_ms;

    // Przemieść każdą nogę o interpolowaną wartość
    for (int i = 0; i <= steps; i++)
    {
        float t = (float)i / (float)steps;

        // Oblicz nową pozycję dla każdej nogi na podstawie przesunięcia
        int interpolated_y_left_front = start_y_left_front + (int)(t * delta_y);
        int interpolated_y_left_back = start_y_left_back + (int)(t * delta_y);
        int interpolated_y_right_middle = start_y_right_middle + (int)(t * delta_y);

        // Aktualizuj pozycję nóg w osi Y, pozostałe osie bez zmian
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_const, interpolated_y_left_front, Hexapod->_legs[LEFT_FRONT]._current_pos[2]);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_const, interpolated_y_left_back, Hexapod->_legs[LEFT_BACK]._current_pos[2]);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_const, interpolated_y_right_middle, Hexapod->_legs[RIGHT_MIDDLE]._current_pos[2]);

        // Czas opóźnienia kontrolujący płynność
        delay(1);
    }
}

// POSE: 1
void setPoseThree(Robot *Hexapod, int z_variable, float speed_factor)
{
    printf("setPoseThree\n");

    int z_move = Hexapod->_legs[0]._target_pos[2] / z_variable;
    z_move = 80;
    //printf("zmienna3= %2.f\n", Hexapod->_legs[0]._target_pos[2]);
    //printf("z move3 = %d\n", z_move);

    for (int i = 0; i < z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_const, Hexapod->_legs[LEFT_FRONT]._current_pos[1] + d3, Hexapod->_legs[LEFT_FRONT]._target_pos[2] - z_variable);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_const, Hexapod->_legs[LEFT_BACK]._current_pos[1] - d3, Hexapod->_legs[LEFT_BACK]._target_pos[2] - z_variable);
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_const, Hexapod->_legs[RIGHT_MIDDLE]._current_pos[1], Hexapod->_legs[RIGHT_MIDDLE]._target_pos[2] - z_variable);

        delay(1 / speed_factor); // Skróć czas opóźnienia zależnie od prędkości
    }
}

// POSE: 1
void setPoseFour(Robot *Hexapod, int z_variable, float speed_factor)
{
    printf("setPoseFour\n");

    int z_move = Hexapod->_legs[0]._current_pos[2] / z_variable;
    //printf("zmienna= %2.f\n", Hexapod->_legs[0]._current_pos[2]);
    //printf("z move = %d\n", z_move);

    for (int i = 0; i < (-1) * z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_const, 0, Hexapod->_legs[LEFT_MIDDLE]._current_pos[2] + z_variable);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_const, -y_const, Hexapod->_legs[RIGHT_BACK]._current_pos[2] + z_variable);
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_const, +y_const, Hexapod->_legs[RIGHT_FRONT]._current_pos[2] + z_variable);

        delay(1 / speed_factor); // Skróć czas opóźnienia zależnie od prędkości
    }
}

void setPoseTwoPrim(Robot *Hexapod, int delta_y, float velocity_mm_per_ms)
{

    printf("setPoseTwoPrim\n");

    // Pobierz aktualne pozycje nóg w osi Y
    int start_y_right_front = Hexapod->_legs[RIGHT_FRONT]._current_pos[1] + d3;
    int start_y_right_back = Hexapod->_legs[RIGHT_BACK]._current_pos[1] - d3;
    int start_y_left_middle = Hexapod->_legs[LEFT_MIDDLE]._current_pos[1];

    // Oblicz docelowe pozycje jako przesunięcie relatywne
    int target_y_right_front = start_y_right_front + delta_y;
    int target_y_right_back = start_y_right_back + delta_y;
    int target_y_left_middle = start_y_left_middle + delta_y;

    // Oblicz dystans do przemieszczenia dla każdej nogi
    int distance_right_front = abs(target_y_right_front - start_y_right_front);
    int distance_right_back = abs(target_y_right_back - start_y_right_back);
    int distance_left_middle = abs(target_y_left_middle - start_y_left_middle);

    // Określ liczbę kroków na podstawie maksymalnego dystansu i prędkości
    int max_distance = (distance_right_front > distance_right_back ? (distance_right_front > distance_left_middle ? distance_right_front : distance_left_middle) : (distance_right_back > distance_left_middle ? distance_right_back : distance_left_middle));
    int steps = max_distance / velocity_mm_per_ms;

    // Przemieść każdą nogę o interpolowaną wartość
    for (int i = 0; i <= steps; i++)
    {
        float t = (float)i / (float)steps;

        // Oblicz nową pozycję dla każdej nogi na podstawie przesunięcia
        int interpolated_y_right_front = start_y_right_front + (int)(t * delta_y);
        int interpolated_y_right_back = start_y_right_back + (int)(t * delta_y);
        int interpolated_y_left_middle = start_y_left_middle + (int)(t * delta_y);

        // Aktualizuj pozycję nóg w osi Y, pozostałe osie bez zmian
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_const, interpolated_y_right_front, Hexapod->_legs[RIGHT_FRONT]._current_pos[2]);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_const, interpolated_y_right_back, Hexapod->_legs[RIGHT_BACK]._current_pos[2]);
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_const, interpolated_y_left_middle, Hexapod->_legs[LEFT_MIDDLE]._current_pos[2]);

        // Czas opóźnienia kontrolujący płynność
        delay(1);
    }
}

void setPoseThreePrim(Robot *Hexapod, int z_variable, float speed_factor)
{
    printf("setPoseThreePrim\n");

    int z_move = Hexapod->_legs[0]._target_pos[2] / z_variable;
    z_move = 80;
    //printf("zmienna3= %2.f\n", Hexapod->_legs[0]._target_pos[2]);
    //printf("z move3 = %d\n", z_move);

    // Pętla do przemieszczania nóg po prawej stronie
    for (int i = 0; i < z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, RIGHT_FRONT, x_const, Hexapod->_legs[RIGHT_FRONT]._current_pos[1] + d3, Hexapod->_legs[RIGHT_FRONT]._target_pos[2] - z_variable);
        evaluatePositionRobotCenter(Hexapod, RIGHT_BACK, x_const, Hexapod->_legs[RIGHT_BACK]._current_pos[1] - d3, Hexapod->_legs[RIGHT_BACK]._target_pos[2] - z_variable);
        evaluatePositionRobotCenter(Hexapod, LEFT_MIDDLE, -x_const, Hexapod->_legs[LEFT_MIDDLE]._current_pos[1], Hexapod->_legs[LEFT_MIDDLE]._target_pos[2] - z_variable);

        // Opóźnienie zależne od prędkości
        delay(1 / speed_factor);
    }
}

void setPoseFourPrim(Robot *Hexapod, int z_variable, float speed_factor)
{
    printf("setPoseFourPrim\n");

    int z_move = Hexapod->_legs[0]._current_pos[2] / z_variable;
    //printf("zmienna= %2.f\n", Hexapod->_legs[0]._current_pos[2]);
    //printf("z move = %d\n", z_move);

    // Pętla do przemieszczania nóg po prawej stronie
    for (int i = 0; i < (-1) * z_move; i++)
    {
        evaluatePositionRobotCenter(Hexapod, RIGHT_MIDDLE, x_const, 0, Hexapod->_legs[RIGHT_MIDDLE]._current_pos[2] + z_variable);
        evaluatePositionRobotCenter(Hexapod, LEFT_BACK, -x_const, -y_const, Hexapod->_legs[LEFT_BACK]._current_pos[2] + z_variable);
        evaluatePositionRobotCenter(Hexapod, LEFT_FRONT, -x_const, +y_const, Hexapod->_legs[LEFT_FRONT]._current_pos[2] + z_variable);

        // Opóźnienie zależne od prędkości
        delay(1 / speed_factor);
    }
}















#endif // MOVES_H