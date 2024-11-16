#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>  // dla sleep()
#include <stdio.h>
#include <time.h>


#define L1 73.5
#define L2 84.5
#define L3 118.5
#define L23 70

#define d1 55.0
#define d2 75.0
#define d3 113.25

#define x_const 245.0
#define y_const (d3 + 50.0) // d3
#define z_const_zero 0.0
#define z_const_stand_up (-60.0)

#define MY_PI 3.14159265358979323846

#define DEG2RAD (MY_PI / 180)
#define RAD2DEG (180 / MY_PI)

int global_error = 0;

typedef struct Vector3
{
    double data[3];
}Vector3;

void printVector(Vector3 vector) {
    printf(" = [");

    printf(" %8.3f, ", vector.data[0]);
    printf(" %8.3f, ", vector.data[1]);
    printf(" %8.3f ", vector.data[2]);

    printf("]\n");
    
}

typedef struct Matrix3
{
    double data[3][3];
}Matrix3;

void printMatrix(Matrix3 matrix) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%8.3f ", matrix.data[i][j]);
        }
        printf("\n");
    }
}


// Funkcja mnożąca macierz 3x3 przez wektor 3
Vector3 multiplyMatrixByVector(Matrix3 matrix, Vector3 vector) {
    Vector3 result;

    // Mnożenie macierzy przez wektor
    for (int i = 0; i < 3; i++) {
        result.data[i] = 0;  // Inicjalizujemy elementy wektora wynikowego
        for (int j = 0; j < 3; j++) {
            result.data[i] += matrix.data[i][j] * vector.data[j];
        }
    }

    return result;
}

Matrix3 createJakobian(double q1, double q2, double q3, double l1, double l2, double l3, double k){ //kąty w radianach

    Matrix3 jacobian;



    double s1 = sin(q1);    
    double c1 = cos(q1);



    double s2 = sin(q2);
    double c2 = cos(q2);


    double s3 = sin(q3);
    double c3 = cos(q3);



    double s23 = sin(q2+q3);
    double c23 = cos(q2+q3);


    double a = c23*l3 + c2*l2 + l1;
    double b = s23*l3 + s2*l2;

    jacobian.data[0][0] = -k*s1*a;      jacobian.data[0][1] = -k*c1*b;          jacobian.data[0][2] = -k*c1*s23*l3; 
    jacobian.data[1][0] = k*c1*a;       jacobian.data[1][1] = -k*s1*b;          jacobian.data[1][2] = -k*s1*s23*l3;  
    jacobian.data[2][0] = 0;            jacobian.data[2][1] = -c23*l3 -c2*l2;   jacobian.data[2][2] = -c23*l3;

    return jacobian;
}


bool inverseMatrix(Matrix3 input_matrix, Matrix3 *inversed_matrix) {
    // Tworzymy rozszerzoną macierz [input_matrix | I] dla metody Gaussa-Jordana
    double augmented[3][6];
    
    // Inicjalizacja rozszerzonej macierzy
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            augmented[i][j] = input_matrix.data[i][j];         // Kopiujemy oryginalną macierz
            augmented[i][j + 3] = (i == j) ? 1.0 : 0.0;       // Dodajemy macierz jednostkową
        }
    }
    
    // Eliminacja Gaussa-Jordana
    for (int i = 0; i < 3; i++) {
        // Znajdujemy maksymalny element w kolumnie do pivotowania
        double pivot = augmented[i][i];
        if (pivot == 0) {
            // Jeśli pivot jest 0, macierz jest nieodwracalna
            return false;
        }
        
        // Normalizujemy wiersz pivotowy
        for (int j = 0; j < 6; j++) {
            augmented[i][j] /= pivot;
        }
        
        // Zerujemy elementy w innych wierszach tej kolumny
        for (int k = 0; k < 3; k++) {
            if (k != i) {
                double factor = augmented[k][i];
                for (int j = 0; j < 6; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }
    
    // Kopiujemy prawą stronę rozszerzonej macierzy do macierzy wyjściowej
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inversed_matrix->data[i][j] = augmented[i][j + 3];
        }
    }
    
    return true;
}





// Funkcja do obliczania pozycji końcówki robota (x, y, z) na podstawie kinematyki forward
Vector3 calculateEndEffectorPosition(double q1, double q2, double q3, double l1, double l2, double l3) {
    Vector3 position;
    
    // Kinematyka forward (pozycja końcówki w przestrzeni 3D)
    position.data[0] = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3);  // X
    position.data[1] = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);  // Y
    position.data[2] = 0;  // Z (przy założeniu 2D lub robota płaskiego)

    return position;
}

// Definicja typu dla stron robota (lewa, prawa)
typedef enum
{
    LEFT,
    RIGHT
} RobotSide;

// Definicja typu dla nóg robota
typedef enum
{
    LEFT_FRONT,   // Lewa przednia
    LEFT_MIDDLE,  // Lewa środkowa
    LEFT_BACK,    // Lewa tylna
    RIGHT_FRONT,  // Prawa przednia
    RIGHT_MIDDLE, // Prawa środkowa
    RIGHT_BACK    // Prawa tylna
} LegPosition;

// Funkcja do ustawiania kątów na serwie (symulacja)
void setServoAngle(LegPosition leg, int joint, double angle) {
    // W rzeczywistości ta funkcja by ustawiła odpowiedni kąt na serwie
    // Tutaj po prostu wypisujemy kąt
    //printf("Na nodze %d Ustawiono kąt na serwie %d: %f°\n", leg, joint, angle );
}

void setServosAngles(LegPosition leg, double q1, double q2, double q3){ //angles in radians
    RobotSide side;
    Vector3 servo_angles;

    if((leg == LEFT_BACK) || (leg == LEFT_MIDDLE) || (leg = LEFT_FRONT)){
        side = LEFT;
    }else{
        side = RIGHT;
    }

    if (side == RIGHT)
    {
       
        servo_angles.data[0] = 135 - (q1 * RAD2DEG);
        servo_angles.data[1] = 135 + (q2 * RAD2DEG);
        servo_angles.data[2] = 135 + (q3 * RAD2DEG);
    }

    if (side == LEFT)
    {
     
        servo_angles.data[0] = 135 + (q1 * RAD2DEG);
        servo_angles.data[1] = 135 - (q2 * RAD2DEG);
        servo_angles.data[2] = 135 - (q3 * RAD2DEG);

    }

    setServoAngle(leg, 0, servo_angles.data[0]);
    setServoAngle(leg, 1, servo_angles.data[1]);
    setServoAngle(leg, 2, servo_angles.data[2]);
}

Vector3 calculateInvertedKinematics(Vector3 leg_pos, LegPosition side){
    Vector3 angles;
    double Q1, Q2, Q3;
    double Q3_1, Q3_2;

    double Xp = leg_pos.data[0];
    double Yp = leg_pos.data[1];
    double Zp = leg_pos.data[2];

    double Xpz;

    if (side == LEFT)
    {
        Xp = -Xp;
    }

    Xpz = sqrt((Xp * Xp) + (Yp * Yp));
    Q1 = atan2(Yp, Xp);
    Q3_1 = MY_PI - acos(((L2 * L2) + (L3 * L3) - (Zp * Zp) - ((Xpz - L1) * (Xpz - L1))) / (2 * L2 * L3));
    Q3_2 = acos(((Zp * Zp) + ((Xpz - L1) * (Xpz - L1)) - (L2 * L2) - (L3 * L3)) / (2 * L2 * L3));
    Q3 = -Q3_2; // lub Q3_2
    Q2 = atan2(Zp, (Xpz - L1)) + atan2(L3 * sin(-Q3), (L2 + L3 * cos(-Q3)));


    angles.data[0] = Q1;
    angles.data[1] = Q2;
    angles.data[2] = Q3;


    printf("Q1 = %.2f\n", angles.data[0]*RAD2DEG);
    printf("Q2 = %.2f\n", angles.data[1]*RAD2DEG);
    printf("Q3 = %.2f\n", angles.data[2]*RAD2DEG);

    return angles;


}



int checkPosition(LegPosition pos, double x, double y, double z)
{

    switch (pos)
    {
    case LEFT_FRONT:
        if (x < -L1 - d1 - L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT FRONT, x = %2.f, warunek: X < %2.f\n", x, -L1 - d1 - L23);
            global_error++;
            return 0;
        }
        break;

    case LEFT_MIDDLE:
        if (x < -L1 - d2 - L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT MIDDLE, x = %2.f, warunek: X < %2.f\n", x, -L1 - d2 - L23);
            global_error++;
            return 0;
        }
        break;

    case LEFT_BACK:
        if (x < -L1 - d1 - L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi LEFT BACK, x = %2.f, warunek: X < %2.f\n", x, -L1 - d1 - L23);
            global_error++;
            return 0;
        }
        break;

    case RIGHT_FRONT:
        if (x > L1 + d1 + L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT FRONT, x = %2.f, warunek: X > %2.f\n", x, L1 + d1 + L23);
            global_error++;
            return 0;
        }
        break;

    case RIGHT_MIDDLE:
        if (x > L1 + d2 + L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT FRONT, x = %2.f, warunek: X > %2.f\n", x, L1 + d2 + L23);
            global_error++;
            return 0;
        }
        break;

    case RIGHT_BACK:
        if (x > L1 + d1 + L23)
        {
            return 1;
        }
        else
        {
            printf("Nieprawidlowa pozycja X dla nogi RIGHT BACK, x = %2.f, warunek: X > %2.f\n", x, L1 + d1 + L23);
            global_error++;
            return 0;
        }
        break;

    default:
        printf("Nieprawidlowa pozycja nogi\n");
        global_error++;
        break;
    }
}

double z_0 = 0;
Vector3 getAnglesRobotCenter(LegPosition pos, RobotSide side, Vector3 global_pos)
{
    double x, y, z;

    x = global_pos.data[0];
    y = global_pos.data[1];
    z = global_pos.data[2];

    double xp, yp, zp;
    zp = z + z_0;

    Vector3 global_pos_return;

    Vector3 leg_pos;

    if (checkPosition(pos, x, y, z))
    {

        

        switch (pos)
        {
        case LEFT_FRONT:
            xp = x + d1;
            yp = y - d3;
            break;

        case LEFT_MIDDLE:
            xp = x + d2;
            yp = y;
            break;

        case LEFT_BACK:
            xp = x + d1;
            yp = y + d3;
            break;

        case RIGHT_FRONT:
            xp = x - d1;
            yp = y - d3;
            break;

        case RIGHT_MIDDLE:
            xp = x - d2;
            yp = y;
            break;

        case RIGHT_BACK:
            xp = x - d1;
            yp = y + d3;
            break;

        default:
            printf("bledna pozycja nogi w liczeniu kinematyki nogi wzgledem srodka\n");
            global_error++;
            break;
        }

        leg_pos.data[0] = xp;
        leg_pos.data[1] = yp;
        leg_pos.data[2] = zp;
        global_pos = calculateInvertedKinematics(leg_pos, side);

        
    }
    return global_pos;
}


Vector3 getPositionFromAngles(LegPosition pos, RobotSide side, double q1, double q2, double q3) {
    double Q1 = q1;
    double Q2 = q2;
    double Q3 = q3;

    Vector3 position;

    // Oblicz współrzędne końcówki nogi w układzie odniesienia nogi
    double Xpz = L1 + L2 * cos(Q2) + L3 * cos(Q2 + Q3);
    double Xp = Xpz * cos(Q1);
    double Yp = Xpz * sin(Q1);
    double Zp = L2 * sin(Q2) + L3 * sin(Q2 + Q3);

    // Dopasuj znak osi X zależnie od strony robota
    if (side == LEFT) {
        Xp = -Xp;
    }

    // Przekształcenie współrzędnych nogi na globalne współrzędne robota
    switch (pos) {
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

// Funkcja pomocnicza do obliczania różnicy czasu w sekundach
double get_time_diff_in_seconds(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
}

#include <stdio.h>
#include <time.h>
#include <unistd.h>

int main() {
    Matrix3 jacobian, inversed_jacobian;
    Vector3 d_positions, d_angles;

    Vector3 start_angles, start_pos;
    RobotSide leg_side;
    LegPosition leg = RIGHT_FRONT;

    if((leg == LEFT_BACK) || (leg == LEFT_MIDDLE) || (leg == LEFT_FRONT)) {
        leg_side = LEFT;
    } else {
        leg_side = RIGHT;
    }

    double k;

    start_pos.data[0] = x_const;
    start_pos.data[1] = y_const;
    start_pos.data[2] = z_const_stand_up;

    printf("pozycja startowa:");
    printVector(start_pos);

    start_angles = getAnglesRobotCenter(leg, leg_side, start_pos);

    double q1 = start_angles.data[0];
    double q2 = start_angles.data[1];
    double q3 = start_angles.data[2];
    
    setServosAngles(leg, q1, q2, q3);

    if (leg_side == RIGHT) {
        k = 1;
    } else {
        k = -1;
    }

    Vector3 q_start, q_end;

    q_start.data[0] = q1 * RAD2DEG;
    q_start.data[1] = q2 * RAD2DEG;
    q_start.data[2] = q3 * RAD2DEG;

    Vector3 move;
    move.data[0] = 0;
    move.data[1] = 0;
    move.data[2] = 10;

    double delta_time = 0.01; // Czas w sekundach na aktualizację kątów
    double total_time = 5; // Całkowity czas ruchu w sekundach

    d_positions.data[0] = move.data[0] / total_time;
    d_positions.data[1] = move.data[1] / total_time;
    d_positions.data[2] = move.data[2] / total_time;

    int num_steps = (int)(total_time / delta_time); // Liczba kroków do wykonania

    jacobian = createJakobian(q1, q2, q3, L1, L2, L3, k);

    if (inverseMatrix(jacobian, &inversed_jacobian)) {
        // Odwrotność Jakobianu
    } else {
        printf("Macierz jest nieodwracalna.\n");
        return 1;
    }

    Vector3 target_velocity;
    target_velocity.data[0] = d_positions.data[0];
    target_velocity.data[1] = d_positions.data[1];
    target_velocity.data[2] = d_positions.data[2];

    Vector3 joint_velocities = multiplyMatrixByVector(inversed_jacobian, target_velocity);

    printf("joint velocities");
    printVector(joint_velocities);

    struct timespec start_time, end_time;

    for (int step = 0; step < num_steps; step++) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        q1 += joint_velocities.data[0] * delta_time;
        q2 += joint_velocities.data[1] * delta_time;
        q3 += joint_velocities.data[2] * delta_time;

        setServosAngles(leg, q1, q2, q3);

        /*Vector3 current_position = getPositionFromAngles(leg, leg_side, q1,q2,q3 );
        printf("Krok %d: aktualna pozycja końcówki: x=%.2f, y=%.2f, z=%.2f\n",
               step, current_position.data[0], current_position.data[1], current_position.data[2]);*/

        clock_gettime(CLOCK_MONOTONIC, &end_time);

        double elapsed_time = (end_time.tv_sec - start_time.tv_sec) +
                              (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
        if (elapsed_time < delta_time) {
            usleep((delta_time - elapsed_time) * 1000000);
        }
    }

    Vector3 expected_end_position = start_pos;
    expected_end_position.data[0] += move.data[0];
    expected_end_position.data[1] += move.data[1];
    expected_end_position.data[2] += move.data[2];

    Vector3 final_position = getPositionFromAngles(leg, leg_side, q1,q2,q3 );
    
        printf("Krok %d: aktualna pozycja końcówki: x=%.2f, y=%.2f, z=%.2f\n",
               0, final_position.data[0], final_position.data[1], final_position.data[2]);

    printf("Różnica: Δx=%.2f, Δy=%.2f, Δz=%.2f\n",
           final_position.data[0] - expected_end_position.data[0],
           final_position.data[1] - expected_end_position.data[1],
           final_position.data[2] - expected_end_position.data[2]);

     printf("Aktualne kąty przegubów: q1 = %.2f°, q2 = %.2f°, q3 = %.2f°\n", q1 * RAD2DEG, q2 * RAD2DEG, q3 * RAD2DEG);


    return 0;
}
