#ifndef ENUMS_H
#define ENUMS_H

// pomocniczy enum do uzywania X Y Z w wektorach pozycji zamiast 0,1,2
typedef enum
{
    X,
    Y,
    Z
} Coord;

// pomocniczy enum do uzywania ROT_XYZ w wektorach kątów zamiast 0,1,2
typedef enum
{
    ROT_X,
    ROT_Y,
    ROT_Z
} Rot;

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
} LegType;

/*
Protraction – This is the phase where the robot’s leg is lifted and moved forward in preparation
for contact with the ground in a new position. During protraction, the leg swings forward to set
up the next step. This phase is essential for redistributing weight and preparing for the next stride.

Retraction – This phase occurs when the robot’s leg presses down against the ground and moves backward
relative to the robot's body, pushing the robot forward. Retraction is when the leg is in contact with
the ground and generates the propulsive force needed for movement.
*/

typedef enum
{
    UNKNOWN,
    TRANSPORT_POSITION,
    WALKING_POSITION,
    STAND_UP,
    SIT_DOWN,
    PREPARE_FOR_WALK,
    STOP_WALK,
    WALK
    
} StepFase;

typedef enum
{
    UNKNOWN_LEG_FASE,
    FRONT_POS,
    IN_RETRACTION,
    BACK_POS,
    IN_PROTRACTION

} LegFase;


#endif // ENUMS.H