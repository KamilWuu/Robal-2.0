#ifndef ENUMS_H
#define ENUMS_H

// Definicja typu dla stron robota (lewa, prawa)
typedef enum
{
    X,
    Y,
    Z
} Coord;

// Definicja typu dla stron robota (lewa, prawa)
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
    // LF_LB_RM_PROT, //przygotowanie do ruchu, protrakcja LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE  bez retrakcji pozostalych
    // LM_RF_RB_PROT, //przygotowanie do ruchu, protrakcja LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK bez retrakcji pozostalych
    // LF_LB_RM_PROT__LM_RF_RB_RETR,   // LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE_PROTRACTION__LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK_RETRACTION
    // LF_LB_RM_RETR__LM_RF_RB_PROT    // LEFT_FRONT_LEFT_BACK_RIGHT_MIDDLE_RETRACTION__LEFT_MIDDLE_RIGHT_FRONT_RIGHT_BACK_PROTRACTION
} StepFase;

typedef enum
{
    UNKNOWN_LEG_FASE,

    FRONT_POS,
    IN_RETRACTION,
    BACK_POS,
    IN_PROTRACTION

} LegFase;

const char *LegFaseTab[] = {
        "UNKNOWN_LEG_FASE",
        "FRONT_POS",
        "IN_RETRACTION",
        "BACK_POS",
        "IN_PROTRACTION"
    };

#endif // ENUMS.H