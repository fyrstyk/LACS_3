#include "hardware_config.h"

// Motor Configuration
motor_config_t motors[11] = {
    // Closed-loop motors (with position feedback)
    {"Leg A",       MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_15, GPIO_NUM_21, GPIO_NUM_17, LEDC_CHANNEL_0, LEDC_CHANNEL_1, NULL, NULL, 1},
    {"Leg B",       MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_36, GPIO_NUM_18, GPIO_NUM_16, LEDC_CHANNEL_2, LEDC_CHANNEL_3, NULL, NULL, 2},
    {"Leg C",       MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_38, GPIO_NUM_34, GPIO_NUM_35, LEDC_CHANNEL_4, LEDC_CHANNEL_5, NULL, NULL, 3},
    {"Leg D",       MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_47, GPIO_NUM_41, GPIO_NUM_42, LEDC_CHANNEL_6, LEDC_CHANNEL_7, NULL, NULL, 4},
    {"Elevation A", MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_33, GPIO_NUM_21, GPIO_NUM_17, LEDC_CHANNEL_0, LEDC_CHANNEL_1, NULL, NULL, 1},
    {"Elevation B", MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_37, GPIO_NUM_18, GPIO_NUM_16, LEDC_CHANNEL_2, LEDC_CHANNEL_3, NULL, NULL, 2},
    {"Slew",        MOTOR_TYPE_CLOSED_LOOP, GPIO_NUM_2,  GPIO_NUM_41, GPIO_NUM_42, LEDC_CHANNEL_6, LEDC_CHANNEL_7, NULL, NULL, 4},
    
    // Open-loop motors (no position feedback)
    {"Left Extension",   MOTOR_TYPE_OPEN_LOOP, -1, GPIO_NUM_34, GPIO_NUM_35, LEDC_CHANNEL_4, LEDC_CHANNEL_5, NULL, NULL, 3},
    {"Right Extension",  MOTOR_TYPE_OPEN_LOOP, -1, GPIO_NUM_34, GPIO_NUM_35, LEDC_CHANNEL_4, LEDC_CHANNEL_5, NULL, NULL, 3},
    {"Top Extension",    MOTOR_TYPE_OPEN_LOOP, -1, GPIO_NUM_41, GPIO_NUM_42, LEDC_CHANNEL_6, LEDC_CHANNEL_7, NULL, NULL, 4},
    {"Bottom Extension", MOTOR_TYPE_OPEN_LOOP, -1, GPIO_NUM_41, GPIO_NUM_42, LEDC_CHANNEL_6, LEDC_CHANNEL_7, NULL, NULL, 4}
};

motor_state_t motor_states[11] = {
    // Closed-loop motors with calibration
    {0, 29.19f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},   // Leg A
    {0, 29.00f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},   // Leg B
    {0, 28.95f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},   // Leg C
    {0, 28.94f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},   // Leg D
    {0, 1.0f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},     // Elev A
    {0, 1.0f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},     // Elev B
    {0, 47.2f, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},    // Slew
    
    // Open-loop motors
    {0, 0, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},        // Left Extension
    {0, 0, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},        // Right Extension
    {0, 0, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false},        // Top Extension
    {0, 0, 0, 0, 0, 0, MAX_SPEED, 0, false, false, false, 0, false}         // Bottom Extension
};

sync_state_t sync_state = {false, 0, 0, 0, 0, 0, false};