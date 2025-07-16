#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware_config.h"

// Function prototypes
void motor_control_init(void);
bool is_motor_active(int motor_id);
void emergency_stop_motor(int motor_id);
void set_motor_speed(int motor_id, int16_t speed);
void update_motor_speed_smooth(int motor_id, int16_t target_speed);
void update_pcnt_direction(int motor_id, int16_t new_speed);
void select_motor_group(motor_group_t group);
void set_expansion_relay(uint8_t relay_mask);
void update_positions(void);
void calculate_sync_speeds(void);
void position_control_task(void *arg);

// Global state variables
extern motor_group_t active_group;
extern uint16_t active_motors;
extern uint8_t mcp_porta;
extern uint8_t mcp_portb;
extern bool sync_mode;
extern uint8_t expansion_relay_state;

#endif // MOTOR_CONTROL_H