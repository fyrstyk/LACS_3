#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

// MCP23017 I2C Configuration
#define I2C_MASTER_SCL_IO      GPIO_NUM_0
#define I2C_MASTER_SDA_IO      GPIO_NUM_1
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define MCP23017_ADDR          0x20

// MCP23017 Registers
#define MCP23017_IODIRA    0x00
#define MCP23017_IODIRB    0x01
#define MCP23017_GPIOA     0x12
#define MCP23017_GPIOB     0x13
#define MCP23017_OLATA     0x14
#define MCP23017_OLATB     0x15

// W5500 Ethernet Pins
#define W5500_MISO_PIN     GPIO_NUM_12
#define W5500_MOSI_PIN     GPIO_NUM_11
#define W5500_SCLK_PIN     GPIO_NUM_13
#define W5500_CS_PIN       GPIO_NUM_14
#define W5500_INT_PIN      GPIO_NUM_10
#define W5500_RST_PIN      GPIO_NUM_9

// Motion control parameters
#define MIN_SPEED_EXTEND       150
#define MIN_SPEED_RETRACT      60
#define MAX_SPEED              250
#define MIN_SPEED              150
#define SPEED_RAMP_RATE        5
#define SYNC_TOLERANCE_LEG     0.5f
#define SYNC_TOLERANCE_ELEV    0.5f
#define APPROACH_DISTANCE      3.0f
#define FINE_APPROACH_DISTANCE 2.0f

// Motor types
typedef enum {
    MOTOR_TYPE_CLOSED_LOOP,
    MOTOR_TYPE_OPEN_LOOP
} motor_type_t;

// Motor Configuration
typedef struct {
    const char *name;
    motor_type_t type;
    int sensor_pin;
    gpio_num_t pwm1_pin;
    gpio_num_t pwm2_pin;
    ledc_channel_t pwm1_channel;
    ledc_channel_t pwm2_channel;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_channel;
    uint8_t h_bridge;
} motor_config_t;

// Motor groups
typedef enum {
    GROUP_NONE = 0,
    GROUP_LEGS = 1,
    GROUP_ELEVATION_PANELS = 2,
    GROUP_SLEW = 3
} motor_group_t;

// Motor state
typedef struct {
    float position;
    float pulses_per_unit;
    int32_t pulse_count;
    int32_t pulse_offset;
    int16_t speed;
    int16_t target_speed;
    int16_t max_speed;
    float target_position;
    bool is_moving;
    bool is_active;
    bool pcnt_initialized;
    int16_t last_direction;
    bool pcnt_inverted;
} motor_state_t;

// Synchronized movement state
typedef struct {
    bool enabled;
    uint16_t motor_mask;
    float target_position;
    float tolerance;
    float max_error;
    float avg_position;
    bool is_moving;
} sync_state_t;

// External declarations
extern motor_config_t motors[11];
extern motor_state_t motor_states[11];
extern sync_state_t sync_state;

// Global hardware resources
extern i2c_master_dev_handle_t mcp_handle;
extern i2c_master_bus_handle_t i2c_bus;

// Hardware function declarations
esp_err_t mcp23017_write(uint8_t reg, uint8_t value);

#endif // HARDWARE_CONFIG_H